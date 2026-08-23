use crate::alloc_prelude::*;
use crate::math::{Real, Vector};

use super::{RoutedRopePulleyGeometry, RoutedRopeStatus, RoutedRopeWinding};

const EPSILON: Real = 1.0e-6;
const TAU: Real = core::f32::consts::TAU;

#[derive(Copy, Clone, Debug)]
struct Contact {
    point: Vector,
    orientation: i8,
}

#[derive(Copy, Clone, Debug)]
struct Segment {
    from: Vector,
    to: Vector,
    length: Real,
    direction: Vector,
}

#[derive(Clone, Debug)]
pub(super) struct RoutedRopeRoute {
    pub total_length: Real,
    /// Endpoint A, endpoint B, then one gradient for every pulley center.
    pub gradients: Vec<Vector>,
}

#[inline]
fn cross(left: Vector, right: Vector) -> Real {
    left.x * right.y - left.y * right.x
}

#[inline]
fn perpendicular(value: Vector) -> Vector {
    Vector::new(-value.y, value.x)
}

#[inline]
fn orientation(winding: RoutedRopeWinding) -> i8 {
    match winding {
        RoutedRopeWinding::Clockwise => -1,
        RoutedRopeWinding::Counterclockwise => 1,
    }
}

fn positive_angle(value: Real) -> Real {
    ((value % TAU) + TAU) % TAU
}

fn tangent_contacts(point: Vector, center: Vector, radius: Real) -> Vec<Contact> {
    let offset = point - center;
    let distance_squared = offset.length_squared();
    if distance_squared <= radius * radius + EPSILON {
        return Vec::new();
    }
    let along = radius * radius / distance_squared;
    let across = radius * (distance_squared - radius * radius).sqrt() / distance_squared;
    let base = center + offset * along;
    let side = perpendicular(offset) * across;
    [base + side, base - side]
        .into_iter()
        .filter_map(|contact| {
            let radial = (contact - center) / radius;
            let delta = contact - point;
            let length = delta.length();
            (length > EPSILON).then(|| Contact {
                point: contact,
                orientation: cross(radial, delta / length).signum() as i8,
            })
        })
        .collect()
}

fn segment(from: Vector, to: Vector) -> Option<Segment> {
    let delta = to - from;
    let length = delta.length();
    (length > EPSILON).then(|| Segment {
        from,
        to,
        length,
        direction: delta / length,
    })
}

fn compare_segments(left: &Segment, right: &Segment) -> core::cmp::Ordering {
    left.length
        .total_cmp(&right.length)
        .then_with(|| left.from.x.total_cmp(&right.from.x))
        .then_with(|| left.from.y.total_cmp(&right.from.y))
}

fn tangent_orientation(direction: Vector, point: Vector, center: Vector) -> i8 {
    let offset = point - center;
    let length = offset.length();
    if length <= EPSILON {
        return 0;
    }
    cross(offset / length, direction).signum() as i8
}

fn point_to_wheel(point: Vector, wheel: &RoutedRopePulleyGeometry) -> Option<Segment> {
    let expected = orientation(wheel.winding);
    let mut candidates: Vec<_> = tangent_contacts(point, wheel.center, wheel.radius)
        .into_iter()
        .filter(|entry| entry.orientation == expected)
        .filter_map(|entry| segment(point, entry.point))
        .collect();
    candidates.sort_by(compare_segments);
    candidates.into_iter().next()
}

fn wheel_to_point(wheel: &RoutedRopePulleyGeometry, point: Vector) -> Option<Segment> {
    let expected = orientation(wheel.winding);
    let mut candidates: Vec<_> = tangent_contacts(point, wheel.center, wheel.radius)
        .into_iter()
        .filter_map(|entry| segment(entry.point, point))
        .filter(|entry| tangent_orientation(entry.direction, entry.from, wheel.center) == expected)
        .collect();
    candidates.sort_by(compare_segments);
    candidates.into_iter().next()
}

fn wheel_to_wheel(
    first: &RoutedRopePulleyGeometry,
    second: &RoutedRopePulleyGeometry,
) -> Option<Segment> {
    let delta = second.center - first.center;
    let distance_squared = delta.length_squared();
    let mut candidates = Vec::new();
    for radius_sign in [1.0, -1.0] {
        let radius_delta = first.radius - radius_sign * second.radius;
        let height_squared = distance_squared - radius_delta * radius_delta;
        if height_squared <= EPSILON {
            continue;
        }
        for side in [1.0, -1.0] {
            let normal = (delta * radius_delta
                + perpendicular(delta) * (height_squared.sqrt() * side))
                / distance_squared;
            let Some(candidate) = segment(
                first.center + normal * first.radius,
                second.center + normal * (radius_sign * second.radius),
            ) else {
                continue;
            };
            if tangent_orientation(candidate.direction, candidate.from, first.center)
                == orientation(first.winding)
                && tangent_orientation(candidate.direction, candidate.to, second.center)
                    == orientation(second.winding)
            {
                candidates.push(candidate);
            }
        }
    }
    candidates.sort_by(compare_segments);
    candidates.into_iter().next()
}

fn segment_clear(
    candidate: &Segment,
    wheels: &[RoutedRopePulleyGeometry],
    adjacent_left: isize,
    adjacent_right: isize,
) -> bool {
    let delta = candidate.to - candidate.from;
    let denominator = delta.length_squared();
    wheels.iter().enumerate().all(|(index, wheel)| {
        if index as isize == adjacent_left || index as isize == adjacent_right {
            return true;
        }
        let amount = ((wheel.center - candidate.from).dot(delta) / denominator).clamp(0.0, 1.0);
        (wheel.center - (candidate.from + delta * amount)).length() > wheel.radius + EPSILON
    })
}

pub(super) fn resolve_route(
    endpoint_a: Vector,
    endpoint_b: Vector,
    wheels: &[RoutedRopePulleyGeometry],
) -> Result<RoutedRopeRoute, RoutedRopeStatus> {
    if !endpoint_a.is_finite()
        || !endpoint_b.is_finite()
        || wheels.iter().any(|wheel| {
            !wheel.center.is_finite() || !wheel.radius.is_finite() || wheel.radius <= 0.0
        })
    {
        return Err(RoutedRopeStatus::InvalidCoordinates);
    }
    if wheels.is_empty() {
        return Err(RoutedRopeStatus::NoPulleys);
    }
    for left in 0..wheels.len() {
        for right in left + 1..wheels.len() {
            if (wheels[left].center - wheels[right].center).length()
                <= wheels[left].radius + wheels[right].radius + EPSILON
            {
                return Err(RoutedRopeStatus::PulleyOverlap);
            }
        }
    }

    let mut segments = Vec::with_capacity(wheels.len() + 1);
    segments
        .push(point_to_wheel(endpoint_a, &wheels[0]).ok_or(RoutedRopeStatus::TangentUnavailable)?);
    for index in 0..wheels.len() - 1 {
        segments.push(
            wheel_to_wheel(&wheels[index], &wheels[index + 1])
                .ok_or(RoutedRopeStatus::TangentUnavailable)?,
        );
    }
    segments.push(
        wheel_to_point(&wheels[wheels.len() - 1], endpoint_b)
            .ok_or(RoutedRopeStatus::TangentUnavailable)?,
    );
    if segments
        .iter()
        .enumerate()
        .any(|(index, entry)| !segment_clear(entry, wheels, index as isize - 1, index as isize))
    {
        return Err(RoutedRopeStatus::SegmentObstructed);
    }

    let mut arc_length = 0.0;
    for (index, wheel) in wheels.iter().enumerate() {
        let from = segments[index].to - wheel.center;
        let to = segments[index + 1].from - wheel.center;
        let from_angle = from.y.atan2(from.x);
        let to_angle = to.y.atan2(to.x);
        let sweep = match wheel.winding {
            RoutedRopeWinding::Counterclockwise => positive_angle(to_angle - from_angle),
            RoutedRopeWinding::Clockwise => -positive_angle(from_angle - to_angle),
        };
        arc_length += sweep.abs() * wheel.radius;
    }
    let total_length = segments.iter().map(|entry| entry.length).sum::<Real>() + arc_length;
    let mut gradients = Vec::with_capacity(wheels.len() + 2);
    gradients.push(-segments[0].direction);
    gradients.push(segments[segments.len() - 1].direction);
    gradients.extend(
        (0..wheels.len()).map(|index| segments[index].direction - segments[index + 1].direction),
    );
    Ok(RoutedRopeRoute {
        total_length,
        gradients,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn numbers(value: &str) -> Vec<Real> {
        value
            .split(':')
            .map(|part| part.parse::<Real>().expect("golden number"))
            .collect()
    }

    #[test]
    fn matches_shared_typescript_golden_routes() {
        for line in include_str!("golden_routes.csv").lines() {
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let columns: Vec<_> = line.split(',').collect();
            let endpoint_a = Vector::new(
                columns[2].parse().expect("a.x"),
                columns[3].parse().expect("a.y"),
            );
            let endpoint_b = Vector::new(
                columns[4].parse().expect("b.x"),
                columns[5].parse().expect("b.y"),
            );
            let wheels: Vec<_> = columns[6]
                .split(';')
                .map(|encoded| {
                    let fields: Vec<_> = encoded.split(':').collect();
                    RoutedRopePulleyGeometry {
                        center: Vector::new(
                            fields[0].parse().expect("wheel.x"),
                            fields[1].parse().expect("wheel.y"),
                        ),
                        radius: fields[2].parse().expect("wheel radius"),
                        winding: if fields[3] == "cw" {
                            RoutedRopeWinding::Clockwise
                        } else {
                            RoutedRopeWinding::Counterclockwise
                        },
                    }
                })
                .collect();
            let resolved = resolve_route(endpoint_a, endpoint_b, &wheels);
            if columns[1] == "pulley-overlap" {
                assert_eq!(resolved.unwrap_err(), RoutedRopeStatus::PulleyOverlap);
                continue;
            }
            let route = resolved.unwrap_or_else(|status| panic!("{}: {status:?}", columns[0]));
            let expected_length: Real = columns[7].parse().expect("length");
            assert!((route.total_length - expected_length).abs() < 2.0e-6);
            let expected_endpoints = [
                Vector::new(columns[8].parse().unwrap(), columns[9].parse().unwrap()),
                Vector::new(columns[10].parse().unwrap(), columns[11].parse().unwrap()),
            ];
            for (actual, expected) in route.gradients[..2].iter().zip(expected_endpoints) {
                assert!((*actual - expected).length() < 2.0e-6);
            }
            for (actual, encoded) in route.gradients[2..].iter().zip(columns[12].split(';')) {
                let expected = numbers(encoded);
                assert!((*actual - Vector::new(expected[0], expected[1])).length() < 2.0e-6);
            }
        }
    }

    #[test]
    fn resolves_a_fixed_pulley_route_and_gradients() {
        let wheel = RoutedRopePulleyGeometry {
            center: Vector::new(0.0, 0.0),
            radius: 0.2,
            winding: RoutedRopeWinding::Clockwise,
        };
        let route = resolve_route(Vector::new(-1.0, 0.0), Vector::new(0.0, -1.0), &[wheel])
            .expect("valid fixed-pulley route");
        assert!(route.total_length > 1.9 && route.total_length < 3.1);
        assert_eq!(route.gradients.len(), 3);
        assert!(route.gradients.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn rejects_overlapping_pulleys() {
        let wheels = [
            RoutedRopePulleyGeometry {
                center: Vector::new(0.0, 0.0),
                radius: 0.2,
                winding: RoutedRopeWinding::Clockwise,
            },
            RoutedRopePulleyGeometry {
                center: Vector::new(0.3, 0.0),
                radius: 0.2,
                winding: RoutedRopeWinding::Clockwise,
            },
        ];
        assert_eq!(
            resolve_route(Vector::new(-1.0, 0.0), Vector::new(1.0, 0.0), &wheels).unwrap_err(),
            RoutedRopeStatus::PulleyOverlap
        );
    }
}

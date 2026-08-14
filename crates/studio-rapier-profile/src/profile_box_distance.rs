use rapier2d::math::{Pose, Real, Vector};
use rapier2d::parry::shape::Cuboid;

use crate::profile::{AnalyticProfile, ProfileSegment, angle_on_sweep};

const DISTANCE_EPSILON: Real = 1.0e-7;

#[derive(Clone, Copy)]
pub(crate) struct ClosestPair {
    pub profile: Vector,
    pub other: Vector,
    pub fallback_normal: Vector,
}

impl ClosestPair {
    fn distance_squared(self) -> Real {
        (self.other - self.profile).length_squared()
    }
}

pub(crate) fn closest_profile_box_pair(
    profile: &AnalyticProfile,
    transform: &Pose,
    cuboid: &Cuboid,
) -> ClosestPair {
    let corners = box_corners(transform, cuboid);
    let mut closest: Option<ClosestPair> = None;
    for segment in profile.segments() {
        for index in 0..4 {
            let start = corners[index];
            let end = corners[(index + 1) % 4];
            let tangent = end - start;
            let inward = normalized_or(Vector::new(-tangent.y, tangent.x), Vector::Y);
            let candidate = match segment {
                ProfileSegment::Line {
                    start: first,
                    end: second,
                } => closest_line_line(*first, *second, start, end, inward),
                ProfileSegment::Arc {
                    center,
                    radius,
                    start_angle,
                    sweep,
                } => closest_arc_line(*center, *radius, *start_angle, *sweep, start, end, inward),
            };
            if closest
                .is_none_or(|current| candidate.distance_squared() < current.distance_squared())
            {
                closest = Some(candidate);
            }
        }
    }
    closest.unwrap_or(ClosestPair {
        profile: Vector::ZERO,
        other: Vector::ZERO,
        fallback_normal: Vector::Y,
    })
}

fn box_corners(transform: &Pose, cuboid: &Cuboid) -> [Vector; 4] {
    let x = cuboid.half_extents.x;
    let y = cuboid.half_extents.y;
    [
        transform * Vector::new(-x, -y),
        transform * Vector::new(x, -y),
        transform * Vector::new(x, y),
        transform * Vector::new(-x, y),
    ]
}

fn closest_point_on_line(point: Vector, start: Vector, end: Vector) -> Vector {
    let tangent = end - start;
    let denominator = tangent.length_squared();
    if denominator <= DISTANCE_EPSILON {
        return start;
    }
    start + tangent * ((point - start).dot(tangent) / denominator).clamp(0.0, 1.0)
}

fn closest_line_line(
    first_start: Vector,
    first_end: Vector,
    second_start: Vector,
    second_end: Vector,
    fallback_normal: Vector,
) -> ClosestPair {
    let first_tangent = first_end - first_start;
    let second_tangent = second_end - second_start;
    let offset = first_start - second_start;
    let a = first_tangent.length_squared();
    let e = second_tangent.length_squared();
    let b = first_tangent.dot(second_tangent);
    let c = first_tangent.dot(offset);
    let f = second_tangent.dot(offset);
    let denominator = a * e - b * b;
    let mut first_t = if denominator.abs() > DISTANCE_EPSILON {
        ((b * f - c * e) / denominator).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let second_t = ((b * first_t + f) / e.max(DISTANCE_EPSILON)).clamp(0.0, 1.0);
    first_t = ((b * second_t - c) / a.max(DISTANCE_EPSILON)).clamp(0.0, 1.0);
    ClosestPair {
        profile: first_start + first_tangent * first_t,
        other: second_start + second_tangent * second_t,
        fallback_normal,
    }
}

#[allow(clippy::too_many_arguments)]
fn closest_arc_line(
    center: Vector,
    radius: Real,
    start_angle: Real,
    sweep: Real,
    line_start: Vector,
    line_end: Vector,
    fallback_normal: Vector,
) -> ClosestPair {
    let arc_point = |angle: Real| center + Vector::new(angle.cos(), angle.sin()) * radius;
    let mut candidates = Vec::with_capacity(6);
    for angle in [start_angle, start_angle + sweep] {
        let profile = arc_point(angle);
        candidates.push(ClosestPair {
            profile,
            other: closest_point_on_line(profile, line_start, line_end),
            fallback_normal,
        });
    }
    for other in [line_start, line_end] {
        let offset = other - center;
        let angle = offset.y.atan2(offset.x);
        let selected = if angle_on_sweep(angle, start_angle, sweep) {
            angle
        } else if (other - arc_point(start_angle)).length_squared()
            <= (other - arc_point(start_angle + sweep)).length_squared()
        {
            start_angle
        } else {
            start_angle + sweep
        };
        candidates.push(ClosestPair {
            profile: arc_point(selected),
            other,
            fallback_normal,
        });
    }
    add_arc_line_interior_candidates(
        &mut candidates,
        center,
        radius,
        start_angle,
        sweep,
        line_start,
        line_end,
        fallback_normal,
    );
    candidates
        .into_iter()
        .min_by(|left, right| left.distance_squared().total_cmp(&right.distance_squared()))
        .unwrap()
}

#[allow(clippy::too_many_arguments)]
fn add_arc_line_interior_candidates(
    candidates: &mut Vec<ClosestPair>,
    center: Vector,
    radius: Real,
    start_angle: Real,
    sweep: Real,
    line_start: Vector,
    line_end: Vector,
    fallback_normal: Vector,
) {
    let tangent = line_end - line_start;
    let tangent_length = tangent.length();
    if tangent_length <= DISTANCE_EPSILON {
        return;
    }
    let unit_tangent = tangent / tangent_length;
    let line_normal = Vector::new(-unit_tangent.y, unit_tangent.x);
    let projection = line_start + unit_tangent * (center - line_start).dot(unit_tangent);
    let projection_t = (projection - line_start).dot(unit_tangent);
    if projection_t < 0.0 || projection_t > tangent_length {
        return;
    }
    for sign in [-1.0, 1.0] {
        let profile = center + line_normal * radius * sign;
        let angle = (profile.y - center.y).atan2(profile.x - center.x);
        if angle_on_sweep(angle, start_angle, sweep) {
            candidates.push(ClosestPair {
                profile,
                other: projection,
                fallback_normal,
            });
        }
    }
}

fn normalized_or(value: Vector, fallback: Vector) -> Vector {
    if value.length() > DISTANCE_EPSILON {
        value.normalize()
    } else {
        fallback
    }
}

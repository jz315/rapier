use std::f32::consts::{FRAC_PI_2, PI};

use rapier2d::math::{Real, Vector};

use crate::profile_segment::ProfileSegment;

const BOUNDARY_EPSILON: Real = 1.0e-6;

pub(crate) fn contains(segments: &[ProfileSegment], point: &Vector) -> bool {
    if segments.iter().any(|segment| {
        (segment.closest_point(point) - point).length_squared()
            <= BOUNDARY_EPSILON * BOUNDARY_EPSILON
    }) {
        return true;
    }
    let mut inside = false;
    for segment in segments {
        match segment {
            ProfileSegment::Line { start, end } => {
                if (start.y > point.y) != (end.y > point.y) {
                    let x = start.x + (point.y - start.y) * (end.x - start.x) / (end.y - start.y);
                    if x > point.x {
                        inside = !inside;
                    }
                }
            }
            ProfileSegment::Arc {
                center,
                radius,
                start_angle,
                sweep,
            } => {
                for (start, end) in monotone_arc_ranges(*start_angle, *sweep) {
                    let start_y = center.y + radius * start.sin();
                    let end_y = center.y + radius * end.sin();
                    if (start_y > point.y) == (end_y > point.y) {
                        continue;
                    }
                    let relative_y = point.y - center.y;
                    let x_offset = (radius * radius - relative_y * relative_y).max(0.0).sqrt();
                    let midpoint = (start + end) / 2.0;
                    let x = center.x + midpoint.cos().signum() * x_offset;
                    if x > point.x {
                        inside = !inside;
                    }
                }
            }
        }
    }
    inside
}

pub(crate) fn signed_area(segments: &[ProfileSegment]) -> Real {
    segments
        .iter()
        .map(|segment| match segment {
            ProfileSegment::Line { start, end } => (start.x * end.y - start.y * end.x) / 2.0,
            ProfileSegment::Arc {
                center,
                radius,
                start_angle,
                sweep,
            } => {
                let end_angle = start_angle + sweep;
                (radius * center.x * (end_angle.sin() - start_angle.sin())
                    - radius * center.y * (end_angle.cos() - start_angle.cos())
                    + radius * radius * sweep)
                    / 2.0
            }
        })
        .sum()
}

fn monotone_arc_ranges(start: Real, sweep: Real) -> Vec<(Real, Real)> {
    let end = start + sweep;
    let minimum = start.min(end);
    let maximum = start.max(end);
    let first_index = ((minimum - FRAC_PI_2) / PI).ceil() as i32;
    let last_index = ((maximum - FRAC_PI_2) / PI).floor() as i32;
    let mut breaks = vec![start];
    for index in first_index..=last_index {
        let angle = FRAC_PI_2 + index as Real * PI;
        if angle > minimum + BOUNDARY_EPSILON && angle < maximum - BOUNDARY_EPSILON {
            breaks.push(angle);
        }
    }
    if sweep < 0.0 {
        breaks[1..].reverse();
    }
    breaks.push(end);
    breaks.windows(2).map(|pair| (pair[0], pair[1])).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn notched_profile() -> Vec<ProfileSegment> {
        vec![
            ProfileSegment::Line {
                start: Vector::new(-2.0, 1.0),
                end: Vector::new(-0.5, 1.0),
            },
            ProfileSegment::Arc {
                center: Vector::new(0.0, 1.0),
                radius: 0.5,
                start_angle: PI,
                sweep: PI,
            },
            ProfileSegment::Line {
                start: Vector::new(0.5, 1.0),
                end: Vector::new(2.0, 1.0),
            },
            ProfileSegment::Line {
                start: Vector::new(2.0, 1.0),
                end: Vector::new(2.0, -1.0),
            },
            ProfileSegment::Line {
                start: Vector::new(2.0, -1.0),
                end: Vector::new(-2.0, -1.0),
            },
            ProfileSegment::Line {
                start: Vector::new(-2.0, -1.0),
                end: Vector::new(-2.0, 1.0),
            },
        ]
    }

    #[test]
    fn classifies_a_semicircular_notch_without_sampling() {
        let profile = notched_profile();
        assert!(contains(&profile, &Vector::new(0.0, 0.0)));
        assert!(!contains(&profile, &Vector::new(0.0, 0.75)));
        assert!(!contains(&profile, &Vector::new(0.0, 1.25)));
        assert!(contains(&profile, &Vector::new(1.0, 0.75)));
        assert!(signed_area(&profile) < 0.0);
    }

    #[test]
    fn classification_does_not_depend_on_contour_direction() {
        let reversed = notched_profile()
            .into_iter()
            .rev()
            .map(|segment| match segment {
                ProfileSegment::Line { start, end } => ProfileSegment::Line {
                    start: end,
                    end: start,
                },
                ProfileSegment::Arc {
                    center,
                    radius,
                    start_angle,
                    sweep,
                } => ProfileSegment::Arc {
                    center,
                    radius,
                    start_angle: start_angle + sweep,
                    sweep: -sweep,
                },
            })
            .collect::<Vec<_>>();
        assert!(contains(&reversed, &Vector::new(0.0, 0.0)));
        assert!(!contains(&reversed, &Vector::new(0.0, 0.75)));
        assert!(signed_area(&reversed) > 0.0);
    }
}

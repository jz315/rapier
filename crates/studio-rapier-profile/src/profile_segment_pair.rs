use rapier2d::math::{Real, Vector};

use crate::profile::{ProfileSegment, angle_on_sweep};

const EPSILON: Real = 1.0e-7;

#[derive(Clone, Copy)]
pub(crate) struct SegmentClosest {
    pub first: Vector,
    pub second: Vector,
}

impl SegmentClosest {
    pub fn distance_squared(self) -> Real {
        (self.second - self.first).length_squared()
    }

    fn flipped(self) -> Self {
        Self {
            first: self.second,
            second: self.first,
        }
    }
}

pub(crate) fn closest_segment_pair(
    first: &ProfileSegment,
    second: &ProfileSegment,
) -> SegmentClosest {
    match (first, second) {
        (ProfileSegment::Line { start: a, end: b }, ProfileSegment::Line { start: c, end: d }) => {
            closest_line_line(*a, *b, *c, *d)
        }
        (ProfileSegment::Arc { .. }, ProfileSegment::Line { start, end }) => {
            closest_arc_line(first, *start, *end)
        }
        (ProfileSegment::Line { start, end }, ProfileSegment::Arc { .. }) => {
            closest_arc_line(second, *start, *end).flipped()
        }
        (ProfileSegment::Arc { .. }, ProfileSegment::Arc { .. }) => closest_arc_arc(first, second),
    }
}

fn closest_line_line(a: Vector, b: Vector, c: Vector, d: Vector) -> SegmentClosest {
    let ab = b - a;
    let cd = d - c;
    let offset = a - c;
    let aa = ab.length_squared();
    let cc = cd.length_squared();
    let ac = ab.dot(cd);
    let ao = ab.dot(offset);
    let co = cd.dot(offset);
    let denominator = aa * cc - ac * ac;
    let mut ta = if denominator.abs() > EPSILON {
        ((ac * co - ao * cc) / denominator).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let tc = ((ac * ta + co) / cc.max(EPSILON)).clamp(0.0, 1.0);
    ta = ((ac * tc - ao) / aa.max(EPSILON)).clamp(0.0, 1.0);
    SegmentClosest {
        first: a + ab * ta,
        second: c + cd * tc,
    }
}

fn closest_arc_line(arc: &ProfileSegment, start: Vector, end: Vector) -> SegmentClosest {
    let mut candidates = Vec::with_capacity(10);
    for first in [arc.start(), arc.end()] {
        candidates.push(SegmentClosest {
            first,
            second: closest_on_line(first, start, end),
        });
    }
    for second in [start, end] {
        candidates.push(SegmentClosest {
            first: arc.closest_point(&second),
            second,
        });
    }
    let ProfileSegment::Arc {
        center,
        radius,
        start_angle,
        sweep,
    } = arc
    else {
        unreachable!()
    };
    let tangent = end - start;
    let length = tangent.length();
    if length > EPSILON {
        let unit = tangent / length;
        let normal = Vector::new(-unit.y, unit.x);
        let projection = start + unit * (*center - start).dot(unit);
        let line_t = (projection - start).dot(unit);
        if (0.0..=length).contains(&line_t) {
            for sign in [-1.0, 1.0] {
                let first = *center + normal * *radius * sign;
                let angle = (first.y - center.y).atan2(first.x - center.x);
                if angle_on_sweep(angle, *start_angle, *sweep) {
                    candidates.push(SegmentClosest {
                        first,
                        second: projection,
                    });
                }
            }
        }
        let relative = start - *center;
        let a = tangent.length_squared();
        let b = 2.0 * relative.dot(tangent);
        let c = relative.length_squared() - radius * radius;
        let discriminant = b * b - 4.0 * a * c;
        if discriminant >= -EPSILON {
            let root = discriminant.max(0.0).sqrt();
            for t in [(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)] {
                if (0.0..=1.0).contains(&t) {
                    let point = start + tangent * t;
                    let angle = (point.y - center.y).atan2(point.x - center.x);
                    if angle_on_sweep(angle, *start_angle, *sweep) {
                        candidates.push(SegmentClosest {
                            first: point,
                            second: point,
                        });
                    }
                }
            }
        }
    }
    minimum(candidates)
}

fn closest_arc_arc(first: &ProfileSegment, second: &ProfileSegment) -> SegmentClosest {
    let mut candidates = Vec::with_capacity(12);
    for point in [first.start(), first.end()] {
        candidates.push(SegmentClosest {
            first: point,
            second: second.closest_point(&point),
        });
    }
    for point in [second.start(), second.end()] {
        candidates.push(SegmentClosest {
            first: first.closest_point(&point),
            second: point,
        });
    }
    let ProfileSegment::Arc {
        center: c1,
        radius: r1,
        start_angle: a1,
        sweep: s1,
    } = first
    else {
        unreachable!()
    };
    let ProfileSegment::Arc {
        center: c2,
        radius: r2,
        start_angle: a2,
        sweep: s2,
    } = second
    else {
        unreachable!()
    };
    let delta = *c2 - *c1;
    let center_distance = delta.length();
    if center_distance > EPSILON {
        let axis = delta / center_distance;
        for sign in [-1.0, 1.0] {
            let p1 = *c1 + axis * *r1 * sign;
            let angle1 = (p1.y - c1.y).atan2(p1.x - c1.x);
            if angle_on_sweep(angle1, *a1, *s1) {
                candidates.push(SegmentClosest {
                    first: p1,
                    second: second.closest_point(&p1),
                });
            }
            let p2 = *c2 + axis * *r2 * sign;
            let angle2 = (p2.y - c2.y).atan2(p2.x - c2.x);
            if angle_on_sweep(angle2, *a2, *s2) {
                candidates.push(SegmentClosest {
                    first: first.closest_point(&p2),
                    second: p2,
                });
            }
        }
        let along =
            (r1 * r1 - r2 * r2 + center_distance * center_distance) / (2.0 * center_distance);
        let height_squared = r1 * r1 - along * along;
        if height_squared >= -EPSILON {
            let base = *c1 + axis * along;
            let perpendicular = Vector::new(-axis.y, axis.x) * height_squared.max(0.0).sqrt();
            for point in [base - perpendicular, base + perpendicular] {
                let angle1 = (point.y - c1.y).atan2(point.x - c1.x);
                let angle2 = (point.y - c2.y).atan2(point.x - c2.x);
                if angle_on_sweep(angle1, *a1, *s1) && angle_on_sweep(angle2, *a2, *s2) {
                    candidates.push(SegmentClosest {
                        first: point,
                        second: point,
                    });
                }
            }
        }
    }
    minimum(candidates)
}

fn closest_on_line(point: Vector, start: Vector, end: Vector) -> Vector {
    let tangent = end - start;
    let length_squared = tangent.length_squared();
    if length_squared <= EPSILON {
        start
    } else {
        start + tangent * ((point - start).dot(tangent) / length_squared).clamp(0.0, 1.0)
    }
}

fn minimum(candidates: Vec<SegmentClosest>) -> SegmentClosest {
    candidates
        .into_iter()
        .min_by(|left, right| left.distance_squared().total_cmp(&right.distance_squared()))
        .expect("analytic profile segments always produce a closest-point candidate")
}

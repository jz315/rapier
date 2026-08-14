use std::f32::consts::TAU;

use rapier2d::math::{Real, Vector};

const GEOMETRY_EPSILON: Real = 1.0e-5;

#[derive(Clone, Debug)]
pub enum ProfileSegment {
    Line {
        start: Vector,
        end: Vector,
    },
    Arc {
        center: Vector,
        radius: Real,
        start_angle: Real,
        sweep: Real,
    },
}

impl ProfileSegment {
    pub fn start(&self) -> Vector {
        match self {
            Self::Line { start, .. } => *start,
            Self::Arc {
                center,
                radius,
                start_angle,
                ..
            } => center + Vector::new(start_angle.cos(), start_angle.sin()) * *radius,
        }
    }

    pub fn end(&self) -> Vector {
        match self {
            Self::Line { end, .. } => *end,
            Self::Arc {
                center,
                radius,
                start_angle,
                sweep,
            } => {
                let angle = start_angle + sweep;
                center + Vector::new(angle.cos(), angle.sin()) * *radius
            }
        }
    }

    pub fn closest_point(&self, point: &Vector) -> Vector {
        match self {
            Self::Line { start, end } => {
                let tangent = end - start;
                let denominator = tangent.length_squared();
                if denominator <= Real::EPSILON {
                    return *start;
                }
                start + tangent * ((point - start).dot(tangent) / denominator).clamp(0.0, 1.0)
            }
            Self::Arc {
                center,
                radius,
                start_angle,
                sweep,
            } => {
                let offset = point - center;
                let angle = if offset.length_squared() <= Real::EPSILON {
                    *start_angle
                } else {
                    offset.y.atan2(offset.x)
                };
                let selected = if angle_on_sweep(angle, *start_angle, *sweep) {
                    angle
                } else {
                    let end_angle = start_angle + sweep;
                    if angular_distance(angle, *start_angle) <= angular_distance(angle, end_angle) {
                        *start_angle
                    } else {
                        end_angle
                    }
                };
                center + Vector::new(selected.cos(), selected.sin()) * *radius
            }
        }
    }

    pub fn tangent_at(&self, point: &Vector) -> Vector {
        match self {
            Self::Line { start, end } => end - start,
            Self::Arc { center, sweep, .. } => {
                let radial = point - center;
                Vector::new(-radial.y, radial.x) * sweep.signum()
            }
        }
    }

    pub(crate) fn include_bounds(&self, minimum: &mut Vector, maximum: &mut Vector) {
        include_point(minimum, maximum, self.start());
        include_point(minimum, maximum, self.end());
        if let Self::Arc {
            center,
            radius,
            start_angle,
            sweep,
        } = self
        {
            for angle in [0.0, TAU / 4.0, TAU / 2.0, TAU * 3.0 / 4.0] {
                if angle_on_sweep(angle, *start_angle, *sweep) {
                    include_point(
                        minimum,
                        maximum,
                        center + Vector::new(angle.cos(), angle.sin()) * *radius,
                    );
                }
            }
        }
    }
}

pub fn angle_on_sweep(angle: Real, start: Real, sweep: Real) -> bool {
    if sweep >= 0.0 {
        (angle - start).rem_euclid(TAU) <= sweep + GEOMETRY_EPSILON
    } else {
        (start - angle).rem_euclid(TAU) <= -sweep + GEOMETRY_EPSILON
    }
}

fn angular_distance(first: Real, second: Real) -> Real {
    let delta = (first - second).rem_euclid(TAU);
    delta.min(TAU - delta)
}

fn include_point(minimum: &mut Vector, maximum: &mut Vector, point: Vector) {
    minimum.x = minimum.x.min(point.x);
    minimum.y = minimum.y.min(point.y);
    maximum.x = maximum.x.max(point.x);
    maximum.y = maximum.y.max(point.y);
}

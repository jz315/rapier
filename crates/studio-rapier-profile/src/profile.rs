use rapier2d::math::{Real, Vector};
use rapier2d::parry::bounding_volume::Aabb;

use crate::profile_interior::{contains, signed_area};
pub use crate::profile_segment::{ProfileSegment, angle_on_sweep};

const GEOMETRY_EPSILON: Real = 1.0e-5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileMode {
    Outline,
    Solid,
}

#[derive(Clone, Copy)]
pub(crate) struct ClosestBoundary {
    pub point: Vector,
    pub tangent: Vector,
}

#[derive(Clone, Debug)]
pub struct AnalyticProfile {
    segments: Vec<ProfileSegment>,
    mode: ProfileMode,
    half_thickness: Real,
    counterclockwise: bool,
    aabb: Aabb,
    bounding_radius: Real,
}

impl AnalyticProfile {
    pub fn new(segments: Vec<ProfileSegment>, thickness: Real, mode: ProfileMode) -> Option<Self> {
        if segments.len() < 2 || !thickness.is_finite() || thickness <= 0.0 {
            return None;
        }
        for (index, segment) in segments.iter().enumerate() {
            let next = &segments[(index + 1) % segments.len()];
            if (segment.end() - next.start()).length() > GEOMETRY_EPSILON {
                return None;
            }
            if let ProfileSegment::Arc { radius, sweep, .. } = segment
                && (!radius.is_finite()
                    || *radius <= 0.0
                    || !sweep.is_finite()
                    || sweep.abs() <= GEOMETRY_EPSILON
                    || sweep.abs() > std::f32::consts::TAU + GEOMETRY_EPSILON)
            {
                return None;
            }
        }
        let half_thickness = thickness / 2.0;
        let mut minimum = Vector::new(Real::MAX, Real::MAX);
        let mut maximum = Vector::new(-Real::MAX, -Real::MAX);
        for segment in &segments {
            segment.include_bounds(&mut minimum, &mut maximum)
        }
        let expansion = Vector::splat(if mode == ProfileMode::Outline {
            half_thickness
        } else {
            0.0
        });
        let aabb = Aabb::new(minimum - expansion, maximum + expansion);
        let bounding_radius = aabb.mins.length().max(aabb.maxs.length());
        let area = signed_area(&segments);
        if mode == ProfileMode::Solid && area.abs() <= GEOMETRY_EPSILON {
            return None;
        }
        Some(Self {
            segments,
            mode,
            half_thickness,
            counterclockwise: area > 0.0,
            aabb,
            bounding_radius,
        })
    }

    pub fn segments(&self) -> &[ProfileSegment] {
        &self.segments
    }
    pub fn half_thickness(&self) -> Real {
        self.half_thickness
    }
    pub fn mode(&self) -> ProfileMode {
        self.mode
    }
    pub(crate) fn local_aabb(&self) -> Aabb {
        self.aabb
    }
    pub(crate) fn bounding_radius(&self) -> Real {
        self.bounding_radius
    }

    pub(crate) fn closest_boundary(&self, point: &Vector) -> ClosestBoundary {
        self.segments
            .iter()
            .map(|segment| {
                let boundary = segment.closest_point(point);
                ClosestBoundary {
                    point: boundary,
                    tangent: segment.tangent_at(&boundary),
                }
            })
            .min_by(|left, right| {
                (left.point - point)
                    .length_squared()
                    .total_cmp(&(right.point - point).length_squared())
            })
            .unwrap_or(ClosestBoundary {
                point: Vector::ZERO,
                tangent: Vector::X,
            })
    }

    pub(crate) fn outward_normal(&self, boundary: ClosestBoundary) -> Vector {
        let tangent = if boundary.tangent.length() > GEOMETRY_EPSILON {
            boundary.tangent.normalize()
        } else {
            Vector::X
        };
        if self.counterclockwise {
            Vector::new(tangent.y, -tangent.x)
        } else {
            Vector::new(-tangent.y, tangent.x)
        }
    }

    pub(crate) fn contains_interior(&self, point: &Vector) -> bool {
        contains(&self.segments, point)
    }
}

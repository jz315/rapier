use rapier2d::math::{Real, Vector};
use rapier2d::parry::bounding_volume::{Aabb, BoundingSphere};
use rapier2d::parry::mass_properties::MassProperties;
use rapier2d::parry::query::{PointProjection, PointQuery, Ray, RayCast, RayIntersection};
use rapier2d::parry::shape::{FeatureId, Shape, ShapeType, TypedShape};

use crate::profile::{AnalyticProfile, ProfileMode};

impl PointQuery for AnalyticProfile {
    fn project_local_point(&self, point: Vector, solid: bool) -> PointProjection {
        let boundary = self.closest_boundary(&point);
        let offset = point - boundary.point;
        let distance = offset.length();
        let normal = if distance > Real::EPSILON {
            offset / distance
        } else {
            self.outward_normal(boundary)
        };
        if self.mode() == ProfileMode::Solid {
            let inside = self.contains_interior(&point);
            return PointProjection::new(
                inside,
                if solid && inside {
                    point
                } else {
                    boundary.point
                },
            );
        }
        PointProjection::new(
            distance <= self.half_thickness(),
            boundary.point + normal * self.half_thickness(),
        )
    }

    fn project_local_point_and_get_feature(&self, point: Vector) -> (PointProjection, FeatureId) {
        (self.project_local_point(point, false), FeatureId::Face(0))
    }

    fn contains_local_point(&self, point: Vector) -> bool {
        if self.mode() == ProfileMode::Solid {
            self.contains_interior(&point)
        } else {
            let boundary = self.closest_boundary(&point);
            (boundary.point - point).length_squared()
                <= self.half_thickness() * self.half_thickness()
        }
    }
}

impl RayCast for AnalyticProfile {
    fn cast_local_ray_and_get_normal(
        &self,
        _ray: &Ray,
        _max_toi: Real,
        _solid: bool,
    ) -> Option<RayIntersection> {
        None
    }
}

impl Shape for AnalyticProfile {
    fn compute_local_aabb(&self) -> Aabb {
        self.local_aabb()
    }
    fn compute_local_bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere::new(Vector::ZERO, self.bounding_radius())
    }
    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(self.clone())
    }
    fn scale_dyn(&self, _scale: Vector, _subdivisions: u32) -> Option<Box<dyn Shape>> {
        None
    }
    fn mass_properties(&self, _density: Real) -> MassProperties {
        MassProperties::new(Vector::ZERO, 0.0, 0.0)
    }
    fn shape_type(&self) -> ShapeType {
        ShapeType::Custom
    }
    fn as_typed_shape(&self) -> TypedShape<'_> {
        TypedShape::Custom(self)
    }
    fn ccd_thickness(&self) -> Real {
        if self.mode() == ProfileMode::Outline {
            self.half_thickness()
        } else {
            0.0
        }
    }
    fn ccd_angular_thickness(&self) -> Real {
        self.bounding_radius()
    }
}

use rapier2d::math::{Pose, Real, Vector};
use rapier2d::parry::shape::Cuboid;

use crate::profile::{AnalyticProfile, ProfileSegment, angle_on_sweep};

const CONTACT_EPSILON: Real = 1.0e-7;

#[derive(Clone, Copy)]
pub(crate) struct Penetration {
    pub point1: Vector,
    pub point2: Vector,
    pub normal1: Vector,
    pub depth: Real,
}

pub(crate) fn solid_profile_box_penetrations(
    profile: &AnalyticProfile,
    profile_to_box: &Pose,
    cuboid: &Cuboid,
) -> Vec<Penetration> {
    let mut candidates = Vec::with_capacity(profile.segments().len() * 3 + 4);
    for corner in box_corners(profile_to_box, cuboid) {
        if !profile.contains_interior(&corner) {
            continue;
        }
        let boundary = profile.closest_boundary(&corner);
        let offset = corner - boundary.point;
        let depth = offset.length();
        if depth <= CONTACT_EPSILON {
            continue;
        }
        candidates.push(Penetration {
            point1: boundary.point,
            point2: corner,
            normal1: -offset / depth,
            depth,
        });
    }
    for point in profile_box_candidates(profile, profile_to_box) {
        if let Some((box_boundary, box_outward, depth)) =
            project_inside_box(point, profile_to_box, cuboid)
        {
            candidates.push(Penetration {
                point1: point,
                point2: box_boundary,
                normal1: -box_outward,
                depth,
            });
        }
    }
    candidates
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

fn project_inside_box(
    point: Vector,
    transform: &Pose,
    cuboid: &Cuboid,
) -> Option<(Vector, Vector, Real)> {
    let local = transform.inverse_transform_point(point);
    let x_depth = cuboid.half_extents.x - local.x.abs();
    let y_depth = cuboid.half_extents.y - local.y.abs();
    if x_depth <= CONTACT_EPSILON || y_depth <= CONTACT_EPSILON {
        return None;
    }
    let (boundary_local, outward_local, depth) = if x_depth < y_depth {
        let sign = if local.x < 0.0 { -1.0 } else { 1.0 };
        (
            Vector::new(sign * cuboid.half_extents.x, local.y),
            Vector::new(sign, 0.0),
            x_depth,
        )
    } else {
        let sign = if local.y < 0.0 { -1.0 } else { 1.0 };
        (
            Vector::new(local.x, sign * cuboid.half_extents.y),
            Vector::new(0.0, sign),
            y_depth,
        )
    };
    Some((
        transform * boundary_local,
        transform.rotation * outward_local,
        depth,
    ))
}

fn profile_box_candidates(profile: &AnalyticProfile, transform: &Pose) -> Vec<Vector> {
    let x_axis = transform.rotation * Vector::X;
    let y_axis = transform.rotation * Vector::Y;
    let directions = [x_axis, -x_axis, y_axis, -y_axis];
    let mut points = Vec::with_capacity(profile.segments().len() * 3);
    for segment in profile.segments() {
        points.push(segment.start());
        if let ProfileSegment::Arc {
            center,
            radius,
            start_angle,
            sweep,
        } = segment
        {
            for direction in directions {
                let angle = direction.y.atan2(direction.x);
                if angle_on_sweep(angle, *start_angle, *sweep) {
                    points.push(center + direction * *radius);
                }
            }
        }
    }
    points
}

use rapier2d::math::{Pose, Real, Vector};
use rapier2d::parry::query::Contact;
use rapier2d::parry::shape::{Ball, Cuboid};

use crate::profile::{AnalyticProfile, ProfileMode};
use crate::profile_box_contacts::solid_profile_box_contact;
use crate::profile_box_distance::closest_profile_box_pair;

const DISTANCE_EPSILON: Real = 1.0e-5;

pub fn profile_ball_contact(
    profile: &AnalyticProfile,
    profile_to_ball: &Pose,
    ball: &Ball,
) -> Contact {
    let center = profile_to_ball.translation;
    let boundary = profile.closest_boundary(&center);
    let offset = center - boundary.point;
    let distance = offset.length();
    let (normal, gap, point1) = if profile.mode() == ProfileMode::Solid {
        let inside = profile.contains_interior(&center);
        let direction = normalized_or(
            if inside { -offset } else { offset },
            profile.outward_normal(boundary),
        );
        let signed_distance = if inside { -distance } else { distance };
        (direction, signed_distance - ball.radius, boundary.point)
    } else {
        let direction = normalized_or(offset, profile.outward_normal(boundary));
        (
            direction,
            distance - profile.half_thickness() - ball.radius,
            boundary.point + direction * profile.half_thickness(),
        )
    };
    let opposite = -normal;
    let normal2 = profile_to_ball.rotation.inverse() * opposite;
    let point2 = normal2 * ball.radius;
    Contact::new(point1, point2, normal, normal2, gap)
}

pub fn profile_box_contact(
    profile: &AnalyticProfile,
    profile_to_box: &Pose,
    cuboid: &Cuboid,
) -> Contact {
    if profile.mode() == ProfileMode::Solid
        && let Some(contact) = solid_profile_box_contact(profile, profile_to_box, cuboid)
    {
        return contact;
    }
    let pair = closest_profile_box_pair(profile, profile_to_box, cuboid);
    let offset = pair.other - pair.profile;
    let distance = offset.length();
    let normal1 = normalized_or(offset, pair.fallback_normal);
    let surface_offset = if profile.mode() == ProfileMode::Outline {
        profile.half_thickness()
    } else {
        0.0
    };
    let gap = distance - surface_offset;
    let point1 = pair.profile + normal1 * surface_offset;
    let point2 = profile_to_box.inverse_transform_point(pair.other);
    let opposite = -normal1;
    let normal2 = profile_to_box.rotation.inverse() * opposite;
    Contact::new(point1, point2, normal1, normal2, gap)
}

fn normalized_or(value: Vector, fallback: Vector) -> Vector {
    if value.length() > DISTANCE_EPSILON {
        value.normalize()
    } else {
        fallback.normalize_or_zero()
    }
}

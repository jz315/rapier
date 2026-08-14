use rapier2d::math::{Pose, Real, Vector};
use rapier2d::parry::query::Contact;
use rapier2d::parry::shape::Cuboid;

use crate::profile::AnalyticProfile;
use crate::profile_box_solid::{Penetration, solid_profile_box_penetrations};

const PATCH_DEPTH_EPSILON: Real = 1.0e-5;
const DUPLICATE_EPSILON_SQUARED: Real = 1.0e-10;
const ALIGNED_NORMAL_COSINE: Real = 0.99;
const MAX_MANIFOLD_CONTACTS: usize = 4;

pub(crate) fn solid_profile_box_contact(
    profile: &AnalyticProfile,
    profile_to_box: &Pose,
    cuboid: &Cuboid,
) -> Option<Contact> {
    let candidates = selected_penetrations(profile, profile_to_box, cuboid);
    let count = candidates.len() as Real;
    if count == 0.0 {
        return None;
    }
    let combined = candidates.iter().fold(
        Penetration {
            point1: Vector::ZERO,
            point2: Vector::ZERO,
            normal1: Vector::ZERO,
            depth: 0.0,
        },
        |sum, candidate| Penetration {
            point1: sum.point1 + candidate.point1 / count,
            point2: sum.point2 + candidate.point2 / count,
            normal1: sum.normal1 + candidate.normal1 / count,
            depth: sum.depth + candidate.depth / count,
        },
    );
    Some(to_contact(combined, profile_to_box))
}

pub(crate) fn solid_profile_box_manifold_contacts(
    profile: &AnalyticProfile,
    profile_to_box: &Pose,
    cuboid: &Cuboid,
) -> Vec<Contact> {
    let selected = selected_penetrations(profile, profile_to_box, cuboid);
    if selected.len() <= MAX_MANIFOLD_CONTACTS {
        return selected
            .into_iter()
            .map(|item| to_contact(item, profile_to_box))
            .collect();
    }
    evenly_spaced(selected, MAX_MANIFOLD_CONTACTS)
        .into_iter()
        .map(|item| to_contact(item, profile_to_box))
        .collect()
}

fn selected_penetrations(
    profile: &AnalyticProfile,
    profile_to_box: &Pose,
    cuboid: &Cuboid,
) -> Vec<Penetration> {
    let mut candidates = solid_profile_box_penetrations(profile, profile_to_box, cuboid);
    let Some(reference) = candidates
        .iter()
        .max_by(|left, right| left.depth.total_cmp(&right.depth))
        .copied()
    else {
        return Vec::new();
    };
    let tangent = Vector::new(-reference.normal1.y, reference.normal1.x);
    candidates.retain(|item| {
        item.depth >= reference.depth - PATCH_DEPTH_EPSILON
            && item.normal1.dot(reference.normal1) >= ALIGNED_NORMAL_COSINE
    });
    candidates.sort_by(|left, right| {
        tangent
            .dot(left.point1)
            .total_cmp(&tangent.dot(right.point1))
    });
    candidates.dedup_by(|left, right| {
        (left.point1 - right.point1).length_squared() <= DUPLICATE_EPSILON_SQUARED
            && (left.point2 - right.point2).length_squared() <= DUPLICATE_EPSILON_SQUARED
    });
    candidates
}

fn evenly_spaced(items: Vec<Penetration>, count: usize) -> Vec<Penetration> {
    let last = items.len() - 1;
    (0..count)
        .map(|index| items[index * last / (count - 1)])
        .collect()
}

fn to_contact(penetration: Penetration, profile_to_box: &Pose) -> Contact {
    let normal1 = penetration.normal1.normalize();
    let opposite = -normal1;
    Contact::new(
        penetration.point1,
        profile_to_box.inverse_transform_point(penetration.point2),
        normal1,
        profile_to_box.rotation.inverse() * opposite,
        -penetration.depth,
    )
}

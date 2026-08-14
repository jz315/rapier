use rapier2d::math::{Pose, Real, Vector};
use rapier2d::parry::query::Contact;

use crate::profile::{AnalyticProfile, ProfileMode, ProfileSegment};
use crate::profile_segment_pair::{SegmentClosest, closest_segment_pair};

const CONTACT_EPSILON: Real = 1.0e-6;

pub(crate) fn profile_profile_contact(
    first: &AnalyticProfile,
    first_to_second: &Pose,
    second: &AnalyticProfile,
) -> Contact {
    if first.mode() == ProfileMode::Outline && second.mode() == ProfileMode::Solid {
        return profile_profile_contact(second, &first_to_second.inverse(), first).flipped();
    }
    let closest = closest_profiles(first, first_to_second, second);
    let offset = closest.second - closest.first;
    let distance = offset.length();
    let boundary = first.closest_boundary(&closest.first);
    let fallback = first.outward_normal(boundary);
    let normal1 = normalized_or(offset, fallback);
    match (first.mode(), second.mode()) {
        (ProfileMode::Outline, ProfileMode::Outline) => contact_from_pair(
            first_to_second,
            closest,
            normal1,
            distance - first.half_thickness() - second.half_thickness(),
            first.half_thickness(),
            second.half_thickness(),
        ),
        (ProfileMode::Solid, ProfileMode::Outline) => {
            let inside = first.contains_interior(&closest.second);
            let gap = if inside {
                -distance - second.half_thickness()
            } else {
                distance - second.half_thickness()
            };
            let normal = if inside { fallback } else { normal1 };
            contact_from_pair(
                first_to_second,
                closest,
                normal,
                gap,
                0.0,
                second.half_thickness(),
            )
        }
        (ProfileMode::Solid, ProfileMode::Solid) => {
            let direct = containment_contact(first, first_to_second, second);
            let reversed = containment_contact(second, &first_to_second.inverse(), first)
                .map(Contact::flipped);
            direct
                .into_iter()
                .chain(reversed)
                .max_by(|left, right| left.dist.total_cmp(&right.dist))
                .unwrap_or_else(|| {
                    contact_from_pair(first_to_second, closest, normal1, distance, 0.0, 0.0)
                })
        }
        (ProfileMode::Outline, ProfileMode::Solid) => unreachable!(),
    }
}

fn closest_profiles(
    first: &AnalyticProfile,
    first_to_second: &Pose,
    second: &AnalyticProfile,
) -> SegmentClosest {
    let transformed: Vec<_> = second
        .segments()
        .iter()
        .map(|segment| transform_segment(first_to_second, segment))
        .collect();
    first
        .segments()
        .iter()
        .flat_map(|left| {
            transformed
                .iter()
                .map(move |right| closest_segment_pair(left, right))
        })
        .min_by(|left, right| left.distance_squared().total_cmp(&right.distance_squared()))
        .expect("validated analytic profiles contain segments")
}

fn containment_contact(
    outer: &AnalyticProfile,
    outer_to_inner: &Pose,
    inner: &AnalyticProfile,
) -> Option<Contact> {
    inner
        .segments()
        .iter()
        .map(ProfileSegment::start)
        .map(|local| (local, outer_to_inner * local))
        .filter(|(_, point)| outer.contains_interior(point))
        .filter_map(|(local, point)| {
            let boundary = outer.closest_boundary(&point);
            let depth = (point - boundary.point).length();
            (depth > CONTACT_EPSILON).then(|| {
                let normal1 = outer.outward_normal(boundary);
                let normal2 = outer_to_inner.rotation.inverse() * -normal1;
                Contact::new(boundary.point, local, normal1, normal2, -depth)
            })
        })
        .max_by(|left, right| left.dist.total_cmp(&right.dist))
}

fn contact_from_pair(
    first_to_second: &Pose,
    closest: SegmentClosest,
    normal1: Vector,
    gap: Real,
    first_offset: Real,
    second_offset: Real,
) -> Contact {
    let point1 = closest.first + normal1 * first_offset;
    let point2_world = closest.second - normal1 * second_offset;
    let normal2 = first_to_second.rotation.inverse() * -normal1;
    Contact::new(
        point1,
        first_to_second.inverse_transform_point(point2_world),
        normal1,
        normal2,
        gap,
    )
}

fn transform_segment(pose: &Pose, segment: &ProfileSegment) -> ProfileSegment {
    match segment {
        ProfileSegment::Line { start, end } => ProfileSegment::Line {
            start: pose * *start,
            end: pose * *end,
        },
        ProfileSegment::Arc {
            center,
            radius,
            start_angle,
            sweep,
        } => ProfileSegment::Arc {
            center: pose * *center,
            radius: *radius,
            start_angle: *start_angle + pose.rotation.angle(),
            sweep: *sweep,
        },
    }
}

fn normalized_or(value: Vector, fallback: Vector) -> Vector {
    if value.length() > CONTACT_EPSILON {
        value.normalize()
    } else if fallback.length() > CONTACT_EPSILON {
        fallback.normalize()
    } else {
        Vector::X
    }
}

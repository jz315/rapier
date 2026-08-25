use std::f32::consts::PI;

use rapier2d::math::{Pose, Vector};

use crate::profile::{AnalyticProfile, ProfileMode, ProfileSegment};
use crate::profile_profile_query::{profile_profile_contact, profile_profile_manifold_contacts};

fn circle(mode: ProfileMode) -> AnalyticProfile {
    AnalyticProfile::new(
        vec![
            ProfileSegment::Arc {
                center: Vector::ZERO,
                radius: 1.0,
                start_angle: 0.0,
                sweep: PI,
            },
            ProfileSegment::Arc {
                center: Vector::ZERO,
                radius: 1.0,
                start_angle: PI,
                sweep: PI,
            },
        ],
        0.02,
        mode,
    )
    .expect("circle profile")
}

fn rectangle(half_width: f32, half_height: f32) -> AnalyticProfile {
    let points = [
        Vector::new(-half_width, -half_height),
        Vector::new(half_width, -half_height),
        Vector::new(half_width, half_height),
        Vector::new(-half_width, half_height),
    ];
    AnalyticProfile::new(
        (0..points.len())
            .map(|index| ProfileSegment::Line {
                start: points[index],
                end: points[(index + 1) % points.len()],
            })
            .collect(),
        0.02,
        ProfileMode::Solid,
    )
    .expect("rectangle profile")
}

fn concave_profile() -> AnalyticProfile {
    let points = [
        Vector::new(-1.0, -1.0),
        Vector::new(1.0, -1.0),
        Vector::new(1.0, 1.0),
        Vector::new(0.2, 1.0),
        Vector::new(0.2, -0.2),
        Vector::new(-1.0, -0.2),
    ];
    AnalyticProfile::new(
        (0..points.len())
            .map(|index| ProfileSegment::Line {
                start: points[index],
                end: points[(index + 1) % points.len()],
            })
            .collect(),
        0.02,
        ProfileMode::Solid,
    )
    .expect("concave profile")
}

#[test]
fn outline_circle_gap_is_analytic() {
    let first = circle(ProfileMode::Outline);
    let second = circle(ProfileMode::Outline);
    let contact = profile_profile_contact(
        &first,
        &Pose::from_translation(Vector::new(2.1, 0.0)),
        &second,
    );
    assert!((contact.dist - 0.08).abs() < 1.0e-5);
    assert!((contact.normal1 - Vector::X).length() < 1.0e-5);
}

#[test]
fn overlapping_solid_circles_report_penetration() {
    let first = circle(ProfileMode::Solid);
    let second = circle(ProfileMode::Solid);
    let contact = profile_profile_contact(
        &first,
        &Pose::from_translation(Vector::new(1.8, 0.0)),
        &second,
    );
    assert!((contact.dist + 0.2).abs() < 1.0e-5);
    assert!((contact.normal1 - Vector::X).length() < 1.0e-5);
}

#[test]
fn parallel_solid_faces_produce_two_stably_ordered_contacts() {
    let ground = rectangle(2.0, 0.2);
    let body = rectangle(0.5, 0.5);
    let pose = Pose::from_translation(Vector::new(0.0, 0.69));
    let contacts = profile_profile_manifold_contacts(&ground, &pose, &body, 0.02);
    assert_eq!(contacts.len(), 2);
    assert!(contacts[0].point1.x < contacts[1].point1.x);
    for contact in &contacts {
        assert!((contact.dist + 0.01).abs() < 1.0e-5);
        assert!((contact.normal1 - Vector::Y).length() < 1.0e-5);
    }
    let shifted = profile_profile_manifold_contacts(
        &ground,
        &Pose::from_translation(Vector::new(0.01, 0.69)),
        &body,
        0.02,
    );
    assert_eq!(shifted.len(), 2);
    assert!(shifted[0].point1.x < shifted[1].point1.x);
    assert!((shifted[0].point2.x + 0.5).abs() < 1.0e-5);
    assert!((shifted[1].point2.x - 0.5).abs() < 1.0e-5);
}

#[test]
fn curved_profile_pair_keeps_one_contact() {
    let first = circle(ProfileMode::Solid);
    let second = circle(ProfileMode::Solid);
    let contacts = profile_profile_manifold_contacts(
        &first,
        &Pose::from_translation(Vector::new(1.8, 0.0)),
        &second,
        0.02,
    );
    assert_eq!(contacts.len(), 1);
    assert!((contacts[0].dist + 0.2).abs() < 1.0e-5);
}

#[test]
fn line_arc_pair_reports_a_finite_contact_without_sampling() {
    let ground = rectangle(2.0, 0.2);
    let body = circle(ProfileMode::Solid);
    let contacts = profile_profile_manifold_contacts(
        &ground,
        &Pose::from_translation(Vector::new(0.0, 1.19)),
        &body,
        0.02,
    );
    assert_eq!(contacts.len(), 1);
    assert!(contacts[0].dist.is_finite());
    assert!(contacts[0].dist < 0.02);
    assert!(contacts[0].normal1.length() > 0.99);
}

#[test]
fn concave_containment_contact_is_finite_and_deterministic() {
    let outer = concave_profile();
    let inner = rectangle(0.15, 0.15);
    let pose = Pose::from_translation(Vector::new(0.1, -0.1));
    let first = profile_profile_manifold_contacts(&outer, &pose, &inner, 0.02);
    let second = profile_profile_manifold_contacts(&outer, &pose, &inner, 0.02);
    assert!(!first.is_empty());
    assert_eq!(first.len(), second.len());
    for (left, right) in first.iter().zip(second.iter()) {
        assert!(left.dist.is_finite());
        assert_eq!(left.point1, right.point1);
        assert_eq!(left.point2, right.point2);
        assert_eq!(left.normal1, right.normal1);
    }
}

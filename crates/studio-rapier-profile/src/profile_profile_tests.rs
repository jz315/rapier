use std::f32::consts::PI;

use rapier2d::math::{Pose, Vector};

use crate::profile::{AnalyticProfile, ProfileMode, ProfileSegment};
use crate::profile_profile_query::profile_profile_contact;

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

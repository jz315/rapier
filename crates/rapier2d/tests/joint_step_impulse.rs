//! Joint step impulses include every internal PGS substep and expose both reaction force and
//! reaction torque without depending on solver iteration counts.

use rapier2d::prelude::*;

fn suspended_revolute_reaction(internal_iterations: usize) -> Vector {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let gravity = Vector::new(0.0, -9.8);
    let mut params = IntegrationParameters::default();
    params.dt = 1.0 / 120.0;
    params.num_internal_pgs_iterations = internal_iterations;

    let body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .additional_mass(1.0)
            .can_sleep(false),
    );
    let anchor = bodies.insert(RigidBodyBuilder::fixed());
    let joint = impulse_joints.insert(body, anchor, RevoluteJointBuilder::new(), true);

    for _ in 0..30 {
        pipeline.step(
            gravity,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
    }

    let impulse = impulse_joints
        .get(joint)
        .expect("joint must remain valid")
        .step_impulses;
    Vector::new(impulse[0] / params.dt, impulse[1] / params.dt)
}

#[test]
fn revolute_step_reaction_is_internal_substep_independent() {
    for internal_iterations in [1, 4, 8, 16] {
        let reaction = suspended_revolute_reaction(internal_iterations);
        assert!(reaction.x.abs() < 1.0e-4);
        assert!(
            (reaction.y - 9.8).abs() < 1.0e-3,
            "{internal_iterations} internal iterations reported {} N instead of 9.8 N",
            reaction.y
        );
    }
}

#[test]
fn fixed_step_reaction_includes_the_constraint_couple() {
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut impulse_joints = ImpulseJointSet::new();
    let mut multibody_joints = MultibodyJointSet::new();
    let mut pipeline = PhysicsPipeline::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut islands = IslandManager::new();
    let mut ccd = CCDSolver::new();
    let gravity = Vector::new(0.0, -9.8);
    let mut params = IntegrationParameters::default();
    params.dt = 1.0 / 120.0;
    params.num_internal_pgs_iterations = 8;

    let body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(1.0, 0.0))
            .additional_mass_properties(MassProperties::new(Vector::ZERO, 1.0, 1.0))
            .can_sleep(false),
    );
    let anchor = bodies.insert(RigidBodyBuilder::fixed());
    let joint = impulse_joints.insert(
        body,
        anchor,
        FixedJointBuilder::new().local_anchor1(Vector::new(-1.0, 0.0)),
        true,
    );

    for _ in 0..30 {
        pipeline.step(
            gravity,
            &params,
            &mut islands,
            &mut broad_phase,
            &mut narrow_phase,
            &mut bodies,
            &mut colliders,
            &mut impulse_joints,
            &mut multibody_joints,
            &mut ccd,
            &(),
            &(),
        );
    }

    let impulse = impulse_joints
        .get(joint)
        .expect("joint must remain valid")
        .step_impulses;
    let force_y = impulse[1] / params.dt;
    let angular_impulse_about_com = impulse[2] / params.dt;
    let reaction_torque = angular_impulse_about_com - (-1.0 * force_y);
    assert!((force_y - 9.8).abs() < 1.0e-3, "reported {force_y} N");
    assert!(
        (reaction_torque - 9.8).abs() < 1.0e-3,
        "reported {reaction_torque} N·m"
    );
}

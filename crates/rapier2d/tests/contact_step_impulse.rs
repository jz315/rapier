//! Contact impulses exposed through the narrow-phase are the impulses applied during the
//! current physics step. They must not count the warm-start seed from the previous step twice.

use rapier2d::prelude::*;

fn resting_contact_force(solver_iterations: usize) -> Real {
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
    params.num_solver_iterations = solver_iterations;

    let ground_body = bodies.insert(RigidBodyBuilder::fixed());
    let ground =
        colliders.insert_with_parent(ColliderBuilder::cuboid(5.0, 0.1), ground_body, &mut bodies);
    let block_body = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(Vector::new(0.0, 0.6))
            .additional_mass(1.0)
            .can_sleep(false)
            .lock_rotations(),
    );
    let block = colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.5, 0.5).density(0.0),
        block_body,
        &mut bodies,
    );

    for _ in 0..240 {
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

    let pair = narrow_phase
        .contact_pair(ground, block)
        .expect("resting block must contact ground");
    let impulse: Real = pair
        .manifolds
        .iter()
        .flat_map(|manifold| &manifold.points)
        .map(|point| point.data.impulse)
        .sum();
    impulse / params.dt
}

#[test]
fn reported_contact_step_impulse_is_solver_iteration_independent() {
    for iterations in [1, 4, 12, 64] {
        let force = resting_contact_force(iterations);
        assert!(
            (force - 9.8).abs() < 1.0e-3,
            "{iterations} solver iterations reported {force} N instead of 9.8 N"
        );
    }
}

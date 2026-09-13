use rapier::math::Vector;
use rapier::pipeline::{ContactModificationContext, PhysicsHooks};

pub struct SurfaceMotionHooks;
impl PhysicsHooks for SurfaceMotionHooks {
    fn modify_solver_contacts(&self, ctxt: &mut ContactModificationContext) {
        apply_surface_motion(ctxt);
    }
}

pub fn apply_surface_motion(ctxt: &mut ContactModificationContext) {
    let mut velocity = Vector::ZERO;
    for (handle, sign) in [(ctxt.collider1, 1.0), (ctxt.collider2, -1.0)] {
        let collider = &ctxt.colliders[handle];
        if let Some((local_velocity, local_normal)) = collider.surface_motion() {
            let rotation = collider.position().rotation;
            let outward = rotation * local_normal;
            // Only the specified face drives contacts. Edges and the return face stay passive.
            if outward.dot(*ctxt.normal) * sign > 1.0 - 1.0e-5 {
                velocity += rotation * local_velocity * sign;
            }
        }
    }
    for contact in ctxt.solver_contacts.iter_mut() {
        contact.tangent_velocity = velocity;
    }
}

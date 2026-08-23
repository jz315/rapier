use crate::alloc_prelude::*;
use crate::data::arena::{Arena, Index};
use crate::dynamics::solver::SolverBodies;
use crate::dynamics::{
    IntegrationParameters, IslandManager, RigidBodyHandle, RigidBodySet, SpringCoefficients,
};
use crate::math::{Real, Vector};

mod route;

use route::resolve_route;

const DENOMINATOR_EPSILON: Real = 1.0e-12;

/// Stable handle of a routed-rope constraint stored by a physics pipeline.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct RoutedRopeConstraintHandle(pub Index);

impl RoutedRopeConstraintHandle {
    /// Splits this handle into its arena index and generation.
    pub fn into_raw_parts(self) -> (u32, u32) {
        self.0.into_raw_parts()
    }

    /// Reconstructs a handle from an arena index and generation.
    pub fn from_raw_parts(id: u32, generation: u32) -> Self {
        Self(Index::from_raw_parts(id, generation))
    }
}

/// A point fixed in the world or attached to a rigid body in that body's local frame.
#[derive(Clone, Debug)]
pub enum RoutedRopePoint {
    /// A point fixed in world coordinates.
    World(Vector),
    /// A point attached to a rigid body.
    Body {
        /// The attached rigid body.
        body: RigidBodyHandle,
        /// The anchor in the rigid body's local frame.
        local_anchor: Vector,
    },
}

/// Configured winding direction around a routed pulley.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RoutedRopeWinding {
    /// The rope follows the wheel clockwise from incoming to outgoing tangent.
    Clockwise,
    /// The rope follows the wheel counterclockwise from incoming to outgoing tangent.
    Counterclockwise,
}

/// One ordered pulley in a routed rope.
#[derive(Clone, Debug)]
pub struct RoutedRopePulley {
    /// World-fixed or body-attached center of the ideal wheel.
    pub center: RoutedRopePoint,
    /// Positive wheel radius.
    pub radius: Real,
    /// Configured direction of the wrapped arc.
    pub winding: RoutedRopeWinding,
}

#[derive(Copy, Clone, Debug)]
pub(super) struct RoutedRopePulleyGeometry {
    pub center: Vector,
    pub radius: Real,
    pub winding: RoutedRopeWinding,
}

/// Deterministic state of routed-rope geometry and runtime availability.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum RoutedRopeStatus {
    /// The current route is valid.
    Valid = 0,
    /// The constraint was explicitly disabled.
    Disabled = 1,
    /// A coordinate, radius, or length was non-finite or invalid.
    InvalidCoordinates = 2,
    /// The route contains no pulleys.
    NoPulleys = 3,
    /// Two pulley rims overlap or touch.
    PulleyOverlap = 4,
    /// A requested tangent cannot be constructed.
    TangentUnavailable = 5,
    /// A straight rope segment intersects a non-adjacent pulley.
    SegmentObstructed = 6,
    /// A body referenced by the route no longer exists or is disabled.
    MissingBody = 7,
    /// The route has no participant with finite inverse mass.
    NoDynamicParticipant = 8,
}

#[derive(Clone, Debug)]
struct PreparedPoint {
    solver_body: Option<u32>,
    local_com_anchor: Vector,
    fixed_world_point: Vector,
    angular_response: bool,
}

impl PreparedPoint {
    fn position(&self, bodies: &SolverBodies) -> Vector {
        self.solver_body
            .map(|id| bodies.poses[id as usize].transform_point(self.local_com_anchor))
            .unwrap_or(self.fixed_world_point)
    }
}

#[derive(Copy, Clone, Debug)]
struct BodyJacobian {
    solver_body: u32,
    linear: Vector,
    angular: Real,
}

/// A native, unilateral total-length constraint routed over ordered pulley wheels.
#[derive(Clone, Debug)]
pub struct RoutedRopeConstraint {
    /// First rope endpoint.
    pub endpoint_a: RoutedRopePoint,
    /// Second rope endpoint.
    pub endpoint_b: RoutedRopePoint,
    /// Ordered ideal pulleys between the endpoints.
    pub pulleys: Vec<RoutedRopePulley>,
    /// Maximum allowed total routed length.
    pub max_length: Real,
    /// Whether this constraint participates in solving.
    pub enabled: bool,
    status: RoutedRopeStatus,
    active: bool,
    current_length: Real,
    length_error: Real,
    constraint_speed: Real,
    step_impulse: Real,
    warmstart_impulse: Real,
    impulse: Real,
    inverse_lhs: Real,
    cfm_gain: Real,
    rhs_bias: Real,
    prepared_points: Vec<PreparedPoint>,
    jacobians: Vec<BodyJacobian>,
}

impl RoutedRopeConstraint {
    /// Creates an enabled unilateral total-length rope constraint.
    pub fn new(
        endpoint_a: RoutedRopePoint,
        endpoint_b: RoutedRopePoint,
        pulleys: Vec<RoutedRopePulley>,
        max_length: Real,
    ) -> Self {
        Self {
            endpoint_a,
            endpoint_b,
            pulleys,
            max_length,
            enabled: true,
            status: RoutedRopeStatus::Valid,
            active: false,
            current_length: 0.0,
            length_error: 0.0,
            constraint_speed: 0.0,
            step_impulse: 0.0,
            warmstart_impulse: 0.0,
            impulse: 0.0,
            inverse_lhs: 0.0,
            cfm_gain: 0.0,
            rhs_bias: 0.0,
            prepared_points: Vec::new(),
            jacobians: Vec::new(),
        }
    }

    /// Returns the latest deterministic route status.
    pub fn status(&self) -> RoutedRopeStatus {
        self.status
    }

    /// Returns whether the rope carried tension during the latest step.
    pub fn active(&self) -> bool {
        self.active
    }

    /// Returns the latest routed length.
    pub fn current_length(&self) -> Real {
        self.current_length
    }

    /// Returns `current_length - max_length`.
    pub fn length_error(&self) -> Real {
        self.length_error
    }

    /// Returns the latest length-rate Jacobian applied to body velocities.
    pub fn constraint_speed(&self) -> Real {
        self.constraint_speed
    }

    /// Returns the sum of actual substep impulses in the latest pipeline step.
    pub fn step_impulse(&self) -> Real {
        self.step_impulse
    }

    /// Enables or disables this constraint and clears carried impulses.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.status = if enabled {
            RoutedRopeStatus::Valid
        } else {
            RoutedRopeStatus::Disabled
        };
        self.active = false;
        self.impulse = 0.0;
        self.warmstart_impulse = 0.0;
        self.step_impulse = 0.0;
    }

    fn fail(&mut self, status: RoutedRopeStatus) {
        self.status = status;
        self.enabled = false;
        self.active = false;
        self.impulse = 0.0;
        self.warmstart_impulse = 0.0;
        self.inverse_lhs = 0.0;
        self.jacobians.clear();
    }

    fn prepare_point(
        point: &RoutedRopePoint,
        angular_response: bool,
        bodies: &RigidBodySet,
    ) -> Result<PreparedPoint, RoutedRopeStatus> {
        match point {
            RoutedRopePoint::World(point) => Ok(PreparedPoint {
                solver_body: None,
                local_com_anchor: Vector::ZERO,
                fixed_world_point: *point,
                angular_response: false,
            }),
            RoutedRopePoint::Body { body, local_anchor } => {
                let rigid_body = bodies.get(*body).ok_or(RoutedRopeStatus::MissingBody)?;
                if !rigid_body.is_enabled() {
                    return Err(RoutedRopeStatus::MissingBody);
                }
                let world_point = rigid_body.position() * *local_anchor;
                let local_com_anchor = rigid_body.position().rotation.inverse()
                    * (world_point - rigid_body.mprops.world_com);
                let solver_body = (!rigid_body.is_fixed() && !rigid_body.is_sleeping())
                    .then_some(rigid_body.ids.active_set_id);
                Ok(PreparedPoint {
                    solver_body,
                    local_com_anchor,
                    fixed_world_point: world_point,
                    angular_response,
                })
            }
        }
    }

    fn prepare(&mut self, bodies: &RigidBodySet) {
        if !self.enabled {
            return;
        }
        if !self.max_length.is_finite() || self.max_length <= 0.0 {
            self.fail(RoutedRopeStatus::InvalidCoordinates);
            return;
        }
        if self.pulleys.is_empty() {
            self.fail(RoutedRopeStatus::NoPulleys);
            return;
        }
        self.prepared_points.clear();
        let endpoint_a = Self::prepare_point(&self.endpoint_a, true, bodies);
        let endpoint_b = Self::prepare_point(&self.endpoint_b, true, bodies);
        let pulley_points: Result<Vec<_>, _> = self
            .pulleys
            .iter()
            .map(|pulley| Self::prepare_point(&pulley.center, false, bodies))
            .collect();
        match (endpoint_a, endpoint_b, pulley_points) {
            (Ok(a), Ok(b), Ok(pulleys)) => {
                self.prepared_points.push(a);
                self.prepared_points.push(b);
                self.prepared_points.extend(pulleys);
                self.status = RoutedRopeStatus::Valid;
            }
            _ => self.fail(RoutedRopeStatus::MissingBody),
        }
    }

    fn update_substep(&mut self, params: &IntegrationParameters, bodies: &SolverBodies) {
        if !self.enabled || self.prepared_points.len() != self.pulleys.len() + 2 {
            return;
        }
        let endpoint_a = self.prepared_points[0].position(bodies);
        let endpoint_b = self.prepared_points[1].position(bodies);
        let pulley_geometry: Vec<_> = self
            .pulleys
            .iter()
            .enumerate()
            .map(|(index, pulley)| RoutedRopePulleyGeometry {
                center: self.prepared_points[index + 2].position(bodies),
                radius: pulley.radius,
                winding: pulley.winding,
            })
            .collect();
        let route = match resolve_route(endpoint_a, endpoint_b, &pulley_geometry) {
            Ok(route) => route,
            Err(status) => {
                self.fail(status);
                return;
            }
        };
        self.current_length = route.total_length;
        self.length_error = route.total_length - self.max_length;
        self.jacobians.clear();
        for (point, gradient) in self.prepared_points.iter().zip(route.gradients) {
            let Some(solver_body) = point.solver_body else {
                continue;
            };
            let pose = &bodies.poses[solver_body as usize];
            let world_offset = pose.rotation * point.local_com_anchor;
            let angular = if point.angular_response {
                world_offset.x * gradient.y - world_offset.y * gradient.x
            } else {
                0.0
            };
            if let Some(existing) = self
                .jacobians
                .iter_mut()
                .find(|entry| entry.solver_body == solver_body)
            {
                existing.linear += gradient;
                existing.angular += angular;
            } else {
                self.jacobians.push(BodyJacobian {
                    solver_body,
                    linear: gradient,
                    angular,
                });
            }
        }
        let denominator = self.jacobians.iter().fold(0.0, |sum, jacobian| {
            let pose = &bodies.poses[jacobian.solver_body as usize];
            sum + jacobian.linear.dot(pose.im * jacobian.linear)
                + pose.ii * jacobian.angular * jacobian.angular
        });
        if denominator <= DENOMINATOR_EPSILON {
            self.fail(RoutedRopeStatus::NoDynamicParticipant);
            return;
        }
        self.constraint_speed = self.speed(bodies);
        let softness = SpringCoefficients::<Real>::joint_defaults();
        let cfm_coeff = softness.cfm_coeff(params.dt);
        self.cfm_gain = denominator * cfm_coeff;
        self.inverse_lhs = 1.0 / (denominator + self.cfm_gain);
        self.rhs_bias = if self.length_error > 0.0 {
            (self.length_error * softness.erp_inv_dt(params.dt))
                .min(params.max_corrective_velocity())
        } else {
            self.length_error / params.dt
        };
        self.impulse = if params.warmstart_joints {
            self.warmstart_impulse * params.warmstart_coefficient
        } else {
            0.0
        };
        self.active = self.length_error >= 0.0
            || self.constraint_speed + self.rhs_bias > 0.0
            || self.impulse > 0.0;
        self.status = RoutedRopeStatus::Valid;
    }

    fn speed(&self, bodies: &SolverBodies) -> Real {
        self.jacobians.iter().fold(0.0, |sum, jacobian| {
            let velocity = &bodies.vels[jacobian.solver_body as usize];
            sum + jacobian.linear.dot(velocity.linear) + jacobian.angular * velocity.angular
        })
    }

    fn apply_impulse(&self, bodies: &mut SolverBodies, impulse: Real) {
        if impulse == 0.0 {
            return;
        }
        for jacobian in &self.jacobians {
            let pose = bodies.poses[jacobian.solver_body as usize];
            let velocity = &mut bodies.vels[jacobian.solver_body as usize];
            velocity.linear -= (pose.im * jacobian.linear) * impulse;
            velocity.angular -= pose.ii * jacobian.angular * impulse;
        }
    }

    fn warmstart(&self, bodies: &mut SolverBodies) {
        self.apply_impulse(bodies, self.impulse);
    }

    fn solve(&mut self, bodies: &mut SolverBodies, without_bias: bool) {
        if !self.enabled || self.inverse_lhs == 0.0 {
            return;
        }
        let rhs = self.speed(bodies) + if without_bias { 0.0 } else { self.rhs_bias };
        let next =
            (self.impulse + self.inverse_lhs * (rhs - self.cfm_gain * self.impulse)).max(0.0);
        let delta = next - self.impulse;
        self.impulse = next;
        self.active |= next > 0.0;
        self.apply_impulse(bodies, delta);
    }

    fn finish_substep(&mut self, bodies: &SolverBodies) {
        if !self.enabled {
            return;
        }
        self.constraint_speed = self.speed(bodies);
        self.step_impulse += self.impulse;
        self.warmstart_impulse = self.impulse;
    }

    fn refresh_observations(&mut self, bodies: &RigidBodySet) {
        if !self.enabled {
            return;
        }
        let resolve_point = |point: &RoutedRopePoint| -> Result<Vector, RoutedRopeStatus> {
            match point {
                RoutedRopePoint::World(point) => Ok(*point),
                RoutedRopePoint::Body { body, local_anchor } => bodies
                    .get(*body)
                    .filter(|body| body.is_enabled())
                    .map(|body| body.position() * *local_anchor)
                    .ok_or(RoutedRopeStatus::MissingBody),
            }
        };
        let endpoint_a = match resolve_point(&self.endpoint_a) {
            Ok(point) => point,
            Err(status) => return self.fail(status),
        };
        let endpoint_b = match resolve_point(&self.endpoint_b) {
            Ok(point) => point,
            Err(status) => return self.fail(status),
        };
        let pulley_geometry: Result<Vec<_>, _> = self
            .pulleys
            .iter()
            .map(|pulley| {
                resolve_point(&pulley.center).map(|center| RoutedRopePulleyGeometry {
                    center,
                    radius: pulley.radius,
                    winding: pulley.winding,
                })
            })
            .collect();
        let route = match pulley_geometry
            .and_then(|pulleys| resolve_route(endpoint_a, endpoint_b, &pulleys))
        {
            Ok(route) => route,
            Err(status) => return self.fail(status),
        };
        self.current_length = route.total_length;
        self.length_error = route.total_length - self.max_length;
        self.constraint_speed = 0.0;
        let points = core::iter::once((&self.endpoint_a, true))
            .chain(core::iter::once((&self.endpoint_b, true)))
            .chain(self.pulleys.iter().map(|pulley| (&pulley.center, false)));
        for ((point, angular_response), gradient) in points.zip(route.gradients) {
            let RoutedRopePoint::Body { body, local_anchor } = point else {
                continue;
            };
            let Some(body) = bodies.get(*body) else {
                return self.fail(RoutedRopeStatus::MissingBody);
            };
            self.constraint_speed += gradient.dot(body.linvel());
            if angular_response {
                let world_point = body.position() * *local_anchor;
                let offset = world_point - body.center_of_mass();
                self.constraint_speed +=
                    (offset.x * gradient.y - offset.y * gradient.x) * body.angvel();
            }
        }
    }
}

/// Deterministic arena of native routed-rope constraints.
#[derive(Clone, Debug, Default)]
pub struct RoutedRopeConstraintSet {
    constraints: Arena<RoutedRopeConstraint>,
}

impl RoutedRopeConstraintSet {
    /// Creates an empty routed-rope constraint set.
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts a routed-rope constraint and returns its stable handle.
    pub fn insert(&mut self, constraint: RoutedRopeConstraint) -> RoutedRopeConstraintHandle {
        RoutedRopeConstraintHandle(self.constraints.insert(constraint))
    }

    /// Removes and returns a constraint when the handle is valid.
    pub fn remove(&mut self, handle: RoutedRopeConstraintHandle) -> Option<RoutedRopeConstraint> {
        self.constraints.remove(handle.0)
    }

    /// Gets a routed-rope constraint by handle.
    pub fn get(&self, handle: RoutedRopeConstraintHandle) -> Option<&RoutedRopeConstraint> {
        self.constraints.get(handle.0)
    }

    /// Gets a mutable routed-rope constraint by handle.
    pub fn get_mut(
        &mut self,
        handle: RoutedRopeConstraintHandle,
    ) -> Option<&mut RoutedRopeConstraint> {
        self.constraints.get_mut(handle.0)
    }

    /// Returns the number of routed-rope constraints.
    pub fn len(&self) -> usize {
        self.constraints.len()
    }

    /// Returns whether this set contains no routed-rope constraints.
    pub fn is_empty(&self) -> bool {
        self.constraints.is_empty()
    }

    pub(crate) fn begin_step(&mut self) {
        for (_, constraint) in self.constraints.iter_mut() {
            constraint.step_impulse = 0.0;
            constraint.active = false;
        }
    }

    pub(crate) fn wake_participants(&self, islands: &mut IslandManager, bodies: &mut RigidBodySet) {
        for (_, constraint) in self.constraints.iter() {
            if !constraint.enabled {
                continue;
            }
            let points = core::iter::once(&constraint.endpoint_a)
                .chain(core::iter::once(&constraint.endpoint_b))
                .chain(constraint.pulleys.iter().map(|pulley| &pulley.center));
            for point in points {
                if let RoutedRopePoint::Body { body, .. } = point {
                    if bodies
                        .get(*body)
                        .is_some_and(|body| !body.is_fixed() && body.is_enabled())
                    {
                        islands.wake_up(bodies, *body, true);
                    }
                }
            }
        }
    }

    pub(crate) fn prepare(&mut self, bodies: &RigidBodySet) {
        for (_, constraint) in self.constraints.iter_mut() {
            constraint.prepare(bodies);
        }
    }

    pub(crate) fn has_enabled(&self) -> bool {
        self.constraints
            .iter()
            .any(|(_, constraint)| constraint.enabled)
    }

    pub(crate) fn update_substep(&mut self, params: &IntegrationParameters, bodies: &SolverBodies) {
        for (_, constraint) in self.constraints.iter_mut() {
            constraint.update_substep(params, bodies);
        }
    }

    pub(crate) fn warmstart(&mut self, bodies: &mut SolverBodies) {
        for (_, constraint) in self.constraints.iter() {
            constraint.warmstart(bodies);
        }
    }

    pub(crate) fn solve(&mut self, bodies: &mut SolverBodies, without_bias: bool) {
        for (_, constraint) in self.constraints.iter_mut() {
            constraint.solve(bodies, without_bias);
        }
    }

    pub(crate) fn finish_substep(&mut self, bodies: &SolverBodies) {
        for (_, constraint) in self.constraints.iter_mut() {
            constraint.finish_substep(bodies);
        }
    }

    pub(crate) fn refresh_observations(&mut self, bodies: &RigidBodySet) {
        for (_, constraint) in self.constraints.iter_mut() {
            constraint.refresh_observations(bodies);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dynamics::RigidBodyBuilder;

    #[test]
    fn arena_handles_survive_removal_without_aliasing() {
        let mut set = RoutedRopeConstraintSet::new();
        let rope = || {
            RoutedRopeConstraint::new(
                RoutedRopePoint::World(Vector::new(-1.0, 0.0)),
                RoutedRopePoint::World(Vector::new(1.0, 0.0)),
                vec![RoutedRopePulley {
                    center: RoutedRopePoint::World(Vector::new(0.0, 1.0)),
                    radius: 0.2,
                    winding: RoutedRopeWinding::Clockwise,
                }],
                3.0,
            )
        };
        let first = set.insert(rope());
        set.remove(first).expect("first constraint");
        let second = set.insert(rope());
        assert_ne!(first, second);
        assert!(set.get(first).is_none());
        assert!(set.get(second).is_some());
    }

    #[test]
    fn missing_body_disables_only_the_constraint() {
        let mut bodies = RigidBodySet::new();
        let body = bodies.insert(RigidBodyBuilder::dynamic());
        let mut constraint = RoutedRopeConstraint::new(
            RoutedRopePoint::Body {
                body,
                local_anchor: Vector::ZERO,
            },
            RoutedRopePoint::World(Vector::new(1.0, 0.0)),
            vec![RoutedRopePulley {
                center: RoutedRopePoint::World(Vector::new(0.5, 1.0)),
                radius: 0.1,
                winding: RoutedRopeWinding::Clockwise,
            }],
            3.0,
        );
        bodies.remove(
            body,
            &mut IslandManager::new(),
            &mut crate::geometry::ColliderSet::new(),
            &mut crate::dynamics::ImpulseJointSet::new(),
            &mut crate::dynamics::MultibodyJointSet::new(),
            true,
        );
        constraint.prepare(&bodies);
        assert_eq!(constraint.status(), RoutedRopeStatus::MissingBody);
        assert!(!constraint.enabled);
    }
}

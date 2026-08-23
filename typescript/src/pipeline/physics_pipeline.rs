use crate::dynamics::{
    RawCCDSolver, RawImpulseJointSet, RawIntegrationParameters, RawIslandManager,
    RawMultibodyJointSet, RawRigidBodySet,
};
use crate::geometry::{RawBroadPhase, RawColliderSet, RawNarrowPhase};
use crate::math::RawVector;
use crate::pipeline::{RawEventQueue, RawPhysicsHooks};
use crate::rapier::pipeline::PhysicsPipeline;
use wasm_bindgen::prelude::*;

#[cfg(feature = "dim2")]
use crate::rapier::dynamics::{
    RoutedRopeConstraint, RoutedRopeConstraintHandle, RoutedRopePoint, RoutedRopePulley,
    RoutedRopeWinding,
};
#[cfg(feature = "dim2")]
use crate::rapier::math::Vector;
#[cfg(feature = "dim2")]
use crate::utils::{self, FlatHandle};

#[cfg(feature = "dim2")]
fn routed_rope_handle(handle: FlatHandle) -> RoutedRopeConstraintHandle {
    RoutedRopeConstraintHandle::from_raw_parts(
        handle.to_bits() as u32,
        (handle.to_bits() >> 32) as u32,
    )
}

#[wasm_bindgen]
pub struct RawPhysicsPipeline(pub(crate) PhysicsPipeline);

#[wasm_bindgen]
impl RawPhysicsPipeline {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        let mut pipeline = PhysicsPipeline::new();
        pipeline.counters.disable(); // Disable perf counters by default.
        RawPhysicsPipeline(pipeline)
    }

    pub fn set_profiler_enabled(&mut self, enabled: bool) {
        if enabled {
            self.0.counters.enable();
        } else {
            self.0.counters.disable();
        }
    }

    pub fn is_profiler_enabled(&self) -> bool {
        self.0.counters.enabled()
    }

    #[cfg(feature = "dim2")]
    pub fn createRoutedRopeConstraint(
        &mut self,
        maxLength: f32,
        pointKinds: Vec<u32>,
        bodyHandles: Vec<FlatHandle>,
        pointCoordinates: Vec<f32>,
        pulleyRadii: Vec<f32>,
        pulleyWindings: Vec<u32>,
    ) -> Result<FlatHandle, JsValue> {
        let point_count = pulleyRadii.len() + 2;
        if !maxLength.is_finite() || maxLength <= 0.0 {
            return Err(JsValue::from_str("maxLength must be finite and positive"));
        }
        if pulleyRadii.is_empty()
            || pointKinds.len() != point_count
            || bodyHandles.len() != point_count
            || pointCoordinates.len() != point_count * 2
            || pulleyWindings.len() != pulleyRadii.len()
        {
            return Err(JsValue::from_str("invalid routed-rope array lengths"));
        }
        let point = |index: usize| -> Result<RoutedRopePoint, JsValue> {
            let coordinates = Vector::new(
                pointCoordinates[index * 2],
                pointCoordinates[index * 2 + 1],
            );
            if !coordinates.is_finite() {
                return Err(JsValue::from_str("routed-rope point must be finite"));
            }
            match pointKinds[index] {
                0 => Ok(RoutedRopePoint::World(coordinates)),
                1 => Ok(RoutedRopePoint::Body {
                    body: utils::body_handle(bodyHandles[index]),
                    local_anchor: coordinates,
                }),
                _ => Err(JsValue::from_str("unknown routed-rope point kind")),
            }
        };
        let endpoint_a = point(0)?;
        let endpoint_b = point(1)?;
        let mut pulleys = Vec::with_capacity(pulleyRadii.len());
        for index in 0..pulleyRadii.len() {
            let radius = pulleyRadii[index];
            if !radius.is_finite() || radius <= 0.0 {
                return Err(JsValue::from_str("pulley radius must be finite and positive"));
            }
            let winding = match pulleyWindings[index] {
                0 => RoutedRopeWinding::Clockwise,
                1 => RoutedRopeWinding::Counterclockwise,
                _ => return Err(JsValue::from_str("unknown pulley winding")),
            };
            pulleys.push(RoutedRopePulley {
                center: point(index + 2)?,
                radius,
                winding,
            });
        }
        let handle = self
            .0
            .routed_rope_constraints
            .insert(RoutedRopeConstraint::new(
                endpoint_a,
                endpoint_b,
                pulleys,
                maxLength,
            ));
        Ok(utils::flat_handle(handle.0))
    }

    #[cfg(feature = "dim2")]
    pub fn removeRoutedRopeConstraint(&mut self, handle: FlatHandle) -> bool {
        self.0
            .routed_rope_constraints
            .remove(routed_rope_handle(handle))
            .is_some()
    }

    #[cfg(feature = "dim2")]
    pub fn setRoutedRopeConstraintEnabled(
        &mut self,
        handle: FlatHandle,
        enabled: bool,
    ) -> bool {
        let Some(constraint) = self
            .0
            .routed_rope_constraints
            .get_mut(routed_rope_handle(handle))
        else {
            return false;
        };
        constraint.set_enabled(enabled);
        true
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintIsValid(&self, handle: FlatHandle) -> bool {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .is_some()
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintStatus(&self, handle: FlatHandle) -> u32 {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .map(|constraint| constraint.status() as u32)
            .unwrap_or(u32::MAX)
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintActive(&self, handle: FlatHandle) -> bool {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .is_some_and(|constraint| constraint.active())
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintCurrentLength(&self, handle: FlatHandle) -> f32 {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .map(|constraint| constraint.current_length())
            .unwrap_or(f32::NAN)
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintError(&self, handle: FlatHandle) -> f32 {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .map(|constraint| constraint.length_error())
            .unwrap_or(f32::NAN)
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintSpeed(&self, handle: FlatHandle) -> f32 {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .map(|constraint| constraint.constraint_speed())
            .unwrap_or(f32::NAN)
    }

    #[cfg(feature = "dim2")]
    pub fn routedRopeConstraintStepImpulse(&self, handle: FlatHandle) -> f32 {
        self.0
            .routed_rope_constraints
            .get(routed_rope_handle(handle))
            .map(|constraint| constraint.step_impulse())
            .unwrap_or(f32::NAN)
    }

    pub fn timing_step(&self) -> f64 {
        self.0.counters.step_time_ms()
    }

    pub fn timing_collision_detection(&self) -> f64 {
        self.0.counters.collision_detection_time_ms()
    }

    pub fn timing_broad_phase(&self) -> f64 {
        self.0.counters.broad_phase_time_ms()
    }

    pub fn timing_narrow_phase(&self) -> f64 {
        self.0.counters.narrow_phase_time_ms()
    }

    pub fn timing_solver(&self) -> f64 {
        self.0.counters.solver_time_ms()
    }

    pub fn timing_velocity_assembly(&self) -> f64 {
        self.0.counters.solver.velocity_assembly_time.time_ms()
    }

    pub fn timing_velocity_resolution(&self) -> f64 {
        self.0.counters.velocity_resolution_time_ms()
    }

    pub fn timing_velocity_update(&self) -> f64 {
        self.0.counters.velocity_update_time_ms()
    }

    pub fn timing_velocity_writeback(&self) -> f64 {
        self.0.counters.solver.velocity_writeback_time.time_ms()
    }

    pub fn timing_ccd(&self) -> f64 {
        self.0.counters.ccd_time_ms()
    }

    pub fn timing_ccd_toi_computation(&self) -> f64 {
        self.0.counters.ccd.toi_computation_time.time_ms()
    }

    pub fn timing_ccd_broad_phase(&self) -> f64 {
        self.0.counters.ccd.broad_phase_time.time_ms()
    }

    pub fn timing_ccd_narrow_phase(&self) -> f64 {
        self.0.counters.ccd.narrow_phase_time.time_ms()
    }

    pub fn timing_ccd_solver(&self) -> f64 {
        self.0.counters.ccd.solver_time.time_ms()
    }

    pub fn timing_island_construction(&self) -> f64 {
        self.0.counters.island_construction_time_ms()
    }

    pub fn timing_user_changes(&self) -> f64 {
        self.0.counters.stages.user_changes.time_ms()
    }

    pub fn step(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
    ) {
        self.0.step(
            gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.0,
            &mut bodies.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &(),
            &(),
        );
    }

    pub fn stepWithEvents(
        &mut self,
        gravity: &RawVector,
        integrationParameters: &RawIntegrationParameters,
        islands: &mut RawIslandManager,
        broadPhase: &mut RawBroadPhase,
        narrowPhase: &mut RawNarrowPhase,
        bodies: &mut RawRigidBodySet,
        colliders: &mut RawColliderSet,
        joints: &mut RawImpulseJointSet,
        articulations: &mut RawMultibodyJointSet,
        ccd_solver: &mut RawCCDSolver,
        eventQueue: &mut RawEventQueue,
        hookObject: js_sys::Object,
        hookFilterContactPair: js_sys::Function,
        hookFilterIntersectionPair: js_sys::Function,
    ) {
        if eventQueue.auto_drain {
            eventQueue.clear();
        }

        let hooks = RawPhysicsHooks {
            this: hookObject,
            filter_contact_pair: hookFilterContactPair,
            filter_intersection_pair: hookFilterIntersectionPair,
        };

        self.0.step(
            gravity.0,
            &integrationParameters.0,
            &mut islands.0,
            &mut broadPhase.0,
            &mut narrowPhase.0,
            &mut bodies.0,
            &mut colliders.0,
            &mut joints.0,
            &mut articulations.0,
            &mut ccd_solver.0,
            &hooks,
            &eventQueue.collector,
        );
    }
}

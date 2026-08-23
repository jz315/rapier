import {RigidBodyHandle} from "../dynamics";
import {Vector} from "../math";

/** Stable native routed-rope handle owned by a World. */
export type RoutedRopeConstraintHandle = number;

/** A routed-rope point in world space or a rigid body's local frame. */
export type RoutedRopePoint =
    | {kind: "world"; point: Vector}
    | {kind: "body"; body: RigidBodyHandle; localAnchor: Vector};

/** One ordered ideal pulley in a native routed rope. */
export interface RoutedRopePulley {
    center: RoutedRopePoint;
    radius: number;
    winding: "clockwise" | "counterclockwise";
}

/** Complete configuration of a native unilateral routed rope. */
export interface RoutedRopeConstraintDesc {
    endpointA: RoutedRopePoint;
    endpointB: RoutedRopePoint;
    pulleys: RoutedRopePulley[];
    maxLength: number;
}

/** Deterministic native route status codes. */
export enum RoutedRopeConstraintStatus {
    Valid = 0,
    Disabled = 1,
    InvalidCoordinates = 2,
    NoPulleys = 3,
    PulleyOverlap = 4,
    TangentUnavailable = 5,
    SegmentObstructed = 6,
    MissingBody = 7,
    NoDynamicParticipant = 8,
    InvalidHandle = 0xffffffff,
}

/** Runtime state produced by the native solver. */
export interface RoutedRopeConstraintState {
    status: RoutedRopeConstraintStatus;
    active: boolean;
    currentLength: number;
    error: number;
    speed: number;
    stepImpulse: number;
}

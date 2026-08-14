use rapier2d::geometry::{ContactData, ContactManifoldData};
use rapier2d::math::{Pose, Real, Vector};
use rapier2d::parry::query::details::{NormalConstraints, ShapeCastOptions};
use rapier2d::parry::query::{
    ClosestPoints, Contact, ContactManifold, ContactManifoldsWorkspace, NonlinearRigidMotion,
    PersistentQueryDispatcher, QueryDispatcher, ShapeCastHit, Unsupported,
};
use rapier2d::parry::shape::Shape;

use crate::contact::write_contacts;
use crate::profile_cast::conservative_cast;
use crate::profile_pair::ProfilePair;

#[derive(Clone, Copy, Debug, Default)]
pub struct AnalyticProfileDispatcher;

impl AnalyticProfileDispatcher {
    fn pair<'a>(
        position12: &Pose,
        shape1: &'a dyn Shape,
        shape2: &'a dyn Shape,
    ) -> Result<ProfilePair<'a>, Unsupported> {
        ProfilePair::from_shapes(position12, shape1, shape2).ok_or(Unsupported)
    }
}

impl QueryDispatcher for AnalyticProfileDispatcher {
    fn intersection_test(
        &self,
        position12: &Pose,
        shape1: &dyn Shape,
        shape2: &dyn Shape,
    ) -> Result<bool, Unsupported> {
        Ok(Self::pair(position12, shape1, shape2)?.gap() <= 0.0)
    }

    fn distance(
        &self,
        position12: &Pose,
        shape1: &dyn Shape,
        shape2: &dyn Shape,
    ) -> Result<Real, Unsupported> {
        Ok(Self::pair(position12, shape1, shape2)?.gap().max(0.0))
    }

    fn contact(
        &self,
        position12: &Pose,
        shape1: &dyn Shape,
        shape2: &dyn Shape,
        prediction: Real,
    ) -> Result<Option<Contact>, Unsupported> {
        let contact = Self::pair(position12, shape1, shape2)?.contact();
        Ok((contact.dist < prediction).then_some(contact))
    }

    fn closest_points(
        &self,
        position12: &Pose,
        shape1: &dyn Shape,
        shape2: &dyn Shape,
        max_distance: Real,
    ) -> Result<ClosestPoints, Unsupported> {
        let contact = Self::pair(position12, shape1, shape2)?.contact();
        if contact.dist <= 0.0 {
            Ok(ClosestPoints::Intersecting)
        } else if contact.dist <= max_distance {
            Ok(ClosestPoints::WithinMargin(contact.point1, contact.point2))
        } else {
            Ok(ClosestPoints::Disjoint)
        }
    }

    fn cast_shapes(
        &self,
        position12: &Pose,
        velocity12: Vector,
        shape1: &dyn Shape,
        shape2: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        Self::pair(position12, shape1, shape2)?;
        let speed = velocity12.length();
        Ok(conservative_cast(
            0.0,
            options.max_time_of_impact,
            speed,
            options.target_distance,
            options.stop_at_penetration,
            |time| {
                let position = Pose::from_parts(
                    position12.translation + velocity12 * time,
                    position12.rotation,
                );
                Self::pair(&position, shape1, shape2)
                    .ok()
                    .map(|pair| pair.contact())
            },
        ))
    }

    fn cast_shapes_nonlinear(
        &self,
        motion1: &NonlinearRigidMotion,
        shape1: &dyn Shape,
        motion2: &NonlinearRigidMotion,
        shape2: &dyn Shape,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        Self::pair(&Pose::IDENTITY, shape1, shape2)?;
        let speed = (motion2.linvel - motion1.linvel).length()
            + motion1.angvel.abs() * shape1.compute_local_bounding_sphere().radius
            + motion2.angvel.abs() * shape2.compute_local_bounding_sphere().radius;
        Ok(conservative_cast(
            start_time,
            end_time,
            speed,
            0.0,
            stop_at_penetration,
            |time| {
                let first = motion1.position_at_time(time);
                let second = motion2.position_at_time(time);
                Self::pair(&first.inv_mul(&second), shape1, shape2)
                    .ok()
                    .map(|pair| pair.contact())
            },
        ))
    }
}

impl PersistentQueryDispatcher<ContactManifoldData, ContactData> for AnalyticProfileDispatcher {
    fn contact_manifolds(
        &self,
        position12: &Pose,
        shape1: &dyn Shape,
        shape2: &dyn Shape,
        prediction: Real,
        manifolds: &mut Vec<ContactManifold<ContactManifoldData, ContactData>>,
        _workspace: &mut Option<ContactManifoldsWorkspace>,
    ) -> Result<(), Unsupported> {
        let contacts = Self::pair(position12, shape1, shape2)?.manifold_contacts(prediction);
        if manifolds.is_empty() {
            manifolds.push(ContactManifold::new());
        }
        manifolds.truncate(1);
        write_contacts(&mut manifolds[0], &contacts);
        Ok(())
    }

    fn contact_manifold_convex_convex(
        &self,
        _position12: &Pose,
        _shape1: &dyn Shape,
        _shape2: &dyn Shape,
        _normal_constraints1: Option<&dyn NormalConstraints>,
        _normal_constraints2: Option<&dyn NormalConstraints>,
        _prediction: Real,
        _manifold: &mut ContactManifold<ContactManifoldData, ContactData>,
    ) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

use rapier2d::math::Real;
use rapier2d::parry::query::{Contact, ShapeCastHit, ShapeCastStatus};

const CCD_TOLERANCE: Real = 1.0e-6;
const CCD_ITERATIONS: usize = 80;

pub(crate) fn conservative_cast(
    start_time: Real,
    end_time: Real,
    speed_bound: Real,
    target_distance: Real,
    stop_at_penetration: bool,
    mut contact_at: impl FnMut(Real) -> Option<Contact>,
) -> Option<ShapeCastHit> {
    let mut time = start_time;
    for iteration in 0..CCD_ITERATIONS {
        let contact = contact_at(time)?;
        let separation = contact.dist - target_distance;
        if separation <= CCD_TOLERANCE {
            if iteration == 0 && !stop_at_penetration {
                return None;
            }
            let status = if iteration == 0 {
                ShapeCastStatus::PenetratingOrWithinTargetDist
            } else {
                ShapeCastStatus::Converged
            };
            return Some(ShapeCastHit {
                time_of_impact: time,
                witness1: contact.point1,
                witness2: contact.point2,
                normal1: contact.normal1,
                normal2: contact.normal2,
                status,
            });
        }
        if speed_bound <= Real::EPSILON {
            return None;
        }
        time += (separation / speed_bound * 0.9).max(CCD_TOLERANCE);
        if time > end_time {
            return None;
        }
    }
    None
}

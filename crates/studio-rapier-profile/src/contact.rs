use rapier2d::parry::query::{Contact, ContactManifold, TrackedContact};
use rapier2d::parry::shape::PackedFeatureId;

pub fn write_contacts<ManifoldData, ContactData: Default + Copy>(
    manifold: &mut ContactManifold<ManifoldData, ContactData>,
    contacts: &[Contact],
) {
    for (index, contact) in contacts.iter().enumerate() {
        let feature = PackedFeatureId::face(index as u32);
        let tracked = TrackedContact::new(
            contact.point1,
            contact.point2,
            feature,
            feature,
            contact.dist,
        );
        if let Some(point) = manifold.points.get_mut(index) {
            point.copy_geometry_from(tracked);
        } else {
            manifold.points.push(tracked);
        }
    }
    manifold.points.truncate(contacts.len());
    if let Some(contact) = contacts.first() {
        manifold.local_n1 = contact.normal1;
        manifold.local_n2 = contact.normal2;
    }
}

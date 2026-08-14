use rapier2d::math::{Pose, Real};
use rapier2d::parry::query::Contact;
use rapier2d::parry::shape::{Ball, Cuboid, Shape};

use crate::profile::{AnalyticProfile, ProfileMode};
use crate::profile_box_contacts::solid_profile_box_manifold_contacts;
use crate::profile_profile_query::profile_profile_contact;
use crate::profile_query::{profile_ball_contact, profile_box_contact};

enum ProfileOther<'a> {
    Ball(&'a Ball),
    Cuboid(&'a Cuboid),
    Profile(&'a AnalyticProfile),
}

pub(crate) struct ProfilePair<'a> {
    profile: &'a AnalyticProfile,
    other: ProfileOther<'a>,
    profile_to_other: Pose,
    reversed: bool,
}

impl<'a> ProfilePair<'a> {
    pub fn from_shapes(
        position12: &Pose,
        shape1: &'a dyn Shape,
        shape2: &'a dyn Shape,
    ) -> Option<Self> {
        if let Some(profile) = shape1.as_shape::<AnalyticProfile>() {
            return Some(Self {
                profile,
                other: other_shape(shape2)?,
                profile_to_other: *position12,
                reversed: false,
            });
        }
        Some(Self {
            profile: shape2.as_shape::<AnalyticProfile>()?,
            other: other_shape(shape1)?,
            profile_to_other: position12.inverse(),
            reversed: true,
        })
    }

    pub fn contact(&self) -> Contact {
        let contact = match self.other {
            ProfileOther::Ball(ball) => {
                profile_ball_contact(self.profile, &self.profile_to_other, ball)
            }
            ProfileOther::Cuboid(cuboid) => {
                profile_box_contact(self.profile, &self.profile_to_other, cuboid)
            }
            ProfileOther::Profile(profile) => {
                profile_profile_contact(self.profile, &self.profile_to_other, profile)
            }
        };
        if self.reversed {
            contact.flipped()
        } else {
            contact
        }
    }

    pub fn gap(&self) -> Real {
        self.contact().dist
    }

    pub fn manifold_contacts(&self, prediction: Real) -> Vec<Contact> {
        let mut contacts = match self.other {
            ProfileOther::Cuboid(cuboid) if self.profile.mode() == ProfileMode::Solid => {
                solid_profile_box_manifold_contacts(self.profile, &self.profile_to_other, cuboid)
            }
            _ => Vec::new(),
        };
        if contacts.is_empty() {
            contacts.push(self.contact());
            return contacts
                .into_iter()
                .filter(|contact| contact.dist < prediction)
                .collect();
        }
        if self.reversed {
            contacts = contacts.into_iter().map(Contact::flipped).collect();
        }
        contacts.retain(|contact| contact.dist < prediction);
        contacts
    }
}

fn other_shape(shape: &dyn Shape) -> Option<ProfileOther<'_>> {
    shape
        .as_shape::<AnalyticProfile>()
        .map(ProfileOther::Profile)
        .or_else(|| {
            shape
                .as_ball()
                .map(ProfileOther::Ball)
                .or_else(|| shape.as_cuboid().map(ProfileOther::Cuboid))
        })
}

use std::sync::Mutex;

use rapier3d::geometry::{CollisionEvent, CollisionEventFlags, ContactPair, SolverFlags};
use rapier3d::prelude::{
    ColliderSet, ContactForceEvent, EventHandler, PhysicsHooks, Real, RigidBodySet,
};

use crate::ffi::WorldHandle;
use crate::ffi::{
    Bool, ColliderHandleRaw, CollisionEventRecord, ContactForceEventRecord, RigidBodyHandleRaw,
    pack_collider_handle, pack_rigid_body_handle, vec3_from_rapier,
};

const EVENT_RETAIN_CAPACITY: usize = 1024;
const EVENT_SHRINK_THRESHOLD: usize = EVENT_RETAIN_CAPACITY * 8;

pub type ContactPairFilterCallback = extern "C" fn(
    usize,
    ColliderHandleRaw,
    ColliderHandleRaw,
    Bool,
    RigidBodyHandleRaw,
    Bool,
    RigidBodyHandleRaw,
) -> u32;
pub type IntersectionPairFilterCallback = extern "C" fn(
    usize,
    ColliderHandleRaw,
    ColliderHandleRaw,
    Bool,
    RigidBodyHandleRaw,
    Bool,
    RigidBodyHandleRaw,
) -> Bool;

#[derive(Default)]
pub(crate) struct CollectingEventHandler {
    collision_events: Mutex<Vec<CollisionEventRecord>>,
    contact_force_events: Mutex<Vec<ContactForceEventRecord>>,
}

impl CollectingEventHandler {
    pub(crate) fn clear(&self) {
        clear_events(&mut self.collision_events.lock().expect("collision events lock"));
        clear_events(
            &mut self
                .contact_force_events
                .lock()
                .expect("contact force events lock"),
        );
    }

    pub(crate) fn collision_event_count(&self) -> usize {
        self.collision_events
            .lock()
            .expect("collision events lock")
            .len()
    }

    pub(crate) fn collision_event(&self, index: usize) -> Option<CollisionEventRecord> {
        self.collision_events
            .lock()
            .expect("collision events lock")
            .get(index)
            .copied()
    }

    pub(crate) fn contact_force_event_count(&self) -> usize {
        self.contact_force_events
            .lock()
            .expect("contact force events lock")
            .len()
    }

    pub(crate) fn contact_force_event(&self, index: usize) -> Option<ContactForceEventRecord> {
        self.contact_force_events
            .lock()
            .expect("contact force events lock")
            .get(index)
            .copied()
    }
}

fn clear_events<T>(events: &mut Vec<T>) {
    events.clear();
    if events.capacity() > EVENT_SHRINK_THRESHOLD {
        events.shrink_to(EVENT_RETAIN_CAPACITY);
    }
}

impl EventHandler for CollectingEventHandler {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        let record = match event {
            CollisionEvent::Started(h1, h2, flags) => CollisionEventRecord {
                started: Bool::TRUE,
                collider1: pack_collider_handle(h1),
                collider2: pack_collider_handle(h2),
                sensor: flags.contains(CollisionEventFlags::SENSOR).into(),
                removed: flags.contains(CollisionEventFlags::REMOVED).into(),
            },
            CollisionEvent::Stopped(h1, h2, flags) => CollisionEventRecord {
                started: Bool::FALSE,
                collider1: pack_collider_handle(h1),
                collider2: pack_collider_handle(h2),
                sensor: flags.contains(CollisionEventFlags::SENSOR).into(),
                removed: flags.contains(CollisionEventFlags::REMOVED).into(),
            },
        };

        self.collision_events
            .lock()
            .expect("collision events lock")
            .push(record);
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        let event = ContactForceEvent::from_contact_pair(dt, contact_pair, total_force_magnitude);
        self.contact_force_events
            .lock()
            .expect("contact force events lock")
            .push(ContactForceEventRecord {
                collider1: pack_collider_handle(event.collider1),
                collider2: pack_collider_handle(event.collider2),
                total_force: vec3_from_rapier(event.total_force),
                total_force_magnitude: event.total_force_magnitude,
                max_force_direction: vec3_from_rapier(event.max_force_direction),
                max_force_magnitude: event.max_force_magnitude,
            });
    }
}

#[derive(Default)]
pub(crate) struct CallbackPhysicsHooks {
    pub(crate) contact_pair_filter: Option<ContactPairFilterCallback>,
    pub(crate) intersection_pair_filter: Option<IntersectionPairFilterCallback>,
    pub(crate) user_data: usize,
}

impl PhysicsHooks for CallbackPhysicsHooks {
    fn filter_contact_pair(
        &self,
        context: &rapier3d::prelude::PairFilterContext,
    ) -> Option<SolverFlags> {
        let Some(callback) = self.contact_pair_filter else {
            return Some(SolverFlags::COMPUTE_IMPULSES);
        };

        let flags = callback(
            self.user_data,
            pack_collider_handle(context.collider1),
            pack_collider_handle(context.collider2),
            context.rigid_body1.is_some().into(),
            context
                .rigid_body1
                .map(pack_rigid_body_handle)
                .unwrap_or_default(),
            context.rigid_body2.is_some().into(),
            context
                .rigid_body2
                .map(pack_rigid_body_handle)
                .unwrap_or_default(),
        );

        if flags == u32::MAX {
            None
        } else {
            Some(SolverFlags::from_bits_truncate(flags))
        }
    }

    fn filter_intersection_pair(&self, context: &rapier3d::prelude::PairFilterContext) -> bool {
        let Some(callback) = self.intersection_pair_filter else {
            return true;
        };

        callback(
            self.user_data,
            pack_collider_handle(context.collider1),
            pack_collider_handle(context.collider2),
            context.rigid_body1.is_some().into(),
            context
                .rigid_body1
                .map(pack_rigid_body_handle)
                .unwrap_or_default(),
            context.rigid_body2.is_some().into(),
            context
                .rigid_body2
                .map(pack_rigid_body_handle)
                .unwrap_or_default(),
        )
        .0 != 0
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_events(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn world_collision_event_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.events.collision_event_count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_collision_event(
    world: *const WorldHandle,
    index: u32,
) -> CollisionEventRecord {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return CollisionEventRecord::default();
    };
    world
        .inner
        .events
        .collision_event(index as usize)
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn world_contact_force_event_count(world: *const WorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.events.contact_force_event_count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn world_get_contact_force_event(
    world: *const WorldHandle,
    index: u32,
) -> ContactForceEventRecord {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return ContactForceEventRecord::default();
    };
    world
        .inner
        .events
        .contact_force_event(index as usize)
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_contact_pair_filter_callback(
    world: *mut WorldHandle,
    callback: ContactPairFilterCallback,
    user_data: usize,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.contact_pair_filter = Some(callback);
    world.inner.hooks.user_data = user_data;
}

#[unsafe(no_mangle)]
pub extern "C" fn world_set_intersection_pair_filter_callback(
    world: *mut WorldHandle,
    callback: IntersectionPairFilterCallback,
    user_data: usize,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.intersection_pair_filter = Some(callback);
    world.inner.hooks.user_data = user_data;
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_contact_pair_filter_callback(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.contact_pair_filter = None;
}

#[unsafe(no_mangle)]
pub extern "C" fn world_clear_intersection_pair_filter_callback(world: *mut WorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.intersection_pair_filter = None;
}

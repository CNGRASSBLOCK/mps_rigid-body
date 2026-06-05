use std::sync::Mutex;

use rapier3d::geometry::{CollisionEvent, CollisionEventFlags, ContactPair, SolverFlags};
use rapier3d::prelude::{ColliderSet, ContactForceEvent, EventHandler, PhysicsHooks, Real, RigidBodySet};

use crate::ffi::{
    RcBool, RcColliderHandle, RcCollisionEventRecord, RcContactForceEventRecord, RcRigidBodyHandle,
    pack_collider_handle, pack_rigid_body_handle, vec3_from_rapier,
};
use crate::ffi::RcWorldHandle;

pub type RcContactPairFilterCallback = extern "C" fn(
    usize,
    RcColliderHandle,
    RcColliderHandle,
    RcBool,
    RcRigidBodyHandle,
    RcBool,
    RcRigidBodyHandle,
) -> u32;
pub type RcIntersectionPairFilterCallback = extern "C" fn(
    usize,
    RcColliderHandle,
    RcColliderHandle,
    RcBool,
    RcRigidBodyHandle,
    RcBool,
    RcRigidBodyHandle,
) -> RcBool;

#[derive(Default)]
pub(crate) struct CollectingEventHandler {
    collision_events: Mutex<Vec<RcCollisionEventRecord>>,
    contact_force_events: Mutex<Vec<RcContactForceEventRecord>>,
}

impl CollectingEventHandler {
    pub(crate) fn clear(&self) {
        self.collision_events.lock().expect("collision events lock").clear();
        self.contact_force_events
            .lock()
            .expect("contact force events lock")
            .clear();
    }

    pub(crate) fn collision_event_count(&self) -> usize {
        self.collision_events
            .lock()
            .expect("collision events lock")
            .len()
    }

    pub(crate) fn collision_event(&self, index: usize) -> Option<RcCollisionEventRecord> {
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

    pub(crate) fn contact_force_event(&self, index: usize) -> Option<RcContactForceEventRecord> {
        self.contact_force_events
            .lock()
            .expect("contact force events lock")
            .get(index)
            .copied()
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
            CollisionEvent::Started(h1, h2, flags) => RcCollisionEventRecord {
                started: RcBool::TRUE,
                collider1: pack_collider_handle(h1),
                collider2: pack_collider_handle(h2),
                sensor: flags.contains(CollisionEventFlags::SENSOR).into(),
                removed: flags.contains(CollisionEventFlags::REMOVED).into(),
            },
            CollisionEvent::Stopped(h1, h2, flags) => RcCollisionEventRecord {
                started: RcBool::FALSE,
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
            .push(RcContactForceEventRecord {
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
    pub(crate) contact_pair_filter: Option<RcContactPairFilterCallback>,
    pub(crate) intersection_pair_filter: Option<RcIntersectionPairFilterCallback>,
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

    fn filter_intersection_pair(
        &self,
        context: &rapier3d::prelude::PairFilterContext,
    ) -> bool {
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
        .0
            != 0
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_clear_events(world: *mut RcWorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.events.clear();
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_collision_event_count(world: *const RcWorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.events.collision_event_count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_get_collision_event(
    world: *const RcWorldHandle,
    index: u32,
) -> RcCollisionEventRecord {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcCollisionEventRecord::default();
    };
    world
        .inner
        .events
        .collision_event(index as usize)
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_contact_force_event_count(world: *const RcWorldHandle) -> u32 {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return 0;
    };
    world.inner.events.contact_force_event_count() as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_get_contact_force_event(
    world: *const RcWorldHandle,
    index: u32,
) -> RcContactForceEventRecord {
    let Some(world) = (unsafe { world.as_ref() }) else {
        return RcContactForceEventRecord::default();
    };
    world
        .inner
        .events
        .contact_force_event(index as usize)
        .unwrap_or_default()
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_set_contact_pair_filter_callback(
    world: *mut RcWorldHandle,
    callback: RcContactPairFilterCallback,
    user_data: usize,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.contact_pair_filter = Some(callback);
    world.inner.hooks.user_data = user_data;
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_set_intersection_pair_filter_callback(
    world: *mut RcWorldHandle,
    callback: RcIntersectionPairFilterCallback,
    user_data: usize,
) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.intersection_pair_filter = Some(callback);
    world.inner.hooks.user_data = user_data;
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_clear_contact_pair_filter_callback(world: *mut RcWorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.contact_pair_filter = None;
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_clear_intersection_pair_filter_callback(world: *mut RcWorldHandle) {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return;
    };
    world.inner.hooks.intersection_pair_filter = None;
}

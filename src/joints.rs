use rapier3d::prelude::{
    FixedJointBuilder, ImpulseJointHandle, PrismaticJointBuilder, RevoluteJointBuilder,
    RopeJointBuilder, SphericalJointBuilder, SpringJointBuilder, Vector,
};

use crate::ffi::{
    RcBool, RcImpulseJointHandle, RcJointAxis, RcJointBuilderHandle, RcJointType,
    RcRigidBodyHandle, RcVec3, RcWorldHandle, joint_axis_to_rapier, pack_impulse_joint_handle,
    unpack_impulse_joint_handle, unpack_rigid_body_handle, vec3_to_rapier,
};

pub(crate) enum JointBuilderKind {
    Fixed(FixedJointBuilder),
    Revolute(RevoluteJointBuilder),
    Prismatic(PrismaticJointBuilder),
    Rope(RopeJointBuilder),
    Spring(SpringJointBuilder),
    Spherical(SphericalJointBuilder),
}

impl JointBuilderKind {
    fn set_contacts_enabled(&mut self, enabled: bool) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => {
                JointBuilderKind::Fixed(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Spring(builder) => {
                JointBuilderKind::Spring(builder.contacts_enabled(enabled))
            }
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.contacts_enabled(enabled))
            }
        };
    }

    fn set_local_anchor1(&mut self, anchor: Vector) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => {
                JointBuilderKind::Fixed(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Spring(builder) => {
                JointBuilderKind::Spring(builder.local_anchor1(anchor))
            }
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.local_anchor1(anchor))
            }
        };
    }

    fn set_local_anchor2(&mut self, anchor: Vector) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => {
                JointBuilderKind::Fixed(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Spring(builder) => {
                JointBuilderKind::Spring(builder.local_anchor2(anchor))
            }
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.local_anchor2(anchor))
            }
        };
    }

    fn set_limits(&mut self, axis: RcJointAxis, min: f64, max: f64) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => JointBuilderKind::Fixed(builder),
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.limits([min, max]))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.limits([min, max]))
            }
            JointBuilderKind::Rope(builder) => JointBuilderKind::Rope(builder),
            JointBuilderKind::Spring(builder) => JointBuilderKind::Spring(builder),
            JointBuilderKind::Spherical(builder) => {
                JointBuilderKind::Spherical(builder.limits(joint_axis_to_rapier(axis), [min, max]))
            }
        };
    }

    fn set_motor_velocity(&mut self, axis: RcJointAxis, target_vel: f64, factor: f64) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => JointBuilderKind::Fixed(builder),
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.motor_velocity(target_vel, factor))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.motor_velocity(target_vel, factor))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.motor_velocity(target_vel, factor))
            }
            JointBuilderKind::Spring(builder) => JointBuilderKind::Spring(builder),
            JointBuilderKind::Spherical(builder) => JointBuilderKind::Spherical(
                builder.motor_velocity(joint_axis_to_rapier(axis), target_vel, factor),
            ),
        };
    }

    fn set_motor_position(
        &mut self,
        axis: RcJointAxis,
        target_pos: f64,
        stiffness: f64,
        damping: f64,
    ) {
        *self = match std::mem::replace(self, JointBuilderKind::Fixed(FixedJointBuilder::new())) {
            JointBuilderKind::Fixed(builder) => JointBuilderKind::Fixed(builder),
            JointBuilderKind::Revolute(builder) => {
                JointBuilderKind::Revolute(builder.motor_position(target_pos, stiffness, damping))
            }
            JointBuilderKind::Prismatic(builder) => {
                JointBuilderKind::Prismatic(builder.motor_position(target_pos, stiffness, damping))
            }
            JointBuilderKind::Rope(builder) => {
                JointBuilderKind::Rope(builder.motor_position(target_pos, stiffness, damping))
            }
            JointBuilderKind::Spring(builder) => JointBuilderKind::Spring(builder),
            JointBuilderKind::Spherical(builder) => JointBuilderKind::Spherical(
                builder.motor_position(joint_axis_to_rapier(axis), target_pos, stiffness, damping),
            ),
        };
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_create(
    joint_type: RcJointType,
    axis_or_primary: RcVec3,
    b: f64,
    c: f64,
) -> *mut RcJointBuilderHandle {
    let inner = match joint_type {
        RcJointType::Fixed => JointBuilderKind::Fixed(FixedJointBuilder::new()),
        RcJointType::Revolute => {
            JointBuilderKind::Revolute(RevoluteJointBuilder::new(vec3_to_rapier(axis_or_primary)))
        }
        RcJointType::Prismatic => {
            JointBuilderKind::Prismatic(PrismaticJointBuilder::new(vec3_to_rapier(axis_or_primary)))
        }
        RcJointType::Rope => JointBuilderKind::Rope(RopeJointBuilder::new(b)),
        RcJointType::Spring => {
            JointBuilderKind::Spring(SpringJointBuilder::new(axis_or_primary.x, b, c))
        }
        RcJointType::Spherical => JointBuilderKind::Spherical(SphericalJointBuilder::new()),
    };

    Box::into_raw(Box::new(RcJointBuilderHandle { inner }))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_destroy(builder: *mut RcJointBuilderHandle) {
    if builder.is_null() {
        return;
    }

    unsafe {
        drop(Box::from_raw(builder));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_set_contacts_enabled(
    builder: *mut RcJointBuilderHandle,
    enabled: RcBool,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder.inner.set_contacts_enabled(enabled.0 != 0);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_set_local_anchor1(
    builder: *mut RcJointBuilderHandle,
    anchor: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder.inner.set_local_anchor1(vec3_to_rapier(anchor));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_set_local_anchor2(
    builder: *mut RcJointBuilderHandle,
    anchor: RcVec3,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder.inner.set_local_anchor2(vec3_to_rapier(anchor));
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_set_limits(
    builder: *mut RcJointBuilderHandle,
    axis: RcJointAxis,
    min: f64,
    max: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder.inner.set_limits(axis, min, max);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_set_motor_velocity(
    builder: *mut RcJointBuilderHandle,
    axis: RcJointAxis,
    target_vel: f64,
    factor: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder.inner.set_motor_velocity(axis, target_vel, factor);
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_joint_builder_set_motor_position(
    builder: *mut RcJointBuilderHandle,
    axis: RcJointAxis,
    target_pos: f64,
    stiffness: f64,
    damping: f64,
) {
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return;
    };
    builder
        .inner
        .set_motor_position(axis, target_pos, stiffness, damping);
}

fn build_and_insert(
    world: &mut RcWorldHandle,
    body1: RcRigidBodyHandle,
    body2: RcRigidBodyHandle,
    builder: JointBuilderKind,
    wake_up: bool,
) -> ImpulseJointHandle {
    let body1 = unpack_rigid_body_handle(body1);
    let body2 = unpack_rigid_body_handle(body2);
    match builder {
        JointBuilderKind::Fixed(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Revolute(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Prismatic(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Rope(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Spring(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
        JointBuilderKind::Spherical(builder) => {
            world
                .inner
                .impulse_joints
                .insert(body1, body2, builder.build(), wake_up)
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_insert_impulse_joint(
    world: *mut RcWorldHandle,
    body1: RcRigidBodyHandle,
    body2: RcRigidBodyHandle,
    builder: *mut RcJointBuilderHandle,
    wake_up: RcBool,
) -> RcImpulseJointHandle {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return 0;
    };
    let Some(builder) = (unsafe { builder.as_mut() }) else {
        return 0;
    };

    let joint = std::mem::replace(
        &mut builder.inner,
        JointBuilderKind::Fixed(FixedJointBuilder::new()),
    );
    pack_impulse_joint_handle(build_and_insert(world, body1, body2, joint, wake_up.0 != 0))
}

#[unsafe(no_mangle)]
pub extern "C" fn rc_world_remove_impulse_joint(
    world: *mut RcWorldHandle,
    handle: RcImpulseJointHandle,
    wake_up: RcBool,
) -> RcBool {
    let Some(world) = (unsafe { world.as_mut() }) else {
        return RcBool::FALSE;
    };

    world
        .inner
        .impulse_joints
        .remove(unpack_impulse_joint_handle(handle), wake_up.0 != 0)
        .is_some()
        .into()
}

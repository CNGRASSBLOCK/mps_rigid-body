#![allow(clippy::missing_safety_doc)]

mod abi;
mod rapier;
mod helper;

pub use rapier::ffi::{
    AabbDesc, BodyStatus, Bool, CRbTreeHandle, Capsule, CharacterCollision,
    CharacterControllerHandle, ColliderBuilderHandle, ColliderHandleRaw, CollisionEventRecord,
    ContactForceEventRecord, Cylinder, EffectiveCharacterMovement, Ellipsoid,
    ImpulseJointHandleRaw, InteractionGroupsDesc, JointAxisDesc, JointBuilderHandle, JointTypeDesc,
    KdopPreset, NeuralActivation, NeuralBoundsDesc, Obb, PointProjection, Prism, Quat,
    QueryFilterDesc, RTreeHandle, RayHit, RigidBodyBuilderHandle, RigidBodyHandleRaw, ShapeCastHit,
    ShapeCastOptionsDesc, ShapeDesc, ShapeType, Sphere, SphericalShell, Ssv, Vec3,
    VoxelColliderMode, VoxelColliderOptions, WorldHandle,
};

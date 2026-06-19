#![allow(clippy::missing_safety_doc)]

use mimalloc::MiMalloc;

#[global_allocator]
static GLOBAL_ALLOCATOR: MiMalloc = MiMalloc;

mod abi;
mod helper;
mod rapier;

pub use rapier::ffi::{
    AabbDesc, AeroForceReport, AeroSurface, AirlockDepressurization, AnvilKitAppHandle,
    AtomicOxygenErosion, BangOffBangProfile, BatteryEquivalentCircuit, BodyStatus, Bool,
    CRbTreeHandle, Capsule, CharacterCollision, CharacterControllerHandle, ChemicalReactionRate,
    CmgExchange, CmgRobustInverse, Co2MassBalance, ColliderBuilderHandle, ColliderHandleRaw,
    CollisionEventRecord, CollisionProbability, ContactForceEventRecord, ContactForceModel,
    CwDerivative, CwState, Cylinder, DhTransform, EffectiveCharacterMovement, Ellipsoid,
    FlexibleModeDerivative, FluidForceReport, FluidLoopHeatTransfer, FluidVolume, FriisLink,
    GnssObservation, HallThrusterPerformance, HohmannTransfer, ImpulseJointHandleRaw,
    InteractionGroupsDesc, JointAxisDesc, JointBuilderHandle, JointTypeDesc, KdopPreset,
    LeastSquaresAttitude, ManipulatorDynamics, MassProperties, NeuralActivation, NeuralBoundsDesc,
    Obb, OrbitalElements, PointProjection, Prism, Quat, QuaternionDerivative, QueryFilterDesc,
    RTreeHandle, RadarMeasurement, RadiatorPower, RayHit, RigidBodyBuilderHandle,
    RigidBodyEulerDerivative, RigidBodyHandleRaw, ScalarKalman, Sgp4SecularRates, ShapeCastHit,
    ShapeCastOptionsDesc, ShapeDesc, ShapeType, SloshPendulumDerivative, SolarPanelPower, Sphere,
    SphericalShell, Ssv, StateVector, ThermalBalance, TrajectoryEnvironment, TrajectoryForceReport,
    TrajectoryGlideEnvironment, TrajectoryGlideReport, TrajectoryGlideState, TrajectoryState,
    VariationalState, Vec3, VoxelBuildStats, VoxelColliderMode, VoxelColliderOptions, WorldHandle,
};

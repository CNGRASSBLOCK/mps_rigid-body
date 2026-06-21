#![allow(clippy::missing_safety_doc)]

use mimalloc::MiMalloc;

#[global_allocator]
static GLOBAL_ALLOCATOR: MiMalloc = MiMalloc;

mod abi;
mod helper;
mod rapier;

pub use rapier::ffi::{
    AabbDesc, AcousticContactDesc, AcousticExcitationReport, AcousticMaterial,
    AcousticResonanceReport, AcousticWaveReport, AeroForceReport, AeroSurface, AirDragLaw,
    AirlockDepressurization, ApertureDesc, AtomicOxygenErosion, BangOffBangProfile,
    BatteryEquivalentCircuit, BernoulliReport, BifurcationPoint, BiotSavartVelocity, BodyStatus, Bool, BorisPusherParams, CRbTreeHandle, CamConstraintDesc,
    CamConstraintReport, Capsule, CatalystEffect, CatalystReport, CharacterCollision,
    CharacterControllerHandle, ChemicalReactionRate, ChaosDetectionParams, ChaosDetectionReport,
    CmgExchange, CmgRobustInverse, Co2MassBalance,
    ColliderBuilderHandle, ColliderHandleRaw, CollisionEventRecord, CollisionProbability,
    ComplexAmplitude, ConcentrationBuoyancyReport, ContactForceEventRecord, ContactForceModel, CoulombFrictionLaw,
    CustomPhysicsReport, CwDerivative, CwState, Cylinder, DensityFieldStats, DhTransform,
    DiffractionPoint, DoublePendulumAccel, DoublePendulumParams, DoublePendulumState,
    EffectiveCharacterMovement, ElectromagneticField, Ellipsoid, ExternalForceLaw,
    FaradayInductionReport, FdtdYeeReport, FemConstitutiveReport, FemHeatDiffusionReport,
    FemHeatEdge, FemHeatNode, FemShapeFunctionReport, FemTetrahedron, FlexibleModeDerivative,
    FluidForceReport, FluidLoopHeatTransfer, FluidVolume, FractureEnergyReport,
    FractureFragmentDesc, FractureMaterial, FractureModeReport, FractureReplaceReport, FresnelZoneReport, FriisLink,
    GearConstraintDesc, GearConstraintReport, GnssObservation, GpEnergyDensity, GpGridPoint,
    GpOrderParameter, GpTimeEvolutionParams,
    GrayScottParams, GrayScottReactionReport, GravitationalTimeDilation, GridField, GriffithReport,
    HallThrusterPerformance, HeatConductionReport,
    HertzContactReport, HillMuscleDesc, HillMuscleReport, HillMuscleState, HohmannTransfer,
    ImpulseJointHandleRaw, InteractionGroupsDesc, JointAxisDesc, JointBuilderHandle, JointTypeDesc,
    KdopPreset, KirchhoffDiffractionPoint,
    LeastSquaresAttitude, LengthContraction, LogisticMapState, LorentzBoost, LorentzForceReport,
    LorentzTransformedFrame, LorenzParams, LorenzState, LorenzStepReport, LyapunovReport,
    MagneticFluxReport, MagneticXPoint, ManipulatorDynamics, MassProperties, MaterialProperties, MaxwellPointReport, MinerDamageReport, ModalAnalysisReport,
    ModalSynthesisReport, MolecularForceLaw, MolecularPairReport, MolecularParticle, MpcConfig,
    MpcReport, NBodyForceReport, NBodyParticle, NBodySolverParams, NavierStokesReport,
    NeuralActivation, NeuralBoundsDesc, NewmarkBetaParameters, NewmarkBetaReport, Obb,
    OrbitalElements, OrbitalResonanceReport, PhaseChangeReport, PidGains, PidReport, PidState,
    PicParticle, PicStepReport, PlaneWaveParams, PlasmaParamsReport, PointProjection, PointSource, Prism, QuantisedCirculation,
    QuantumBarrier, QuantumOscillatorReport, QuantumTunnelingReport,
    QuantumWaveFunction, Quat, QuaternionDerivative, QueryFilterDesc, RTreeHandle,
    RadarMeasurement, RadiatorPower, RayHit, ReactionDiffusionReport, RelativisticOrbitReport,
    RelativisticParticle, RigidBodyBuilderHandle, RigidBodyEulerDerivative, RigidBodyHandleRaw,
    RocheLimitReport, ScalarKalman, SchwarzschildMetric, ScrewConstraintDesc, ScrewConstraintReport, Sgp4SecularRates, ShapeCastHit,
    ShapeCastOptionsDesc, ShapeDesc, ShapeType, SimpMaterialReport, SkeletalConstraintReport,
    SkeletalJointLimit, SloshPendulumDerivative, SnCurveReport, SoftBendingConstraint,
    SoftBodyStepReport, SoftDistanceConstraint, SoftSphereCollision, SoftSpring,
    SoftVolumeConstraint, SolarPanelPower, SpatializedSample, SphForceReport, SphParticle, Sphere,
    SphericalShell, SphericalWavePoint, SpiralConstraintDesc, SpiralConstraintReport, Ssv, StateSpaceReport,
    StateVector, StressIntensityReport, StressStrainReport, StructuralModeReport, ThermalBalance,
    ThermalRadiationReport, ThermalStressReport, ThermoelasticReport, ThinFilmInterferenceReport,
    ThinFilmParams, TopologyOptimizationParams,
    TopologyOptimizationReport, TrajectoryEnvironment, TrajectoryForceReport,
    TrajectoryGlideEnvironment, TrajectoryGlideReport, TrajectoryGlideState, TrajectoryState,
    VariationalState, Vec3, VlasovMomentReport, VortexReconnectionReport, VortexRing, VortexSegment, VortexTangleStats,
    VoxelBuildStats, VoxelColliderMode, VoxelColliderOptions, WorldHandle, YoungSlitPoint,
};

#[cfg(feature = "anvilkit-bridge")]
pub use rapier::ffi::AnvilKitAppHandle;

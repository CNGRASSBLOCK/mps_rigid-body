# mps_rigid_body 项目分析

## 1. 项目定位

`mps_rigid_body` 是一个基于 `rapier3d-f64` 的 Rust 原生物理运行时封装。
它同时构建为 `rlib` 和 `cdylib`，其中 `cdylib` 供 Java 侧通过 JNI 或 FFM 调用。

项目不是一个独立 Rust 应用，而是跨语言物理与工程公式运行时：

```text
Java 21 JNI / Java 25 FFM
  -> Rust C ABI / JNI wrappers
    -> src/rapier 功能模块
      -> rapier3d-f64 / 工程公式 / AnvilKit ECS
```

Rust 侧负责持有物理世界、刚体、碰撞体、关节、事件队列、查询管线、空间索引和工程公式实现。
Java 侧只保存不透明指针、packed handle 或 `MemorySegment`，所有实际计算和状态变更都回调到 Rust。

## 2. 源码结构

```text
src/
  lib.rs
  abi/
    ffm.rs
    jni.rs
    mod.rs
  helper/
    helper.rs
    mod.rs
  rapier/
    world.rs
    rigid_body.rs
    collider.rs
    query.rs
    events.rs
    joints.rs
    controller.rs
    voxel.rs
    bounds.rs
    dop.rs
    neural.rs
    compat.rs
    aerodynamics.rs
    fluid.rs
    trajectory.rs
    spaceflight.rs
    control_theory.rs
    anvilkit.rs
    ffi.rs
    error.rs
    mod.rs
    acoustics.rs
    astrophysics.rs
    biomechanics.rs
    chaos.rs
    continuum.rs
    electromagnetism.rs
    fracture.rs
    molecular.rs
    physchem.rs
    plasma.rs
    quantum.rs
    relativity.rs
    softbody.rs
    superfluidity.rs
    thermodynamics.rs
    topology.rs
    transmission.rs
    wave_optics.rs

test21/
  Java 21 JNI smoke test 和 Java 工具封装

test25/
  Java 25 FFM smoke test 和 FFM 绑定示例
```

`src/rapier` 是核心实现层。`src/abi` 将 Rust ABI 适配到 Java 入口。`src/helper` 主要提供 JNI 数组和本地内存辅助函数。

## 3. ABI 设计

`src/rapier/ffi.rs` 是 ABI 类型中心，定义所有跨 C ABI、JNI、Java FFM 使用的 `#[repr(C)]` 结构。

关键类型：

- `Vec3`：三维向量。
- `Quat`：四元数。
- `Bool`：单字节布尔封装。
- `ShapeDesc`：通用基础形状描述。
- `InteractionGroupsDesc`：碰撞组和求解组描述。
- `QueryFilterDesc`：查询过滤参数。
- `RayHit`、`PointProjection`、`ShapeCastHit`：查询结果。
- `CollisionEventRecord`、`ContactForceEventRecord`：事件记录。
- `VoxelColliderOptions`、`VoxelBuildStats`：体素构建配置和统计。
- `TrajectoryState`、`TrajectoryEnvironment`、`TrajectoryForceReport`：质点弹道力模型。
- `TrajectoryGlideState`、`TrajectoryGlideEnvironment`、`TrajectoryGlideReport`：助推滑翔/跳跃滑翔弹道模型。
- `OrbitalElements`、`StateVector`：轨道根数和笛卡尔状态。
- `ScalarKalman`、`LeastSquaresAttitude`、`GnssObservation`：估计与导航公式结果。
- `CmgExchange`、`CmgRobustInverse`、`RigidBodyEulerDerivative`、`QuaternionDerivative`：姿态动力学与控制结果。
- `HohmannTransfer`、`FriisLink`、`SolarPanelPower`、`ThermalBalance`、`RadiatorPower`：任务工程公式结果。
- `MassProperties`、`ContactForceModel`、`BatteryEquivalentCircuit`、`HallThrusterPerformance`：子系统工程公式结果。

设计原则是：不直接暴露 Rapier 内部 Rust 类型。外部调用方只使用稳定、C 兼容的结构和句柄。

### 3.1 不透明指针

Rust 长生命周期对象通过不透明指针暴露：

- `WorldHandle`
- `RigidBodyBuilderHandle`
- `ColliderBuilderHandle`
- `JointBuilderHandle`
- `CharacterControllerHandle`
- `RTreeHandle`
- `CRbTreeHandle`
- `AnvilKitAppHandle`

JNI 侧通常保存为 `long`，FFM 侧保存为 `MemorySegment`。

### 3.2 Packed Rapier Handle

Rapier 内部 handle 被编码成非零 `u64`：

```text
packed = ((generation << 32) | id) + 1
```

`0` 保留为 null/invalid。刚体、碰撞体、冲量关节都使用这种 packed handle。

### 3.3 FFM 友好 out-pointer

较大的结构体优先通过 out pointer 返回，避免 Java FFM 对按值返回大结构的 ABI 差异。

典型例子：

- `world_get_gravity_out`
- `rigid_body_get_translation_out`
- `query_cast_ray_out`
- `query_project_point_out`
- `query_cast_shape_out`
- `voxel_aabb_build_stats_out`

对于成功/失败返回，很多函数提供 `_flag` 版本，直接返回 `u8`，比返回 `Bool` 结构更适合 FFM。

## 4. Java 访问层

### 4.1 JNI

`src/abi/jni.rs` 暴露 `Java_*` JNI 函数，并委托到 Rust C ABI 实现。

职责：

- 将 Java primitive 转成 Rust ABI 结构。
- 将 Java 数组或 native address 转成 Rust slice。
- 将 Rust 结果写回 Java 数组或 native out buffer。
- 在 JNI 边界捕获 Rust panic。

Java 21 的工具封装在 `test21/src/main/java/.../util`。

### 4.2 Java 25 FFM

`test25/.../RigidBodyFfm.java` 是 Java 25 FFM 绑定示例。
它用 Java `MemoryLayout` 描述 Rust `repr(C)` ABI，并通过 downcall handle 调用 native symbol。

当前 FFM 示例覆盖：

- 世界创建、销毁、step。
- 重力读写、碰撞体数量。
- 刚体创建、插入、位姿/速度/力/冲量控制。
- 碰撞体创建、插入、运行时材质/分组/事件设置。
- CRbTree、RTree 基础操作。
- 体素 AABB/OBB 构建统计、碰撞体创建和查询。
- ray、point、AABB、OBB、sphere、shape cast 查询。
- 碰撞事件和接触力事件批量读取。

生成的 `include/rigid_body.h` 已暴露 `space_*`、`trajectory_*` 等公式函数，但 Java 25 示例还没有完整包装这些公式接口。需要时可以直接按头文件布局手写 FFM downcall。

## 5. Rapier 世界与刚体

`src/rapier/world.rs` 持有完整 Rapier 物理世界：

```text
PhysicsPipeline
gravity
IntegrationParameters
IslandManager
BroadPhaseBvh
NarrowPhase
RigidBodySet
ColliderSet
ImpulseJointSet
MultibodyJointSet
CCDSolver
hooks
event handler
```

主要能力：

- 创建/销毁世界。
- 设置和读取重力。
- 设置积分参数。
- 推进 `world_step`。
- 读取刚体和碰撞体数量。
- 批量快照刚体位姿和速度。
- 批量更新刚体位姿和速度。

`src/rapier/rigid_body.rs` 支持动态、固定、基于位置的运动学、基于速度的运动学刚体。
运行时可读写状态、位姿、线速度、角速度，并支持 force、torque、impulse、torque impulse、CCD、sleep/wakeup。

## 6. 碰撞体、查询和事件

### 6.1 碰撞体

`src/rapier/collider.rs` 支持基础形状：

- ball
- cuboid
- capsule
- cylinder
- cone
- round 系列形状

扩展构建器：

- OBB
- sphere
- heightmap
- convex hull
- point-cloud bounds
- double BV
- skewed/discrete OBB
- fused collapsing bounds
- edge BVH
- medial spheres

### 6.2 查询

`src/rapier/query.rs` 支持：

- ray cast 和 batch ray cast
- point projection
- point intersection count
- AABB / OBB / sphere intersection
- shape cast

查询使用 `QueryFilterDesc` 控制 flags、interaction groups、排除 collider、排除 rigid body。

### 6.3 事件

`src/rapier/events.rs` 收集：

- collision events
- contact-force events

事件队列使用 `parking_lot::Mutex` 包装，最大存储数量是 `16_384`。

## 7. 体素系统

`src/rapier/voxel.rs` 支持从以下来源构建体素碰撞体：

- 原始 occupancy grid 内存。
- JNI Java `byte[]`。
- AABB。
- OBB。

构建模式：

- `Auto`
- `Cuboids`
- `GreedyCuboids`
- `SurfaceMesh`

AABB 路径会生成全实心体素网格。OBB 路径先生成包围它的世界 AABB，再按 OBB 局部坐标采样体素中心。

重要函数：

- `collider_builder_create_voxels`
- `collider_builder_create_voxels_auto`
- `collider_builder_create_voxel_aabb`
- `collider_builder_create_voxel_obb`
- `voxel_build_stats`
- `voxel_aabb_build_stats`
- `voxel_obb_build_stats`
- `query_intersect_voxel_aabb`
- `query_intersect_voxel_obb`
- `world_insert_static_voxel_aabb`
- `world_insert_dynamic_voxel_obb`

## 8. 空间索引、扩展几何和控制器

空间索引：

- `src/rapier/crbtree.rs`
- `src/rapier/rtree.rs`

扩展几何：

- `bounds.rs`：capsule、SSV、ellipsoid、prism、cylinder、spherical shell。
- `dop.rs`：kDOP 和 FDH。
- `neural.rs`：neural bounds 构建和查询。
- `compat.rs`：动态 cuboid 批量插入、静态 trimesh 插入等兼容辅助。

关节和角色控制器：

- `joints.rs`：fixed、revolute、prismatic、rope、spring、spherical 等冲量关节。
- `controller.rs`：Rapier character controller 封装，支持移动、碰撞记录、impulse solving。

## 9. 公式层总览

项目现在不只是 Rapier wrapper，还包含大量工程公式模块。

公式层分为两类：

1. 纯 ABI 公式：输入参数，返回标量或 out-pointer 结构。
2. Rapier 集成公式：计算 force/torque 后直接作用到 `WorldHandle` 内的刚体。

### 9.1 trajectory.rs

`src/rapier/trajectory.rs` 包含：

- 质点轨迹力模型：重力、二次阻力、升力。
- `trajectory_apply_forces_to_body`：将力直接施加到 Rapier 刚体。
- 助推滑翔/跳跃滑翔模型：
  - speed
  - flight-path angle
  - altitude
  - downrange
  - exponential atmosphere
  - ballistic coefficient
  - lift-to-drag ratio
  - bank angle
  - RK4 单步积分

### 9.2 spaceflight.rs

`src/rapier/spaceflight.rs` 是航天和工程公式主模块。

轨道与相对运动：

- 开普勒第三定律。
- 轨道根数和笛卡尔状态互转。
- J2 摄动加速度。
- Clohessy-Wiltshire 相对运动。
- Lambert 椭圆转移时间方程。
- 霍曼转移速度增量。
- 半长轴衰减率。
- SGP4 风格 J2 长期摄动率。

姿态动力学和控制：

- 四元数姿态运动学。
- 刚体欧拉动力学。
- CMG 角动量交换。
- CMG 奇异鲁棒伪逆。
- 重力梯度力矩。
- 磁力矩器控制。
- Bang-Off-Bang 姿态机动。
- 太阳帆板 PD 定向控制。

导航与估计：

- TRIAD 姿态确定。
- 双矢量最小二乘姿态确定。
- 标量 EKF predict/gain/update。
- GNSS 伪距。
- 双差载波相位。
- Sagnac 效应。
- 微波雷达测距测速。

环境和结构：

- 大气密度标高模型。
- 大气阻力加速度。
- 太阳辐射压力。
- 辐射吸收剂量。
- 原子氧侵蚀。
- 表面充电电流平衡。
- Whipple 防护临界穿孔估算。
- 结构特征频率。
- 柔性太阳翼模态振动。
- 推进剂晃动等效摆。
- Hunt-Crossley 接触力模型。

热控、ECLSS、电源和推进：

- 热管热阻。
- 辐射器散热功率。
- 航天器热平衡。
- 单相流体回路传热。
- CO2 质量守恒。
- Sabatier 甲烷化。
- SPE 电解水制氧。
- 太阳能帆板输出功率。
- 锂离子电池等效电路。
- 霍尔推进器性能。
- 齐奥尔科夫斯基火箭方程。

机器人与对接：

- D-H 齐次变换矩阵。
- 六轴机械臂第一关节逆解。
- 第三关节角。
- 简化对角机械臂动力学。
- 人工势场制导。
- 对接滑翔斜率制导。
- 对接缓冲能量吸收。
- 气闸舱泄压。

### 9.4 新增领域模块：混沌、相对论、超流体、等离子体、波动光学

项目新增五个工程公式模块，均为纯 ABI 公式层：

- `chaos.rs` — 混沌与非线性动力学
  - **洛伦兹吸引子**：RK4 积分，单步和 N 步批量输出。
  - **李雅普诺夫指数**：轨道发散法（参考+扰动轨迹、周期性重归一化）；Rosenstein 算法
    （延迟坐标嵌入、最近邻发散跟踪）适用于任意时间序列。
  - **分岔图**：洛伦兹参数扫描（σ/ρ/β，Poincaré 截面在 x 局部最大值）；Logistic 映射 r 扫描。
  - **双摆（拉格朗日力学）**：从欧拉-拉格朗日方程推导 α₁、α₂，RK4 积分，能量守恒验证。
  - **混沌检测**：综合李雅普诺夫指数 + Grassberger–Procaccia 关联维 + 置信度评分。
  - **Logistic 映射**：单步、N 步迭代、分岔图。

- `relativity.rs` — 相对论效应
  - **洛伦兹运动学**：γ = 1/√(1−β²)、4×4 洛伦兹推进矩阵（三维速度）、四矢量变换、
    相对论速度叠加（三维通用公式）、快度 φ = artanh(β)、β 反推。
  - **史瓦西度规**：g_tt、g_rr 分量、史瓦西半径、光线偏折 δφ = 4GM/(bc²)、
    有效势（含 GR 修正项 −GM L²/c²r³）。
  - **引力时间膨胀**：静止因子 √(1−rs/r)、轨道因子 √(1−3rs/2r)、ISCO 边界检测。
  - **长度收缩**：L = L₀/γ 沿运动方向。
  - **相对论粒子**：γ、E = γmc²、K = (γ−1)mc²、p = γmv、快度；零质量光子支持；
    不变质量 E²/c⁴ − p²/c²，含快子检测。

- `superfluidity.rs` — 超流体与量子涡旋
  - **Biot–Savart 涡旋动力学**：直线涡旋段的诱导速度（解析公式，含端点/共线奇异性处理）；
    涡旋环自诱导速度 v = (κ/4πR)[ln(8R/ξ) − ½]。
  - **量子化环流**：环流量子 κ₀ = h/m（⁴He）；环路积分环流+量子化判定；
    切向速度采样估计绕数 n。
  - **Gross–Pitaevskii 模型**：序参量 ψ = √n·exp(iφ)，涡旋芯密度耗尽（Pade 近似），
    相位 2π 包裹；能量密度（动能 + 相互作用 + 捕获势）；虚时振幅演化。
  - **涡旋重连**：双线段最近点检测 + X 拓扑交换重连 + 能量耗散估计。
  - **涡旋缠结统计**：总长度、平均曲率、动能、线密度。
  - **GP 网格采样**：2D 截面波函数可视化。
  - **物性常数**：ξ = ħ/√(2mgn)、c = √(gn/m)、⁴He 原子质量与散射长度。

- `plasma.rs` — 等离子体物理
  - **等离子体参数**：德拜长度 λ_D、电子/离子等离子体频率 ω_pe/ω_pi、德拜球粒子数 N_D、热速度 v_th。
  - **PIC 基础组件**：Boris 推进器（二阶辛算法：半 E → B 旋转 → 半 E）；三线性场插值；
    电荷沉积 ρ = q·w/V_cell、j = ρ·v。
  - **弗拉索夫矩**：从粒子系综计算密度 n、体速度 u、温度 T、热流 q。
  - **自洽场求解**：1D 泊松方程（Thomas 三对角算法）、电场 E = −dφ/dx。
  - **磁重连**：2D X 点检测（B≈0 + ∇B Jacobian 行列式<0）；Sweet–Parker 率 1/√S；
    Petschek 快速重连率 π/4lnS；Alfvén 速度 v_A；Lundquist 数 S。
  - **PIC 步骤报告**：动能、场能、最大 E/B 场。

- `wave_optics.rs` — 波动光学与衍射
  - **波基础**：波数 k = 2π/λ、平面波、球面波（1/r 衰减）。
  - **惠更斯–菲涅尔原理**：N 点源叠加求任意观察点场。
  - **菲涅尔衍射**：矩形孔径近场衍射（抛物线相位近似、数值黎曼和）。
  - **菲涅尔–基尔霍夫衍射积分**：含倾斜因子 cosθ 和 −i/λ 前置因子。
  - **杨氏双缝干涉**：I = cos²(πdx/λD)·sinc²(πax/λD)，含单缝包络；1D 阵列批处理。
  - **薄膜干涉**：OPD = 2n₂t·cosθₜ、半波损失分析、Fresnel 反射系数、
    归一化强度；多波长光谱批处理（彩虹色合成）。
  - **菲涅尔波带**：rₙ = √(nλD)、相位交替 π、前 N 带相量和。
  - **网格采样**：2D 菲涅尔衍射图样缓存。

## 10. AnvilKit 集成

`src/rapier/anvilkit.rs` 持有 AnvilKit ECS app，并维护 entity 到 Rapier body/collider/joint 的映射。

当前能力：

- 创建/销毁 AnvilKit app。
- spawn body。
- spawn body with collider。
- 设置 entity transform。
- 设置材质。
- sync 到 Rapier world。
- entity -> body handle。
- entity -> collider handle。
- 创建和移除 constraint。
- 应用气动力、流体力和轨迹力。

已有 AnvilKit 施力函数：

- `anvilkit_app_apply_aero_surfaces`
- `anvilkit_app_apply_aero_voxel_grid`
- `anvilkit_app_apply_fluid_aabb_forces`
- `anvilkit_app_apply_trajectory_forces`

当前集成模式：

```text
AnvilKit entity_bits
  -> anvilkit_app_entity_to_body
  -> RigidBodyHandleRaw
  -> trajectory / spaceflight / aerodynamics / fluid 函数
  -> Rapier add_force / add_torque
```

下一步应增加 AnvilKit wrapper：

- `anvilkit_app_apply_space_j2_force`
- `anvilkit_app_apply_space_atmospheric_drag`
- `anvilkit_app_apply_space_solar_radiation_pressure`
- `anvilkit_app_apply_space_gravity_gradient_torque`
- `anvilkit_app_apply_space_magnetic_torquer`
- `anvilkit_app_apply_space_cmg_torque`

这样公式能力就形成三层复用：

```text
纯公式 ABI
  -> Rapier body force/torque 应用函数
    -> AnvilKit entity 级 API
```

## 11. Java 25 FFM 覆盖情况

Java 25 FFM 当前主要覆盖物理基础能力：

- ABI version。
- world create/destroy/step。
- gravity get/set。
- rigid body 创建、插入、位姿、速度、力、冲量。
- collider 创建、插入、材质、分组、事件设置。
- CRbTree / RTree。
- voxel AABB/OBB。
- ray、point、AABB、OBB、sphere、shape cast 查询。
- collision/contact-force event 批量读取。

公式函数已经出现在 `include/rigid_body.h`，但 `test25` 的 Java helper 还没完整封装。
如果 Java 侧要正式使用公式层，建议新增 `FfmSpaceflight` 或 `FfmFormula` 工具类。

## 12. 验证状态

已验证命令：

```powershell
cargo check
cargo test
```

最近结果：

```text
cargo check: passed
cargo test: passed, 172 tests
```

历史文档还列出 Java smoke test：

```powershell
cd test21
.\gradlew.bat check

cd ..\test25
.\gradlew.bat check
```

## 13. 后续建议

### 13.1 高优先级

1. AnvilKit spaceflight wrapper

   把已有 `space_apply_*_to_body` 包装成 `anvilkit_app_apply_space_*`。
   这样 Java 或上层 ECS 可以直接按 entity 施加 J2、大气阻力、太阳光压、重力梯度力矩、磁力矩器、CMG 力矩。

2. Java 25 FFM 公式封装

   增加 `FfmSpaceflight` / `FfmFormula`，封装 `space_*` 和 `trajectory_*`。
   这样 Java 侧不需要手写大量 downcall handle 和 `MemoryLayout`。

3. Java 25 FFM character controller

   JNI 已经有 controller，FFM 还应补齐 create/destroy、配置项、move shape、collision read、solve impulses。

4. Java 25 FFM joints

   补齐 joint builder、anchor、limit、motor、insert/remove。

### 13.2 中优先级

1. 高级 collider builder FFM 覆盖

   包括 heightmap、convex hull、point cloud bounds、kDOP、FDH、neural bounds 等。

2. Java 25 高层工具层

   建议提供：

   - `FfmPhysicsWorld`
   - `FfmRigidBody`
   - `FfmCollider`
   - `FfmQuery`
   - `FfmSpatialIndex`
   - `FfmVoxelGrid`
   - `FfmSpaceflight`

3. 批量查询 API

   增加 batch AABB、batch OBB、batch sphere、batch shape cast，降低 Java/native 调用开销。

### 13.3 长期方向

1. 体素资产管线

   支持 mesh/heightmap voxelize、bitset/RLE 压缩体素、chunked voxel streaming、material ID。

2. 材质表和接触行为配置

   用 ABI 安全的 material table 替代跨语言 callback。

3. 调试几何导出

   输出 collider AABB、接触点/法线、voxel 生成结果、query debug 信息。

4. 序列化快照

   保存和恢复 body、collider、joint、integration parameters，用于 replay、rollback、测试和编辑器。

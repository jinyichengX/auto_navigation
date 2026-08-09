import { ComponentDef } from '../types';

export const componentDefs: Record<string, ComponentDef> = {
  // ==================== ROS 2 核心组件 ====================
  xacro: {
    id: 'xacro',
    name: 'xacro 处理器',
    category: 'ros2_core',
    package: 'xacro',
    executable: 'xacro',
    description: 'Xacro（XML Macro）是 URDF 的宏预处理工具。它将 .xacro 文件中的宏展开为完整的 URDF XML 字符串，通过 launch 文件中的 Command 调用。\n\n你写的 fishbot.xacro 包含 <xacro:include> 和 <xacro:macro>，需要 xacro 工具展开成纯 URDF 后，Gazebo 才能理解。',
    subscribes: [],
    publishes: [
      { topic: '/robot_description', type: 'URDF string', direction: 'out', transport: 'ros2_dds' }
    ],
    belongsTo: 'ros2_dds',
    color: 'bg-emerald-900/60',
    borderColor: 'border-emerald-500/60',
  },
  robot_description: {
    id: 'robot_description',
    name: '/robot_description',
    category: 'concept',
    package: '-',
    executable: '-',
    description: 'ROS 2 参数服务器上的一个字符串参数，存储完整的 URDF 机器人描述。\n\n它不是"节点"，也不是"话题"——它就是 launch 文件中通过 ParameterValue 注入的一段 URDF 文本。robot_state_publisher 和 spawn_robot 都从这里读取机器人几何信息。',
    subscribes: [],
    publishes: [],
    belongsTo: 'concept',
    color: 'bg-slate-800/80',
    borderColor: 'border-slate-500/50',
  },
  robot_state_publisher: {
    id: 'robot_state_publisher',
    name: 'robot_state_publisher',
    category: 'ros2_core',
    package: 'robot_state_publisher',
    executable: 'robot_state_publisher',
    description: '功能：根据 URDF 中的关节定义，广播所有 link 之间的 TF 变换。\n\n输入：\n- /robot_description（URDF 参数，一次性读取）\n- /joint_states（关节位置/速度）\n\n输出：\n- /tf：非固定关节的实时变换（如轮子转动）\n- /tf_static：固定关节的变换（如 base_footprint→base_link），只发一次\n\n它不控制机器人，只管 TF 树。',
    subscribes: [
      { topic: '/joint_states', type: 'sensor_msgs/msg/JointState', direction: 'in', transport: 'ros2_dds' }
    ],
    publishes: [
      { topic: '/tf', type: 'tf2_msgs/msg/TFMessage', direction: 'out', transport: 'ros2_dds' },
      { topic: '/tf_static', type: 'tf2_msgs/msg/TFMessage', direction: 'out', transport: 'ros2_dds' }
    ],
    belongsTo: 'ros2_dds',
    color: 'bg-emerald-900/60',
    borderColor: 'border-emerald-500/60',
  },
  joint_state_publisher: {
    id: 'joint_state_publisher',
    name: 'joint_state_publisher',
    category: 'ros2_core',
    package: 'joint_state_publisher',
    executable: 'joint_state_publisher',
    description: '功能：读取 URDF 中所有非固定关节，发布它们的默认状态。\n\n在 Gazebo 仿真模式下，use_sim_time=True 且 rate=50Hz。\n\n在没有 Gazebo 的真机模式下，如果是带 GUI 的版本（joint_state_publisher_gui），用户可手动拖动滑块控制关节角度进行测试。\n\n通常发布所有关节值为 0.0（默认位置）。',
    subscribes: [],
    publishes: [
      { topic: '/joint_states', type: 'sensor_msgs/msg/JointState', direction: 'out', transport: 'ros2_dds' }
    ],
    belongsTo: 'ros2_dds',
    color: 'bg-emerald-900/60',
    borderColor: 'border-emerald-500/60',
  },
  teleop: {
    id: 'teleop',
    name: 'teleop_twist_keyboard',
    category: 'ros2_core',
    package: 'teleop_twist_keyboard',
    executable: 'teleop_twist_keyboard',
    description: '键盘遥控节点，捕获键盘按键转换为 /cmd_vel 速度指令。\n\n按 i 前进，, 后退，j 左转，l 右转，k 停止。\n\n它是 /cmd_vel 的生产者（发布者）。',
    subscribes: [],
    publishes: [
      { topic: '/cmd_vel', type: 'geometry_msgs/msg/Twist', direction: 'out', transport: 'ros2_dds' }
    ],
    belongsTo: 'ros2_dds',
    color: 'bg-emerald-900/60',
    borderColor: 'border-emerald-500/60',
  },

  // ==================== Gazebo 组件 ====================
  gz_sim: {
    id: 'gz_sim',
    name: 'Gazebo Fortress',
    category: 'gazebo',
    package: 'ros_gz_sim',
    executable: 'gz sim',
    description: '仿真器本体。加载 .sdf 世界文件，启动物理引擎（ODE/Bullet/DART）。\n\n内部运行 Gazebo Transport（零拷贝消息系统）、场景广播、物理仿真循环。\n\n通过 ros_gz_sim 的 IncludeLaunchDescription 启动。',
    subscribes: [],
    publishes: [],
    belongsTo: 'gz_transport',
    color: 'bg-cyan-900/50',
    borderColor: 'border-cyan-500/50',
  },
  gz_transport_layer: {
    id: 'gz_transport_layer',
    name: 'Gazebo Transport',
    category: 'concept',
    package: '-',
    executable: '-',
    description: 'Gazebo 内部的消息传输层，类似 ROS 2 的 DDS，但是 Gazebo 专用的零拷贝消息系统。\n\nGazebo 内部所有插件（DiffDrive、gazebo_ros2_control、传感器）都通过 Gazebo Transport 通信。\n\n关键：ROS 2 节点不能直接订阅 Gazebo Transport 话题！必须通过 ros_gz_bridge 翻译。\n\n用 ign topic -l 可以列出所有 Gazebo Transport 话题。',
    subscribes: [],
    publishes: [],
    belongsTo: 'gz_transport',
    color: 'bg-slate-800/80',
    borderColor: 'border-slate-500/50',
  },
  physics_engine: {
    id: 'physics_engine',
    name: '物理引擎',
    category: 'gazebo',
    package: '-',
    executable: '-',
    description: 'Gazebo 底层的物理仿真引擎（ODE/Bullet/DART）。\n\n负责：碰撞检测、摩擦力计算、重力模拟、刚体动力学。\n\nDiffDrive 插件或 gazebo_ros2_control 通过设置关节速度来驱动物理引擎，物理引擎再根据轮子与地面的摩擦力计算实际位移。',
    subscribes: [],
    publishes: [],
    belongsTo: 'gz_transport',
    color: 'bg-cyan-950/60',
    borderColor: 'border-cyan-700/50',
  },
  spawn_robot: {
    id: 'spawn_robot',
    name: 'spawn_robot\n(ros_gz_sim create)',
    category: 'gazebo',
    package: 'ros_gz_sim',
    executable: 'create',
    description: '功能：从 /robot_description 读取 URDF → 内部转换为 SDF → 在 Gazebo 世界中生成机器人模型。\n\n生成的模型在 Gazebo 侧以指定名称命名（如 fishbot），Gazebo 会自动创建 /model/fishbot/* 话题。\n\n参数：-topic robot_description（数据源）、-name fishbot（Gazebo 中名称）、-x/-y/-z（出生位置）。',
    subscribes: [
      { topic: '/robot_description', type: 'URDF string', direction: 'in', transport: 'ros2_dds' }
    ],
    publishes: [],
    belongsTo: 'both',
    color: 'bg-cyan-900/50',
    borderColor: 'border-cyan-500/50',
  },
  diff_drive_plugin: {
    id: 'diff_drive_plugin',
    name: 'DiffDrive 系统插件',
    category: 'gazebo',
    package: '-',
    executable: 'libignition-gazebo-diff-drive-system.so',
    description: 'Gazebo 系统插件，在 fishbot.xacro 的 <gazebo> 标签内通过 <plugin> 加载。\n\n功能：\n1. 订阅 gz.msgs.Twist → 差速解算（v_x, ω_z → 左右轮速度）→ 设置四个轮子关节速度\n2. 读取关节实际转角 → 反向推算里程计 → 发布 gz.msgs.Odometry\n3. 发布 /model/fishbot/tf（odom → base_footprint 的 TF）\n\n所有发布/订阅都在 Gazebo Transport 侧进行，必须通过 ros_gz_bridge 才能到达 ROS 2。',
    subscribes: [
      { topic: '/cmd_vel', type: 'gz.msgs.Twist', direction: 'in', transport: 'gz_transport' }
    ],
    publishes: [
      { topic: '/odom', type: 'gz.msgs.Odometry', direction: 'out', transport: 'gz_transport' },
      { topic: '/tf', type: 'gz.msgs.Pose_V', direction: 'out', transport: 'gz_transport' }
    ],
    belongsTo: 'gz_transport',
    color: 'bg-cyan-900/50',
    borderColor: 'border-cyan-500/50',
  },
  gz_ros2_control_plugin: {
    id: 'gz_ros2_control_plugin',
    name: 'gazebo_ros2_control\n系统插件',
    category: 'gazebo',
    package: 'gazebo_ros2_control',
    executable: 'libgazebo_ros2_control.so',
    description: 'Gazebo 系统插件，在 URDF <gazebo> 标签中加载。\n\n这个插件不直接控制机器人！它只是 gazebo_ros2_control 框架在 Gazebo 侧的"硬件接口桥梁"。\n\n它的角色：\n- 读取 URDF 中 <ros2_control> 标签，注册硬件接口（velocity/position 等）\n- write()：接收 controller_manager 下发的关节速度指令，设置 Gazebo 关节\n- read()：从 Gazebo 读取关节实际位置/速度，回传给 controller_manager\n\n类比：Gazebo 就是"虚拟硬件"，这个插件就是"虚拟硬件驱动"。',
    subscribes: [],
    publishes: [],
    belongsTo: 'gz_transport',
    color: 'bg-cyan-900/50',
    borderColor: 'border-cyan-500/50',
  },

  // ==================== 桥接组件 ====================
  ros_gz_bridge: {
    id: 'ros_gz_bridge',
    name: 'ros_gz_bridge\n(parameter_bridge)',
    category: 'bridge',
    package: 'ros_gz_bridge',
    executable: 'parameter_bridge',
    description: 'ROS 2 DDS 与 Gazebo Transport 之间的消息翻译器。\n\n核心概念：它不是"转发"，是"翻译"——两边消息格式不同。\n- ROS 2 侧：geometry_msgs/msg/Twist（DDS 序列化）\n- Gazebo 侧：gz.msgs.Twist（Protobuf 序列化）\n\n每条桥接规则格式：/话题@ROS类型@Gazebo类型\n\n当前桥接的 3 条：\n- /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist（下行：ROS→GZ）\n- /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry（上行：GZ→ROS）\n- /model/fishbot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V（上行：GZ→ROS）\n\n⚠️ 用 ros2_control 方案后，这三条桥接都可以删掉！',
    subscribes: [
      { topic: '/cmd_vel', type: 'geometry_msgs/msg/Twist', direction: 'in', transport: 'ros2_dds' },
      { topic: '/odom', type: 'gz.msgs.Odometry', direction: 'in', transport: 'gz_transport' },
      { topic: '/tf', type: 'gz.msgs.Pose_V', direction: 'in', transport: 'gz_transport' }
    ],
    publishes: [
      { topic: '/cmd_vel', type: 'gz.msgs.Twist', direction: 'out', transport: 'gz_transport' },
      { topic: '/odom', type: 'nav_msgs/msg/Odometry', direction: 'out', transport: 'ros2_dds' },
      { topic: '/tf', type: 'tf2_msgs/msg/TFMessage', direction: 'out', transport: 'ros2_dds' }
    ],
    belongsTo: 'both',
    color: 'bg-orange-900/50',
    borderColor: 'border-orange-500/60',
  },

  // ==================== ros2_control 组件 ====================
  controller_manager: {
    id: 'controller_manager',
    name: 'controller_manager',
    category: 'ros2_control',
    package: 'controller_manager',
    executable: 'controller_manager',
    description: 'ros2_control 框架的核心管理节点。\n\n功能：\n1. 启动时读取 URDF 中的 <ros2_control> 硬件接口定义\n2. 根据 YAML 配置文件加载和启动 controller\n3. 在 controller 和 hardware_interface 之间传递数据（update 循环）\n4. 管理 controller 的生命周期（加载/激活/停用/卸载/切换）\n\n它通过 service 调用驱动 controller 的切换。',
    subscribes: [],
    publishes: [],
    belongsTo: 'ros2_dds',
    color: 'bg-purple-900/50',
    borderColor: 'border-purple-500/50',
  },
  diff_drive_controller: {
    id: 'diff_drive_controller',
    name: 'diff_drive_controller',
    category: 'ros2_control',
    package: 'ros2_controllers',
    executable: 'diff_drive_controller',
    description: '差速驱动控制器，由 controller_manager 加载和管理。\n\n功能（与 DiffDrive 插件类似，但跑在 ROS 2 侧！）：\n1. 订阅 /cmd_vel（直接 ROS 2 DDS，不经过 bridge！）\n2. 差速解算：v_left, v_right = f(v_x, ω_z)\n3. write() 到硬件接口设置关节速度\n4. read() 从硬件接口读取关节实际状态\n5. 反向推算里程计 → 直接发布 /odom（ROS 2 DDS）\n6. 直接发布 /tf（odom → base_footprint，ROS 2 DDS）\n\n关键区别：所有通信都在 ROS 2 侧，不经过 Gazebo Transport，不需要 ros_gz_bridge！',
    subscribes: [
      { topic: '/cmd_vel', type: 'geometry_msgs/msg/Twist', direction: 'in', transport: 'ros2_dds' }
    ],
    publishes: [
      { topic: '/odom', type: 'nav_msgs/msg/Odometry', direction: 'out', transport: 'ros2_dds' },
      { topic: '/tf', type: 'tf2_msgs/msg/TFMessage', direction: 'out', transport: 'ros2_dds' }
    ],
    belongsTo: 'ros2_dds',
    color: 'bg-purple-900/50',
    borderColor: 'border-purple-500/50',
  },
  hardware_interface_gz: {
    id: 'hardware_interface_gz',
    name: 'GazeboSystem\n（硬件接口）',
    category: 'ros2_control',
    package: 'gazebo_ros2_control',
    executable: 'GazeboSystem',
    description: 'gazebo_ros2_control 提供的硬件接口实现。\n\n功能（与 controller 对接）：\n- 对外 export 速度指令接口（command_interface）和位置/速度状态接口（state_interface）\n- write(speed)：将 controller 计算的速度设置到 Gazebo 关节\n- read(state)：从 Gazebo 物理引擎读取关节实际位置和速度\n\n它不关心差速解算、不关心里程计——那些是 diff_drive_controller 的工作。它只管底层：设置关节速度，读取关节状态。\n\n类比：操作系统的"驱动层"——上层发指令，它操作硬件。',
    subscribes: [],
    publishes: [],
    belongsTo: 'both',
    color: 'bg-purple-900/50',
    borderColor: 'border-purple-500/50',
  },

  // ==================== 真机硬件组件 ====================
  real_motor_driver: {
    id: 'real_motor_driver',
    name: '真实电机驱动板',
    category: 'hardware',
    package: '-',
    executable: '-',
    description: '真实的电机驱动板（如 L298N、ODrive 等）。\n\n接收 PWM 信号或 CAN 总线指令，驱动直流电机转动。\n\n与仿真不同：这里的"物理引擎"是真实世界的物理学。',
    subscribes: [],
    publishes: [],
    belongsTo: 'real_hardware',
    color: 'bg-red-900/50',
    borderColor: 'border-red-500/50',
  },
  real_encoder: {
    id: 'real_encoder',
    name: '真实编码器',
    category: 'hardware',
    package: '-',
    executable: '-',
    description: '真实的电机编码器（霍尔传感器/光栅编码器）。\n\n读取轮子实际转动角度，反馈给 ros2_control 硬件接口。\n\n与仿真对比：Gazebo 物理引擎直接提供关节角度，无需编码器。',
    subscribes: [],
    publishes: [],
    belongsTo: 'real_hardware',
    color: 'bg-red-900/50',
    borderColor: 'border-red-500/50',
  },
  real_imu: {
    id: 'real_imu',
    name: '真实 IMU',
    category: 'hardware',
    package: '-',
    executable: '-',
    description: '真实的惯性测量单元（如 MPU6050、BMI160）。\n\n测量角速度和线加速度，数据通过 I2C/SPI 传输给主控。\n\n在仿真中，Gazebo 的 IMU 传感器插件直接计算并发布数据，无需真实传感器。',
    subscribes: [],
    publishes: [],
    belongsTo: 'real_hardware',
    color: 'bg-red-900/50',
    borderColor: 'border-red-500/50',
  },
  hardware_interface_real: {
    id: 'hardware_interface_real',
    name: 'DiffBotSystemHardware\n（真实硬件接口）',
    category: 'ros2_control',
    package: 'diffbot_hardware',
    executable: 'DiffBotSystemHardware',
    description: '真实机器人的 ros2_control 硬件接口实现。\n\n功能：\n- write(speed)：通过串口/CAN/GPIO 发送指令到真实电机驱动板\n- read(state)：通过串口/CAN/GPIO 从真实编码器读取角度和速度\n\n与 GazeboSystem 对比：\n- GazeboSystem：操作虚拟关节\n- DiffBotSystemHardware：操作真实硬件\n\n上层 diff_drive_controller 完全不知道下面是真机还是仿真——这就是 ros2_control 的抽象之美！',
    subscribes: [],
    publishes: [],
    belongsTo: 'real_hardware',
    color: 'bg-red-900/50',
    borderColor: 'border-red-500/50',
  },
};

export const categoryInfo: Record<ComponentCategory, { label: string; color: string; icon: string }> = {
  ros2_core: { label: 'ROS 2 核心节点', color: '#34d399', icon: '◉' },
  gazebo: { label: 'Gazebo 组件', color: '#00d4ff', icon: '◆' },
  bridge: { label: '桥接组件', color: '#ff6b35', icon: '⇄' },
  ros2_control: { label: 'ros2_control 组件', color: '#a78bfa', icon: '▣' },
  hardware: { label: '真实硬件', color: '#f87171', icon: '⬡' },
  concept: { label: '概念说明', color: '#94a3b8', icon: '◎' },
};

export const colorMap: Record<BelongsTo, string> = {
  ros2_dds: '#34d399',
  gz_transport: '#00d4ff',
  both: '#ff6b35',
  real_hardware: '#f87171',
  concept: '#94a3b8',
};

# ROS 2 机器人仿真标签大全（全层级 + 谁读取）

> 涵盖：URDF、Xacro、SDF、Gazebo 扩展、ros2_control、ROS 2 Launch
> 缩进表示父子层级关系

---

## 一、URDF 标准标签
### 读取者：`robot_state_publisher`、`ros_gz_sim create`、`rviz2`、`joint_state_publisher`

```
<robot name="xxx">                           ← 根元素，定义一个机器人模型
  │                                           谁读：几乎所有 ROS 节点（robot_state_publisher、rviz2、gz spawner）
  │
  ├── <link name="xxx">                      ← 定义一个刚体连杆（形状+惯性+碰撞+视觉）
  │   │                                        谁读：robot_state_publisher（TF）、rviz2（渲染）、gz（物理）
  │   │
  │   ├── <visual>                           ← 外观（给 rviz2/gz 看的，不影响物理）
  │   │   ├── <geometry>                     ← 几何形状（box/sphere/cylinder/mesh）
  │   │   │   ├── <box size="x y z"/>
  │   │   │   ├── <sphere radius="r"/>
  │   │   │   ├── <cylinder radius="r" length="l"/>
  │   │   │   └── <mesh filename="xxx.stl"/>
  │   │   ├── <origin xyz="..." rpy="..."/>  ← 该部分相对 link 原点的偏移
  │   │   └── <material name="xxx">          ← 颜色/纹理
  │   │       └── <color rgba="r g b a"/>
  │   │
  │   ├── <collision>                        ← 碰撞体（给 Gazebo 物理引擎用的，与物理计算相关）
  │   │   ├── <geometry>                     ← 同上（可以比 visual 更简化，减少计算量）
  │   │   └── <origin xyz="..." rpy="..."/>
  │   │
  │   └── <inertial>                         ← 惯性参数（质量+惯性矩阵，Gazebo 物理用）
  │       ├── <origin xyz="..." rpy="..."/>  ← 质心位置
  │       ├── <mass value="x"/>              ← 质量，单位 kg
  │       └── <inertia ixx="..." ixy="..." .../> ← 3x3 惯性张量
  │
  ├── <joint name="xxx" type="revolute|continuous|prismatic|fixed|floating|planar">
  │   │                                        谁读：robot_state_publisher（TF）、gz（物理关节约束）
  │   │
  │   ├── <parent link="xxx"/>               ← 父连杆名
  │   ├── <child link="xxx"/>                ← 子连杆名
  │   ├── <origin xyz="..." rpy="..."/>      ← 关节相对父连杆的位置
  │   ├── <axis xyz="x y z"/>                ← 关节旋转/移动轴（如绕z轴转动写 0 0 1）
  │   ├── <limit lower="..." upper="..." effort="..." velocity="..."/>
  │   │                                       ← 关节限位（下限/上限/最大力矩/最大速度）
  │   ├── <dynamics damping="..." friction="..."/>
  │   │                                       ← 关节阻尼和摩擦（仿真用）
  │   └── <mimic joint="xxx" multiplier="1" offset="0"/>
  │                                           ← 关节跟随（一个关节随另一个成比例运动）
  │
  └── <ros2_control name="xxx" type="system|actuator|sensor">
                                              ← 见本文「五、ros2_control 标签」
</robot>
```

---

## 二、Xacro 宏语言标签
### 读取者：`xacro` 工具（命令行或 launch 中的 Command）

Xacro 是 XML 宏预处理器，它在 launch 启动阶段被调用，把 `.xacro` 展开成纯 `.urdf` 再传给下游。下游从来没见过 xacro 标签。

```
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  │                                              ← 必须声明 xacro 命名空间
  │
  ├── <xacro:property name="xxx" value="yyy"/>   ← 定义变量，${xxx} 引用
  │
  ├── <xacro:macro name="xxx" params="a b c:=默认值">
  │   │                                            ← 定义宏（可复用组件），params 支持默认值
  │   └── ...宏体内容...                            ← 宏体内可以写任意 URDF 标签
  │   </xacro:macro>
  │
  ├── <xacro:xxx a="..." b="..."/>                ← 调用宏（xacro:宏名）
  │
  ├── <xacro:include filename="xxx.xacro"/>       ← 引入另一个 xacro 文件
  │
  ├── <xacro:if value="${条件表达式}">              ← 条件判断
  │       ...条件成立时的内容...
  │   </xacro:if>
  │
  ├── <xacro:unless value="${条件表达式}">          ← 条件取反
  │       ...条件不成立时的内容...
  │   </xacro:unless>
  │
  ├── ${表达式}                                     ← 数学表达式求值：${pi} ${width/2} ${len*cos(angle)}
  │
  └── <xacro:arg name="xxx" default="yyy"/>      ← 给 xacro 传参（命令行 xacro xxx.xacro xxx:=值）
</robot>
```

---

## 三、SDF 标准标签（Gazebo 原生格式）
### 读取者：Gazebo Sim（gz sim）

SDF 是 Gazebo 自己的世界/模型描述格式。URDF 加载到 Gazebo 时内部会自动转为 SDF。

```
<sdf version="1.8">                             ← SDF 版本号，根元素
  │
  ├── <world name="xxx">                        ← 定义一个世界（仿真场景）
  │   │                                          谁读：Gazebo Server（gz sim）
  │   │
  │   ├── <gravity>0 0 -9.8</gravity>           ← 重力加速度
  │   ├── <magnetic_field>0 0 0</magnetic_field>
  │   ├── <atmosphere type="adiabatic"/>
  │   ├── <physics type="ode">                  ← 物理引擎选型
  │   │   ├── <max_step_size>0.001</max_step_size>  ← 最大仿真步长
  │   │   ├── <real_time_factor>1</real_time_factor>← 实时因子（1=实时）
  │   │   └── <real_time_update_rate>1000</real_time_update_rate>
  │   ├── <scene>                               ← 场景设置
  │   │   ├── <ambient>r g b a</ambient>        ← 环境光
  │   │   ├── <background>r g b a</background>  ← 背景色
  │   │   ├── <shadows>true</shadows>
  │   │   └── <sky>                             ← 天空材质
  │   ├── <light name="xxx" type="directional|point|spot">
  │   │   ├── <pose>x y z roll pitch yaw</pose>
  │   │   ├── <diffuse>r g b a</diffuse>
  │   │   ├── <specular>r g b a</specular>
  │   │   ├── <attenuation>                     ← 光照衰减
  │   │   └── <direction>x y z</direction>
  │   │
  │   ├── <model name="xxx">                    ← 定义一个模型（可以是机器人、物体、地面等）
  │   │   │                                      谁读：Gazebo Server
  │   │   │
  │   │   ├── <static>true|false</static>       ← 静态模型不受物理影响
  │   │   ├── <pose>x y z roll pitch yaw</pose> ← 初始位姿
  │   │   │
  │   │   ├── <link name="xxx">                 ← SDF 连杆（与 URDF link 类似）
  │   │   │   ├── <pose>...</pose>
  │   │   │   ├── <inertial>
  │   │   │   │   ├── <pose>...</pose>           ← 质心位置
  │   │   │   │   ├── <mass>...</mass>
  │   │   │   │   └── <inertia>...</inertia>
  │   │   │   ├── <visual name="xxx">
  │   │   │   │   ├── <pose>...</pose>
  │   │   │   │   ├── <geometry>...</geometry>
  │   │   │   │   └── <material>...</material>
  │   │   │   ├── <collision name="xxx">
  │   │   │   │   ├── <pose>...</pose>
  │   │   │   │   ├── <geometry>...</geometry>
  │   │   │   │   └── <surface>                 ← 表面属性
  │   │   │   │       ├── <friction>
  │   │   │   │       │   ├── <ode>             ← ODE 引擎摩擦参数
  │   │   │   │       │   │   ├── <mu>1.0</mu>           ← 库伦摩擦系数
  │   │   │   │       │   │   └── <mu2>1.0</mu2>         ← 第二方向摩擦系数
  │   │   │   │       │   └── <torsional>       ← 扭转摩擦
  │   │   │   │       └── <bounce>
  │   │   │   │           ├── <restitution_coefficient>0</restitution_coefficient>  ← 弹性恢复系数
  │   │   │   │           └── <threshold>100000</threshold>
  │   │   │   └── <sensor ...>                   ← 见「四、Gazebo 扩展标签」
  │   │   │
  │   │   ├── <joint name="xxx" type="revolute|prismatic|fixed|ball|screw|universal|...">
  │   │   │   ├── <pose>...</pose>
  │   │   │   ├── <parent>xxx</parent>
  │   │   │   ├── <child>xxx</child>
  │   │   │   ├── <axis>
  │   │   │   │   ├── <xyz>x y z</xyz>
  │   │   │   │   ├── <use_parent_model_frame>true|false</use_parent_model_frame>
  │   │   │   │   ├── <limit>
  │   │   │   │   │   ├── <lower>-1.57</lower>
  │   │   │   │   │   ├── <upper>1.57</upper>
  │   │   │   │   │   ├── <effort>10</effort>
  │   │   │   │   │   └── <velocity>1</velocity>
  │   │   │   │   └── <dynamics>
  │   │   │   │       ├── <spring_reference>0</spring_reference>
  │   │   │   │       ├── <spring_stiffness>0</spring_stiffness>
  │   │   │   │       ├── <damping>0</damping>
  │   │   │   │       └── <friction>0</friction>
  │   │   │   └── <physics>
  │   │   │       └── <ode>
  │   │   │           ├── <cfm_damping>0</cfm_damping>
  │   │   │           ├── <limit>
  │   │   │           │   ├── <cfm>0</cfm>
  │   │   │           │   └── <erp>0.2</erp>
  │   │   │           └── <suspension>
  │   │   │               ├── <cfm>0</cfm>
  │   │   │               └── <erp>0.2</erp>
  │   │   │
  │   │   └── <plugin .../>                      ← 见「四、Gazebo 扩展标签」
  │   │
  │   ├── <gui fullscreen='0'>                   ← Gazebo GUI 设置
  │   │   └── <camera name='xxx'>
  │   │       └── <pose>...</pose>
  │   │
  │   └── <plugin .../>                          ← 世界级插件（物理引擎、场景等）
  │
  └── <model name="xxx">                         ← 独立模型文件（.sdf），可以被 include 引用
      └── ...同 <world> 下的 <model>
</sdf>
```

---

## 四、Gazebo 扩展标签（URDF 中的 `<gazebo>` 以及 Plugin）
### 读取者：Gazebo（通过 `ros_gz_sim create` 加载 URDF 时自动解析 `<gazebo>` 标签）

URDF 标准里没有 `<gazebo>` 标签，这是 **Gazebo 对 URDF 的扩展**。当 URDF 被 spawn 到 Gazebo 时，Gazebo 会扫描 URDF 里的 `<gazebo>` 标签来加载插件和设置物理参数。

```
<robot>
  │
  └── <gazebo>                                   ← Gazebo 扩展根标签（可以出现多次）
      │                                           谁读：Gazebo Server（加载 URDF 时）
      │
      ├── <plugin filename="libxxx.so" name="命名空间::插件类名">
      │   │                                       谁读：Gazebo Server 的插件加载器
      │   │                                        filename = .so 文件路径
      │   │                                        name = 插件全限定类名
      │   │
      │   └── ...插件特定参数...                    ← 各插件内部自定参数，没有统一 schema
      │
      └── ...物理参数...                           ← 不通过 plugin，直接设置 Gazebo 物理属性
```

### 4.1 常用 Gazebo 内置插件（Ignition / Fortress / Garden）

```
插件1：差速驱动 DiffDrive
══════════════════════════════
谁读：Gazebo Server
插件位置：libignition-gazebo-diff-drive-system.so
作用：接收 /cmd_vel，驱动轮子，发布 /odom + TF

<plugin filename="libignition-gazebo-diff-drive-system.so"
        name="ignition::gazebo::systems::DiffDrive">
    <left_joint>xxx</left_joint>          ← 左轮关节名（可以有多个）
    <right_joint>xxx</right_joint>        ← 右轮关节名
    <wheel_separation>0.5</wheel_separation>  ← 左右轮距 (m)
    <wheel_radius>0.032</wheel_radius>    ← 轮半径 (m)
    <odom_publish_frequency>30</odom_publish_frequency>  ← 里程计发布频率 (Hz)
    <topic>cmd_vel</topic>                ← 订阅的 Gazebo Transport 话题（默认 /cmd_vel）
    <odom_topic>/odom</odom_topic>        ← （可选）自定义里程计话题
    <robot_base_frame>base_footprint</robot_base_frame>  ← （可选）基坐标系名
    <max_linear_velocity>1.0</max_linear_velocity>       ← 最大线速度
    <max_angular_velocity>2.0</max_angular_velocity>     ← 最大角速度
    <max_linear_acceleration>3.0</max_linear_acceleration>
    <max_angular_acceleration>6.0</max_angular_acceleration>
    <min_velocity_cmd>0.01</min_velocity_cmd>  ← 最小速度指令（低于此为静止）
    <odom_frame_id>odom</odom_frame_id>
</plugin>


插件2：关节状态发布 JointStatePublisher
════════════════════════════════════════
谁读：Gazebo Server
插件位置：libignition-gazebo-joint-state-publisher-system.so
作用：读取 Gazebo 中关节的实际状态，发布到 Gazebo Transport 话题

<plugin filename="libignition-gazebo-joint-state-publisher-system.so"
        name="ignition::gazebo::systems::JointStatePublisher">
    <joint_name>xxx</joint_name>          ← 要发布的关节名
    <update_rate>50</update_rate>         ← 发布频率
    <topic>/joint_states</topic>          ← 发布话题
</plugin>


插件3：IMU 传感器
══════════════════
谁读：Gazebo Server
插件位置：libignition-gazebo-imu-system.so

<plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
</plugin>


插件4：激光雷达 (GPU Lidar)
══════════════════════════
谁读：Gazebo Server
插件位置：libignition-gazebo-gpu-lidar-sensor-system.so

<plugin filename="libignition-gazebo-gpu-lidar-sensor-system.so"
        name="ignition::gazebo::systems::GpuLidar">
    <topic>/scan</topic>                  ← 发布话题
    <update_rate>10</update_rate>         ← 扫描频率 (Hz)
    <lidar>
        <scan>
            <horizontal>
                <samples>360</samples>    ← 每圈采样点数
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
            </horizontal>
        </scan>
        <range>
            <min>0.1</min>
            <max>12.0</max>
            <resolution>0.01</resolution>
        </range>
    </lidar>
</plugin>


插件5：相机
══════════════
谁读：Gazebo Server
插件位置：libignition-gazebo-camera-sensor-system.so

<plugin filename="libignition-gazebo-camera-sensor-system.so"
        name="ignition::gazebo::systems::Camera">
    <topic>/camera</topic>
    <update_rate>30</update_rate>
    <camera>
        <horizontal_fov>1.047</horizontal_fov>  ← 水平视场角 (rad)
        <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
        </image>
        <clip>
            <near>0.1</near>
            <far>100</far>
        </clip>
    </camera>
</plugin>


插件6：接触传感器 ContactSensor
══════════════════════════════════
谁读：Gazebo Server
插件位置：libignition-gazebo-contact-system.so

<plugin filename="libignition-gazebo-contact-system.so"
        name="ignition::gazebo::systems::Contact">
    <contact>
        <collision>xxx_collision</collision>  ← 监控哪个碰撞体
    </contact>
    <topic>/contact</topic>
</plugin>
```

### 4.2 不在 `<plugin>` 里的物理 `<gazebo>` 标签（设置材料/摩擦/颜色）

```xml
<gazebo reference="link_name">                ← reference 指定某个 link
    <material>Gazebo/Red</material>           ← Gazebo 预设材质（影响摩擦力但不影响外观）
    <mu1>1.0</mu1>                            ← 摩擦系数 μ₁
    <mu2>1.0</mu2>                            ← 摩擦系数 μ₂
    <kp>1000000.0</kp>                        ← 接触刚度
    <kd>1.0</kd>                              ← 接触阻尼
    <maxVel>0.01</maxVel>                     ← 摩擦切换最大速度
    <minDepth>0.001</minDepth>                ← 最小穿透深度
</gazebo>
```

---

## 五、ros2_control 标签（写于 URDF 的 `<ros2_control>` 中）
### 读取者：`gazebo_ros2_control` 插件（仿真）/ 自定义 Hardware Interface（真机）

这些标签是 ros2_control 框架的硬件清单声明，写在 URDF/xacro 的 `<ros2_control>` 标签对中。

```
<ros2_control name="xxx" type="system|actuator|sensor">
  │                                           谁读：gazebo_ros2_control 系统插件（或真机自定义驱动）
  │                                           type=system    → 整体驱动（多关节统一管理，如四轮差速底盘）
  │                                           type=actuator  → 单独执行器（每个关节独立）
  │                                           type=sensor    → 纯传感器（只读不写）
  │
  ├── <hardware>                               ← 硬件驱动声明
  │   │                                         谁读：controller_manager → resource_manager
  │   │
  │   ├── <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  │   │                                          ← 硬件驱动类全限定名（仿真用这个，真机换成你自己的）
  │   │                                         谁读：resource_manager（通过 pluginlib 加载）
  │   │
  │   ├── <param name="robot_description">/robot_description</param>
  │   │                                          ← （可选）URDF 参数名，默认 /robot_description
  │   │
  │   └── ...其他硬件特定参数...
  │       <param name="xxx">yyy</param>          ← key-value 参数，传给硬件驱动
  │
  ├── <joint name="xxx">                       ← 声明一个关节
  │   │                                         谁读：controller_manager → resource_manager
  │   │
  │   ├── <command_interface name="velocity|position|effort">
  │   │   │                                      ← 声明该关节支持哪种控制模式
  │   │   │                                      （一个关节可以有多个 command_interface）
  │   │   │                                      谁读：controller_manager（检查控制器需要的接口是否可用）
  │   │   │
  │   │   ├── <param name="min">-1</param>       ← 最小速度/位置/力矩
  │   │   ├── <param name="max">1</param>        ← 最大速度/位置/力矩
  │   │   └── <param name="...">...</param>      ← 其他硬件相关限制参数
  │   │
  │   ├── <state_interface name="position|velocity|effort"/>
  │   │                                          ← 声明该关节支持读取哪些状态
  │   │                                          （可以同时声明多个 state_interface）
  │   │                                          谁读：controller_manager（检查状态接口）
  │   │
  │   └── ...更多 state_interface...
  │
  ├── <sensor name="xxx">                       ← （可选）声明传感器
  │   │                                         谁读：controller_manager
  │   │
  │   └── <state_interface name="imu|scan|..."/>
  │                                              ← 传感器数据接口类型
  │
  └── <gpio name="xxx">                         ← （可选）声明 GPIO
      └── <state_interface name="..."/>
</ros2_control>
```

### ros2_control 中 `command_interface` 和 `state_interface` 的 `name` 固定可选值

| name 值 | 含义 | 方向 |
|---------|------|------|
| `position` | 位置控制/读取 | 读写 |
| `velocity` | 速度控制/读取 | 读写 |
| `effort` | 力/力矩控制/读取 | 读写 |
| `acceleration` | 加速度控制 | 写 |

> 差速轮常用：`command_interface name="velocity"` + `state_interface name="position"` + `state_interface name="velocity"`

---

## 六、ROS 2 Launch 标签（Python Launch 文件）
### 读取者：`ros2 launch` 命令

Launch 文件本质是 Python 脚本，标签是 Python 对象。下方用"标签"的方式描述，实际对应 Python 类。

```
LaunchDescription                         ← 根对象，包含所有启动项
  │                                        谁读：ros2 launch 命令
  │
  ├── DeclareLaunchArgument               ← 声明可命令行覆盖的参数
  │   name='xxx'                          ← 参数名
  │   default_value='yyy'                 ← 默认值
  │   description='...'                   ← 描述文本
  │                                        谁读：ros2 launch（解析命令行参数）
  │
  ├── SetEnvironmentVariable              ← 设置环境变量
  │   name='LIBGL_ALWAYS_SOFTWARE'
  │   value='1'
  │                                        谁读：Shell / 操作系统
  │
  ├── IncludeLaunchDescription            ← 嵌套另一个 launch 文件
  │   PythonLaunchDescriptionSource(path) ← 被嵌套的 launch 路径
  │   launch_arguments={...}              ← 传给子 launch 的参数
  │                                        谁读：ros2 launch（递归解析）
  │
  ├── Node                                ← 启动一个 ROS 2 节点
  │   │                                    谁读：ros2 launch → ros2 run
  │   │
  │   ├── package='xxx'                  ← 包名
  │   ├── executable='xxx'                ← 可执行文件名
  │   ├── name='xxx'                      ← （可选）节点名（默认=executable）
  │   ├── namespace='xxx'                 ← （可选）命名空间
  │   ├── parameters=[{key: value}, ...]  ← 参数列表（dict 或 yaml 文件路径）
  │   ├── arguments=['-a', 'b', ...]      ← 命令行参数
  │   ├── remappings=[('from', 'to'), ...]← 话题/服务重映射
  │   ├── output='screen|log|both'       ← 输出方式
  │   ├── respawn=True|False              ← 节点挂掉后是否自动重启
  │   └── condition=IfCondition(...)      ← 条件启动
  │
  ├── ParameterValue                      ← 声明一个 ROS 参数值（给 Node 的 parameters 用）
  │   Command(['xacro ', ...])            ← 运行 shell 命令，输出作为参数值
  │   value_type=str                      ← 参数类型
  │
  ├── LaunchConfiguration('xxx')          ← 引用 launch 参数值（运行时求值）
  │   ← 用法：嵌入到 Node 参数或 arguments 中
  │
  ├── Command(['xacro ', ...])            ← 执行 shell 命令（如 xacro 转 urdf）
  │   ← 返回命令的标准输出
  │
  └── TimerAction                          ← 延迟启动
      period=5.0                           ← 延迟秒数
      actions=[Node(...), ...]             ← 延迟后执行的动作
```

---

## 七、各标签读取链路总结（谁在什么时候读什么）

```
阶段1：编译期
═══════════
colcon build
  └── CMakeLists.txt → 安装 urdf/xacro/launch/world 到 install 目录

阶段2：启动期（ros2 launch）
══════════════════════
ros2 launch robot_simulation gazebo_sim.launch.py
  │
  ├── SetEnvironmentVariable          ← Shell 环境变量
  ├── DeclareLaunchArgument           ← 命令行参数
  ├── Command('xacro fishbot.xacro')  ← xacro 工具展开 xacro→urdf（此时 xacro 标签消失）
  ├── ParameterValue                  ← 把 urdf 注入 robot_description 参数
  │
  ├── Node: robot_state_publisher     ← 读 robot_description，算 TF
  ├── Node: joint_state_publisher     ← 发 /joint_states
  ├── IncludeLaunch: gz_sim.launch    ← 启动 Gazebo Server + GUI
  │   └── Gazebo 读 world1.sdf       ← 加载世界（SDF 标签被解析）
  │
  ├── Node: ros_gz_sim create         ← 把 robot_description 发给 Gazebo
  │   └── Gazebo 读 <gazebo> 标签     ← 加载 DiffDrive 等插件（Gazebo 扩展标签）
  │   └── Gazebo 读 <ros2_control>    ← 加载硬件清单（ros2_control 标签，如果配了）
  │
  └── Node: ros_gz_bridge             ← 桥接 ROS ↔ Gazebo Transport 话题

阶段3：运行期
═══════════
  /cmd_vel (ROS) ──→ ros_gz_bridge ──→ Gazebo Transport ──→ DiffDrive 插件
                                                                 │
                                                           驱动关节转动
                                                                 │
  /odom + TF ←── ros_gz_bridge ←── Gazebo Transport ←── 插件读取关节位置推算里程计

  /tf (link间) ←── robot_state_publisher ← /joint_states + URDF 中的 <joint> 标签

  RViz2 ← 读 /robot_description（URDF 的 <link><joint> 标签）渲染模型
       ← 读 /tf 显示坐标系
```

---

## 八、快速查错表

| 现象 | 可能缺少的标签 |
|------|---------------|
| Gazebo 里看不到机器人 | URDF 缺少 `<collision>` 或 `<visual>` |
| 机器人满天飞/抖动 | `<inertial>` 参数不对，或 `<gazebo>` 没配摩擦 |
| `ros2 topic list` 没有 `/cmd_vel` | 没配 `<plugin>DiffDrive` 或没桥接 `ros_gz_bridge` |
| 机器人不动 | DiffDrive 的 `<topic>` 或 `ros_gz_bridge` 桥接配置不对 |
| rviz2 不显示模型 | `robot_state_publisher` 没拿到 `robot_description`，或 `<link>` 缺少 `<visual>` |
| 差速转向不对 | `<wheel_separation>` 或 `<wheel_radius>` 配错 |
| 控制器启动报 "找不到接口" | `<ros2_control>` 里的 `<command_interface>` 和控制器 yaml 要求的不匹配 |

---

> 文档位置：`auto_navigation/docs/tags_comprehensive_reference.md`
> 写作风格：暴躁但全面，适合被教程折磨后的知识梳理
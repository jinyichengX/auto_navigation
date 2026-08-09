// ===== 组件定义类型 =====
export type ComponentCategory = 'ros2_core' | 'gazebo' | 'bridge' | 'ros2_control' | 'hardware' | 'concept';
export type BelongsTo = 'ros2_dds' | 'gz_transport' | 'both' | 'real_hardware' | 'concept';
export type Transport = 'ros2_dds' | 'gz_transport';

export interface TopicRef {
  topic: string;
  type: string;
  direction: 'in' | 'out';
  transport: Transport;
}

export interface ComponentDef {
  id: string;
  name: string;
  category: ComponentCategory;
  package: string;
  executable?: string;
  description: string;
  subscribes: TopicRef[];
  publishes: TopicRef[];
  belongsTo: BelongsTo;
  color: string;
  borderColor: string;
}

// ===== 画布状态类型 =====
export interface CanvasNode {
  instanceId: string;
  defId: string;
  x: number;
  y: number;
  robotId?: string;
}

export type ConnectionType = 'cmd' | 'state' | 'tf' | 'bridge';
export type ConnectionTransport = 'ros2_dds' | 'gz_transport' | 'cross';

export interface Connection {
  id: string;
  fromNodeId: string;
  fromTopic: string;
  toNodeId: string;
  toTopic: string;
  topic: string;
  type: ConnectionType;
  transport: ConnectionTransport;
}

// ===== 步骤引导类型 =====
export interface GuideStep {
  step: number;
  title: string;
  description: string;
  addNodes: string[];
  removeNodes: string[];
  highlightConnections: string[];
}

// ===== 模式类型 =====
export type AppMode = 'simulation' | 'real_robot' | 'compare' | 'multi_robot';
export type GuideMode = 'guided' | 'free';

// ===== SVGCircle 连线端点 =====
export interface PortPosition {
  nodeId: string;
  topic: string;
  direction: 'in' | 'out';
}

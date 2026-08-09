import { create } from 'zustand';
import { CanvasNode, Connection, AppMode, GuideMode, ConnectionType, ConnectionTransport } from '../types';
import { componentDefs } from '../data/components';
import { simulationSteps, realRobotSteps, multiRobotSteps } from '../data/steps';

interface CanvasState {
  mode: AppMode;
  guideMode: GuideMode;
  currentStep: number;
  nodes: CanvasNode[];
  connections: Connection[];
  selectedNodeId: string | null;

  setMode: (mode: AppMode) => void;
  setGuideMode: (mode: GuideMode) => void;
  addNode: (defId: string, robotId?: string) => void;
  removeNode: (instanceId: string) => void;
  moveNode: (instanceId: string, x: number, y: number) => void;
  selectNode: (instanceId: string | null) => void;
  nextStep: () => void;
  prevStep: () => void;
  jumpToStep: (step: number) => void;
  recomputeConnections: () => void;
  addNodeWithPosition: (defId: string, x: number, y: number, robotId?: string) => string;
}

let nodeCounter = 0;
function nextId(): string {
  return `node_${++nodeCounter}`;
}

const LAYOUT_POSITIONS: Record<string, { x: number; y: number }> = {
  // ROS 2 DDS 侧 (左)
  xacro: { x: 60, y: 50 },
  robot_description: { x: 270, y: 50 },
  robot_state_publisher: { x: 60, y: 180 },
  joint_state_publisher: { x: 270, y: 180 },
  teleop: { x: 60, y: 380 },
  controller_manager: { x: 60, y: 500 },
  diff_drive_controller: { x: 60, y: 620 },

  // Gazebo Transport 侧 (右)
  gz_sim: { x: 750, y: 30 },
  gz_transport_layer: { x: 750, y: 140 },
  physics_engine: { x: 750, y: 240 },
  spawn_robot: { x: 950, y: 50 },
  diff_drive_plugin: { x: 950, y: 310 },
  gz_ros2_control_plugin: { x: 950, y: 420 },

  // 桥接 (中间)
  ros_gz_bridge: { x: 530, y: 310 },
  hardware_interface_gz: { x: 950, y: 500 },

  // 真实硬件侧
  hardware_interface_real: { x: 600, y: 500 },
  real_motor_driver: { x: 800, y: 450 },
  real_encoder: { x: 800, y: 560 },
  real_imu: { x: 800, y: 200 },
};

export const useStore = create<CanvasState>((set, get) => ({
  mode: 'simulation',
  guideMode: 'guided',
  currentStep: 0,
  nodes: [],
  connections: [],
  selectedNodeId: null,

  setMode: (mode) => {
    set({
      mode,
      currentStep: 0,
      nodes: [],
      connections: [],
      selectedNodeId: null,
    });
    // Recompute after setting mode
    setTimeout(() => get().recomputeConnections(), 0);
  },

  setGuideMode: (mode) => set({ guideMode: mode }),

  addNode: (defId, robotId) => {
    const pos = LAYOUT_POSITIONS[defId] || { x: Math.random() * 600 + 50, y: Math.random() * 500 + 50 };
    get().addNodeWithPosition(defId, pos.x, pos.y, robotId);
  },

  addNodeWithPosition: (defId, x, y, robotId) => {
    const instanceId = nextId();
    const def = componentDefs[defId];
    if (!def) return instanceId;

    const node: CanvasNode = {
      instanceId,
      defId,
      x,
      y,
      robotId,
    };

    set((s) => ({
      nodes: [...s.nodes, node],
    }));

    // Automatically recompute connections
    setTimeout(() => get().recomputeConnections(), 0);
    return instanceId;
  },

  removeNode: (instanceId) => {
    set((s) => ({
      nodes: s.nodes.filter((n) => n.instanceId !== instanceId),
      connections: s.connections.filter(
        (c) => c.fromNodeId !== instanceId && c.toNodeId !== instanceId
      ),
      selectedNodeId: s.selectedNodeId === instanceId ? null : s.selectedNodeId,
    }));
    setTimeout(() => get().recomputeConnections(), 0);
  },

  moveNode: (instanceId, x, y) => {
    set((s) => ({
      nodes: s.nodes.map((n) =>
        n.instanceId === instanceId ? { ...n, x, y } : n
      ),
    }));
  },

  selectNode: (instanceId) => {
    set({ selectedNodeId: instanceId });
  },

  nextStep: () => {
    const state = get();
    const { mode, currentStep } = state;
    let steps;
    if (mode === 'real_robot') steps = realRobotSteps;
    else if (mode === 'multi_robot') steps = multiRobotSteps;
    else steps = simulationSteps;

    const stepIdx = currentStep + 1;
    if (stepIdx >= steps.length) return;

    const step = steps[stepIdx];
    const removeIds = step.removeNodes || [];
    const existingDefIds = new Set(state.nodes.map(n => n.defId));

    // Remove nodes marked for removal
    const remainingNodes = state.nodes.filter(n => !removeIds.includes(n.defId));

    // Add new nodes (skip if already present)
    const newAddNodes: CanvasNode[] = [];
    for (const defId of step.addNodes) {
      if (!existingDefIds.has(defId)) {
        const pos = LAYOUT_POSITIONS[defId] || { x: 50, y: 300 };
        newAddNodes.push({ instanceId: nextId(), defId, x: pos.x, y: pos.y });
      }
    }

    const allNodes = [...remainingNodes, ...newAddNodes];

    set({
      currentStep: stepIdx,
      nodes: allNodes,
      connections: [],
    });

    setTimeout(() => get().recomputeConnections(), 0);
  },

  prevStep: () => {
    const state = get();
    if (state.currentStep <= 0) return;

    const prev = state.currentStep - 1;
    const { mode } = state;
    let steps;
    if (mode === 'real_robot') steps = realRobotSteps;
    else if (mode === 'multi_robot') steps = multiRobotSteps;
    else steps = simulationSteps;

    const currentStepData = steps[state.currentStep];

    // Re-add nodes that were removed in the current step
    const reAddDefIds = currentStepData.removeNodes || [];
    const existingDefIds = new Set(state.nodes.map(n => n.defId));
    const reAddNodes: CanvasNode[] = [];
    for (const defId of reAddDefIds) {
      if (!existingDefIds.has(defId)) {
        const pos = LAYOUT_POSITIONS[defId] || { x: 50, y: 300 };
        reAddNodes.push({ instanceId: nextId(), defId, x: pos.x, y: pos.y });
      }
    }

    // Remove nodes that were added in the current step (and not in reAdd)
    const removeDefIds = currentStepData.addNodes.filter(
      id => !reAddDefIds.includes(id)
    );
    const filteredNodes = [...state.nodes, ...reAddNodes].filter(
      n => !removeDefIds.includes(n.defId)
    );

    set({
      currentStep: prev,
      nodes: filteredNodes,
      connections: [],
    });

    setTimeout(() => get().recomputeConnections(), 0);
  },

  jumpToStep: (step) => {
    const { mode } = get();
    let steps;
    if (mode === 'real_robot') steps = realRobotSteps;
    else if (mode === 'multi_robot') steps = multiRobotSteps;
    else steps = simulationSteps;

    if (step < 0 || step >= steps.length) return;

    // Rebuild from step 0 to target: track set of currently present defIds
    let currentDefIds = new Set<string>();
    const nodeMap = new Map<string, CanvasNode>();

    for (let i = 0; i <= step; i++) {
      const s = steps[i];
      // Remove
      for (const rid of (s.removeNodes || [])) {
        currentDefIds.delete(rid);
      }
      // Add
      for (const defId of s.addNodes) {
        if (!currentDefIds.has(defId)) {
          currentDefIds.add(defId);
          const pos = LAYOUT_POSITIONS[defId] || { x: 50, y: 300 };
          nodeMap.set(defId, { instanceId: nextId(), defId, x: pos.x, y: pos.y });
        }
      }
    }

    const finalNodes = Array.from(nodeMap.values()).filter(n => currentDefIds.has(n.defId));

    set({
      currentStep: step,
      nodes: finalNodes,
      connections: [],
    });

    setTimeout(() => get().recomputeConnections(), 0);
  },

  recomputeConnections: () => {
    const { nodes } = get();
    const newConnections: Connection[] = [];

    for (const fromNode of nodes) {
      const fromDef = componentDefs[fromNode.defId];
      if (!fromDef) continue;

      for (const pub of fromDef.publishes) {
        for (const toNode of nodes) {
          if (fromNode.instanceId === toNode.instanceId) continue;
          const toDef = componentDefs[toNode.defId];
          if (!toDef) continue;

          for (const sub of toDef.subscribes) {
            if (pub.topic === sub.topic) {
              // Determine connection type
              let ctype: ConnectionType = 'state';
              if (pub.topic === '/cmd_vel') ctype = 'cmd';
              else if (pub.topic === '/tf' || pub.topic === '/tf_static') ctype = 'tf';

              // Determine transport
              let ctransport: ConnectionTransport = 'ros2_dds';
              if (pub.transport !== sub.transport) {
                ctransport = 'cross';
              } else if (pub.transport === 'gz_transport') {
                ctransport = 'gz_transport';
              }

              // Check if bridge node is involved
              if (fromDef.id === 'ros_gz_bridge' || toDef.id === 'ros_gz_bridge') {
                ctype = 'bridge';
              }

              // Avoid duplicates
              const exists = newConnections.find(
                c => c.fromNodeId === fromNode.instanceId &&
                     c.toNodeId === toNode.instanceId &&
                     c.topic === pub.topic
              );
              if (!exists) {
                newConnections.push({
                  id: `conn_${fromNode.instanceId}_${toNode.instanceId}_${pub.topic.replace(/\//g, '_')}`,
                  fromNodeId: fromNode.instanceId,
                  fromTopic: pub.topic,
                  toNodeId: toNode.instanceId,
                  toTopic: sub.topic,
                  topic: pub.topic,
                  type: ctype,
                  transport: ctransport,
                });
              }
            }
          }
        }
      }
    }

    set({ connections: newConnections });
  },
}));

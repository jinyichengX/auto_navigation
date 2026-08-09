import { useStore } from '../store/useStore';
import type { Connection as ConnectionType } from '../types';
import { getNodeEdge } from './CanvasNode';

interface Props {
  conn: ConnectionType;
}

const colorMap: Record<string, string> = {
  ros2_dds: '#34d399',
  gz_transport: '#00d4ff',
  cross: '#ff6b35',
};

const markerMap: Record<string, string> = {
  ros2_dds: 'url(#arrow-ros2)',
  gz_transport: 'url(#arrow-gz)',
  cross: 'url(#arrow-cross)',
};

export default function ConnectionLine({ conn }: Props) {
  const { nodes } = useStore();
  const fromNode = nodes.find((n) => n.instanceId === conn.fromNodeId);
  const toNode = nodes.find((n) => n.instanceId === conn.toNodeId);

  if (!fromNode || !toNode) return null;

  const from = getNodeEdge(fromNode, 'right');
  const to = getNodeEdge(toNode, 'left');

  const dx = to.x - from.x;
  const isCrossTransport = conn.transport === 'cross';

  let path: string;
  if (dx > 30) {
    const cpOffset = Math.min(Math.abs(dx) * 0.4, 100);
    path = `M ${from.x} ${from.y} C ${from.x + cpOffset} ${from.y}, ${to.x - cpOffset} ${to.y}, ${to.x} ${to.y}`;
  } else {
    path = `M ${from.x} ${from.y} L ${to.x} ${to.y}`;
  }

  const strokeColor = colorMap[conn.transport] || '#94a3b8';
  const strokeWidth = isCrossTransport ? 2 : 1.5;
  const strokeDash = isCrossTransport ? '6,3' : 'none';

  return (
    <g>
      <path
        d={path}
        fill="none"
        stroke={strokeColor}
        strokeWidth={strokeWidth}
        strokeDasharray={strokeDash}
        markerEnd={markerMap[conn.transport] || 'url(#arrow-default)'}
        className={isCrossTransport ? 'animate-pulse-bridge' : ''}
        opacity={conn.transport === 'cross' ? 0.8 : 0.5}
      />
      {/* Topic label */}
      <text
        x={(from.x + to.x) / 2}
        y={(from.y + to.y) / 2 - 8}
        textAnchor="middle"
        fill={strokeColor}
        fontSize="9"
        fontFamily="JetBrains Mono, monospace"
        opacity={0.8}
      >
        {conn.topic}
      </text>
    </g>
  );
}

import { useStore } from '../store/useStore';
import { componentDefs } from '../data/components';
import type { CanvasNode as CanvasNodeType } from '../types';
import { useRef, useCallback, useState } from 'react';

interface Props {
  node: CanvasNodeType;
  scale: number;
}

export default function CanvasNode({ node, scale }: Props) {
  const { selectNode, selectedNodeId, moveNode } = useStore();
  const def = componentDefs[node.defId];
  const isSelected = selectedNodeId === node.instanceId;
  const [isDragging, setIsDragging] = useState(false);
  const [showContext, setShowContext] = useState(false);
  const dragRef = useRef<{ startX: number; startY: number; nodeX: number; nodeY: number } | null>(null);

  const handleContextMenu = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setShowContext(true);
    selectNode(node.instanceId);

    const closeMenu = () => {
      setShowContext(false);
      document.removeEventListener('click', closeMenu);
    };
    setTimeout(() => document.addEventListener('click', closeMenu), 0);
  }, [node.instanceId, selectNode]);

  const handleDelete = useCallback((e: React.MouseEvent) => {
    e.stopPropagation();
    useStore.getState().removeNode(node.instanceId);
    setShowContext(false);
  }, [node.instanceId]);

  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    e.stopPropagation();
    selectNode(node.instanceId);
    setIsDragging(true);
    dragRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      nodeX: node.x,
      nodeY: node.y,
    };

    const handleMouseMove = (ev: MouseEvent) => {
      if (!dragRef.current) return;
      // Screen delta / scale = canvas delta
      const dx = (ev.clientX - dragRef.current.startX) / scale;
      const dy = (ev.clientY - dragRef.current.startY) / scale;
      moveNode(node.instanceId, dragRef.current.nodeX + dx, dragRef.current.nodeY + dy);
    };

    const handleMouseUp = () => {
      setIsDragging(false);
      dragRef.current = null;
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
  }, [node.instanceId, node.x, node.y, scale, selectNode, moveNode]);

  if (!def) return null;

  const width = def.id === 'robot_description' || def.id === 'gz_transport_layer' ? 150 : 170;
  const height = def.id === 'ros_gz_bridge' || def.id === 'gz_ros2_control_plugin' || def.id === 'hardware_interface_real' ? 75 : 65;

  const isConcept = def.category === 'concept';

  return (
    <div
      className={`absolute rounded-lg border cursor-grab active:cursor-grabbing
        transition-shadow duration-200 select-none
        ${isSelected ? 'ring-2 ring-accent-cyan shadow-lg shadow-accent-cyan/20 z-10' : 'z-0'}
        ${isDragging ? 'opacity-90 shadow-xl' : ''}
        ${def.borderColor} ${def.color}
        ${isConcept ? 'border-dashed' : 'border-solid'}`}
      style={{
        left: node.x,
        top: node.y,
        width,
        minHeight: height,
      }}
      onMouseDown={handleMouseDown}
      onContextMenu={handleContextMenu}
      onClick={(e) => e.stopPropagation()}
    >
      {/* Context menu */}
      {showContext && (
        <div
          className="absolute top-full left-0 mt-1 bg-bg-card border border-border-subtle rounded shadow-xl z-50 py-1 min-w-[100px]"
          onClick={(e) => e.stopPropagation()}
        >
          <button
            onClick={handleDelete}
            className="w-full text-left px-3 py-1.5 text-xs text-red-400 hover:bg-red-900/30 transition-colors flex items-center gap-2"
          >
            <span>✕</span> 删除此节点
          </button>
        </div>
      )}
      <div className="p-2">
        <div className="text-xs font-medium text-gray-100 whitespace-pre-line leading-tight"
          style={{ fontSize: def.id === 'gz_transport_layer' ? '11px' : '12px' }}>
          {def.name}
        </div>
        {def.package && def.package !== '-' && (
          <div className="text-[10px] text-gray-500 font-mono mt-0.5 truncate">
            {def.package}
          </div>
        )}
      </div>
    </div>
  );
}

// Export port position calculator
export function getNodeCenter(node: CanvasNodeType): { x: number; y: number } {
  const def = componentDefs[node.defId];
  const width = (def?.id === 'robot_description' || def?.id === 'gz_transport_layer') ? 150 : 170;
  const height = (def?.id === 'ros_gz_bridge' || def?.id === 'gz_ros2_control_plugin' || def?.id === 'hardware_interface_real') ? 75 : 65;
  return {
    x: node.x + width / 2,
    y: node.y + height / 2,
  };
}

export function getNodeEdge(node: CanvasNodeType, direction: 'left' | 'right' | 'top' | 'bottom'): { x: number; y: number } {
  const center = getNodeCenter(node);
  const def = componentDefs[node.defId];
  const width = (def?.id === 'robot_description' || def?.id === 'gz_transport_layer') ? 150 : 170;
  const height = (def?.id === 'ros_gz_bridge' || def?.id === 'gz_ros2_control_plugin' || def?.id === 'hardware_interface_real') ? 75 : 65;

  switch (direction) {
    case 'right': return { x: node.x + width, y: center.y };
    case 'left': return { x: node.x, y: center.y };
    case 'bottom': return { x: center.x, y: node.y + height };
    case 'top': return { x: center.x, y: node.y };
  }
}

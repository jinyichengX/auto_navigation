import { useRef, useState, useCallback } from 'react';
import { useStore } from '../store/useStore';
import { componentDefs } from '../data/components';
import CanvasNodeComponent from './CanvasNode';
import ConnectionLineComponent from './ConnectionLine';

export default function Canvas() {
  const { nodes, connections, addNodeWithPosition, selectNode, guideMode } = useStore();
  const canvasRef = useRef<HTMLDivElement>(null);
  const [scale, setScale] = useState(0.9);
  const [offset, setOffset] = useState({ x: 0, y: 30 });
  const [isPanning, setIsPanning] = useState(false);
  const panRef = useRef({ startX: 0, startY: 0, startOffsetX: 0, startOffsetY: 0 });

  // Zoom
  const handleWheel = useCallback((e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? -0.05 : 0.05;
    setScale((s) => Math.max(0.2, Math.min(2, s + delta)));
  }, []);

  // Pan - use functional setOffset to avoid stale closure
  const handleMouseDownCanvas = useCallback((e: React.MouseEvent) => {
    if ((e.target as HTMLElement).closest('[data-node]')) return;
    setIsPanning(true);
    panRef.current = {
      startX: e.clientX,
      startY: e.clientY,
      startOffsetX: 0, // filled below
      startOffsetY: 0,
    };
    // Capture current offset via getState-like approach
    selectNode(null);

    const handleMouseMove = (ev: MouseEvent) => {
      const dx = ev.clientX - panRef.current.startX;
      const dy = ev.clientY - panRef.current.startY;
      setOffset((prev) => ({
        x: panRef.current.startOffsetX + dx,
        y: panRef.current.startOffsetY + dy,
      }));
    };

    const handleMouseUp = () => {
      setIsPanning(false);
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };

    // Initialize offset snapshot after state settles
    const capture = (s: { x: number; y: number }) => {
      panRef.current.startOffsetX = s.x;
      panRef.current.startOffsetY = s.y;
    };
    setOffset((prev) => {
      capture(prev);
      return prev;
    });

    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
  }, [selectNode]);

  // Drop from sidebar
  const scaleRef = useRef(scale);
  const offsetRef = useRef(offset);
  scaleRef.current = scale;
  offsetRef.current = offset;

  const handleDrop = useCallback((e: React.DragEvent) => {
    e.preventDefault();
    const defId = e.dataTransfer.getData('text/plain');
    if (!defId || !componentDefs[defId]) return;

    const rect = canvasRef.current?.getBoundingClientRect();
    if (!rect) return;

    const s = scaleRef.current;
    const off = offsetRef.current;
    const x = (e.clientX - rect.left - off.x) / s;
    const y = (e.clientY - rect.top - off.y) / s;

    addNodeWithPosition(defId, x, y);
  }, [addNodeWithPosition]);

  const handleDragOver = useCallback((e: React.DragEvent) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = 'copy';
  }, []);

  // Determine domain boundary line X position (between ROS 2 DDS and Gazebo Transport)
  const boundaryX = 530;

  // Count nodes in each domain
  const ros2Nodes = nodes.filter(n => {
    const def = componentDefs[n.defId];
    return def && (def.belongsTo === 'ros2_dds' || def.id === 'robot_description');
  });
  const gzNodes = nodes.filter(n => {
    const def = componentDefs[n.defId];
    return def && (def.belongsTo === 'gz_transport' || def.belongsTo === 'both');
  });

  const showBoundary = ros2Nodes.length > 0 || gzNodes.length > 0;

  return (
    <div
      ref={canvasRef}
      className="flex-1 relative overflow-hidden"
      style={{
        background: 'radial-gradient(circle at 50% 50%, #141e2b 0%, #0f1419 100%)',
        backgroundImage: `
          linear-gradient(rgba(45, 58, 74, 0.15) 1px, transparent 1px),
          linear-gradient(90deg, rgba(45, 58, 74, 0.15) 1px, transparent 1px)
        `,
        backgroundSize: '30px 30px',
        cursor: isPanning ? 'grabbing' : 'grab',
      }}
      onWheel={handleWheel}
      onMouseDown={handleMouseDownCanvas}
      onDrop={handleDrop}
      onDragOver={handleDragOver}
    >
      {/* Transformed layer */}
      <div
        style={{
          transform: `scale(${scale}) translate(${offset.x / scale}px, ${offset.y / scale}px)`,
          transformOrigin: '0 0',
          width: '100%',
          height: '100%',
          position: 'absolute',
          top: 0,
          left: 0,
        }}
      >
        {/* Domain boundary */}
        {showBoundary && (
          <div
            className="absolute top-0 bottom-0 border-l-2 border-dashed border-gray-600/40 flex items-center"
            style={{ left: boundaryX, height: '100vh' }}
          >
            <div className="absolute -left-[70px] top-4 text-[10px] text-emerald-500/60 font-mono uppercase tracking-widest">
              ROS 2 DDS
            </div>
            <div className="absolute left-4 top-4 text-[10px] text-cyan-500/60 font-mono uppercase tracking-widest">
              Gazebo Transport
            </div>
          </div>
        )}

        {/* SVG connections layer */}
        <svg
          className="absolute top-0 left-0 pointer-events-none"
          style={{ width: '100%', height: '100%', minWidth: '2000px', minHeight: '1500px' }}
        >
          <defs>
            <marker id="arrow-ros2" viewBox="0 0 10 8" refX="10" refY="4" markerWidth="8" markerHeight="6" orient="auto">
              <path d="M 0 0 L 10 4 L 0 8 z" fill="#34d399" />
            </marker>
            <marker id="arrow-gz" viewBox="0 0 10 8" refX="10" refY="4" markerWidth="8" markerHeight="6" orient="auto">
              <path d="M 0 0 L 10 4 L 0 8 z" fill="#00d4ff" />
            </marker>
            <marker id="arrow-cross" viewBox="0 0 10 8" refX="10" refY="4" markerWidth="8" markerHeight="6" orient="auto">
              <path d="M 0 0 L 10 4 L 0 8 z" fill="#ff6b35" />
            </marker>
            <marker id="arrow-default" viewBox="0 0 10 8" refX="10" refY="4" markerWidth="8" markerHeight="6" orient="auto">
              <path d="M 0 0 L 10 4 L 0 8 z" fill="#94a3b8" />
            </marker>
          </defs>
          {connections.map((conn) => (
            <ConnectionLineComponent key={conn.id} conn={conn} />
          ))}
        </svg>

        {/* Nodes */}
        {nodes.map((node) => (
          <div key={node.instanceId} data-node="true">
            <CanvasNodeComponent node={node} scale={scale} />
          </div>
        ))}

        {/* Empty state */}
        {nodes.length === 0 && (
          <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-center">
            <div className="text-gray-600 text-sm font-mono mb-2">
              {guideMode === 'guided'
                ? '点击「下一步」开始逐步引导'
                : '从左侧拖拽组件到此处'}
            </div>
            <div className="text-gray-700 text-xs">
              滚轮缩放 · 按住空白处拖拽平移
            </div>
          </div>
        )}
      </div>

      {/* Zoom controls */}
      <div className="absolute bottom-3 right-3 flex gap-1 z-10">
        <button
          onClick={() => setScale((s) => Math.min(2, s + 0.1))}
          className="w-7 h-7 rounded bg-bg-card border border-border-subtle text-gray-400 hover:text-gray-200 text-sm flex items-center justify-center"
        >
          +
        </button>
        <button
          onClick={() => setScale((s) => Math.max(0.2, s - 0.1))}
          className="w-7 h-7 rounded bg-bg-card border border-border-subtle text-gray-400 hover:text-gray-200 text-sm flex items-center justify-center"
        >
          -
        </button>
        <div className="text-[10px] text-gray-500 flex items-center px-1 font-mono">
          {Math.round(scale * 100)}%
        </div>
      </div>

      {/* Legend */}
      <div className="absolute top-3 right-3 bg-bg-card/90 border border-border-subtle rounded p-2 text-[10px] z-10 font-mono space-y-1">
        <div className="flex items-center gap-1.5">
          <div className="w-3 h-0.5 rounded" style={{ backgroundColor: '#34d399' }} />
          <span className="text-gray-400">ROS 2 DDS 通信</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-3 h-0.5 rounded" style={{ backgroundColor: '#00d4ff' }} />
          <span className="text-gray-400">Gazebo Transport 通信</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-3 h-0.5 rounded" style={{ backgroundColor: '#ff6b35', border: 'none' }} />
          <span className="text-gray-400">跨边界桥接 (bridge)</span>
        </div>
      </div>
    </div>
  );
}

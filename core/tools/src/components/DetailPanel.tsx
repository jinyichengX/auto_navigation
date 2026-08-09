import { useStore } from '../store/useStore';
import { componentDefs } from '../data/components';

export default function DetailPanel() {
  const { selectedNodeId, nodes } = useStore();

  if (!selectedNodeId) {
    return (
      <aside className="w-72 bg-bg-card border-l border-border-subtle shrink-0 p-4 overflow-y-auto">
        <div className="text-xs text-gray-500 uppercase tracking-wider mb-4 font-mono">
          组件详情
        </div>
        <div className="text-xs text-gray-500 leading-relaxed">
          点击画布中的组件查看详细信息。
        </div>
      </aside>
    );
  }

  const node = nodes.find((n) => n.instanceId === selectedNodeId);
  if (!node) return null;
  const def = componentDefs[node.defId];
  if (!def) return null;

  return (
    <aside className="w-72 bg-bg-card border-l border-border-subtle shrink-0 overflow-y-auto animate-slide-in">
      <div className="p-4">
        <div className="flex items-center justify-between mb-3">
          <span className="text-xs text-gray-500 uppercase tracking-wider font-mono">组件详情</span>
          <button
            onClick={() => useStore.getState().selectNode(null)}
            className="text-gray-500 hover:text-gray-300 text-xs"
          >
            ✕
          </button>
        </div>

        <div className={`rounded-lg border ${def.borderColor} ${def.color} p-3 mb-4`}>
          <div className="text-sm font-bold text-gray-100 whitespace-pre-line">{def.name}</div>
          <div className="text-xs text-gray-400 mt-1 font-mono">{def.package}</div>
        </div>

        {def.executable && (
          <div className="mb-3">
            <div className="text-xs text-accent-cyan font-mono mb-1">可执行文件</div>
            <code className="text-xs bg-black/30 rounded px-2 py-1 block text-gray-300">{def.executable}</code>
          </div>
        )}

        {def.subscribes.length > 0 && (
          <div className="mb-3">
            <div className="text-xs text-accent-green font-mono mb-1">订阅 (Subscribe)</div>
            <div className="space-y-1">
              {def.subscribes.map((s, i) => (
                <div key={i} className="text-xs bg-black/30 rounded px-2 py-1">
                  <span className="text-gray-300 font-mono">{s.topic}</span>
                  <span className="text-gray-500 ml-1">({s.type})</span>
                  <span className="text-gray-600 ml-1 text-[10px]">[{s.transport}]</span>
                </div>
              ))}
            </div>
          </div>
        )}

        {def.publishes.length > 0 && (
          <div className="mb-3">
            <div className="text-xs text-accent-orange font-mono mb-1">发布 (Publish)</div>
            <div className="space-y-1">
              {def.publishes.map((p, i) => (
                <div key={i} className="text-xs bg-black/30 rounded px-2 py-1">
                  <span className="text-gray-300 font-mono">{p.topic}</span>
                  <span className="text-gray-500 ml-1">({p.type})</span>
                  <span className="text-gray-600 ml-1 text-[10px]">[{p.transport}]</span>
                </div>
              ))}
            </div>
          </div>
        )}

        <div className="mb-3">
          <div className="text-xs text-accent-purple font-mono mb-1">所属域</div>
          <span className={`text-xs px-2 py-0.5 rounded ${
            def.belongsTo === 'ros2_dds' ? 'bg-emerald-900/40 text-emerald-400' :
            def.belongsTo === 'gz_transport' ? 'bg-cyan-900/40 text-cyan-400' :
            def.belongsTo === 'both' ? 'bg-orange-900/40 text-orange-400' :
            def.belongsTo === 'real_hardware' ? 'bg-red-900/40 text-red-400' :
            'bg-slate-700/40 text-slate-400'
          }`}>
            {def.belongsTo === 'ros2_dds' ? 'ROS 2 DDS' :
             def.belongsTo === 'gz_transport' ? 'Gazebo Transport' :
             def.belongsTo === 'both' ? '跨边界' :
             def.belongsTo === 'real_hardware' ? '真实硬件' : '概念'}
          </span>
        </div>

        <div>
          <div className="text-xs text-gray-400 font-mono mb-1">详细解释</div>
          <div className="text-xs text-gray-300 leading-relaxed whitespace-pre-line">
            {def.description}
          </div>
        </div>
      </div>
    </aside>
  );
}

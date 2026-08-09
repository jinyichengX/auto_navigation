import { useStore } from '../store/useStore';
import { componentDefs, categoryInfo } from '../data/components';
import type { ComponentCategory } from '../types';

export default function Sidebar() {
  const { guideMode } = useStore();
  const categories = ['ros2_core', 'gazebo', 'bridge', 'ros2_control', 'hardware', 'concept'] as ComponentCategory[];

  const groupedComponents: Record<string, string[]> = {};
  for (const [id, def] of Object.entries(componentDefs)) {
    if (!groupedComponents[def.category]) groupedComponents[def.category] = [];
    groupedComponents[def.category].push(id);
  }

  if (guideMode === 'guided') {
    return (
      <aside className="w-56 bg-bg-card border-r border-border-subtle shrink-0 overflow-y-auto p-3">
        <div className="text-xs text-gray-500 uppercase tracking-wider mb-3 font-mono">
          逐步引导模式
        </div>
        <div className="text-xs text-gray-400 leading-relaxed">
          组件将按照预设顺序自动添加到画布中。
          <br /><br />
          使用底部的「下一步」按钮推进。
          <br /><br />
          你也可以切换到「自由探索」模式自行拖拽组件。
        </div>
      </aside>
    );
  }

  const handleDragStart = (e: React.DragEvent, defId: string) => {
    e.dataTransfer.setData('text/plain', defId);
    e.dataTransfer.effectAllowed = 'copy';
  };

  return (
    <aside className="w-56 bg-bg-card border-r border-border-subtle shrink-0 overflow-y-auto">
      <div className="p-3">
        <div className="text-xs text-gray-500 uppercase tracking-wider mb-3 font-mono">
          组件库
        </div>
        <div className="text-xs text-gray-500 mb-3">
          拖拽到画布中
        </div>
        {categories.map((cat) => {
          const info = categoryInfo[cat];
          const items = groupedComponents[cat] || [];
          if (items.length === 0) return null;

          return (
            <div key={cat} className="mb-3">
              <div className="flex items-center gap-1.5 mb-2">
                <span style={{ color: info.color }} className="text-xs">{info.icon}</span>
                <span className="text-xs font-medium" style={{ color: info.color }}>{info.label}</span>
              </div>
              <div className="space-y-1">
                {items.map((defId) => {
                  const def = componentDefs[defId];
                  return (
                    <div
                      key={defId}
                      draggable
                      onDragStart={(e) => handleDragStart(e, defId)}
                      className={`px-2.5 py-1.5 rounded text-xs cursor-grab active:cursor-grabbing
                        hover:bg-white/5 transition-colors border ${def.borderColor} ${def.color}
                        truncate`}
                      title={def.description}
                    >
                      {def.name}
                    </div>
                  );
                })}
              </div>
            </div>
          );
        })}
      </div>
    </aside>
  );
}

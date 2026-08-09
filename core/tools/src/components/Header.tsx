import { useStore } from '../store/useStore';

export default function Header() {
  const { mode, setMode, guideMode, setGuideMode } = useStore();

  const modes: { id: 'simulation' | 'real_robot' | 'compare' | 'multi_robot'; label: string }[] = [
    { id: 'simulation', label: '仿真模式' },
    { id: 'real_robot', label: '真机模式' },
    { id: 'compare', label: '仿真 vs 真机' },
    { id: 'multi_robot', label: '多机器人' },
  ];

  return (
    <header className="h-12 bg-bg-card border-b border-border-subtle flex items-center px-4 shrink-0 z-20">
      <div className="flex items-center gap-2 mr-6">
        <span className="text-accent-cyan font-mono text-sm font-bold tracking-wide">ROS2×GZ</span>
        <span className="text-gray-500 text-xs">架构可视化</span>
      </div>

      <div className="flex gap-1">
        {modes.map((m) => (
          <button
            key={m.id}
            onClick={() => setMode(m.id)}
            className={`px-3 py-1 rounded text-xs font-medium transition-all duration-200 ${
              mode === m.id
                ? 'bg-accent-cyan/20 text-accent-cyan border border-accent-cyan/40'
                : 'text-gray-400 hover:text-gray-200 border border-transparent hover:border-border-subtle'
            }`}
          >
            {m.label}
          </button>
        ))}
      </div>

      <div className="ml-auto flex items-center gap-2">
        <span className="text-gray-500 text-xs">模式：</span>
        <button
          onClick={() => setGuideMode('guided')}
          className={`px-3 py-1 rounded text-xs font-medium transition-all ${
            guideMode === 'guided'
              ? 'bg-accent-purple/20 text-accent-purple border border-accent-purple/40'
              : 'text-gray-400 hover:text-gray-200 border border-transparent hover:border-border-subtle'
          }`}
        >
          逐步引导
        </button>
        <button
          onClick={() => setGuideMode('free')}
          className={`px-3 py-1 rounded text-xs font-medium transition-all ${
            guideMode === 'free'
              ? 'bg-accent-purple/20 text-accent-purple border border-accent-purple/40'
              : 'text-gray-400 hover:text-gray-200 border border-transparent hover:border-border-subtle'
          }`}
        >
          自由探索
        </button>
      </div>
    </header>
  );
}

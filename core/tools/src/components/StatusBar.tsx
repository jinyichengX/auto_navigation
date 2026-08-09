import { useStore } from '../store/useStore';

export default function StatusBar() {
  const { mode, guideMode, currentStep, nodes, connections } = useStore();

  const modeLabel = {
    simulation: '仿真模式',
    real_robot: '真机模式',
    compare: '仿真 vs 真机对比',
    multi_robot: '多机器人模式',
  }[mode];

  return (
    <footer className="h-7 bg-bg-card border-t border-border-subtle flex items-center px-4 text-[11px] text-gray-500 font-mono shrink-0 z-20">
      <span className="mr-4">
        模式: <span className="text-accent-cyan">{modeLabel}</span>
      </span>
      <span className="mr-4">
        引导: <span className={guideMode === 'guided' ? 'text-accent-purple' : 'text-gray-400'}>
          {guideMode === 'guided' ? '逐步引导' : '自由探索'}
        </span>
      </span>
      {guideMode === 'guided' && (
        <span className="mr-4">
          步骤: <span className="text-gray-300">{currentStep}</span>
        </span>
      )}
      <span className="mr-4">
        节点: <span className="text-gray-300">{nodes.length}</span>
      </span>
      <span className="mr-4">
        连线: <span className="text-gray-300">{connections.length}</span>
      </span>
      <span className="ml-auto text-gray-600">
        滚轮缩放 · 拖拽节点移动 · 点击查看详情 · 拖拽侧边栏组件到画布
      </span>
    </footer>
  );
}

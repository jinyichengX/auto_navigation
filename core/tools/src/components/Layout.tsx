import { useStore } from '../store/useStore';
import Header from './Header';
import Sidebar from './Sidebar';
import Canvas from './Canvas';
import DetailPanel from './DetailPanel';
import StepGuide from './StepGuide';
import StatusBar from './StatusBar';
import { useEffect } from 'react';

export default function Layout() {
  const { mode, guideMode, nextStep } = useStore();

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (guideMode === 'guided') {
        if (e.key === 'ArrowRight' || e.key === ' ') {
          e.preventDefault();
          nextStep();
        }
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [guideMode, nextStep]);

  if (mode === 'compare') {
    return (
      <div className="h-screen flex flex-col bg-bg-dark">
        <Header />
        <div className="flex-1 flex overflow-hidden">
          <Sidebar />
          <div className="flex-1 relative">
            <Canvas />
            {guideMode === 'guided' && <StepGuide />}
          </div>
          <div className="w-80 bg-bg-card border-l border-border-subtle overflow-y-auto p-4 shrink-0">
            <div className="text-xs text-gray-500 uppercase tracking-wider mb-4 font-mono">
              仿真 vs 真机对比
            </div>
            <div className="space-y-4 text-xs text-gray-300 leading-relaxed">
              <div>
                <div className="text-accent-cyan font-mono mb-1">仿真模式 (Gazebo)</div>
                <ul className="list-disc list-inside space-y-1 text-gray-400">
                  <li>硬件接口: gazebo_ros2_control/GazeboSystem</li>
                  <li>物理层: Gazebo 物理引擎 (ODE/Bullet/DART)</li>
                  <li>跨域通信需要 ros_gz_bridge</li>
                  <li>世界/模型可通过 .sdf/.urdf 随意修改</li>
                </ul>
              </div>
              <div>
                <div className="text-accent-orange font-mono mb-1">真机模式 (无 Gazebo)</div>
                <ul className="list-disc list-inside space-y-1 text-gray-400">
                  <li>硬件接口: 自定义实现 (如 DiffBotSystemHardware)</li>
                  <li>物理层: 真实世界</li>
                  <li>全部 ROS 2 DDS 通信，无需 bridge</li>
                  <li>需要真实的电机驱动板、编码器、IMU</li>
                </ul>
              </div>
              <div className="pt-2 border-t border-border-subtle">
                <div className="text-accent-green font-mono mb-1">两者完全相同</div>
                <ul className="list-disc list-inside space-y-1 text-gray-400">
                  <li>diff_drive_controller 代码</li>
                  <li>controller_manager 配置</li>
                  <li>URDF 机器人描述</li>
                  <li>robot_state_publisher</li>
                  <li>/cmd_vel, /odom, /tf 话题接口</li>
                </ul>
              </div>
              <div className="pt-2 border-t border-border-subtle">
                <div className="text-accent-purple font-mono mb-1">核心价值</div>
                <p className="text-gray-400">
                  ros2_control 的精髓：上层 controller 代码在仿真和真机之间完全复用。
                  切换环境时只换硬件接口实现，不需要改任何 controller 代码。
                </p>
              </div>
            </div>
          </div>
        </div>
        <StatusBar />
      </div>
    );
  }

  return (
    <div className="h-screen flex flex-col bg-bg-dark">
      <Header />
      <div className="flex-1 flex overflow-hidden">
        <Sidebar />
        <div className="flex-1 relative">
          <Canvas />
          {guideMode === 'guided' && <StepGuide />}
        </div>
        <DetailPanel />
      </div>
      <StatusBar />
    </div>
  );
}

import { useStore } from '../store/useStore';
import { simulationSteps, realRobotSteps, multiRobotSteps } from '../data/steps';
import { useEffect, useRef } from 'react';

export default function StepGuide() {
  const { mode, guideMode, currentStep, nodes, nextStep, prevStep, jumpToStep } = useStore();
  const hasInitialized = useRef(false);

  // Auto-start: if in guided mode and canvas is empty, jump to step 0
  useEffect(() => {
    if (guideMode === 'guided' && nodes.length === 0 && !hasInitialized.current) {
      hasInitialized.current = true;
      jumpToStep(0);
    }
  }, [guideMode, nodes.length, jumpToStep]);

  // Reset when mode changes
  useEffect(() => {
    hasInitialized.current = false;
  }, [mode]);

  if (guideMode !== 'guided') return null;

  let steps;
  if (mode === 'real_robot') steps = realRobotSteps;
  else if (mode === 'multi_robot') steps = multiRobotSteps;
  else if (mode === 'compare') steps = simulationSteps; // compare uses sim steps
  else steps = simulationSteps;

  if (currentStep < 0 || currentStep >= steps.length) return null;

  const step = steps[currentStep];
  const isFirst = currentStep === 0;
  const isLast = currentStep === steps.length - 1;

  return (
    <div className="absolute bottom-10 left-1/2 -translate-x-1/2 z-30 animate-fade-in">
      <div className="bg-bg-card border border-border-subtle rounded-lg shadow-2xl shadow-black/50 max-w-lg">
        <div className="p-4">
          {/* Progress bar */}
          <div className="flex items-center gap-2 mb-3">
            <div className="flex-1 h-1 bg-bg-dark rounded-full overflow-hidden">
              <div
                className="h-full bg-accent-cyan rounded-full transition-all duration-500"
                style={{ width: `${((currentStep + 1) / steps.length) * 100}%` }}
              />
            </div>
            <span className="text-xs text-gray-500 font-mono">
              {currentStep + 1}/{steps.length}
            </span>
          </div>

          {/* Step title */}
          <h3 className="text-sm font-bold text-gray-100 mb-2">
            步骤 {step.step}：{step.title}
          </h3>

          {/* Step description */}
          <p className="text-xs text-gray-300 leading-relaxed whitespace-pre-line max-h-36 overflow-y-auto">
            {step.description}
          </p>

          {/* Navigation */}
          <div className="flex items-center justify-between mt-3 pt-3 border-t border-border-subtle">
            <div className="flex gap-1">
              <button
                onClick={prevStep}
                disabled={isFirst}
                className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                  isFirst
                    ? 'text-gray-600 cursor-not-allowed'
                    : 'text-gray-300 hover:text-gray-100 hover:bg-white/5 border border-border-subtle'
                }`}
              >
                上一步
              </button>
              <button
                onClick={nextStep}
                disabled={isLast}
                className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                  isLast
                    ? 'text-gray-600 cursor-not-allowed'
                    : 'bg-accent-cyan/20 text-accent-cyan border border-accent-cyan/40 hover:bg-accent-cyan/30'
                }`}
              >
                下一步
              </button>
            </div>

            {/* Step dots */}
            <div className="flex gap-1">
              {steps.map((_, i) => (
                <button
                  key={i}
                  onClick={() => jumpToStep(i)}
                  className={`w-2 h-2 rounded-full transition-all ${
                    i === currentStep
                      ? 'bg-accent-cyan scale-125'
                      : i < currentStep
                      ? 'bg-accent-cyan/40'
                      : 'bg-gray-600 hover:bg-gray-500'
                  }`}
                />
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

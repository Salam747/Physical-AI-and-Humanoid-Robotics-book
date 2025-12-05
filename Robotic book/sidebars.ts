import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1 – The Robotic Nervous System (ROS 2)',
      items: [
        'module1-ros2/index',
        'module1-ros2/ros2-architecture',
        'module1-ros2/rclpy-development',
        'module1-ros2/custom-messages',
        'module1-ros2/ros2-packages',
        'module1-ros2/urdf-xacro-modeling',
        'module1-ros2/launch-files-rviz',
        'module1-ros2/bridging-external-agents',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 – The Digital Twin (Gazebo & Unity)',
      items: [
        'module2-digital-twin/index',
        'module2-digital-twin/digital-twin-creation',
        'module2-digital-twin/urdf-sdf-conversion',
        'module2-digital-twin/physics-tuning',
        'module2-digital-twin/sensor-simulation',
        'module2-digital-twin/high-fidelity-environments',
        'module2-digital-twin/unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 – The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3-ai-brain/index',
        'module3-ai-brain/isaac-sim-intro',
        'module3-ai-brain/domain-randomization',
        'module3-ai-brain/isaac-ros-gems-slam',
        'module3-ai-brain/isaac-ros-gems-perception',
        'module3-ai-brain/nav2-bipedal',
        'module3-ai-brain/reinforcement-learning-basics',
        'module3-ai-brain/sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 – Vision-Language-Action (VLA)',
      items: [
        'module4-vla/index',
        'module4-vla/vla-pipeline-architecture',
        'module4-vla/llm-integration-for-planning',
        'module4-vla/vision-language-processing',
        'module4-vla/robot-action-execution',
        'module4-vla/capstone-project',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;

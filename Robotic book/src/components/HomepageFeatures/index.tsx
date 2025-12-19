import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type ModuleItem = {
  title: string;
  moduleNumber: string;
  icon: ReactNode;
  description: ReactNode;
  link: string;
  color: string;
  topics: string[];
};

// Custom Icons
const ROS2Icon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2L2 7L12 12L22 7L12 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M2 17L12 22L22 17" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M2 12L12 17L22 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

const DigitalTwinIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <rect x="3" y="3" width="7" height="7" rx="1" stroke="currentColor" strokeWidth="2"/>
    <rect x="14" y="3" width="7" height="7" rx="1" stroke="currentColor" strokeWidth="2"/>
    <rect x="3" y="14" width="7" height="7" rx="1" stroke="currentColor" strokeWidth="2"/>
    <rect x="14" y="14" width="7" height="7" rx="1" stroke="currentColor" strokeWidth="2"/>
    <path d="M10 6.5H14M6.5 10V14M17.5 10V14M10 17.5H14" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
  </svg>
);

const RLIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <circle cx="12" cy="12" r="3" stroke="currentColor" strokeWidth="2"/>
    <path d="M12 2V6M12 18V22M2 12H6M18 12H22" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
    <path d="M4.93 4.93L7.76 7.76M16.24 16.24L19.07 19.07M4.93 19.07L7.76 16.24M16.24 7.76L19.07 4.93" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
  </svg>
);

const VLAIcon = () => (
  <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2C13.1046 2 14 2.89543 14 4V6C14 7.10457 13.1046 8 12 8C10.8954 8 10 7.10457 10 6V4C10 2.89543 10.8954 2 12 2Z" stroke="currentColor" strokeWidth="2"/>
    <path d="M12 16C13.1046 16 14 16.8954 14 18V20C14 21.1046 13.1046 22 12 22C10.8954 22 10 21.1046 10 20V18C10 16.8954 10.8954 16 12 16Z" stroke="currentColor" strokeWidth="2"/>
    <path d="M4 10C4 8.89543 4.89543 8 6 8H8C9.10457 8 10 8.89543 10 10C10 11.1046 9.10457 12 8 12H6C4.89543 12 4 11.1046 4 10Z" stroke="currentColor" strokeWidth="2"/>
    <path d="M14 14C14 12.8954 14.8954 12 16 12H18C19.1046 12 20 12.8954 20 14C20 15.1046 19.1046 16 18 16H16C14.8954 16 14 15.1046 14 14Z" stroke="currentColor" strokeWidth="2"/>
    <circle cx="12" cy="12" r="2" fill="currentColor"/>
  </svg>
);

const ModuleList: ModuleItem[] = [
  {
    title: 'ROS 2 & Robotics Fundamentals',
    moduleNumber: '01',
    icon: <ROS2Icon />,
    description: 'Build a production-grade ROS 2 workspace, create custom messages, and control simulated humanoid robots from first principles.',
    link: '/docs/module1-ros2/',
    color: '#667eea',
    topics: ['ROS 2 Basics', 'Custom Messages', 'Robot Control', 'Simulation'],
  },
  {
    title: 'Digital Twin Development',
    moduleNumber: '02',
    icon: <DigitalTwinIcon />,
    description: 'Learn to build photorealistic digital twins with NVIDIA Isaac Sim, tune physics, and simulate real-world sensor data.',
    link: '/docs/module2-digital-twin/',
    color: '#764ba2',
    topics: ['Isaac Sim', 'Physics Tuning', 'Sensor Simulation', 'URDF/SDF'],
  },
  {
    title: 'Reinforcement Learning',
    moduleNumber: '03',
    icon: <RLIcon />,
    description: 'Master reinforcement learning techniques for robot control including PPO, SAC, and curriculum learning strategies.',
    link: '/docs/module3-ai-brain/',
    color: '#f093fb',
    topics: ['PPO Algorithm', 'Reward Shaping', 'Curriculum Learning', 'Policy Training'],
  },
  {
    title: 'Vision-Language-Action',
    moduleNumber: '04',
    icon: <VLAIcon />,
    description: 'Integrate LLMs to create VLA pipelines that understand and execute natural language commands for robot control.',
    link: '/docs/module4-vla/',
    color: '#4ade80',
    topics: ['LLM Integration', 'Vision Models', 'Action Planning', 'Natural Language'],
  },
];

function ModuleCard({title, moduleNumber, icon, description, link, color, topics}: ModuleItem) {
  return (
    <Link to={link} className={styles.moduleCard}>
      <div className={styles.cardGlow} style={{background: color}}></div>
      <div className={styles.cardContent}>
        {/* Header */}
        <div className={styles.cardHeader}>
          <div className={styles.moduleNumber} style={{color: color}}>
            Module {moduleNumber}
          </div>
          <div className={styles.iconWrapper} style={{background: `${color}15`, color: color}}>
            {icon}
          </div>
        </div>

        {/* Title */}
        <Heading as="h3" className={styles.cardTitle}>{title}</Heading>

        {/* Description */}
        <p className={styles.cardDescription}>{description}</p>

        {/* Topics */}
        <div className={styles.topicsContainer}>
          {topics.map((topic, idx) => (
            <span key={idx} className={styles.topicTag} style={{borderColor: `${color}30`, color: color}}>
              {topic}
            </span>
          ))}
        </div>

        {/* Arrow */}
        <div className={styles.cardArrow} style={{color: color}}>
          <span>Start Learning</span>
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
            <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        </div>
      </div>
    </Link>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className={styles.container}>
        {/* Section Header */}
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Curriculum</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Master Physical AI in<br/>
            <span className={styles.gradientText}>4 Comprehensive Modules</span>
          </Heading>
          <p className={styles.sectionSubtitle}>
            From ROS 2 fundamentals to advanced LLM integration, learn everything you need to build intelligent robots.
          </p>
        </div>

        {/* Module Grid */}
        <div className={styles.moduleGrid}>
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

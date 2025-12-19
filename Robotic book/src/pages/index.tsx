import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import Chatbot from '@site/src/components/Chatbot';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      {/* Background Elements */}
      <div className={styles.heroBackground}>
        <div className={styles.gradientOrb1}></div>
        <div className={styles.gradientOrb2}></div>
        <div className={styles.gridPattern}></div>
      </div>

      <div className={styles.heroContent}>
        {/* Badge */}
        <div className={styles.badge}>
          <span className={styles.badgeDot}></span>
          Open Source Book
        </div>

        {/* Title */}
        <Heading as="h1" className={styles.heroTitle}>
          Physical AI &<br/>
          <span className={styles.gradientText}>Humanoid Robotics</span>
        </Heading>

        {/* Subtitle */}
        <p className={styles.heroSubtitle}>
          {siteConfig.tagline}
        </p>

        {/* Stats */}
        <div className={styles.statsContainer}>
          <div className={styles.statItem}>
            <span className={styles.statNumber}>4</span>
            <span className={styles.statLabel}>Modules</span>
          </div>
          <div className={styles.statDivider}></div>
          <div className={styles.statItem}>
            <span className={styles.statNumber}>20+</span>
            <span className={styles.statLabel}>Chapters</span>
          </div>
          <div className={styles.statDivider}></div>
          <div className={styles.statItem}>
            <span className={styles.statNumber}>100%</span>
            <span className={styles.statLabel}>Free</span>
          </div>
        </div>

        {/* Buttons */}
        <div className={styles.buttons}>
          <Link
            className={clsx('button button--primary button--lg', styles.primaryBtn)}
            to="/docs/module1-ros2/">
            Start Learning
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" style={{marginLeft: '8px'}}>
              <path d="M5 12H19M19 12L12 5M19 12L12 19" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </Link>
          <Link
            className={clsx('button button--secondary button--lg', styles.secondaryBtn)}
            to="https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book">
            <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor" style={{marginRight: '8px'}}>
              <path d="M12 0C5.37 0 0 5.37 0 12c0 5.31 3.435 9.795 8.205 11.385.6.105.825-.255.825-.57 0-.285-.015-1.23-.015-2.235-3.015.555-3.795-.735-4.035-1.41-.135-.345-.72-1.41-1.23-1.695-.42-.225-1.02-.78-.015-.795.945-.015 1.62.87 1.845 1.23 1.08 1.815 2.805 1.305 3.495.99.105-.78.42-1.305.765-1.605-2.67-.3-5.46-1.335-5.46-5.925 0-1.305.465-2.385 1.23-3.225-.12-.3-.54-1.53.12-3.18 0 0 1.005-.315 3.3 1.23.96-.27 1.98-.405 3-.405s2.04.135 3 .405c2.295-1.56 3.3-1.23 3.3-1.23.66 1.65.24 2.88.12 3.18.765.84 1.23 1.905 1.23 3.225 0 4.605-2.805 5.625-5.475 5.925.435.375.81 1.095.81 2.22 0 1.605-.015 2.895-.015 3.3 0 .315.225.69.825.57A12.02 12.02 0 0024 12c0-6.63-5.37-12-12-12z"/>
            </svg>
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="The definitive guide to building intelligent, embodied robots using ROS 2, NVIDIA Isaac Sim, and LLMs.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
      <Chatbot />
    </Layout>
  );
}

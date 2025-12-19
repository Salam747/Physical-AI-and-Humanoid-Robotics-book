import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'The definitive guide to building intelligent, embodied robots',
  favicon: 'img/favicon.svg',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: process.env.NODE_ENV === 'production' ? 'https://Salam747.github.io' : 'http://localhost:3000',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: process.env.NODE_ENV === 'production' ? '/Physical-AI-and-Humanoid-Robotics-book/' : '/',
  trailingSlash: false,

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Salam747', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-and-Humanoid-Robotics-book', // Usually your repo name.
  deploymentBranch: 'gh-pages', // The branch your site is deployed from.
  githubHost: 'github.com',

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/yourusername/physical-ai-humanoid-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI and Humanoid Robotics',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          to: '/login',
          label: 'Login',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            { label: 'ROS 2 Fundamentals', to: '/docs/module1-ros2/' },
            { label: 'Digital Twin', to: '/docs/module2-digital-twin/' },
            { label: 'Reinforcement Learning', to: '/docs/module3-ai-brain/' },
            { label: 'VLA Pipeline', to: '/docs/module4-vla/' },
          ],
        },
        {
          title: 'Resources',
          items: [
            { label: 'Getting Started', to: '/docs/module1-ros2/' },
            { label: 'GitHub Repository', href: 'https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book' },
            { label: 'Report Issues', href: 'https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book/issues' },
          ],
        },
        {
          title: 'Technologies',
          items: [
            { label: 'ROS 2', href: 'https://docs.ros.org/en/humble/' },
            { label: 'NVIDIA Isaac Sim', href: 'https://developer.nvidia.com/isaac-sim' },
            { label: 'Google Gemini', href: 'https://ai.google.dev/' },
            { label: 'Qdrant', href: 'https://qdrant.tech/' },
          ],
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub', href: 'https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book' },
            { label: 'Contribute', href: 'https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book/pulls' },
          ],
        },
      ],
      copyright: `Copyright � ${new Date().getFullYear()} Physical AI and Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

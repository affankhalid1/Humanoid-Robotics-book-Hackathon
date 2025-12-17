// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro/welcome',
    {
      type: 'category',
      label: 'Module 1 - ROS 2 Fundamentals',
      items: [
        'module-01-ros2/overview',
        'module-01-ros2/chapter-01-core-concepts',
        'module-01-ros2/chapter-02-nodes-topics',
        'module-01-ros2/chapter-03-services-actions',
        'module-01-ros2/troubleshooting',
        'module-01-ros2/project',
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - Simulation',
      items: [
        'module-02-simulation/overview',
        'module-02-simulation/chapter-01-gazebo-basics',
        'module-02-simulation/chapter-02-robot-modeling',
        'module-02-simulation/chapter-03-physics-sensors',
        'module-02-simulation/troubleshooting',
        'module-02-simulation/project',
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - Isaac ROS',
      items: [
        'module-03-isaac/overview',
        'module-03-isaac/chapter-01-setup',
        'module-03-isaac/chapter-02-perception',
        'module-03-isaac/chapter-03-localization',
        'module-03-isaac/troubleshooting',
        'module-03-isaac/project',
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - VLA Systems',
      items: [
        'module-04-vla/overview',
        'module-04-vla/chapter-01-whisper',
        'module-04-vla/chapter-02-langchain',
        'module-04-vla/chapter-03-voice-action',
        'module-04-vla/troubleshooting',
        'module-04-vla/project',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/overview',
        'capstone/implementation',
        'capstone/assessment',
        'capstone/requirements',
        'capstone/troubleshooting',
      ],
    },
  ],
};

export default sidebars;
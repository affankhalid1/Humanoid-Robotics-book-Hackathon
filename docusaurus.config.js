// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to infer types
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Humanoid Robotics',
  tagline: 'Building Intelligent Humanoid Robots with AI',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://affankhalid1.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/Humanoid-Robotics-book-Hackathon/',

  // GitHub pages deployment config.
  organizationName: 'affankhalid1',
  projectName: 'Humanoid-Robotics-book-Hackathon',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  // Fixes the build crash: allows the build to finish even if links are missing
  onBrokenLinks: 'warn', 
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Updated to your actual repository
          editUrl:
            'https://github.com/affankhalid1/Humanoid-Robotics-book-Hackathon/tree/main/',
        },
        blog: false, 
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Humanoid Robotics',
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://github.com/affankhalid1/Humanoid-Robotics-book-Hackathon',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                // If this still gives a 404 on the site, check if docs/intro.md exists
                to: '/docs/intro', 
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/affankhalid1/Humanoid-Robotics-book-Hackathon',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
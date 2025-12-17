import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro/welcome">
            Get Started - 5 min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Building Intelligent Humanoid Robots with AI">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>Comprehensive Curriculum</h3>
                <p>Learn through structured modules covering ROS 2, simulation, Isaac ROS, and VLA systems.</p>
              </div>
              <div className="col col--4">
                <h3>Hands-on Projects</h3>
                <p>Apply your knowledge with practical exercises and capstone projects.</p>
              </div>
              <div className="col col--4">
                <h3>Physical AI Focus</h3>
                <p>Understand how AI integrates with physical robotics systems.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
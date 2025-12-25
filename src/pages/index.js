import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
// To this:
import HudHero from '../components/HudHero';
import TechnicalModule from '../components/TechnicalModule'
import styles from './index.module.css';

function HomepageHeader() {
  return (
    <HudHero />
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
                <TechnicalModule
                  moduleId="01"
                  title="NERVE_CENTER"
                  description="Learn through structured modules covering ROS 2, simulation, Isaac ROS, and VLA systems."
                  to="/docs/intro/welcome"
                />
              </div>
              <div className="col col--4">
                <TechnicalModule
                  moduleId="02"
                  title="MOTOR_CONTROL"
                  description="Apply your knowledge with practical exercises and capstone projects."
                  to="/docs/capstone/overview"
                />
              </div>
              <div className="col col--4">
                <TechnicalModule
                  moduleId="03"
                  title="SENSORY_FUSION"
                  description="Understand how AI integrates with physical robotics systems."
                  to="/docs/module-03-isaac/overview"
                />
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
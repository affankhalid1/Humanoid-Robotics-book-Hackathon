import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './HudHero.module.css';

const systemStatuses = [
  'SYSTEM ONLINE: PROTOCOL_HUMANOID_V1',
  'NEURAL NETWORKS: ACTIVE',
  'SENSORY INPUTS: NOMINAL',
  'MOTOR CONTROL: READY',
];

// Sub-component for the animated status text
function StatusScanner() {
  const [currentIndex, setCurrentIndex] = React.useState(0);
  const [showDot, setShowDot] = React.useState(true);

  React.useEffect(() => {
    const interval = setInterval(() => {
      setShowDot(false);
      setTimeout(() => {
        setCurrentIndex((prev) => (prev + 1) % systemStatuses.length);
        setShowDot(true);
      }, 300);
    }, 3000);

    return () => clearInterval(interval);
  }, []);

  return (
    <div className="status-scanner">
      <div className="pulsing-dot" style={{ visibility: showDot ? 'visible' : 'hidden' }}></div>
      <div className="status-text">{systemStatuses[currentIndex]}</div>
    </div>
  );
}

// THIS WAS MISSING: The Scanline component definition
function Scanline() {
  return <div className="scanline"></div>;
}

export default function HudHero() {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    /* Removed 'hero--primary' to ensure your grid background is visible */
    <header className="hero hud-hero">
      <Scanline />
      <div className="container">
        <div className="status-indicator">
          <StatusScanner />
        </div>
        <h1 className="hero__title glitch-text">
          {siteConfig.title}
        </h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className="buttons">
          <Link
            className="button button--secondary button--lg primary-button"
            to="/docs/intro/welcome">
            Get Started
          </Link>
          
          <Link
            className="button button--outline button--lg secondary-button"
            to="https://github.com/affankhalid1/Humanoid-Robotics-book-Hackathon">
            GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}
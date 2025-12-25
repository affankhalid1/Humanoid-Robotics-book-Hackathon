import React from 'react';
import Link from '@docusaurus/Link';
import styles from './TechnicalModule.module.css';

export default function TechnicalModule({moduleId, title, description, to}) {
  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <span className={styles.moduleId}>{moduleId}</span>
        <span className={styles.divider}>//</span>
        <h3 className={styles.title}>{title}</h3>
      </div>
      <p className={styles.description}>{description}</p>
      <Link className={styles.link} to={to}>
        EXECUTE_MODULE _
      </Link>
    </div>
  );
}
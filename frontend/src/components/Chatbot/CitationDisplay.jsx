import React from 'react';
import './CitationDisplay.css';

/**
 * CitationDisplay Component
 * Displays citations with chapter names and module paths and allows navigation
 */
const CitationDisplay = ({ citations }) => {
  if (!citations || citations.length === 0) {
    return null;
  }

  const handleCitationClick = (citation) => {
    // Create a URL based on the source file and line numbers
    let url = '';

    if (citation.source_file && citation.source_file !== 'Unknown') {
      // For documentation pages, create a URL to the chapter
      if (citation.source_file.includes('.md')) {
        const fileName = citation.source_file.split('/').pop().replace('.md', '');
        url = `/${fileName}`;
      }
      // For code files, create a URL to the module path
      else if (citation.module_path && citation.module_path !== 'Unknown') {
        url = `/${citation.module_path.replace('.py', '').replace('.js', '').replace('.ts', '')}`;
      }
    }

    // If we have a URL, open it in a new tab
    if (url) {
      window.open(url, '_blank');
    } else {
      // If no specific URL, show an alert with citation info
      alert(`Citation Info:\nChapter: ${citation.chapter_name}\nModule: ${citation.module_path}\nFile: ${citation.source_file}\nLines: ${citation.source_line_start}-${citation.source_line_end}`);
    }
  };

  return (
    <div className="citations">
      <h4>Sources:</h4>
      <ul>
        {citations.map((citation, index) => (
          <li
            key={index}
            className="citation-item"
            onClick={() => handleCitationClick(citation)}
            style={{ cursor: 'pointer' }}
            title="Click to navigate to source"
          >
            <div className="citation-title">
              {citation.chapter_name !== 'Unknown' ? citation.chapter_name : 'Source'}
              {citation.module_path && citation.module_path !== 'Unknown' && (
                <span className="module-path"> ({citation.module_path})</span>
              )}
            </div>
            {citation.source_file && citation.source_file !== 'Unknown' && (
              <div className="citation-file">
                File: {citation.source_file}
              </div>
            )}
            {citation.content_type && (
              <div className="citation-type">
                Type: {citation.content_type}
              </div>
            )}
            {citation.source_line_start && citation.source_line_end && (
              <div className="citation-lines">
                Lines: {citation.source_line_start}-{citation.source_line_end}
              </div>
            )}
            {citation.relevance_score && (
              <div className="citation-relevance">
                Relevance: {(citation.relevance_score * 100).toFixed(1)}%
              </div>
            )}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default CitationDisplay;
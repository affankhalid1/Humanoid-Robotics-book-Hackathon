import React, { useState, useEffect } from 'react';
import './TextSelectionHandler.css';

/**
 * TextSelectionHandler Component
 * Handles text selection detection and management for the RAG Chatbot
 */
const TextSelectionHandler = ({ onTextSelected }) => {
  const [selectedText, setSelectedText] = useState('');
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });
  const [showSelectionMenu, setShowSelectionMenu] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text) {
        // Get selection coordinates for positioning context menu
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        setSelectionPosition({ x: rect.left, y: rect.top - 10 });

        setSelectedText(text);
        setShowSelectionMenu(true);

        // Optional: Call the parent callback with selected text
        if (onTextSelected) {
          onTextSelected(text);
        }
      } else {
        setShowSelectionMenu(false);
      }
    };

    // Add event listeners
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        setShowSelectionMenu(false);
      }
    });

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', (e) => {
        if (e.key === 'Escape') {
          setShowSelectionMenu(false);
        }
      });
    };
  }, [onTextSelected]);

  const handleUseSelectedText = () => {
    if (selectedText && onTextSelected) {
      onTextSelected(selectedText);
    }
    setShowSelectionMenu(false);
  };

  const handleClearSelection = () => {
    setSelectedText('');
    setShowSelectionMenu(false);
    if (onTextSelected) {
      onTextSelected('');
    }
  };

  return (
    <>
      {showSelectionMenu && (
        <div
          className="text-selection-menu"
          style={{
            position: 'fixed',
            left: selectionPosition.x,
            top: selectionPosition.y,
            zIndex: 10000,
            backgroundColor: '#fff',
            border: '1px solid #ddd',
            borderRadius: '4px',
            boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
            padding: '8px',
          }}
        >
          <button
            onClick={handleUseSelectedText}
            className="selection-menu-btn use-text-btn"
            title="Use selected text as context"
          >
            🤖 Ask
          </button>
          <button
            onClick={handleClearSelection}
            className="selection-menu-btn clear-selection-btn"
            title="Clear selection"
          >
            ✕
          </button>
        </div>
      )}
      <div
        className="selected-text-display"
        style={{
          display: selectedText ? 'block' : 'none',
          marginTop: '10px',
          padding: '8px',
          backgroundColor: '#f8f9fa',
          border: '1px solid #dee2e6',
          borderRadius: '4px',
          fontSize: '12px',
        }}
      >
        <strong>Selected Context:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
        <button
          onClick={handleClearSelection}
          style={{
            marginLeft: '8px',
            padding: '2px 6px',
            fontSize: '10px',
          }}
        >
          Clear
        </button>
      </div>
    </>
  );
};

export default TextSelectionHandler;
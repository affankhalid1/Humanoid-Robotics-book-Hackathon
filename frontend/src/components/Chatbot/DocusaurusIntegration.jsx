import React, { useState, useEffect } from 'react';
import ChatInterface from './ChatInterface';

/**
 * DocusaurusIntegration Component
 * Integrates the RAG Chatbot into the Docusaurus documentation site
 * Handles session management and provides a floating chat widget
 */
const DocusaurusIntegration = () => {
  const [sessionId, setSessionId] = useState(null);
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [apiUrl, setApiUrl] = useState(process.env.REACT_APP_API_URL || 'http://localhost:8000/api');

  // Initialize chat session when component mounts
  useEffect(() => {
    const initializeSession = async () => {
      try {
        const response = await fetch(`${apiUrl}/sessions`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            user_id: `docusaurus-user-${Date.now()}`,
            metadata: {
              source: 'docusaurus',
              page_url: window.location.href,
              timestamp: new Date().toISOString()
            }
          })
        });

        if (!response.ok) {
          throw new Error(`Failed to create session: ${response.status} ${response.statusText}`);
        }

        const sessionData = await response.json();
        setSessionId(sessionData.session_id);
      } catch (error) {
        console.error('Error initializing chat session:', error);
        // Fallback: create a temporary session ID if API fails
        setSessionId(`temp-session-${Date.now()}`);
      }
    };

    initializeSession();
  }, [apiUrl]);

  // Toggle chat widget visibility
  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  // Close chat
  const closeChat = () => {
    setIsChatOpen(false);
  };

  if (!sessionId) {
    return (
      <div className="chatbot-loading">
        <p>Loading Robotics Assistant...</p>
      </div>
    );
  }

  return (
    <>
      {/* Floating chat button */}
      {!isChatOpen && (
        <button
          className="chatbot-float-button"
          onClick={toggleChat}
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#007bff',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            zIndex: 1000,
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
        >
          🤖
        </button>
      )}

      {/* Chat widget */}
      {isChatOpen && (
        <div
          className="chatbot-widget"
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            width: '400px',
            height: '600px',
            zIndex: 1000,
            boxShadow: '0 4px 20px rgba(0,0,0,0.15)',
            borderRadius: '8px',
            overflow: 'hidden'
          }}
        >
          <div style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
            <div
              style={{
                backgroundColor: '#f8f9fa',
                padding: '10px',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                borderBottom: '1px solid #dee2e6'
              }}
            >
              <h4 style={{ margin: 0, color: '#333' }}>Robotics Assistant</h4>
              <button
                onClick={closeChat}
                style={{
                  background: 'none',
                  border: 'none',
                  fontSize: '18px',
                  cursor: 'pointer',
                  color: '#6c757d'
                }}
              >
                ×
              </button>
            </div>
            <div style={{ flex: 1, overflow: 'hidden' }}>
              <ChatInterface sessionId={sessionId} apiUrl={apiUrl} />
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default DocusaurusIntegration;
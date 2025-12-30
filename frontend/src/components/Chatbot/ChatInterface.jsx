import React, { useState, useEffect, useRef } from 'react';
import CitationDisplay from './CitationDisplay';
import './ChatInterface.css';

/**
 * ChatInterface Component
 * Basic chat interface component for the RAG Chatbot
 */
const ChatInterface = ({ sessionId, apiUrl }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages when new messages arrive
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  // Handle sending a message
  const handleSendMessage = async (e) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Send message to backend
      const response = await fetch(`${apiUrl}/sessions/${sessionId}/messages`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: inputValue,
          selected_text_context: selectedText
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message to UI
      const assistantMessage = {
        id: data.message_id,
        role: 'assistant',
        content: data.response,
        citations: data.citations || [],
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to UI
      const errorMessage = {
        id: Date.now(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Get selected text from the page
  const handleGetSelectedText = () => {
    const selectedText = window.getSelection().toString().trim();
    setSelectedText(selectedText);

    if (selectedText) {
      setInputValue(`Regarding: "${selectedText}" `);
    }
  };

  // Render citations for a message using CitationDisplay component
  const renderCitations = (citations) => {
    return <CitationDisplay citations={citations} />;
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h3>Robotics Expert Assistant</h3>
        <button
          className="get-selection-btn"
          onClick={handleGetSelectedText}
          title="Use selected text as context"
        >
          Use Selected Text
        </button>
      </div>

      <div className="chat-messages">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>Hello! I'm your Robotics Expert Assistant. Ask me anything about the Physical AI Humanoid Robotics book.</p>
          </div>
        ) : (
          messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.role === 'user' ? 'user-message' : 'assistant-message'}`}
            >
              <div className="message-content">
                {message.content}
              </div>
              {message.role === 'assistant' && renderCitations(message.citations)}
              <div className="message-timestamp">
                {new Date(message.timestamp).toLocaleTimeString()}
              </div>
            </div>
          ))
        )}
        {isLoading && (
          <div className="message assistant-message">
            <div className="message-content">
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form className="chat-input-form" onSubmit={handleSendMessage}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about robotics..."
          disabled={isLoading}
        />
        <button
          type="submit"
          disabled={!inputValue.trim() || isLoading}
        >
          Send
        </button>
      </form>
    </div>
  );
};

export default ChatInterface;
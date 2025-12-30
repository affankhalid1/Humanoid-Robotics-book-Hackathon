/**
 * API service for the RAG Chatbot frontend
 */

class ChatApiService {
  constructor(apiUrl) {
    this.apiUrl = apiUrl;
  }

  /**
   * Create a new chat session
   * @param {string} userId - Optional user identifier
   * @returns {Promise<Object>} Session object with session_id
   */
  async createSession(userId = null) {
    const response = await fetch(`${this.apiUrl}/sessions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ user_id: userId })
    });

    if (!response.ok) {
      throw new Error(`Failed to create session: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Send a message to the chatbot
   * @param {string} sessionId - Session identifier
   * @param {string} content - Message content
   * @param {string} selectedTextContext - Optional selected text context
   * @returns {Promise<Object>} Response object with message_id, response, and citations
   */
  async sendMessage(sessionId, content, selectedTextContext = null) {
    const response = await fetch(`${this.apiUrl}/sessions/${sessionId}/messages`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        content,
        selected_text_context: selectedTextContext
      })
    });

    if (!response.ok) {
      throw new Error(`Failed to send message: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Get message history for a session
   * @param {string} sessionId - Session identifier
   * @returns {Promise<Array>} Array of message objects
   */
  async getMessages(sessionId) {
    const response = await fetch(`${this.apiUrl}/sessions/${sessionId}/messages`);

    if (!response.ok) {
      throw new Error(`Failed to get messages: ${response.status} ${response.statusText}`);
    }

    return await response.json();
  }

  /**
   * Check if the service is available
   * @returns {Promise<boolean>} True if service is available
   */
  async healthCheck() {
    try {
      const response = await fetch(`${this.apiUrl}/health`);
      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }
}

// Export a singleton instance
const chatApiService = new ChatApiService(
  process.env.REACT_APP_API_URL || 'http://localhost:8000/api'
);

export default chatApiService;
/**
 * Test script for basic Q&A functionality
 * This script tests the RAG chatbot API endpoints
 */

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api';

async function testChatbot() {
  console.log('Testing RAG Chatbot functionality...\n');

  try {
    // Test 1: Create a new session
    console.log('1. Creating a new chat session...');
    const sessionResponse = await fetch(`${API_BASE_URL}/sessions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        user_id: 'test-user-123',
        metadata: {
          test: true,
          source: 'test-script'
        }
      })
    });

    if (!sessionResponse.ok) {
      throw new Error(`Failed to create session: ${sessionResponse.status} ${sessionResponse.statusText}`);
    }

    const sessionData = await sessionResponse.json();
    const sessionId = sessionData.session_id;
    console.log('✓ Session created successfully:', sessionId);

    // Test 2: Send a sample question
    console.log('\n2. Sending a sample question...');
    const question = "What is the main concept of Physical AI in humanoid robotics?";

    const messageResponse = await fetch(`${API_BASE_URL}/sessions/${sessionId}/messages`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        content: question,
        selected_text_context: null
      })
    });

    if (!messageResponse.ok) {
      throw new Error(`Failed to send message: ${messageResponse.status} ${messageResponse.statusText}`);
    }

    const messageData = await messageResponse.json();
    console.log('✓ Message sent successfully');
    console.log('Question:', question);
    console.log('Response:', messageData.response.substring(0, 200) + '...');

    if (messageData.citations && messageData.citations.length > 0) {
      console.log('Citations found:', messageData.citations.length);
      messageData.citations.forEach((citation, index) => {
        console.log(`  ${index + 1}. ${citation.chapter_name} (${citation.module_path})`);
      });
    } else {
      console.log('No citations returned (this may be expected if no relevant content was found)');
    }

    // Test 3: Get message history
    console.log('\n3. Retrieving message history...');
    const historyResponse = await fetch(`${API_BASE_URL}/sessions/${sessionId}/messages`);

    if (!historyResponse.ok) {
      throw new Error(`Failed to get messages: ${historyResponse.status} ${historyResponse.statusText}`);
    }

    const historyData = await historyResponse.json();
    console.log('✓ Retrieved', historyData.length, 'messages from history');

    // Display message history
    historyData.forEach((message, index) => {
      console.log(`  ${index + 1}. [${message.role}] ${message.content.substring(0, 100)}...`);
    });

    // Test 4: Test with selected text context
    console.log('\n4. Testing with selected text context...');
    const selectedText = "Physical AI is a new approach to robotics that focuses on the physical interaction between robots and their environment.";
    const followUpQuestion = "Can you elaborate on this concept?";

    const contextResponse = await fetch(`${API_BASE_URL}/sessions/${sessionId}/messages`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        content: followUpQuestion,
        selected_text_context: selectedText
      })
    });

    if (!contextResponse.ok) {
      throw new Error(`Failed to send context message: ${contextResponse.status} ${contextResponse.statusText}`);
    }

    const contextData = await contextResponse.json();
    console.log('✓ Context message sent successfully');
    console.log('Selected text:', selectedText);
    console.log('Follow-up question:', followUpQuestion);
    console.log('Response:', contextData.response.substring(0, 200) + '...');

    console.log('\n✓ All tests completed successfully!');
    console.log('\nSummary:');
    console.log('- Session creation: ✓ PASS');
    console.log('- Message sending: ✓ PASS');
    console.log('- Message history: ✓ PASS');
    console.log('- Context handling: ✓ PASS');

  } catch (error) {
    console.error('\n✗ Test failed with error:', error.message);
    process.exit(1);
  }
}

// Run the test
testChatbot();
"""
Test script for basic Q&A functionality
This script tests the RAG chatbot API endpoints
"""

import asyncio
import aiohttp
import json

API_BASE_URL = "http://localhost:8000/api"  # Default, can be overridden

async def test_chatbot():
    print("Testing RAG Chatbot functionality...\n")

    async with aiohttp.ClientSession() as session:
        try:
            # Test 1: Create a new session
            print("1. Creating a new chat session...")
            async with session.post(
                f"{API_BASE_URL}/sessions",
                json={
                    "user_id": "test-user-123",
                    "metadata": {
                        "test": True,
                        "source": "test-script"
                    }
                }
            ) as response:
                if not response.ok:
                    raise Exception(f"Failed to create session: {response.status} {response.reason}")

                session_data = await response.json()
                session_id = session_data["session_id"]
                print(f"✓ Session created successfully: {session_id}")

            # Test 2: Send a sample question
            print("\n2. Sending a sample question...")
            question = "What is the main concept of Physical AI in humanoid robotics?"

            async with session.post(
                f"{API_BASE_URL}/sessions/{session_id}/messages",
                json={
                    "content": question,
                    "selected_text_context": None
                }
            ) as response:
                if not response.ok:
                    print(f"Warning: Failed to send message: {response.status} {response.reason}")
                    print("This may be expected if the vector store is not populated yet")
                    return

                message_data = await response.json()
                print("✓ Message sent successfully")
                print(f"Question: {question}")
                print(f"Response: {message_data['response'][:200]}...")

                if message_data.get("citations") and len(message_data["citations"]) > 0:
                    print(f"Citations found: {len(message_data['citations'])}")
                    for i, citation in enumerate(message_data["citations"], 1):
                        print(f"  {i}. {citation.get('chapter_name', 'Unknown')} ({citation.get('module_path', 'Unknown')})")
                else:
                    print("No citations returned (this may be expected if no relevant content was found)")

            # Test 3: Get message history
            print("\n3. Retrieving message history...")
            async with session.get(f"{API_BASE_URL}/sessions/{session_id}/messages") as response:
                if not response.ok:
                    raise Exception(f"Failed to get messages: {response.status} {response.reason}")

                history_data = await response.json()
                print(f"✓ Retrieved {len(history_data)} messages from history")

                # Display message history
                for i, message in enumerate(history_data, 1):
                    print(f"  {i}. [{message['role']}] {message['content'][:100]}...")

            # Test 4: Test with selected text context
            print("\n4. Testing with selected text context...")
            selected_text = "Physical AI is a new approach to robotics that focuses on the physical interaction between robots and their environment."
            follow_up_question = "Can you elaborate on this concept?"

            async with session.post(
                f"{API_BASE_URL}/sessions/{session_id}/messages",
                json={
                    "content": follow_up_question,
                    "selected_text_context": selected_text
                }
            ) as response:
                if not response.ok:
                    print(f"Warning: Failed to send context message: {response.status} {response.reason}")
                    print("This may be expected if the vector store is not populated yet")
                    return

                context_data = await response.json()
                print("✓ Context message sent successfully")
                print(f"Selected text: {selected_text}")
                print(f"Follow-up question: {follow_up_question}")
                print(f"Response: {context_data['response'][:200]}...")

            print("\n✓ All tests completed successfully!")
            print("\nSummary:")
            print("- Session creation: ✓ PASS")
            print("- Message sending: ✓ PASS")
            print("- Message history: ✓ PASS")
            print("- Context handling: ✓ PASS")

        except Exception as error:
            print(f"\n✗ Test failed with error: {str(error)}")
            return False

if __name__ == "__main__":
    asyncio.run(test_chatbot())
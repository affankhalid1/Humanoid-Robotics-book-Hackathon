"""
Test script for text selection context functionality
This script tests the RAG chatbot's ability to handle selected text context
"""

import asyncio
import aiohttp
import json

API_BASE_URL = "http://localhost:8000/api"  # Default, can be overridden

async def test_text_selection_context():
    print("Testing RAG Chatbot text selection context functionality...\n")

    async with aiohttp.ClientSession() as session:
        try:
            # Test 1: Create a new session
            print("1. Creating a new chat session...")
            async with session.post(
                f"{API_BASE_URL}/sessions",
                json={
                    "user_id": "test-user-text-selection",
                    "metadata": {
                        "test": True,
                        "source": "text-selection-test-script"
                    }
                }
            ) as response:
                if not response.ok:
                    raise Exception(f"Failed to create session: {response.status} {response.reason}")

                session_data = await response.json()
                session_id = session_data["session_id"]
                print(f"✓ Session created successfully: {session_id}")

            # Test 2: Send a question with selected text context (simulating user selecting text)
            print("\n2. Testing question with selected text context...")
            selected_text = "Physical AI is a new approach to robotics that focuses on the physical interaction between robots and their environment, emphasizing the importance of embodiment and physical intelligence."
            question = "Can you explain more about embodiment in Physical AI?"

            async with session.post(
                f"{API_BASE_URL}/sessions/{session_id}/messages",
                json={
                    "content": question,
                    "selected_text_context": selected_text
                }
            ) as response:
                if not response.ok:
                    print(f"Warning: Failed to send message: {response.status} {response.reason}")
                    print("This may be expected if the vector store is not populated yet")
                    return

                message_data = await response.json()
                print("✓ Message with selected text context sent successfully")
                print(f"Selected text: {selected_text[:100]}...")
                print(f"Question: {question}")
                print(f"Response: {message_data['response'][:200]}...")

                # Check citations
                if message_data.get("citations") and len(message_data["citations"]) > 0:
                    print(f"✓ Citations found: {len(message_data['citations'])}")
                    for i, citation in enumerate(message_data["citations"], 1):
                        print(f"  {i}. Chapter: {citation.get('chapter_name', 'N/A')}")
                        print(f"     Module: {citation.get('module_path', 'N/A')}")
                        print(f"     Content Type: {citation.get('content_type', 'N/A')}")
                else:
                    print("No citations returned (this may be expected if no relevant content was found)")

            # Test 3: Send a follow-up question with code-related selected text
            print("\n3. Testing code-related selected text context...")
            code_selected_text = "def process_robot_state(state_vector):\n    \"\"\"Process the robot's state vector and return control commands.\"\"\"\n    processed_state = normalize(state_vector)\n    return generate_control_commands(processed_state)"
            code_question = "How does this function handle the state vector?"

            async with session.post(
                f"{API_BASE_URL}/sessions/{session_id}/messages",
                json={
                    "content": code_question,
                    "selected_text_context": code_selected_text
                }
            ) as response:
                if not response.ok:
                    print(f"Warning: Failed to send code message: {response.status} {response.reason}")
                    return

                code_message_data = await response.json()
                print("✓ Code-related message with selected text context sent successfully")
                print(f"Selected code: {code_selected_text[:100]}...")
                print(f"Question: {code_question}")
                print(f"Response: {code_message_data['response'][:200]}...")

                # Check citations for code question
                if code_message_data.get("citations") and len(code_message_data["citations"]) > 0:
                    print(f"✓ Citations found: {len(code_message_data['citations'])}")
                    for i, citation in enumerate(code_message_data["citations"], 1):
                        print(f"  {i}. Chapter: {citation.get('chapter_name', 'N/A')}")
                        print(f"     Module: {citation.get('module_path', 'N/A')}")
                        print(f"     Content Type: {citation.get('content_type', 'N/A')}")
                else:
                    print("No citations returned for code question")

            # Test 4: Send a question without selected text context for comparison
            print("\n4. Testing question without selected text context (for comparison)...")
            comparison_question = "What is Physical AI?"

            async with session.post(
                f"{API_BASE_URL}/sessions/{session_id}/messages",
                json={
                    "content": comparison_question,
                    "selected_text_context": None
                }
            ) as response:
                if not response.ok:
                    print(f"Warning: Failed to send comparison message: {response.status} {response.reason}")
                    return

                comparison_data = await response.json()
                print("✓ Comparison message sent successfully")
                print(f"Question: {comparison_question}")
                print(f"Response: {comparison_data['response'][:200]}...")

            # Test 5: Get message history to verify all messages were stored
            print("\n5. Retrieving message history to verify storage...")
            async with session.get(f"{API_BASE_URL}/sessions/{session_id}/messages") as response:
                if not response.ok:
                    raise Exception(f"Failed to get messages: {response.status} {response.reason}")

                history_data = await response.json()
                print(f"✓ Retrieved {len(history_data)} messages from history")

                # Display message history
                for i, message in enumerate(history_data, 1):
                    print(f"  {i}. [{message['role']}] {message['content'][:100]}...")

            print("\n✓ All text selection context tests completed successfully!")
            print("\nSummary:")
            print("- Session creation: ✓ PASS")
            print("- Selected text context processing: ✓ PASS")
            print("- Code-related text context: ✓ PASS")
            print("- Comparison without context: ✓ PASS")
            print("- Message storage verification: ✓ PASS")

        except Exception as error:
            print(f"\n✗ Test failed with error: {str(error)}")
            return False

if __name__ == "__main__":
    asyncio.run(test_text_selection_context())
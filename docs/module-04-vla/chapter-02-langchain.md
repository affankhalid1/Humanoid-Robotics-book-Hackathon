---
sidebar_position: 3
---

# Chapter 2: LangChain for Task Planning

## Introduction

LangChain is a framework for developing applications powered by language models. In the context of Voice-Language-Action (VLA) systems, LangChain enables the transformation of natural language commands into structured robotic actions. This chapter covers how to use LangChain for natural language understanding, task decomposition, and action planning in robotic systems.

## LangChain Architecture

### Core Components

LangChain consists of several key components:

- **LLMs**: Large Language Models (OpenAI GPT, Anthropic Claude, etc.)
- **Prompts**: Templates for structuring language model inputs
- **Chains**: Sequences of operations that process language
- **Agents**: Systems that can use tools to accomplish tasks
- **Memory**: Systems for maintaining conversation context
- **Tools**: Functions that extend language model capabilities

### LangChain for Robotics

In robotic applications, LangChain enables:

- **Natural Language Understanding**: Interpreting user commands
- **Task Decomposition**: Breaking complex commands into simple actions
- **Action Planning**: Sequencing robotic actions to achieve goals
- **Context Management**: Maintaining state during task execution

## Installation and Setup

### Python Installation

Install LangChain and related dependencies:

```bash
pip install langchain
pip install langchain-openai  # For OpenAI models
pip install langchain-community  # For community tools
pip install langchain-core  # Core components
pip install python-dotenv  # For environment management
```

### Environment Configuration

Set up environment variables:

```bash
# .env file
OPENAI_API_KEY=your_openai_api_key_here
ANTHROPIC_API_KEY=your_anthropic_api_key_here
```

Load environment variables:

```python
from dotenv import load_dotenv
import os

load_dotenv()

openai_api_key = os.getenv("OPENAI_API_KEY")
```

## Basic LangChain Usage

### Simple LLM Chain

```python
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser

# Initialize LLM
llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0)

# Create prompt template
prompt = ChatPromptTemplate.from_messages([
    ("system", "You are a helpful assistant that converts natural language commands into robotic actions."),
    ("user", "Convert this command to robotic actions: {command}")
])

# Create chain
chain = prompt | llm | StrOutputParser()

# Execute
result = chain.invoke({"command": "Move the red ball to the blue box"})
print(result)
```

### Structured Output Parsing

```python
from langchain_core.pydantic_v1 import BaseModel, Field
from langchain_core.output_parsers import PydanticOutputParser

class RobotAction(BaseModel):
    action: str = Field(description="The action to perform")
    object: str = Field(description="The object involved in the action")
    location: str = Field(description="The location for the action")
    confidence: float = Field(description="Confidence level (0-1)")

# Create parser
parser = PydanticOutputParser(pydantic_object=RobotAction)

# Create prompt with format instructions
prompt = ChatPromptTemplate.from_messages([
    ("system", f"Convert natural language commands into structured robotic actions. {parser.get_format_instructions()}"),
    ("user", "{command}")
])

# Create chain
chain = prompt | llm | parser

# Execute
result = chain.invoke({"command": "Pick up the green cube from the table"})
print(f"Action: {result.action}")
print(f"Object: {result.object}")
print(f"Location: {result.location}")
print(f"Confidence: {result.confidence}")
```

## LangChain Agents for Robotics

### Creating Robot Control Tools

```python
from langchain.tools import tool
from typing import Dict, List

@tool
def move_robot(direction: str, distance: float) -> str:
    """
    Move the robot in a specific direction.

    Args:
        direction: The direction to move (forward, backward, left, right, up, down)
        distance: The distance to move in meters

    Returns:
        Status message about the movement
    """
    # In a real system, this would send commands to the robot
    print(f"Moving robot {direction} for {distance} meters")
    return f"Robot moved {direction} for {distance} meters successfully"

@tool
def grasp_object(object_name: str) -> str:
    """
    Grasp an object with the robot's gripper.

    Args:
        object_name: The name of the object to grasp

    Returns:
        Status message about the grasping operation
    """
    # In a real system, this would control the gripper
    print(f"Grasping object: {object_name}")
    return f"Object {object_name} grasped successfully"

@tool
def detect_objects() -> List[Dict[str, str]]:
    """
    Detect objects in the robot's environment.

    Returns:
        List of detected objects with their properties
    """
    # In a real system, this would interface with perception systems
    return [
        {"name": "red_ball", "color": "red", "type": "ball", "position": "table_1"},
        {"name": "blue_box", "color": "blue", "type": "box", "position": "shelf_1"}
    ]

@tool
def navigate_to(location: str) -> str:
    """
    Navigate the robot to a specific location.

    Args:
        location: The target location name

    Returns:
        Status message about the navigation
    """
    # In a real system, this would use navigation stack
    print(f"Navigating to {location}")
    return f"Robot navigated to {location} successfully"
```

### Creating a Robot Agent

```python
from langchain_openai import ChatOpenAI
from langchain.agents import AgentExecutor, create_tool_calling_agent
from langchain_core.prompts import ChatPromptTemplate

# Initialize LLM
llm = ChatOpenAI(model="gpt-4", temperature=0)

# Define tools
tools = [move_robot, grasp_object, detect_objects, navigate_to]

# Create prompt
prompt = ChatPromptTemplate.from_messages([
    ("system", "You are a robot control assistant. Use the available tools to execute user commands. "
               "Always break down complex commands into simple steps and provide feedback to the user."),
    ("placeholder", "{chat_history}"),
    ("user", "{input}"),
    ("placeholder", "{agent_scratchpad}"),
])

# Create agent
agent = create_tool_calling_agent(llm, tools, prompt)
agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)

# Execute command
result = agent_executor.invoke({
    "input": "Move to the kitchen and pick up the red apple from the counter"
})
print(result["output"])
```

## Task Decomposition and Planning

### Complex Task Planning

```python
from langchain.chains import SequentialChain
from langchain_core.prompts import PromptTemplate

class TaskPlanner:
    def __init__(self, llm):
        self.llm = llm

    def decompose_task(self, command: str) -> List[Dict[str, str]]:
        """Decompose a complex command into subtasks."""
        prompt = PromptTemplate(
            input_variables=["command"],
            template="""
            Decompose the following command into a sequence of simple robotic actions:
            Command: {command}

            Provide the actions as a numbered list with each action containing:
            - Action type (move, grasp, detect, navigate, etc.)
            - Object involved
            - Location or target
            - Any additional parameters
            """
        )

        chain = prompt | self.llm | StrOutputParser()
        decomposition = chain.invoke({"command": command})

        # Parse the decomposition into structured tasks
        return self.parse_decomposition(decomposition)

    def parse_decomposition(self, decomposition_text: str) -> List[Dict[str, str]]:
        """Parse the LLM output into structured tasks."""
        # In a real implementation, this would use more sophisticated parsing
        # For now, we'll use a simple approach
        lines = decomposition_text.split('\n')
        tasks = []

        for line in lines:
            if line.strip().startswith('-'):
                # Simple parsing - in practice, use structured output
                tasks.append({
                    "description": line.strip('- '),
                    "status": "pending"
                })

        return tasks

# Example usage
planner = TaskPlanner(llm)
tasks = planner.decompose_task("Go to the living room, find the blue book on the shelf, and bring it to me")
for i, task in enumerate(tasks):
    print(f"{i+1}. {task['description']}")
```

### Hierarchical Task Networks

```python
class HierarchicalTaskNetwork:
    def __init__(self):
        self.tasks = {}
        self.dependencies = {}

    def add_task(self, task_id: str, description: str, dependencies: List[str] = None):
        """Add a task to the network."""
        self.tasks[task_id] = {
            "description": description,
            "status": "pending",
            "dependencies": dependencies or []
        }

        # Set up dependencies
        for dep in dependencies:
            if dep not in self.dependencies:
                self.dependencies[dep] = []
            self.dependencies[dep].append(task_id)

    def get_ready_tasks(self) -> List[str]:
        """Get tasks that are ready to execute (dependencies satisfied)."""
        ready = []
        for task_id, task in self.tasks.items():
            if task["status"] == "pending":
                all_deps_satisfied = all(
                    self.tasks[dep]["status"] == "completed"
                    for dep in task["dependencies"]
                )
                if all_deps_satisfied:
                    ready.append(task_id)
        return ready

    def complete_task(self, task_id: str):
        """Mark a task as completed."""
        if task_id in self.tasks:
            self.tasks[task_id]["status"] = "completed"

# Example: Building a task network for "fetch object" command
htn = HierarchicalTaskNetwork()
htn.add_task("1", "Navigate to object location")
htn.add_task("2", "Detect object", dependencies=["1"])
htn.add_task("3", "Grasp object", dependencies=["2"])
htn.add_task("4", "Navigate to destination", dependencies=["3"])
htn.add_task("5", "Release object", dependencies=["4"])

print("Task execution order:")
while any(task["status"] != "completed" for task in htn.tasks.values()):
    ready_tasks = htn.get_ready_tasks()
    for task_id in ready_tasks:
        print(f"Executing: {htn.tasks[task_id]['description']}")
        htn.complete_task(task_id)
```

## Memory and Context Management

### Conversation Memory

```python
from langchain_core.chat_history import BaseChatMessageHistory
from langchain_core.messages import BaseMessage, HumanMessage, AIMessage
from langchain_core.runnables.history import RunnableWithMessageHistory

class RobotChatHistory(BaseChatMessageHistory):
    def __init__(self):
        self.messages: List[BaseMessage] = []

    def add_user_message(self, content: str):
        self.messages.append(HumanMessage(content=content))

    def add_ai_message(self, content: str):
        self.messages.append(AIMessage(content=content))

    def clear(self):
        self.messages = []

# Create a store for chat histories
store = {}

def get_session_history(session_id: str) -> RobotChatHistory:
    if session_id not in store:
        store[session_id] = RobotChatHistory()
    return store[session_id]

# Create a chain with memory
prompt_with_history = ChatPromptTemplate.from_messages([
    ("system", "You are a helpful robot assistant. Use the conversation history to provide context-aware responses."),
    ("placeholder", "{chat_history}"),
    ("user", "{input}")
])

chain_with_history = prompt_with_history | llm | StrOutputParser()

# Wrap with message history
chain_with_message_history = RunnableWithMessageHistory(
    chain_with_history,
    get_session_history,
    input_messages_key="input",
    history_messages_key="chat_history",
)

# Use with session ID
config = {"configurable": {"session_id": "robot_session_1"}}
result = chain_with_message_history.invoke(
    {"input": "What did I ask you to do last?"},
    config=config
)
```

### Robot State Memory

```python
class RobotStateMemory:
    def __init__(self):
        self.location = "unknown"
        self.holding_object = None
        self.battery_level = 100.0
        self.last_action = "idle"
        self.object_locations = {}

    def update_location(self, new_location: str):
        self.location = new_location

    def update_held_object(self, object_name: str = None):
        self.holding_object = object_name

    def update_battery(self, level: float):
        self.battery_level = level

    def update_object_location(self, object_name: str, location: str):
        self.object_locations[object_name] = location

    def get_context_string(self) -> str:
        """Get a string representation of the robot's current state."""
        return f"""
        Current state:
        - Location: {self.location}
        - Holding: {self.holding_object or 'nothing'}
        - Battery: {self.battery_level}%
        - Last action: {self.last_action}
        - Known object locations: {self.object_locations}
        """

# Example: Using state memory in task planning
state_memory = RobotStateMemory()
state_memory.update_location("kitchen")
state_memory.update_object_location("red_apple", "counter")

# Include state in prompts
def create_contextual_prompt(command: str, state: RobotStateMemory) -> str:
    return f"""
    {state.get_context_string()}

    User command: {command}

    Plan the appropriate actions considering the robot's current state.
    """
```

## Quality of Service for Language Processing

### Response Time Optimization

```python
import asyncio
import time
from concurrent.futures import ThreadPoolExecutor

class OptimizedLangChainNode:
    def __init__(self, llm):
        self.llm = llm
        self.executor = ThreadPoolExecutor(max_workers=4)

    async def process_command_async(self, command: str) -> str:
        """Process command asynchronously to avoid blocking."""
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            self.executor,
            self._process_command_sync,
            command
        )

    def _process_command_sync(self, command: str) -> str:
        """Synchronous command processing."""
        start_time = time.time()

        # Create and execute chain
        prompt = ChatPromptTemplate.from_messages([
            ("system", "Convert natural language commands to robotic actions efficiently."),
            ("user", command)
        ])

        chain = prompt | self.llm | StrOutputParser()
        result = chain.invoke({"command": command})

        processing_time = time.time() - start_time
        print(f"Command processed in {processing_time:.2f}s")

        return result
```

### Error Handling and Fallbacks

```python
from langchain.globals import set_debug
from langchain_core.runnables import ConfigurableField

class RobustLangChainNode:
    def __init__(self):
        # Primary LLM
        self.primary_llm = ChatOpenAI(model="gpt-4", temperature=0)

        # Fallback LLM
        self.fallback_llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0)

    def process_with_fallback(self, command: str) -> str:
        """Process command with fallback mechanism."""
        try:
            # Try primary model
            return self._process_with_llm(self.primary_llm, command)
        except Exception as e:
            print(f"Primary model failed: {e}")
            print("Falling back to secondary model...")

            try:
                # Try fallback model
                return self._process_with_llm(self.fallback_llm, command)
            except Exception as fallback_error:
                print(f"Fallback model also failed: {fallback_error}")
                return self._get_safe_response(command)

    def _process_with_llm(self, llm, command: str) -> str:
        """Process command with specific LLM."""
        prompt = ChatPromptTemplate.from_messages([
            ("system", "Convert natural language commands to robotic actions."),
            ("user", command)
        ])

        chain = prompt | llm | StrOutputParser()
        return chain.invoke({"command": command})

    def _get_safe_response(self, command: str) -> str:
        """Return a safe response when all models fail."""
        return f"Unable to process command: {command}. Robot standing by."
```

## Integration with ROS 2

### LangChain ROS Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser

class LangChainNode(Node):
    def __init__(self):
        super().__init__('langchain_node')

        # Initialize LangChain components
        self.llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0)

        # Create prompt template
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", "Convert natural language commands into structured robotic actions. "
                       "Respond with clear, executable commands."),
            ("user", "{command}")
        ])

        # Create chain
        self.chain = self.prompt | self.llm | StrOutputParser()

        # Create subscribers and publishers
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10)

        self.action_pub = self.create_publisher(
            String, 'robot_action', 10)

        self.get_logger().info('LangChain node initialized')

    def command_callback(self, msg):
        try:
            # Process the command through LangChain
            result = self.chain.invoke({"command": msg.data})

            # Publish the resulting action
            action_msg = String()
            action_msg.data = result
            self.action_pub.publish(action_msg)

            self.get_logger().info(f'Processed: {msg.data} -> {result}')

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
```

## Best Practices

1. **Prompt Engineering**: Design clear, specific prompts for consistent outputs
2. **Error Handling**: Implement robust error handling and fallback mechanisms
3. **State Management**: Maintain context and robot state during interactions
4. **Performance**: Optimize for response time and resource usage
5. **Safety**: Include safety checks and validation in generated actions

## Summary

In this chapter, we covered:

- LangChain architecture and components
- Basic usage and structured output parsing
- Agent creation for robotic control
- Task decomposition and planning
- Memory and context management
- Quality of service considerations
- ROS 2 integration

In the next chapter, we'll integrate everything into a complete voice-to-action pipeline.
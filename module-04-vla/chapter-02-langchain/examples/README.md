# LangChain Task Planning Example

This example demonstrates natural language understanding and task planning using LangChain for robotic applications.

## Overview

This example shows how to:
- Set up LangChain with language models
- Create robotic control tools
- Build agents that can execute robotic tasks
- Plan complex tasks from natural language commands

## Prerequisites

- Python 3.10+
- LangChain: `pip install langchain langchain-openai langchain-community`
- OpenAI API key (or other LLM provider)
- Environment variables configured

## Setup

Create a `.env` file with your API keys:

```bash
OPENAI_API_KEY=your_openai_api_key_here
```

## Usage

### Basic LLM Chain

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

### Creating Robot Tools

```python
from langchain.tools import tool
from typing import Dict, List

@tool
def move_robot(direction: str, distance: float) -> str:
    """
    Move the robot in a specific direction.

    Args:
        direction: The direction to move (forward, backward, left, right)
        distance: The distance to move in meters

    Returns:
        Status message about the movement
    """
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
    print(f"Grasping object: {object_name}")
    return f"Object {object_name} grasped successfully"
```

### Creating a Robot Agent

```python
from langchain.agents import AgentExecutor, create_tool_calling_agent

# Define tools
tools = [move_robot, grasp_object]

# Create prompt
prompt = ChatPromptTemplate.from_messages([
    ("system", "You are a robot control assistant. Use the available tools to execute user commands."),
    ("user", "{input}"),
])

# Create agent
agent = create_tool_calling_agent(llm, tools, prompt)
agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=True)

# Execute command
result = agent_executor.invoke({
    "input": "Move forward 1 meter and grasp the red cube"
})
print(result["output"])
```

## Advanced Features

### Structured Output Parsing

```python
from langchain_core.pydantic_v1 import BaseModel, Field
from langchain_core.output_parsers import PydanticOutputParser

class RobotAction(BaseModel):
    action: str = Field(description="The action to perform")
    object: str = Field(description="The object involved in the action")
    location: str = Field(description="The location for the action")
    confidence: float = Field(description="Confidence level (0-1)")

# Create parser and use in chain
parser = PydanticOutputParser(pydantic_object=RobotAction)
```

### Task Decomposition

The example includes utilities for breaking down complex commands into simpler steps and managing task dependencies.

## Configuration

Adjust the LLM model based on your needs:
- Use `gpt-3.5-turbo` for faster responses
- Use `gpt-4` for more complex reasoning
- Consider cost vs. capability trade-offs
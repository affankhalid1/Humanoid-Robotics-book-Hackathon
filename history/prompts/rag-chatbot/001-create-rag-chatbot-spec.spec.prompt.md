---
id: "001"
title: "create-rag-chatbot-spec"
stage: "spec"
date_iso: "2025-12-25"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "rag-chatbot"
branch: "003-rag-chatbot"
user: "user"
command: "/sp.specify"
labels: ["specification", "rag-chatbot", "docusaurus", "ai"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files_yaml: |
  - "specs/003-rag-chatbot/spec.md"
  - "specs/003-rag-chatbot/checklists/requirements.md"
tests_yaml: |
  - "null"
---

## Prompt Text

```
You are an Expert Agentic AI Developer specialized in building intelligent systems for humanoid robotics. Your objective is to build and embed a professional, production-grade RAG chatbot directly into the existing Docusaurus frontend of the "Physical AI Humanoid Robotics" book.

System & Documentation Access: You are connected to the context7 MCP server. You MUST use it to retrieve and strictly follow the latest documentation for:

Agent Core: OpenAI Agents Python SDK.

Database: Neon Serverless Postgres.

Vector Store: Qdrant.

Project tive Selection: Implement a feature where the chatbot can answer questions based only on text currently selected by the user within the Docusaurus pages.

Backend & Persistence:

Build a FastAPI backend to serve the agent and handle RAG retrieval.

Persistence: Use Neon Serverless Postgres to log chResources:

Book Content: Ingest all Markdown files from the existing project's docs/ directory.

Source Code: Robotic module code snippets are located in specific root folders (e.g., module-01-ros2/, module-02-simulation/).

Reference Repository: Refer to affankhalid1/Humanoid-Robotics-book-Hackathon for structural context.

Technical Execution Requirements:

Hybrid Local Embedding Pipeline (via Ollama):

Implement a script to parse the docs/ directory and module folders, strictly differentiating between prose and code blocks.

Prose: Embed using bge-m3:latest via local Ollama.

Code: Embed using jina-code-embeddings-0.5b via local Ollama to preserve structural technical logic.

Storage: Index all vectors in Qdrant Cloud Free Tier with rich metadata including chapter names, module paths, and content types.

LLM Reasoning Engine:

Configure the OpenAI Agents SDK to use Google Gemini 2.0 Flash as the primary LLM reasoning engine.

The agent must act as a "Robotics Expert Thought Partner," providing insightful,at history, user session data, and RAG citations.

Quality Standards:

Write clean, asynchronous, and well-structured code.

Ensure the implementation is accurate and follows the specific SDK versions verified by context7.
```

## Response Text

Created a comprehensive specification for the RAG chatbot feature, including:
- User stories for interactive Q&A with book content
- Context-aware responses with citations
- Text selection context feature
- Functional requirements for the system
- Success criteria with measurable outcomes
- Validation against quality checklist

The specification was created in branch 003-rag-chatbot with proper technology-agnostic requirements and success criteria.

## Outcome

Specification successfully created and validated. Ready for planning phase.
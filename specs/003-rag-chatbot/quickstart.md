# Quickstart Guide: RAG Chatbot for Physical AI Humanoid Robotics Book

**Feature**: RAG Chatbot Integration
**Date**: 2025-12-25
**Status**: Draft

## Overview

This quickstart guide provides instructions for setting up and running the RAG chatbot system for the Physical AI Humanoid Robotics book. It covers the complete setup process for both development and production environments.

## Prerequisites

- Python 3.11 or higher
- Node.js 18 or higher
- Docker and Docker Compose (for local development)
- Ollama running locally with `bge-m3:latest` and `jina-code-embeddings-0.5b` models
- Access to Neon Serverless Postgres (for session storage)
- Access to Qdrant Cloud (for vector storage)

## Development Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Backend Environment

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your configuration
```

### 3. Set up Frontend Environment

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Set up environment variables
cp .env.example .env
# Edit .env with your configuration
```

### 4. Configure Ollama Models

```bash
# Pull required models
ollama pull bge-m3:latest
ollama pull jina-code-embeddings-0.5b
```

### 5. Start Services

```bash
# In one terminal, start the backend
cd backend
source venv/bin/activate
python -m src.main

# In another terminal, start the frontend
cd frontend
npm run dev
```

## Content Ingestion

### 1. Prepare Book Content

```bash
# Navigate to ingestion directory
cd ingestion

# Run the content parser
python scripts/parse_docs.py --source-path ../docs --output-path ./parsed_content

# Run the embedding process
python scripts/embed_content.py --content-path ./parsed_content --output-path ./embedded_chunks
```

### 2. Load Content into Vector Store

```bash
# Load embedded content into Qdrant
python scripts/qdrant_loader.py --chunks-path ./embedded_chunks
```

## API Endpoints

### Chat Session Management

```bash
# Create a new session
curl -X POST http://localhost:8000/api/sessions \
  -H "Content-Type: application/json" \
  -d '{"user_id": "optional-user-id"}'

# Send a message
curl -X POST http://localhost:8000/api/sessions/{session_id}/messages \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Your question here",
    "selected_text_context": "Optional selected text context"
  }'

# Get session history
curl -X GET http://localhost:8000/api/sessions/{session_id}/messages
```

## Docusaurus Integration

### 1. Install the Chatbot Component

```bash
# Copy the component to your Docusaurus project
cp frontend/src/components/Chatbot/* docs/src/components/Chatbot/
```

### 2. Use the Component in Docusaurus Pages

```jsx
import Chatbot from '@site/src/components/Chatbot';

function MyPage() {
  return (
    <div>
      <h1>My Content</h1>
      <p>Some content here...</p>

      <Chatbot
        sessionId="unique-session-id"
        apiUrl="http://localhost:8000/api"
      />
    </div>
  );
}
```

## Configuration

### Backend Configuration (.env)

```bash
# Database Configuration
DATABASE_URL=postgresql://user:password@localhost:5432/chatbot_db

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key

# Ollama Configuration
OLLAMA_HOST=http://localhost:11434

# LLM Configuration
LLM_PROVIDER=gemini
LLM_MODEL=gemini-2.0-flash
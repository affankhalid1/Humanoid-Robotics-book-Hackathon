# Physical AI Humanoid Robotics Book

A comprehensive guide to building humanoid robots with AI capabilities.

## Overview

This repository contains the documentation and code examples for the Physical AI Humanoid Robotics book. The book teaches how to build intelligent humanoid robots using modern AI techniques and robotic frameworks.

## Structure

- `docs/` - Book content in Markdown format organized by modules
- `module-01-ros2/` - ROS 2 code examples
- `module-02-simulation/` - Simulation code examples
- `module-03-isaac/` - Isaac ROS code examples
- `module-04-vla/` - Vision-Language-Action code examples
- `docker/` - Containerized development environments
- `scripts/` - Utility scripts for testing and validation

## Modules

1. **Module 1 - ROS 2 Fundamentals**: Learn the Robot Operating System (ROS 2) framework
2. **Module 2 - Simulation**: Master robot simulation techniques using Gazebo and Unity
3. **Module 3 - Isaac ROS**: Explore NVIDIA's Isaac ecosystem for AI perception
4. **Module 4 - Vision-Language-Action (VLA)**: Implement voice-language-action systems

## Prerequisites

- Basic Python programming knowledge
- Understanding of Linux command line
- Familiarity with version control systems (Git)
- Minimum hardware requirements:
  - 8+ core CPU
  - 16GB+ RAM
  - NVIDIA GPU with 8GB+ VRAM (for Isaac operations)

## Getting Started

1. Clone the repository
2. Follow the setup instructions in the Introduction module
3. Work through the modules sequentially

## Contributing

See CONTRIBUTING.md for contribution guidelines.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## RAG Chatbot Integration

The Physical AI Humanoid Robotics book now includes an integrated RAG (Retrieval-Augmented Generation) chatbot that allows readers to ask questions about the book content and receive contextual answers with proper citations.

### Features

- **Interactive Q&A**: Ask questions about the Physical AI Humanoid Robotics book content
- **Citation Tracking**: Responses include proper citations to book chapters and modules
- **Text Selection Context**: Ask questions about selected text on the page
- **Docusaurus Integration**: Seamless integration with the book's documentation site
- **Code and Prose Differentiation**: Handles both code snippets and prose content differently
- **Session Management**: Persistent chat sessions with history

### Architecture

The chatbot system consists of:

- **Frontend**: React components integrated into Docusaurus
- **Backend**: FastAPI server with PostgreSQL, Qdrant vector store
- **LLM Integration**: Google Gemini 2.0 Flash for responses
- **Embeddings**: Local Ollama embeddings (bge-m3 for prose, jina-code-embeddings for code)
- **Content Ingestion**: Parses Markdown and code files from the book

### Setup Instructions

#### Prerequisites

- Python 3.10+
- Node.js (for Docusaurus integration)
- Ollama (for local embeddings)
- PostgreSQL server
- Qdrant vector database
- Google Gemini API key

#### Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create environment file:
   ```bash
   cp .env.example .env
   ```

4. Update `.env` with your configuration:
   ```env
   # Database
   DATABASE_URL=postgresql://username:password@localhost/dbname

   # Qdrant
   QDRANT_HOST=localhost
   QDRANT_PORT=6333
   QDRANT_API_KEY=your_api_key
   QDRANT_COLLECTION_NAME=book_content

   # LLM
   LLM_PROVIDER=gemini
   LLM_MODEL=gemini-2.0-flash
   GEMINI_API_KEY=your_gemini_api_key

   # App
   APP_HOST=0.0.0.0
   APP_PORT=8000
   APP_DEBUG=true
   ```

#### Content Ingestion

1. Run the content ingestion pipeline:
   ```bash
   cd ingestion/scripts
   python parse_docs.py --docs-path ../docs
   python embed_content.py --chunks-path ../data/chunks.json
   python qdrant_loader.py --chunks-path ../data/embeddings.json
   ```

#### Running the Application

1. Start the backend server:
   ```bash
   cd backend
   python -m uvicorn src.main:app --host 0.0.0.0 --port 8000
   ```

### API Endpoints

#### Sessions

- `POST /api/sessions` - Create a new chat session
- `GET /api/sessions/{session_id}` - Get session details

#### Messages

- `POST /api/sessions/{session_id}/messages` - Send a message and get response
- `GET /api/sessions/{session_id}/messages` - Get message history

### Environment Variables

#### Backend (.env)

- `DATABASE_URL` - PostgreSQL connection string
- `QDRANT_HOST` - Qdrant server host
- `QDRANT_PORT` - Qdrant server port
- `QDRANT_API_KEY` - Qdrant API key (if required)
- `QDRANT_COLLECTION_NAME` - Qdrant collection name
- `LLM_PROVIDER` - LLM provider (currently supports "gemini")
- `LLM_MODEL` - LLM model name
- `GEMINI_API_KEY` - Google Gemini API key
- `APP_HOST` - Application host
- `APP_PORT` - Application port
- `APP_DEBUG` - Debug mode (true/false)

### Troubleshooting

#### Common Issues

1. **Ollama not responding**: Ensure Ollama is running and the required models are pulled:
   ```bash
   ollama serve
   ollama pull bge-m3:latest
   ollama pull jina-code-embeddings-0.5b
   ```

2. **Qdrant connection issues**: Verify Qdrant server is running and credentials are correct.

3. **Database connection errors**: Check PostgreSQL server is running and credentials are correct.
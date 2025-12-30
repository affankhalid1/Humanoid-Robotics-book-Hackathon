"""
Main application file for the RAG Chatbot
========================================

This is the main FastAPI application that serves as the entry point for the
RAG (Retrieval-Augmented Generation) chatbot system for the Physical AI
Humanoid Robotics book. The application provides API endpoints for:

- Session management (creating and retrieving chat sessions)
- Message management (sending and receiving messages)
- RAG functionality (retrieving relevant content from the book)
- Citation management (tracking and displaying sources)

The application uses:
- FastAPI for the web framework
- SQLAlchemy for database operations
- Qdrant for vector storage and similarity search
- Ollama for local embeddings
- Google Gemini for LLM responses
- PostgreSQL for session and message storage

Endpoints:
- /api/sessions - Manage chat sessions
- /api/sessions/{session_id}/messages - Manage messages within a session
- /health - Health check endpoint
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import asyncio

from .api.sessions import router as sessions_router
from .api.messages import router as messages_router
from .config import settings
from .vector_store.qdrant_client import QdrantClientWrapper
from .services.rag_service import rag_service


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for FastAPI application.

    This function handles application startup and shutdown events:

    Startup:
    - Initializes the vector store (Qdrant collection)
    - Ensures the required collection exists and is properly configured

    Shutdown:
    - Performs cleanup operations (currently none required)

    Args:
        app: The FastAPI application instance
    """
    # Startup
    print("Initializing vector store...")
    await rag_service.initialize_vector_store()
    print("Vector store initialized successfully!")

    yield

    # Shutdown
    print("Shutting down...")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG chatbot system for the Physical AI Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(sessions_router)
app.include_router(messages_router)


@app.get("/")
async def root():
    """
    Root endpoint for health check
    """
    return {"message": "RAG Chatbot API is running!"}


@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "version": "1.0.0"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=settings.app_host,
        port=settings.app_port,
        reload=True if settings.app_debug else False
    )
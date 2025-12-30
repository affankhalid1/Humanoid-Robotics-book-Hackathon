import os
from typing import Optional
from pydantic import BaseSettings


class Settings(BaseSettings):
    # Database Configuration
    database_url: str = os.getenv("DATABASE_URL", "postgresql://username:password@localhost/chatbot_db")

    # Qdrant Configuration
    qdrant_host: str = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port: int = int(os.getenv("QDRANT_PORT", "6333"))
    qdrant_api_key: Optional[str] = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

    # Ollama Configuration
    ollama_host: str = os.getenv("OLLAMA_HOST", "http://localhost:11434")

    # LLM Configuration
    llm_provider: str = os.getenv("LLM_PROVIDER", "gemini")
    llm_model: str = os.getenv("LLM_MODEL", "gemini-2.0-flash")
    gemini_api_key: Optional[str] = os.getenv("GEMINI_API_KEY")

    # Application Configuration
    app_env: str = os.getenv("APP_ENV", "development")
    app_debug: bool = os.getenv("APP_DEBUG", "true").lower() == "true"
    app_host: str = os.getenv("APP_HOST", "0.0.0.0")
    app_port: int = int(os.getenv("APP_PORT", "8000"))
    secret_key: str = os.getenv("SECRET_KEY", "your-secret-key-here")

    # Session Configuration
    session_expiry_hours: int = int(os.getenv("SESSION_EXPIRY_HOURS", "24"))

    class Config:
        env_file = ".env"


settings = Settings()
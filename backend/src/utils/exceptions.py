"""
Custom exceptions for the RAG Chatbot application
"""


class ChatbotError(Exception):
    """Base exception for the chatbot application"""
    def __init__(self, message: str, error_code: str = None, details: dict = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.details = details or {}


class ValidationError(ChatbotError):
    """Exception raised when validation fails"""
    def __init__(self, message: str, field: str = None, details: dict = None):
        super().__init__(message, "VALIDATION_ERROR", details)
        self.field = field


class DatabaseError(ChatbotError):
    """Exception raised when database operations fail"""
    def __init__(self, message: str, details: dict = None):
        super().__init__(message, "DATABASE_ERROR", details)


class VectorStoreError(ChatbotError):
    """Exception raised when vector store operations fail"""
    def __init__(self, message: str, details: dict = None):
        super().__init__(message, "VECTOR_STORE_ERROR", details)


class LLMError(ChatbotError):
    """Exception raised when LLM operations fail"""
    def __init__(self, message: str, details: dict = None):
        super().__init__(message, "LLM_ERROR", details)
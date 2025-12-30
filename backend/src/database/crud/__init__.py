"""
CRUD operations for all entities
"""

from .chat_session import (
    create_chat_session,
    get_chat_session,
    update_chat_session,
    delete_chat_session
)

from .message import (
    create_message,
    get_message,
    get_messages_by_session,
    update_message,
    delete_message
)

from .knowledge_chunk import (
    create_knowledge_chunk,
    get_knowledge_chunk,
    get_knowledge_chunks,
    update_knowledge_chunk,
    delete_knowledge_chunk
)

from .citation_reference import (
    create_citation_reference,
    get_citation_reference,
    get_citations_by_message,
    update_citation_reference,
    delete_citation_reference
)

from .text_selection_context import (
    create_text_selection_context,
    get_text_selection_context,
    update_text_selection_context,
    delete_text_selection_context
)

__all__ = [
    # ChatSession CRUD
    "create_chat_session",
    "get_chat_session",
    "update_chat_session",
    "delete_chat_session",

    # Message CRUD
    "create_message",
    "get_message",
    "get_messages_by_session",
    "update_message",
    "delete_message",

    # KnowledgeChunk CRUD
    "create_knowledge_chunk",
    "get_knowledge_chunk",
    "get_knowledge_chunks",
    "update_knowledge_chunk",
    "delete_knowledge_chunk",

    # CitationReference CRUD
    "create_citation_reference",
    "get_citation_reference",
    "get_citations_by_message",
    "update_citation_reference",
    "delete_citation_reference",

    # TextSelectionContext CRUD
    "create_text_selection_context",
    "get_text_selection_context",
    "update_text_selection_context",
    "delete_text_selection_context"
]
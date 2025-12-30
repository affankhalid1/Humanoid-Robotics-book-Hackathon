"""
RAG (Retrieval-Augmented Generation) Service for the RAG Chatbot
Handles content retrieval and similarity search
"""

from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
import numpy as np
from uuid import UUID

from ..vector_store.qdrant_client import QdrantClientWrapper
from ..utils import get_logger
from .text_selection_service import text_selection_service


class RAGService:
    """
    Service class for RAG (Retrieval-Augmented Generation) operations
    """
    def __init__(self):
        self.qdrant_client = QdrantClientWrapper()
        self.logger = get_logger(__name__)

    async def initialize_vector_store(self):
        """
        Initialize the vector store collection
        """
        await self.qdrant_client.initialize_collection()

    async def retrieve_relevant_content(
        self,
        query: str,
        limit: int = 5,
        content_types: Optional[List[str]] = None,
        chapter_names: Optional[List[str]] = None,
        selected_text_context: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve content relevant to the query

        Args:
            query: Query text to search for
            limit: Maximum number of results to return
            content_types: Optional list of content types to filter by ("prose", "code")
            chapter_names: Optional list of chapter names to filter by
            selected_text_context: Optional selected text context to prioritize in search

        Returns:
            List of relevant content chunks with metadata
        """
        try:
            # Enhance the query with selected text context if provided
            if selected_text_context:
                # Use the text selection service to enhance the query
                search_query = text_selection_service.enhance_query_with_context(query, selected_text_context)
                self.logger.info(f"Using enhanced search query with selected text context: {search_query[:100]}...")
            else:
                search_query = query

            # Get embedding for the search query
            query_embedding = await self._get_query_embedding(search_query)

            # Prepare filters
            filters = {}
            if content_types:
                filters["content_type"] = content_types
            if chapter_names:
                filters["chapter_name"] = chapter_names

            # Search for similar content in vector store
            results = await self.qdrant_client.search_similar(
                query_embedding=query_embedding,
                limit=limit,
                filters=filters
            )

            # If selected text context was provided, we might want to boost results that are similar
            # to the selected text, but for now we'll just return the results as is
            if selected_text_context:
                self.logger.info(f"Retrieved {len(results)} relevant content chunks for query with selected text context: {query[:50]}...")
            else:
                self.logger.info(f"Retrieved {len(results)} relevant content chunks for query: {query[:50]}...")

            return results

        except Exception as e:
            self.logger.error(f"Error retrieving relevant content: {str(e)}")
            raise

    async def _get_query_embedding(self, query: str) -> List[float]:
        """
        Get embedding for a query using Ollama.
        This is a simplified version - in a real implementation,
        you would call the Ollama API to generate embeddings.

        Args:
            query: Query text

        Returns:
            Embedding vector as a list of floats
        """
        # Placeholder implementation - in real scenario, call Ollama API
        # This is just for demonstration purposes
        import hashlib
        import struct

        # Create a deterministic "embedding" based on the hash of the query
        # In a real implementation, this would call the Ollama embedding API
        query_hash = hashlib.md5(query.encode()).hexdigest()

        # Convert hex hash to a vector of floats
        embedding = []
        for i in range(0, len(query_hash), 2):
            hex_pair = query_hash[i:i+2]
            float_val = struct.unpack('!f', bytes.fromhex(hex_pair.ljust(8, '0')[:8]))[0]
            embedding.append(float_val)

        # Normalize to reasonable embedding size (e.g., 1536)
        # This is just a placeholder to simulate real embeddings
        while len(embedding) < 1536:
            embedding.append(0.0)

        return embedding[:1536]

    async def retrieve_content_by_id(self, content_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve specific content by its ID

        Args:
            content_id: ID of the content to retrieve

        Returns:
            Content chunk with metadata or None if not found
        """
        try:
            result = await self.qdrant_client.get_by_id(content_id)
            if result:
                self.logger.debug(f"Retrieved content by ID: {content_id}")
            else:
                self.logger.warning(f"Content not found by ID: {content_id}")

            return result

        except Exception as e:
            self.logger.error(f"Error retrieving content by ID {content_id}: {str(e)}")
            raise

    async def search_by_filters(
        self,
        content_type: Optional[str] = None,
        chapter_name: Optional[str] = None,
        module_path: Optional[str] = None,
        limit: int = 10
    ) -> List[Dict[str, Any]]:
        """
        Search content by specific filters

        Args:
            content_type: Filter by content type ("prose", "code")
            chapter_name: Filter by chapter name
            module_path: Filter by module path
            limit: Maximum number of results

        Returns:
            List of content chunks matching the filters
        """
        try:
            filters = {}
            if content_type:
                filters["content_type"] = [content_type]
            if chapter_name:
                filters["chapter_name"] = [chapter_name]
            if module_path:
                filters["module_path"] = [module_path]

            # For this placeholder, we'll use a simple search
            # In a real implementation, this would use the proper filtering
            results = await self.qdrant_client.search_similar(
                query_embedding=[0.0] * 1536,  # Placeholder embedding
                limit=limit,
                filters=filters
            )

            self.logger.debug(f"Found {len(results)} results with filters: {filters}")
            return results

        except Exception as e:
            self.logger.error(f"Error searching with filters: {str(e)}")
            raise


# Global instance for easy access
rag_service = RAGService()
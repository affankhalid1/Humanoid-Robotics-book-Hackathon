"""
Citation Service for the RAG Chatbot
Handles creation and management of citation references
"""

from typing import List, Optional, Dict, Any
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime
from uuid import UUID
from sqlalchemy.future import select

from ..models.citation_reference import CitationReference
from ..models.knowledge_chunk import KnowledgeChunk
from ..database.crud import (
    create_citation_reference, get_citation_reference,
    get_citations_by_message, update_citation_reference, delete_citation_reference
)
from ..utils import get_logger


class CitationService:
    """
    Service class for managing citation references
    """
    def __init__(self):
        self.logger = get_logger(__name__)

    async def create_citation_reference(
        self,
        db: AsyncSession,
        session_id: str,
        message_id: str,
        knowledge_chunk_id: str,
        relevance_score: float
    ) -> CitationReference:
        """
        Create a new citation reference

        Args:
            db: Database session
            session_id: Session UUID as string
            message_id: Message UUID as string
            knowledge_chunk_id: Knowledge chunk UUID as string
            relevance_score: Relevance score (0.0 to 1.0)

        Returns:
            Created CitationReference object
        """
        try:
            # Validate relevance score
            if not 0.0 <= relevance_score <= 1.0:
                raise ValueError(f"Relevance score must be between 0.0 and 1.0, got {relevance_score}")

            # Create the citation reference
            citation = await create_citation_reference(
                db=db,
                session_id=session_id,
                message_id=message_id,
                knowledge_chunk_id=knowledge_chunk_id,
                relevance_score=relevance_score
            )

            self.logger.info(f"Created citation reference {citation.id} for message {message_id}")
            return citation

        except Exception as e:
            self.logger.error(f"Error creating citation reference: {str(e)}")
            raise

    async def create_citation_with_chunk_info(
        self,
        db: AsyncSession,
        session_id: str,
        message_id: str,
        knowledge_chunk_id: str,
        relevance_score: float
    ) -> Dict[str, Any]:
        """
        Create a citation reference and return enhanced citation info with chunk metadata

        Args:
            db: Database session
            session_id: Session UUID as string
            message_id: Message UUID as string
            knowledge_chunk_id: Knowledge chunk UUID as string
            relevance_score: Relevance score (0.0 to 1.0)

        Returns:
            Dictionary with citation information including chapter name and module path
        """
        try:
            # Create the citation reference
            citation = await create_citation_reference(
                db=db,
                session_id=session_id,
                message_id=message_id,
                knowledge_chunk_id=knowledge_chunk_id,
                relevance_score=relevance_score
            )

            # Fetch the knowledge chunk to get chapter name and module path
            stmt = select(KnowledgeChunk).where(KnowledgeChunk.id == UUID(knowledge_chunk_id))
            result = await db.execute(stmt)
            knowledge_chunk = result.scalar_one_or_none()

            if not knowledge_chunk:
                self.logger.warning(f"Knowledge chunk not found: {knowledge_chunk_id}")
                # Return basic citation info if knowledge chunk not found
                return {
                    "citation_id": str(citation.id),
                    "knowledge_chunk_id": knowledge_chunk_id,
                    "relevance_score": relevance_score,
                    "chapter_name": "Unknown",
                    "module_path": "Unknown",
                    "source_file": "Unknown"
                }

            # Return enhanced citation info
            citation_info = {
                "citation_id": str(citation.id),
                "knowledge_chunk_id": knowledge_chunk_id,
                "relevance_score": relevance_score,
                "chapter_name": knowledge_chunk.chapter_name or "Unknown",
                "module_path": knowledge_chunk.module_path or "Unknown",
                "source_file": knowledge_chunk.source_file or "Unknown",
                "content_type": knowledge_chunk.content_type,
                "source_line_start": knowledge_chunk.source_line_start,
                "source_line_end": knowledge_chunk.source_line_end
            }

            self.logger.info(f"Created enhanced citation reference {citation.id} for message {message_id}")
            return citation_info

        except Exception as e:
            self.logger.error(f"Error creating enhanced citation reference: {str(e)}")
            raise

    async def get_citations_with_chunk_info(
        self,
        db: AsyncSession,
        message_id: str
    ) -> List[Dict[str, Any]]:
        """
        Retrieve citations for a message with enhanced chunk information

        Args:
            db: Database session
            message_id: Message UUID as string

        Returns:
            List of citation dictionaries with enhanced information
        """
        try:
            # Get citations for the message
            citations = await self.get_citations_by_message(db, message_id)

            enhanced_citations = []
            for citation in citations:
                # Fetch the knowledge chunk to get chapter name and module path
                stmt = select(KnowledgeChunk).where(KnowledgeChunk.id == citation.knowledge_chunk_id)
                result = await db.execute(stmt)
                knowledge_chunk = result.scalar_one_or_none()

                if knowledge_chunk:
                    citation_info = {
                        "citation_id": str(citation.id),
                        "knowledge_chunk_id": str(citation.knowledge_chunk_id),
                        "relevance_score": citation.relevance_score,
                        "chapter_name": knowledge_chunk.chapter_name or "Unknown",
                        "module_path": knowledge_chunk.module_path or "Unknown",
                        "source_file": knowledge_chunk.source_file or "Unknown",
                        "content_type": knowledge_chunk.content_type,
                        "source_line_start": knowledge_chunk.source_line_start,
                        "source_line_end": knowledge_chunk.source_line_end
                    }
                else:
                    # Fallback if knowledge chunk not found
                    citation_info = {
                        "citation_id": str(citation.id),
                        "knowledge_chunk_id": str(citation.knowledge_chunk_id),
                        "relevance_score": citation.relevance_score,
                        "chapter_name": "Unknown",
                        "module_path": "Unknown",
                        "source_file": "Unknown",
                        "content_type": "Unknown",
                        "source_line_start": None,
                        "source_line_end": None
                    }

                enhanced_citations.append(citation_info)

            self.logger.debug(f"Retrieved {len(enhanced_citations)} enhanced citations for message {message_id}")
            return enhanced_citations

        except Exception as e:
            self.logger.error(f"Error retrieving enhanced citations for message {message_id}: {str(e)}")
            raise

    async def get_citation_reference(self, db: AsyncSession, citation_id: str) -> Optional[CitationReference]:
        """
        Retrieve a citation reference by ID

        Args:
            db: Database session
            citation_id: Citation UUID as string

        Returns:
            CitationReference object if found, None otherwise
        """
        try:
            citation = await get_citation_reference(db, citation_id)
            if citation:
                self.logger.debug(f"Retrieved citation reference: {citation.id}")
            else:
                self.logger.warning(f"Citation reference not found: {citation_id}")

            return citation

        except Exception as e:
            self.logger.error(f"Error retrieving citation reference {citation_id}: {str(e)}")
            raise

    async def get_citations_by_message(self, db: AsyncSession, message_id: str) -> List[CitationReference]:
        """
        Retrieve all citations for a message

        Args:
            db: Database session
            message_id: Message UUID as string

        Returns:
            List of CitationReference objects
        """
        try:
            citations = await get_citations_by_message(db, message_id)
            self.logger.debug(f"Retrieved {len(citations)} citations for message {message_id}")
            return citations

        except Exception as e:
            self.logger.error(f"Error retrieving citations for message {message_id}: {str(e)}")
            raise

    async def get_citations_by_session(self, db: AsyncSession, session_id: str) -> List[CitationReference]:
        """
        Retrieve all citations for a session

        Args:
            db: Database session
            session_id: Session UUID as string

        Returns:
            List of CitationReference objects
        """
        try:
            # This would require a custom query since the CRUD module doesn't have this function
            from sqlalchemy.future import select
            stmt = select(CitationReference).where(CitationReference.session_id == UUID(session_id))
            result = await db.execute(stmt)
            citations = result.scalars().all()

            self.logger.debug(f"Retrieved {len(citations)} citations for session {session_id}")
            return citations

        except Exception as e:
            self.logger.error(f"Error retrieving citations for session {session_id}: {str(e)}")
            raise


# Global instance for easy access
citation_service = CitationService()
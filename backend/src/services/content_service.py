"""
Content Service for the RAG Chatbot
Handles content differentiation logic (prose vs code)
"""

from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from ..models.knowledge_chunk import KnowledgeChunk
from ..database.crud import (
    create_knowledge_chunk, get_knowledge_chunk,
    get_knowledge_chunks, update_knowledge_chunk, delete_knowledge_chunk
)
from ..utils import get_logger
import re


class ContentService:
    """
    Service class for managing content differentiation logic
    """
    def __init__(self):
        self.logger = get_logger(__name__)

    def identify_content_type(self, content: str) -> str:
        """
        Identify whether content is prose or code based on patterns

        Args:
            content: Text content to analyze

        Returns:
            "prose" or "code"
        """
        # Check for common code patterns
        code_indicators = [
            r'\b(if|else|for|while|def|class|function|import|from|import|const|let|var)\b',
            r'(\{|\}|\[|\]|\(|\))',
            r'(\w+\s*=\s*|\w+\s*==\s*|\w+\s*!=\s*|\w+\s*<\s*|\w+\s*>)',
            r'(\/\/|#|\/\*|\*\/)',
            r'(\w+\.\w+)',
            r'(\w+\(\s*\w+\s*\))',
            r'(public|private|protected|static|final|abstract|interface|extends|implements)',
            r'(\d+\.\d+|\d+)',
        ]

        # Check for prose patterns
        prose_indicators = [
            r'([A-Z][a-z]+\s+){2,}',  # Multiple capitalized words
            r'[.!?]\s+[A-Z]',  # Sentence endings followed by capital letters
            r'(\w+ing\s|\w+ed\s)',  # Present/past participles
            r'(the|a|an|and|or|but|in|on|at|to|for|of|with|by)\s+',  # Common English words
        ]

        code_score = 0
        prose_score = 0

        for pattern in code_indicators:
            if re.search(pattern, content):
                code_score += 1

        for pattern in prose_indicators:
            if re.search(pattern, content):
                prose_score += 1

        # If there are clear code indicators, classify as code
        if code_score > prose_score and code_score > 0:
            return "code"
        else:
            return "prose"

    def extract_code_blocks(self, content: str) -> List[Dict[str, str]]:
        """
        Extract code blocks from content

        Args:
            content: Content that may contain code blocks

        Returns:
            List of dictionaries with code content and language info
        """
        # Pattern to match code blocks (```)
        pattern = r'```(\w*)\n(.*?)```'
        matches = re.findall(pattern, content, re.DOTALL)

        code_blocks = []
        for match in matches:
            language = match[0] if match[0] else "unknown"
            code = match[1].strip()
            code_blocks.append({
                "language": language,
                "content": code
            })

        return code_blocks

    def extract_prose_content(self, content: str) -> str:
        """
        Extract prose content by removing code blocks

        Args:
            content: Content that may contain code blocks

        Returns:
            Prose content with code blocks removed
        """
        # Remove code blocks (```)
        pattern = r'```(\w*)\n(.*?)```'
        prose_content = re.sub(pattern, '', content, flags=re.DOTALL)
        return prose_content.strip()

    async def create_knowledge_chunk(
        self,
        db: AsyncSession,
        content: str,
        content_type: Optional[str] = None,
        chapter_name: Optional[str] = None,
        module_path: Optional[str] = None,
        source_file: Optional[str] = None,
        source_line_start: Optional[int] = None,
        source_line_end: Optional[int] = None,
        metadata: Optional[dict] = None
    ) -> KnowledgeChunk:
        """
        Create a knowledge chunk with automatic content type detection

        Args:
            db: Database session
            content: Content to store
            content_type: Optional content type ("prose" or "code"), auto-detected if not provided
            chapter_name: Chapter name
            module_path: Module path
            source_file: Source file
            source_line_start: Start line in source file
            source_line_end: End line in source file
            metadata: Additional metadata

        Returns:
            Created KnowledgeChunk object
        """
        try:
            # Auto-detect content type if not provided
            if content_type is None:
                content_type = self.identify_content_type(content)

            # Validate content type
            if content_type not in ["prose", "code"]:
                raise ValueError(f"Invalid content type: {content_type}. Must be 'prose' or 'code'")

            # Create the knowledge chunk
            chunk = await create_knowledge_chunk(
                db=db,
                content=content,
                content_type=content_type,
                chapter_name=chapter_name,
                module_path=module_path,
                source_file=source_file,
                source_line_start=source_line_start,
                source_line_end=source_line_end,
                metadata=metadata or {}
            )

            self.logger.info(f"Created knowledge chunk {chunk.id} with type '{content_type}'")
            return chunk

        except Exception as e:
            self.logger.error(f"Error creating knowledge chunk: {str(e)}")
            raise

    async def process_content_for_storage(self, content: str, **kwargs) -> List[KnowledgeChunk]:
        """
        Process content for storage by separating prose and code blocks

        Args:
            content: Content to process
            **kwargs: Additional arguments to pass to create_knowledge_chunk

        Returns:
            List of created KnowledgeChunk objects
        """
        chunks = []

        # Extract code blocks
        code_blocks = self.extract_code_blocks(content)

        # Create chunks for each code block
        for block in code_blocks:
            code_chunk = await self.create_knowledge_chunk(
                db=kwargs.get('db'),
                content=block['content'],
                content_type='code',
                **{k: v for k, v in kwargs.items() if k != 'db'}
            )
            chunks.append(code_chunk)

        # Extract and create chunk for prose content
        prose_content = self.extract_prose_content(content)
        if prose_content.strip():
            prose_chunk = await self.create_knowledge_chunk(
                db=kwargs.get('db'),
                content=prose_content,
                content_type='prose',
                **{k: v for k, v in kwargs.items() if k != 'db'}
            )
            chunks.append(prose_chunk)

        return chunks


# Global instance for easy access
content_service = ContentService()
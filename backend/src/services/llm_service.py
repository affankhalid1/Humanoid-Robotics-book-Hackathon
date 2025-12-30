"""
LLM Integration Service for the RAG Chatbot
Handles communication with the LLM provider (Google Gemini)
"""

import asyncio
from typing import List, Dict, Any, Optional
from openai import AsyncOpenAI
from ..config import settings
from ..utils import get_logger


class LLMService:
    """
    Service class for interacting with the LLM provider
    """
    def __init__(self):
        self.logger = get_logger(__name__)
        self.provider = settings.llm_provider
        self.model = settings.llm_model
        self.api_key = settings.gemini_api_key

        # Initialize client based on provider
        if self.provider == "gemini":
            # Using OpenAI compatible interface for Gemini
            self.client = AsyncOpenAI(
                api_key=self.api_key,
                base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
            )
        else:
            raise ValueError(f"Unsupported LLM provider: {self.provider}")

    async def generate_response(
        self,
        prompt: str,
        context: Optional[List[Dict[str, str]]] = None,
        selected_text_context: Optional[str] = None
    ) -> str:
        """
        Generate a response from the LLM based on the prompt and context

        Args:
            prompt: User's question or prompt
            context: List of context chunks retrieved from the vector store
            selected_text_context: Optional text selection context

        Returns:
            Generated response from the LLM
        """
        try:
            # Build the full prompt with context
            full_prompt = self._build_prompt(prompt, context, selected_text_context)

            # Create messages for the API call
            messages = [
                {
                    "role": "system",
                    "content": "You are a Robotics Expert Thought Partner. Provide insightful, accurate answers derived from the provided book content. Always include proper citations to specific chapters, sections, or code examples when available."
                },
                {
                    "role": "user",
                    "content": full_prompt
                }
            ]

            # Call the LLM
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )

            # Extract and return the response
            llm_response = response.choices[0].message.content
            self.logger.info(f"Generated LLM response for prompt: {prompt[:50]}...")
            return llm_response

        except Exception as e:
            self.logger.error(f"Error generating LLM response: {str(e)}")
            raise

    async def generate_response_with_citations(
        self,
        prompt: str,
        context: Optional[List[Dict[str, str]]] = None,
        selected_text_context: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate a response from the LLM with citation information

        Args:
            prompt: User's question or prompt
            context: List of context chunks retrieved from the vector store
            selected_text_context: Optional text selection context

        Returns:
            Dictionary containing the response and citation information
        """
        try:
            # Build the full prompt with context
            full_prompt = self._build_prompt(prompt, context, selected_text_context)

            # Create messages for the API call
            messages = [
                {
                    "role": "system",
                    "content": "You are a Robotics Expert Thought Partner. Provide insightful, accurate answers derived from the provided book content. Always include proper citations to specific chapters, sections, or code examples when available. Format your response to indicate which parts come from which sources."
                },
                {
                    "role": "user",
                    "content": full_prompt
                }
            ]

            # Call the LLM
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )

            # Extract the response
            llm_response = response.choices[0].message.content

            # Extract citation information from the response
            citations = self._extract_citations(llm_response, context)

            result = {
                "response": llm_response,
                "citations": citations
            }

            self.logger.info(f"Generated LLM response with citations for prompt: {prompt[:50]}...")
            return result

        except Exception as e:
            self.logger.error(f"Error generating LLM response with citations: {str(e)}")
            raise

    def _build_prompt(
        self,
        prompt: str,
        context: Optional[List[Dict[str, str]]] = None,
        selected_text_context: Optional[str] = None
    ) -> str:
        """
        Build the full prompt with context and selected text

        Args:
            prompt: User's question or prompt
            context: List of context chunks retrieved from the vector store
            selected_text_context: Optional text selection context

        Returns:
            Formatted prompt string
        """
        full_prompt = ""

        # Add selected text context if provided
        if selected_text_context:
            full_prompt += f"User has selected the following text for context: {selected_text_context}\n\n"
            full_prompt += "Please focus your answer on this selected text and provide specific citations to relevant sections.\n\n"

        # Add retrieved context
        if context:
            full_prompt += "Relevant content from the book:\n"
            for i, chunk in enumerate(context):
                content = chunk.get('content', '')[:500]  # Limit context length
                metadata = chunk.get('metadata', {})
                source = metadata.get('chapter_name', 'Unknown')
                module_path = metadata.get('module_path', 'Unknown')
                content_type = metadata.get('content_type', 'Unknown')

                full_prompt += f"{i+1}. From {source} ({content_type} content"
                if module_path != 'Unknown':
                    full_prompt += f", module: {module_path}"
                full_prompt += f"): {content}\n\n"

        # Add user's original prompt
        full_prompt += f"Question: {prompt}\n\n"
        full_prompt += "Please provide an accurate, contextual answer based on the above information, including proper citations to the book content. If the user selected specific text, focus your response on that text and provide specific references."

        return full_prompt

    def _extract_citations(
        self,
        response: str,
        context: Optional[List[Dict[str, str]]] = None
    ) -> List[Dict[str, Any]]:
        """
        Extract citation information from the LLM response

        Args:
            response: LLM response text
            context: List of context chunks that were used

        Returns:
            List of citation dictionaries
        """
        citations = []

        if context:
            # Create citations for each context chunk that was used
            # We'll create citations based on the context chunks that were provided to the LLM
            for i, chunk in enumerate(context):
                # Extract the metadata for the citation
                metadata = chunk.get('metadata', {})
                citation = {
                    "chunk_id": chunk.get('id', f"context_chunk_{i}"),  # Use index if no ID provided
                    "chapter_name": metadata.get('chapter_name', 'Unknown'),
                    "module_path": metadata.get('module_path', 'Unknown'),
                    "relevance_score": chunk.get('score', 0.0),
                    "source_file": metadata.get('source_file', 'Unknown'),
                    "source_line_start": metadata.get('source_line_start', None),
                    "source_line_end": metadata.get('source_line_end', None),
                    "content_type": metadata.get('content_type', 'Unknown'),
                    "source_url": self._generate_source_url(metadata),
                    "source_text": chunk.get('content', '')[:200] + "..."
                }
                citations.append(citation)

        return citations

    def _generate_source_url(self, metadata: Dict[str, Any]) -> str:
        """
        Generate a source URL based on the metadata

        Args:
            metadata: Metadata dictionary containing source information

        Returns:
            Generated source URL
        """
        chapter_name = metadata.get('chapter_name', 'Unknown')
        module_path = metadata.get('module_path', 'Unknown')
        source_file = metadata.get('source_file', 'Unknown')

        # If we have a chapter name, use it to generate a URL
        if chapter_name and chapter_name != 'Unknown':
            # Convert to URL-friendly format
            url_part = chapter_name.replace(' ', '-').replace('_', '-').lower()
            return f"/docs/{url_part}"

        # If we have a module path, use it
        elif module_path and module_path != 'Unknown':
            url_part = module_path.replace(' ', '-').replace('_', '-').replace('.', '/').lower()
            return f"/docs/{url_part}"

        # If we have a source file, try to create a URL from it
        elif source_file and source_file != 'Unknown':
            # Extract the filename without extension
            import os
            filename = os.path.splitext(os.path.basename(source_file))[0]
            url_part = filename.replace(' ', '-').replace('_', '-').lower()
            return f"/docs/{url_part}"

        # Default fallback
        return "/docs"


# Global instance for easy access
llm_service = LLMService()
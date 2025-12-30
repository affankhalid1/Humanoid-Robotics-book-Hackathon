"""
Text Selection Context Service for the RAG Chatbot
Handles processing and management of selected text context
"""

from typing import Optional, Dict, Any, List
import re
from ..utils import get_logger


class TextSelectionService:
    """
    Service class for handling text selection context
    """
    def __init__(self):
        self.logger = get_logger(__name__)

    def process_selected_text(self, selected_text: str) -> Dict[str, Any]:
        """
        Process selected text to extract useful context information

        Args:
            selected_text: The text that was selected by the user

        Returns:
            Dictionary containing processed text and metadata
        """
        if not selected_text or not selected_text.strip():
            return {
                "original_text": "",
                "cleaned_text": "",
                "word_count": 0,
                "character_count": 0,
                "is_code_snippet": False,
                "key_terms": [],
                "context_type": "empty"
            }

        # Clean the text (remove extra whitespace, normalize)
        cleaned_text = self._clean_text(selected_text)

        # Analyze the text to determine its characteristics
        word_count = len(cleaned_text.split())
        character_count = len(cleaned_text)
        is_code_snippet = self._is_code_snippet(cleaned_text)
        key_terms = self._extract_key_terms(cleaned_text)
        context_type = self._determine_context_type(cleaned_text, is_code_snippet)

        result = {
            "original_text": selected_text,
            "cleaned_text": cleaned_text,
            "word_count": word_count,
            "character_count": character_count,
            "is_code_snippet": is_code_snippet,
            "key_terms": key_terms,
            "context_type": context_type
        }

        self.logger.debug(f"Processed selected text: {cleaned_text[:100]}... (type: {context_type})")
        return result

    def _clean_text(self, text: str) -> str:
        """
        Clean and normalize the selected text

        Args:
            text: Raw selected text

        Returns:
            Cleaned and normalized text
        """
        # Remove extra whitespace and normalize line breaks
        cleaned = re.sub(r'\s+', ' ', text.strip())
        return cleaned

    def _is_code_snippet(self, text: str) -> bool:
        """
        Determine if the selected text is likely a code snippet

        Args:
            text: Text to analyze

        Returns:
            True if the text appears to be code, False otherwise
        """
        # Look for common code patterns
        code_indicators = [
            r'\b(def|class|import|from|return|if|else|elif|for|while|try|except|with|async|await)\b',  # Python keywords
            r'\{.*\}',  # Curly braces
            r'\(.*\)',  # Parentheses
            r'\[.*\]',  # Square brackets
            r'=\s*[^,;]*',  # Assignment patterns
            r'//|#|/\*|\*/',  # Comment patterns
            r';',  # Semicolons
            r'\.',  # Dots (method calls)
        ]

        text_lower = text.lower()
        for pattern in code_indicators:
            if re.search(pattern, text_lower):
                # Additional check: if there are more special characters than words, it's likely code
                special_char_ratio = len(re.findall(r'[^\w\s]', text)) / len(text) if text else 0
                word_ratio = len(text.split()) / len(text) if text else 0
                if special_char_ratio > 0.1 or word_ratio < 0.1:  # Adjusted threshold
                    return True

        return False

    def _extract_key_terms(self, text: str) -> List[str]:
        """
        Extract key terms from the selected text

        Args:
            text: Text to extract terms from

        Returns:
            List of key terms
        """
        # Simple key term extraction - could be enhanced with NLP techniques
        words = re.findall(r'\b\w+\b', text.lower())
        # Filter out common stop words and return unique terms
        stop_words = {'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by', 'is', 'are', 'was', 'were', 'be', 'been', 'being', 'have', 'has', 'had', 'do', 'does', 'did', 'will', 'would', 'could', 'should', 'may', 'might', 'must', 'can', 'this', 'that', 'these', 'those', 'i', 'you', 'he', 'she', 'it', 'we', 'they', 'me', 'him', 'her', 'us', 'them', 'my', 'your', 'his', 'its', 'our', 'their', 'myself', 'yourself', 'himself', 'herself', 'itself', 'ourselves', 'yourselves', 'themselves'}
        key_terms = [word for word in set(words) if len(word) > 2 and word not in stop_words]
        return key_terms[:10]  # Limit to top 10 terms

    def _determine_context_type(self, text: str, is_code: bool) -> str:
        """
        Determine the type of context in the selected text

        Args:
            text: Text to analyze
            is_code: Whether the text appears to be code

        Returns:
            Context type (e.g., 'code', 'prose', 'configuration', 'documentation')
        """
        if is_code:
            return 'code'
        elif len(text) < 50:
            # Likely a short phrase or sentence
            return 'short_text'
        else:
            # Assume it's prose/documentation
            return 'prose'

    def enhance_query_with_context(self, query: str, selected_text_context: Optional[str]) -> str:
        """
        Enhance a query with selected text context

        Args:
            query: Original user query
            selected_text_context: Selected text context

        Returns:
            Enhanced query string
        """
        if not selected_text_context or not selected_text_context.strip():
            return query

        # Process the selected text
        processed_context = self.process_selected_text(selected_text_context)

        # Enhance the query based on the context
        if processed_context['context_type'] == 'code':
            enhanced_query = f"Regarding the code: '{processed_context['cleaned_text'][:200]}...', {query}"
        else:
            enhanced_query = f"Regarding the text: '{processed_context['cleaned_text'][:200]}...', {query}"

        self.logger.info(f"Enhanced query with selected text context: {enhanced_query[:100]}...")
        return enhanced_query


# Global instance for easy access
text_selection_service = TextSelectionService()
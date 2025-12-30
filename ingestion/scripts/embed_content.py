#!/usr/bin/env python3
"""
Embedding functionality for the RAG Chatbot.
Implements embedding for both prose and code content using different models.
"""

import asyncio
import json
import requests
from typing import List, Dict, Any
from pathlib import Path
import numpy as np
from .parse_docs import ContentParser


class ContentEmbedder:
    """
    Embedder for content chunks using different models for prose and code.
    """

    def __init__(self, ollama_host: str = "http://localhost:11434"):
        self.ollama_host = ollama_host
        self.session = requests.Session()

    def _call_ollama_embedding(self, text: str, model: str) -> List[float]:
        """
        Call Ollama API to generate embeddings for the given text.

        Args:
            text: Text to embed
            model: Ollama model to use

        Returns:
            Embedding vector as a list of floats
        """
        url = f"{self.ollama_host}/api/embeddings"
        payload = {
            "model": model,
            "prompt": text
        }

        response = self.session.post(url, json=payload)
        response.raise_for_status()

        result = response.json()
        return result.get("embedding", [])

    def embed_prose_content(self, content: str) -> List[float]:
        """
        Embed prose content using the bge-m3 model.

        Args:
            content: Prose content to embed

        Returns:
            Embedding vector as a list of floats
        """
        # Use bge-m3 model for prose content
        return self._call_ollama_embedding(content, "bge-m3:latest")

    def embed_code_content(self, content: str) -> List[float]:
        """
        Embed code content using the jina-code-embeddings model.

        Args:
            content: Code content to embed

        Returns:
            Embedding vector as a list of floats
        """
        # Use jina-code-embeddings model for code content
        return self._call_ollama_embedding(content, "jina-code-embeddings-0.5b:latest")

    def embed_content_chunk(self, chunk: Dict[str, Any]) -> Dict[str, Any]:
        """
        Embed a content chunk based on its type.

        Args:
            chunk: Content chunk with metadata

        Returns:
            Chunk with embedding added
        """
        content_type = chunk.get('content_type', 'prose')
        content = chunk.get('content', '')

        if not content.strip():
            chunk['embedding'] = []
            return chunk

        # Embed content based on type
        if content_type == 'code':
            embedding = self.embed_code_content(content)
        else:  # prose or other content types
            embedding = self.embed_prose_content(content)

        chunk['embedding'] = embedding
        return chunk

    def embed_content_chunks(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Embed a list of content chunks.

        Args:
            chunks: List of content chunks to embed

        Returns:
            List of content chunks with embeddings
        """
        embedded_chunks = []
        for chunk in chunks:
            embedded_chunk = self.embed_content_chunk(chunk)
            embedded_chunks.append(embedded_chunk)

        return embedded_chunks


def main():
    """
    Main function to run the content embedder.
    """
    import argparse

    parser = argparse.ArgumentParser(description='Embed content chunks for RAG Chatbot')
    parser.add_argument('--input-path', required=True, help='Path to input JSON file with parsed content')
    parser.add_argument('--output-path', required=True, help='Path to output directory')

    args = parser.parse_args()

    # Load parsed content
    with open(args.input_path, 'r', encoding='utf-8') as f:
        chunks = json.load(f)

    # Initialize embedder
    embedder = ContentEmbedder()

    # Embed all chunks
    embedded_chunks = embedder.embed_content_chunks(chunks)

    # Create output directory if it doesn't exist
    output_dir = Path(args.output_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save embedded chunks to a JSON file
    output_file = output_dir / 'embedded_chunks.json'
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(embedded_chunks, f, indent=2, ensure_ascii=False)

    print(f"Embedded {len(embedded_chunks)} content chunks")
    print(f"Output saved to {output_file}")


if __name__ == "__main__":
    main()
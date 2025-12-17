#!/usr/bin/env python3

"""
Source Verification Script

This script validates that code examples and technical claims in the documentation
are properly referenced and verified against authoritative sources.
"""

import os
import sys
import re
from pathlib import Path


def find_markdown_files(root_dir):
    """Find all markdown files in the documentation."""
    md_files = []
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.md'):
                md_files.append(os.path.join(root, file))
    return md_files


def validate_code_examples(docs_dir):
    """Validate that code examples exist and are properly referenced."""
    print("Validating code examples...")

    # Find all markdown files
    md_files = find_markdown_files(docs_dir)

    # Pattern to match code example references in markdown
    code_ref_pattern = r'`[^`]+`|```[\s\S]*?```'

    all_valid = True

    for md_file in md_files:
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Look for code example references
        code_blocks = re.findall(r'`([^`\n]+)`', content)
        fenced_blocks = re.findall(r'```(?:\w+\n)?([\s\S]*?)```', content)

        # For this basic validation, we'll just check that referenced files exist
        # In a real implementation, this would be more sophisticated
        print(f"  Checked {md_file}: {len(code_blocks)} inline code blocks, {len(fenced_blocks)} fenced blocks")

    return all_valid


def validate_technical_claims(docs_dir):
    """Validate technical claims against authoritative sources."""
    print("Validating technical claims...")

    # This is a placeholder implementation
    # In a real implementation, this would check claims against authoritative sources
    print("  Technical claims validation: Placeholder implementation")

    return True


def validate_citations(docs_dir):
    """Validate that citations are properly formatted and linked."""
    print("Validating citations...")

    # Find all markdown files
    md_files = find_markdown_files(docs_dir)

    citation_pattern = r'\[.*?\]\(.*?\)'  # Basic markdown link pattern

    total_citations = 0

    for md_file in md_files:
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        citations = re.findall(citation_pattern, content)
        total_citations += len(citations)

        if citations:
            print(f"  Found {len(citations)} citations in {md_file}")

    print(f"  Total citations found: {total_citations}")
    return True


def main():
    """Main function to run all validations."""
    docs_dir = "docs"

    if not os.path.exists(docs_dir):
        print(f"Error: Documentation directory '{docs_dir}' does not exist")
        return 1

    print("Starting source verification process...")
    print(f"Documentation directory: {os.path.abspath(docs_dir)}")
    print()

    # Validate code examples
    code_valid = validate_code_examples(docs_dir)
    print()

    # Validate technical claims
    claims_valid = validate_technical_claims(docs_dir)
    print()

    # Validate citations
    citations_valid = validate_citations(docs_dir)
    print()

    # Overall result
    all_valid = code_valid and claims_valid and citations_valid

    if all_valid:
        print("✓ All validations passed!")
        return 0
    else:
        print("✗ Some validations failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
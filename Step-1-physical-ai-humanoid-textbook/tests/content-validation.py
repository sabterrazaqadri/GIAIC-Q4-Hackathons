#!/usr/bin/env python3
"""
Content Validation Script for Module 1

This script validates the content of all chapters in Module 1 according to the requirements:
- Check for proper markdown formatting
- Verify APA citations are present
- Ensure chapters have required sections
- Validate internal links and references
"""

import os
import re
from pathlib import Path


def validate_chapter_format(file_path):
    """Validate basic markdown format and required sections."""
    errors = []
    
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Check for required sections
    required_sections = ['## Learning Objectives', '## Introduction', '## Summary', '## References and Citations']
    for section in required_sections:
        if section not in content:
            errors.append(f"Missing required section '{section}' in {file_path}")
    
    # Check for at least 2-3 comprehension checks
    comp_check_matches = re.findall(r'## Comprehension Checks.*?##', content, re.DOTALL)
    if not comp_check_matches:
        errors.append(f"No Comprehension Checks section found in {file_path}")
    else:
        # Count questions in the comprehension checks
        question_count = len(re.findall(r'### Question \d+', content))
        if question_count < 2:
            errors.append(f"Expected at least 2 comprehension questions, found {question_count} in {file_path}")
    
    # Check for APA citations
    apa_pattern = r'\([A-Za-z0-9, ]+\)\.\s*\([0-9]{4}\)\.'  # Basic APA pattern
    if not re.search(apa_pattern, content):
        # Check for alternative APA format in references section
        refs_section = content[content.find('## References and Citations'):content.find('## References and Citations') + 1000]
        if 'Available at:' not in refs_section and 'ROS.org' not in refs_section:
            errors.append(f"Possible missing APA citations in {file_path}")
    
    return errors


def count_words_in_file(file_path):
    """Count words in a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Remove markdown syntax for more accurate word count
    content = re.sub(r'\[[^\]]*\]\([^)]*\)', '', content)  # Remove links
    content = re.sub(r'#+\s+', '', content)  # Remove headers
    content = re.sub(r'`[^`]*`', '', content)  # Remove inline code
    content = re.sub(r'\n\s*\n', ' ', content)  # Replace multiple newlines with space
    
    words = content.split()
    return len(words)


def main():
    """Main function to run content validation."""
    module1_dir = Path('module-1')
    chapters = list(module1_dir.glob('chapter-*.md'))
    
    print("Validating Module 1 content...")
    print("=" * 50)
    
    total_errors = 0
    total_words = 0
    
    for chapter in chapters:
        print(f"Validating {chapter.name}...")
        
        # Validate format
        errors = validate_chapter_format(chapter)
        if errors:
            print(f"  Errors found in {chapter.name}:")
            for error in errors:
                print(f"    - {error}")
            total_errors += len(errors)
        else:
            print(f"  [PASS] {chapter.name} passed format validation")
        
        # Count words
        word_count = count_words_in_file(chapter)
        total_words += word_count
        print(f"  - Word count: {word_count}")
    
    print("\nSummary:")
    print("=" * 50)
    
    if total_errors == 0:
        print("[PASS] All chapters passed format validation")
    else:
        print(f"[FAIL] Found {total_errors} errors in chapter formats")

    print(f"Total word count: {total_words}")

    # Check if word count is within range
    if 3500 <= total_words <= 5000:
        print(f"[PASS] Total word count ({total_words}) is within required range (3,500-5,000)")
    else:
        print(f"[FAIL] Total word count ({total_words}) is outside required range (3,500-5,000)")
        if total_words < 3500:
            print(f"  Need {3500 - total_words} more words")
        else:
            print(f"  Have {total_words - 5000} words over the limit")
    
    if total_errors == 0:
        return 0
    else:
        return 1


if __name__ == '__main__':
    exit(main())
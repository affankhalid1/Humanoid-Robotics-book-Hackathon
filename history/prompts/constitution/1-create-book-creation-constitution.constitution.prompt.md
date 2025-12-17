---
id: 1
title: Create Book Creation Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,book,documentation]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - null
---

## Prompt

```text
Constitution Prompt for AI/Spec-Driven Book Creation
Project Overview
Project Type: Technical Book Development using Docusaurus
Deployment Target: GitHub Pages
Tools: Spec-Kit Plus + Claude Code
Format: Web-based documentation (Docusaurus static site)

Core Principles
1. Accuracy through Source Verification

All technical information must be verifiable through official documentation, standards, or authoritative sources
Claims about technologies, frameworks, or methodologies require direct rmation
Modular organization for independent chapter updates


Key Standards
Documentation Standards

Citation Format: Inline links to official documentation and authoritative sources
Code Format: Syntax-highlighted code blocks with language specification
File Structure: Follow Docusaurus conventions and Spec-Kit Plus patterns
Naming Conventions: Clear, descriptive filenames and section headers

Source Requirements

Primary Sources (minimum 60%):

Official documentation
Technical specifications
API references
Standards organizations (W3C, IETF, etc.)


Secondary Sources (maximum 40%):

Technical blogs from recognized experts
GitHub repositories with substantial usage
Academic papers for foundational concepts
Conference talks from reputable events



Quality Assurance

Code Validation: All code examples tested before publication
Link Verification: All external links checked for validity
Technical Accuracy: Peer review by domain experts preferred
Originality: All content original or properly attributed with clear licensing

Writing Standards

Readability: Flesch-Kincaid Reading Ease score 50-70 (college level)
Consistency: Uniform terminology, style, and formatting throughout
Accessibility: Alt text for images, semantic HTML structure
SEO Optimization: Meaningful headers, meta descriptions, keywords


Constraints
Structural Constraints

Chapter Length: 1,500-3,000 words per major section
Total Book Length: Minimum 20,000 words (flexible based on scope)
Code-to-Text Ratio: 30-40% code examples/demos
Minimum Chapters: 8-12 main chapters + introduction/conclusion

Source Constraints

Minimum Sources per Chapter: 5-10 authoritative references
Documentation Links: Every technical claim linked to source
Code Repository: Companion GitHub repo with all examples
Asset Management: All images, diagrams, and media properly licensed

Technical Constraints

Docusaurus Version: Specify and maintain consistency
Browser Compatibility: Modern browsers (last 2 versions)
Mobile Responsive: Full mobile optimization required
Build Time: Keep under 2 minutes for full site generation
Page Load: Target under 3 seconds per page

Deployment Constraints

GitHub Pages Compatibility: Static site only, no server-side code
Repository Structure: Follow Spec-Kit Plus conventions
CI/CD Pipeline: Automated deployment on merge to main branch
Version Control: Semantic versioning for book releases


Success Criteria
Content Quality Metrics
✅ Technical Accuracy: All code examples execute without errors
✅ Source Verification: Every technical claim traceable to authoritative source
✅ Completeness: All promised topics covered with adequate depth
✅ Clarity: Technical reviewers confirm content is understandable
✅ Originality: No plagiarism, all content original or properly attributed
Deployment Metrics
✅ Build Success: Docusaurus builds without errors or warnings
✅ GitHub Pages Live: Site accessible and fully functional
✅ Navigation: All internal links work correctly
✅ External Links: 100% of external references valid and accessible
✅ Mobile Friendly: Passes Google Mobile-Friendly Test
User Experience Metrics
✅ Search Functionality: Built-in search returns relevant results
✅ Load Performance: Lighthouse score 85+ for performance
✅ Accessibility: WCAG 2.1 Level AA compliance
✅ Cross-Browser: Tested on Chrome, Firefox, Safari, Edge
✅ Print-Friendly: CSS print styles for offline reading
Documentation Metrics
✅ README Completeness: Setup instructions clear and complete
✅ Contributing Guide: Guidelines for contributors provided
✅ License: Appropriate open-source license selected and applied
✅ Changelog: Version history documented
✅ Code Comments: All complex code adequately commented

Additional Guidelines
Content Development Process

Planning Phase: Create detailed specification document
Outline Phase: Structure chapters with learning objectives
Draft Phase: Write content with inline source citations
Review Phase: Technical review and fact-checking
Refinement Phase: Incorporate feedback and polish
Testing Phase: Validate all code and links
Deployment Phase: Build and publish to GitHub Pages

Spec-Kit Plus Integration

Follow the spec-first development approach
Use Claude Code for content generation and refinement
Maintain specification documents alongside content
Track changes and versions systematically

Maintenance Plan

Quarterly Review: Check for outdated information
Dependency Updates: Monitor and update library versions
Link Validation: Automated monthly link checking
Community Feedback: GitHub Issues for reader feedback
Continuous Improvement: Iterative updates based on feedback


Output Format Requirements
File Organization
book-project/
├── docs/               # Docusaurus content
├── src/               # Custom components
├── static/            # Assets (images, files)
├── blog/              # Optional blog section
├── specs/             # Specification documents
└── examples/          # Code examples repository
```

## Response snapshot

A comprehensive constitution for AI/Spec-Driven Book Creation has been created, along with updates to related templates to ensure consistency. The constitution includes:

1. Core Principles:
   - Accuracy through Source Verification
   - Documentation Standards Compliance
   - Source Hierarchy Management
   - Quality Assurance Validation
   - Technical Constraint Adherence
   - Content Development Process Discipline

2. Success Criteria and Metrics:
   - Content Quality Standards
   - Deployment Quality Standards
   - User Experience Standards

3. Development Workflow and Quality Gates:
   - Content Creation Process
   - Review and Validation Requirements
   - Deployment Approval Process

4. Governance section with versioning and compliance requirements

The related templates (plan, spec, tasks) have been updated to reflect the new constitution's requirements, ensuring that all development activities will comply with the established principles and standards.

## Outcome

- ✅ Impact: Created foundational constitution for technical book development project with Docusaurus
- 🧪 Tests: N/A (documentation task)
- 📁 Files: Updated constitution.md and 3 template files to maintain consistency
- 🔁 Next prompts: Continue with feature specification and planning
- 🧠 Reflection: Constitution provides clear governance framework for book development with emphasis on accuracy and source verification

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification following the new constitution
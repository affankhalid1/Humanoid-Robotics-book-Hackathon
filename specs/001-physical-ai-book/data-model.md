# Data Model: Physical AI Humanoid Robotics Book

## Core Entities

### Book Module
- **Description**: Represents a major section of the book (e.g., ROS 2, Simulation, Isaac, VLA)
- **Attributes**:
  - id: string (unique identifier, e.g., "module-01-ros2")
  - title: string (display title)
  - description: string (brief overview)
  - wordCount: integer (estimated word count)
  - learningTime: string (estimated time to complete, e.g., "3-5 weeks")
  - prerequisites: array of strings (required knowledge/skills)
  - learningObjectives: array of strings (skills to acquire)
  - chapters: array of Chapter entities
  - projects: array of Project entities
- **Relationships**: Contains many Chapters and Projects

### Chapter
- **Description**: Represents a chapter within a book module
- **Attributes**:
  - id: string (unique identifier, e.g., "module-01-ros2/chapter-01")
  - title: string (display title)
  - description: string (brief overview)
  - wordCount: integer (estimated word count)
  - learningTime: string (estimated time to complete, e.g., "2-3 hours")
  - learningObjectives: array of strings (skills to acquire)
  - content: string (path to markdown content file)
  - codeExamples: array of CodeExample entities
  - exercises: array of Exercise entities
- **Relationships**: Belongs to one BookModule, contains many CodeExamples and Exercises

### Code Example
- **Description**: Represents executable code blocks with specific functionality that demonstrates concepts from the text
- **Attributes**:
  - id: string (unique identifier, e.g., "module-01-ros2/hello-robot-world")
  - title: string (display title)
  - description: string (what the example demonstrates)
  - language: string (programming language)
  - filePath: string (path to source file)
  - dependencies: array of strings (required packages/libraries)
  - expectedOutput: string (description of expected output)
  - difficulty: enum ("beginner", "intermediate", "advanced")
  - testingInstructions: string (how to run and verify)
- **Relationships**: Belongs to one Chapter

### Exercise
- **Description**: Represents practice problems for students to solve
- **Attributes**:
  - id: string (unique identifier)
  - title: string (display title)
  - description: string (problem statement)
  - difficulty: enum ("beginner", "intermediate", "advanced")
  - solutionPath: string (path to solution file)
  - hints: array of strings (helpful hints)
- **Relationships**: Belongs to one Chapter

### Project
- **Description**: Represents hands-on projects that reinforce the learning objectives
- **Attributes**:
  - id: string (unique identifier)
  - title: string (display title)
  - description: string (project requirements)
  - difficulty: enum ("beginner", "intermediate", "advanced")
  - estimatedTime: string (time to complete)
  - requirements: array of strings (what needs to be built/implemented)
  - acceptanceCriteria: array of strings (how to verify completion)
  - solutionPath: string (path to complete solution)
- **Relationships**: Belongs to one BookModule

### Docker Configuration
- **Description**: Represents containerized environments that provide consistent development and simulation setups
- **Attributes**:
  - id: string (unique identifier, e.g., "ros2-humble", "isaac-sim")
  - name: string (display name)
  - description: string (what the container provides)
  - dockerfilePath: string (path to Dockerfile)
  - composePath: string (path to docker-compose file if applicable)
  - baseImage: string (base image used)
  - installedPackages: array of strings (software installed in container)
  - ports: array of integers (ports exposed)
  - volumes: array of strings (mounted volumes)
  - requirements: string (minimum hardware requirements)
- **Relationships**: Used by multiple BookModules

### Learning Objective
- **Description**: Represents specific skills or knowledge that users should acquire from each module
- **Attributes**:
  - id: string (unique identifier)
  - description: string (what the learner should be able to do)
  - module: string (which module this belongs to)
  - chapter: string (which chapter this belongs to, optional)
  - measurable: boolean (can this be objectively measured)
  - assessmentMethod: string (how to assess achievement)
- **Relationships**: Associated with BookModules and Chapters

## System Architecture Data Flow

### Content Creation Workflow
1. **Content Author** creates Chapter entities with text content
2. **Developer** creates Code Example entities with source code
3. **QA Team** verifies Code Examples and creates Exercise entities
4. **Editor** ensures Learning Objectives align with content
5. **Reviewer** validates all content against requirements

### Student Learning Path
1. **Student** accesses Book Module via Docusaurus interface
2. **System** presents Chapter content in sequential order
3. **Student** runs Code Examples in Docker environment
4. **System** tracks completion of Exercises and Projects
5. **Student** demonstrates mastery through Capstone Project

## Validation Rules

### Content Validation
- Each Chapter must have 3+ Code Examples
- Each Book Module must have 1+ Project
- All Code Examples must have verified functionality
- All content must meet Flesch-Kincaid grade 10-12 readability
- All external links must be valid

### Technical Validation
- All Docker configurations must work on Ubuntu 22.04
- All code examples must run with specified ROS 2 Humble
- All Isaac Sim examples must work with RTX GPU requirements
- All Whisper integration must handle audio input correctly
- All Nav2 navigation examples must work with specified parameters

## State Transitions

### Content Development States
- `draft` → `review` → `approved` → `published`
- `published` → `deprecated` (when technology changes)

### Student Progress States
- `not_started` → `reading` → `coding` → `exercising` → `project_work` → `completed`

## Relationships Summary

- **BookModule** 1 → * **Chapter**
- **BookModule** 1 → * **Project**
- **Chapter** 1 → * **CodeExample**
- **Chapter** 1 → * **Exercise**
- **LearningObjective** 1 → 1 **BookModule** (optional)
- **LearningObjective** 1 → 1 **Chapter** (optional)
- **DockerConfiguration** * → * **BookModule** (many-to-many)
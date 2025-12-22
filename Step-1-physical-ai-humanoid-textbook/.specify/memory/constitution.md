<!-- Sync Impact Report:
Version change: v0.0.0 (initial) -> v0.0.1 (patch)
Modified principles:
- [PROJECT_NAME] -> Physical-AI & Humanoid-Robotics — AI-Native Textbook + Embedded RAG Chatbot
- [PRINCIPLE_1_NAME] -> Verifiability
- [PRINCIPLE_1_DESCRIPTION] -> Every factual claim must be traceable to an authoritative source.
- [PRINCIPLE_2_NAME] -> Reproducibility
- [PRINCIPLE_2_DESCRIPTION] -> Code, commands, and data manifests must let reviewers reproduce results.
- [PRINCIPLE_3_NAME] -> Clarity
- [PRINCIPLE_3_DESCRIPTION] -> Technical writing aimed at an audience with computer-science background; use clear examples and runnable labs.
- [PRINCIPLE_4_NAME] -> Rigor
- [PRINCIPLE_4_DESCRIPTION] -> Prefer peer-reviewed sources and official docs for standards and APIs.
- [PRINCIPLE_5_NAME] -> Security & Privacy
- [PRINCIPLE_5_DESCRIPTION] -> User data (signup, embeddings, logs) must follow privacy-by-design: minimal retention, hashed identifiers, and privacy notes.
- [SECTION_2_NAME] -> Key Standards & Formatting
- [SECTION_2_CONTENT] -> (New content from user input)
- [SECTION_3_NAME] -> Core Deliverables
- [SECTION_3_CONTENT] -> (New content from user input)
- [GOVERNANCE_RULES] -> (New content from derived rules)
Added sections: None
Removed sections: [PRINCIPLE_6_NAME], [PRINCIPLE__DESCRIPTION]
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
- README.md: ⚠ pending
Follow-up TODOs: TODO(RATIFICATION_DATE): Original adoption date unknown
-->
# Physical-AI & Humanoid-Robotics — AI-Native Textbook + Embedded RAG Chatbot Constitution

## Core Principles

### I. Verifiability
Every factual claim must be traceable to an authoritative source.

### II. Reproducibility
Code, commands, and data manifests must let reviewers reproduce results.

### III. Clarity
Technical writing aimed at an audience with computer-science background; use clear examples and runnable labs.

### IV. Rigor
Prefer peer-reviewed sources and official docs for standards and APIs.

### V. Security & Privacy
User data (signup, embeddings, logs) must follow privacy-by-design: minimal retention, hashed identifiers, and privacy notes.

## Key Standards & Formatting

- Citation style: APA for all inline citations and reference lists.
- Minimum sources: 15 distinct sources; ≥50% must be peer-reviewed or official technical docs (ROS, NVIDIA, OpenAI).
- All code snippets must include exact runtime commands, minimum required versions, and expected outputs.
- Writing readability target: Flesch-Kincaid grade 10–12.
- Plagiarism tolerance: 0% — run automated checks before final submission.
- Deliverable format for book: Docusaurus site deployed to GitHub Pages (public repo). Provide a production build and a working live URL.
- Backup deliverable: downloadable PDF export of the book with embedded citations and accessible image alt text.

## Core Deliverables

1. Public GitHub repository with:
   - Docusaurus source in `/docusaurus` (docs, sidebars, config).
   - `spec/` directory with Spec-Kit Plus artifacts.
   - `agents/` directory with Claude Code subagent manifests and agent skill stubs.
   - `assets/` with diagrams and small demo assets (keep large models off-repo).
   - README with install/run/deploy steps and demo script for a ≤90s video.

2. Live published site (GitHub Pages or Vercel) showing the textbook content and sidebar layout for Modules 1–4 + Capstone + Hardware.

3. Integrated RAG chatbot:
   - Frontend widget embedded in the book UI that accepts free-form questions and highlighted-text context.
   - Backend service (FastAPI) implementing: ingestion pipeline, embeddings storage in Qdrant Cloud, Postgres (Neon Serverless) for metadata, and an OpenAI Agents/ChatKit-based orchestrator to assemble context + generate answers.
   - The chatbot must support a “use only selected text” mode where the agent answers solely from explicitly highlighted doc text.

4. Minimal tests & manifests:
   - Ingestion test that verifies each doc page is vectorized and queryable.
   - Example queries + expected evidence passages.
   - Dockerfile(s) or Runbook for running the FastAPI + vector DB locally (or via credentials for Qdrant/Neon).

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan. All PRs/reviews must verify compliance; Complexity must be justified.

**Version**: v0.0.1 | **Ratified**: TODO(RATIFICATION_DATE): Original adoption date unknown | **Last Amended**: 2025-12-08

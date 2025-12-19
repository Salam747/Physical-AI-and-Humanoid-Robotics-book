# Research & Decisions: Free RAG Chatbot Backend

**Date**: 2025-12-09
**Status**: Completed

This document summarizes the key technical decisions made during the planning phase. Given the detailed nature of the specification, no major unknowns required deep research. The focus was on confirming the suitability of the chosen free-tier services.

---

### Decision 1: Backend Framework

-   **Decision**: Use **FastAPI**.
-   **Rationale**:
    -   It is a modern, high-performance Python web framework.
    -   Excellent support for asynchronous operations, which is ideal for I/O-bound tasks like calling external APIs (Gemini, Qdrant).
    -   Automatic generation of interactive API documentation (Swagger UI), which is great for development and testing.
    -   It is lightweight and well-suited for serverless deployment on Vercel.
-   **Alternatives Considered**:
    -   **Flask**: A solid choice, but requires more boilerplate for async operations and API documentation. FastAPI is more "out-of-the-box" for this API-centric use case.
    -   **Django**: Too large and complex for a simple microservice-style backend. It includes many features (ORM, admin panel) that are not needed for this project.

---

### Decision 2: Deployment Platform

-   **Decision**: Target **Vercel** as the primary deployment platform.
-   **Rationale**:
    -   Vercel's free tier is generous and supports serverless Python functions seamlessly.
    -   Excellent integration with GitHub for continuous deployment (Git-push-to-deploy).
    -   Natively understands how to run a FastAPI application by converting its routes into serverless functions, requiring minimal configuration (`vercel.json`).
-   **Alternatives Considered**:
    -   **Render**: Also an excellent choice with a strong free tier. It's a valid fallback, but Vercel's workflow is slightly more streamlined for this type of serverless backend.
    -   **AWS/GCP/Azure**: Too complex and require significant setup for a beginner-friendly, 100% free project. The goal is to avoid manual cloud infrastructure management.

---

### Decision 3: RAG Orchestration

-   **Decision**: Use the **LangChain** community library.
-   **Rationale**:
    -   It provides standard, reusable components for building RAG pipelines (`PromptTemplate`, `StrOutputParser`).
    -   The `langchain-google-genai` integration simplifies calls to the Gemini API.
    -   While the entire pipeline could be written manually, using LangChain makes the code more modular and easier to understand for beginners familiar with the framework.
-   **Alternatives Considered**:
    -   **Manual HTTP Requests**: Writing the calls to Gemini and the logic for constructing prompts manually. This is a valid approach but results in more boilerplate code. LangChain provides a good level of abstraction without adding significant complexity.

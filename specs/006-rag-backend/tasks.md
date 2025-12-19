---

## Phase 5: User Story 3 - Query with Selected Text Override (Priority: P2)

**Goal**: Provide a specific piece of text to guide the chatbot's answer, overriding full book content search.
**Independent Test**: When `selected_text` is provided, the answer is strictly based on that text.

### Implementation for User Story 3

- [x] T020 [US3] Modify the RAG pipeline function in `backend/rag/pipeline.py` to accept an optional `selected_text` parameter and pass it to the generator.
- [x] T021 [US3] Update the generator logic in `backend/rag/generator.py` to check for `selected_text`; if present, use it as the primary context for Gemini 1.5 Flash, bypassing Qdrant retrieval.
- [x] T022 [US3] Update the `POST /query` API endpoint in `backend/app.py` to include the optional `selected_text` field in the request body, as defined in `query.yaml`.
- [x] T023 [US3] Create unit tests in `backend/tests/test_api.py` to verify that when a `selected_text` is provided, the generated answer is entirely derived from it, ignoring other book content.

**Checkpoint**: All user stories should now be independently functional

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T024 [P] Implement the `GET /health` API endpoint in `backend/app.py` returning a status 200 OK with `{"status": "healthy"}` as per `health.yaml`.
- [x] T025 Enhance error handling in `/query` for cases where no relevant chunks are found (e.g., cosine similarity below 0.7) and implement a retry mechanism or graceful degradation for Gemini API rate limits in `backend/app.py` and `backend/rag/pipeline.py`.
- [x] T026 Ensure all Python functions and significant code blocks across the `backend/` directory include comments with both Urdu and English explanations, as per FR-014.
- [x] T027 Review and refine `backend/README.md` to ensure it is beginner-friendly and covers all setup and usage instructions, including environment variables, API key acquisition, local running, ingestion, and query examples, aligning with `quickstart.md`.
- [x] T028 Implement CORS middleware in `backend/app.py` to allow requests from the specific GitHub Pages domain where the Docusaurus frontend will be hosted (NFR-006).

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Stories (Phase 3-5)**: All depend on Foundational phase completion.
    -   User stories can then proceed in parallel (if staffed)
    -   Or sequentially in priority order (P1 → P1 → P2).
-   **Polish (Final Phase)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1 - Ingest Book Content)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
-   **User Story 2 (P1 - Query Chatbot)**: Can start after Foundational (Phase 2) - Depends on US1 for ingested data.
-   **User Story 3 (P2 - Selected Text Override)**: Can start after Foundational (Phase 2) - Depends on US2 for base query functionality.

### Within Each User Story

-   Tests (if included) MUST be written and FAIL before implementation.
-   Models before services.
-   Services before endpoints.
-   Core implementation before integration.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   T007, T008, T009 in Foundational phase can run in parallel.
-   Once Foundational phase completes, User Story 1 (P1) can start.
-   Once User Story 1 (P1) completes, User Story 2 (P1) can start.
-   Once User Story 2 (P1) completes, User Story 3 (P2) can start.
-   T024 in Final Phase can run in parallel.

---

## Parallel Example: User Story 1

```bash
# Implement Foundational tasks in parallel (T007, T008, T009)
Task: "T007 [P] Implement a utility function `get_qdrant_client()` for initializing Qdrant client connection..."
Task: "T008 [P] Implement a utility function `get_db_session()` for initializing Neon Postgres connection..."
Task: "T009 [P] Implement the Gemini embedding function `get_gemini_embedding()` using `google-generativeai`..."

# Implement US1 implementation tasks
Task: "T010 [US1] Implement text chunking logic..."
Task: "T011 [US1] Develop the ingestion script to read Markdown files..."

# Implement US1 test task
Task: "T012 [US1] Create unit tests for the ingestion script..."

# Implement US1 API endpoint task
Task: "T013 [US1] Implement the `POST /ingest` API endpoint..."
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all user stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Add User Story 3 → Test independently → Deploy/Demo
5.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    -   Developer A: User Story 1
    -   Developer B: User Story 2 (after US1 is completed for data dependency)
    -   Developer C: User Story 3 (after US2 is completed for base query functionality)
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable (with noted dependencies)
-   Verify tests fail before implementing
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
# Performance Audit Brief Revision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a concise, evidence-ranked C++ safety and architecture review layer to the existing performance audit while preserving its full technical analysis.

**Architecture:** Extend the audit's executive section with seven short subsections. Each requested concern receives a confirmed, conditional, not-found, or not-applicable status linked to repository evidence; existing detailed sections remain the supporting record.

**Tech Stack:** Markdown, PROS C++ source inspection, Git, PowerShell, Python read-only validation, MinGW host tests.

## Global Constraints

- Modify audit documentation only; do not change competition source, dependencies, or build behavior.
- Preserve the audit base at `ac0fedcb1627a7fd0208ca8eaf62a3e2f9117f3c`.
- Preserve the existing 17-area audit, measurements, limitations, backlog, and roadmap.
- Keep new prose paragraphs at or below 240 characters; use tables for dense evidence.
- Do not present a general C++ concern as a repository defect without source evidence.
- Preserve useful comments and documentation; comments are not a runtime or binary-size optimization target.
- Preserve unrelated working-tree changes and stage only files named by each task.

---

### Task 1: Add Memory and Object-Lifetime Briefs

**Files:**
- Modify: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Inspect: `src/**/*.cpp`, `include/aon/**/*.hpp`, `common.mk`

**Interfaces:**
- Consumes: the approved revision design and pinned source evidence.
- Produces: executive evidence standard plus memory/resource and C++ lifetime matrices.

- [x] **Step 1: Verify ownership and lifetime evidence**

Run targeted searches for `new`, `delete`, `malloc`, `free`, smart pointers, C file handles, raw owning pointers, custom destructors, copy/move operations, exceptions, and potentially uninitialized locals.

- [x] **Step 2: Trace every matching first-party resource path**

Read allocation, file-handle, task-capture, and custom-resource sites in context. Classify each requested concern as confirmed, conditional, not found, or not applicable.

- [x] **Step 3: Add the evidence standard and memory/resource brief**

Add concise front-layer prose and a table covering leaks, manual allocation, dangling pointers, use-after-free, uninitialized variables, and RAII. Cite exact files and state scan limits.

- [x] **Step 4: Add the object-lifetime brief**

Cover Rule of 3/5/0, object slicing, and destructor exception behavior. Distinguish repository evidence from generic C++ risk and note compiler exception settings where relevant.

- [x] **Step 5: Validate and commit**

Check paragraph lengths, table structure, cited paths, and `git diff --check`. Stage only the audit and plan, mark Task 1 complete, then commit `docs: audit C++ resource and lifetime risks`.

### Task 2: Add Architecture, Build, and Comment Briefs

**Files:**
- Modify: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Modify: `docs/superpowers/plans/2026-08-10-performance-audit-brief-revision.md`
- Inspect: major class headers, `Makefile`, `common.mk`, `project.pros`, dependency metadata, comments, and include relationships.

**Interfaces:**
- Consumes: Task 1 evidence statuses and the existing performance findings.
- Produces: architecture, build ecosystem, priority, and comment/style briefs.

- [x] **Step 1: Inspect class and dependency structure**

Measure inheritance depth, large implementation headers, duplicated abstractions, dependency declarations, and source-comment markers. Avoid equating line count with a defect without usage evidence.

- [x] **Step 2: Add the code-design brief**

Cover OOP misuse, deep inheritance, virtual dispatch, excessive getter/setter boundaries, and over-engineering. Link confirmed concerns to the existing odometry and dual-motion-stack findings.

- [x] **Step 3: Add the build/ecosystem brief**

Cover compile-time evidence, header/source organization, C++ modules, and dependency management. Mark unavailable timing data clearly and avoid recommending unsupported ecosystem changes.

- [x] **Step 4: Add priority and comments/style briefs**

Map new conclusions into P0-P3 without duplicating the full backlog. State which comments help safety, which stale markers require review, and why deleting comments cannot improve the binary.

- [x] **Step 5: Validate and commit**

Check logical order, paragraph lengths, table structure, citations, and `git diff --check`. Stage only the audit and plan, mark Task 2 complete, then commit `docs: add architecture and build audit briefs`.

### Task 3: Logic, Concision, and Final Verification

**Files:**
- Modify: `docs/performance/2026-08-10-performance-real-time-audit.md`
- Modify: `docs/superpowers/plans/2026-08-10-performance-audit-brief-revision.md`

**Interfaces:**
- Consumes: the complete concise review layer and original detailed audit.
- Produces: a consistent, verified final report and completion record.

- [ ] **Step 1: Review information order and eliminate contradictions**

Ensure conclusions precede evidence, definitions precede statuses, and the concise layer agrees with detailed memory, concurrency, compiler, and backlog sections.

- [ ] **Step 2: Tighten new prose**

Remove repetition, vague intensifiers, and generic advice. Keep each new prose paragraph at or below 240 characters without weakening evidence or limitations.

- [ ] **Step 3: Validate coverage mechanically**

Confirm all user-supplied criteria appear, all explicit source paths exist, all new tables have consistent columns, and no placeholder language remains.

- [ ] **Step 4: Rebuild and run host tests**

Compile all 13 host executables with `-std=c++17 -Wall -Wextra -Werror -Iinclude`, run each executable, and record the embedded-toolchain limitation if it remains unavailable.

- [ ] **Step 5: Review and commit**

Run `git diff --check`, verify audit-owned changes are documentation-only, inspect repository status, mark Task 3 complete, and commit `docs: finalize concise performance audit revision`.

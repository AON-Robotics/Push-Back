# Working Safely From Two Computers

GitHub is the shared state between the lab laptop and apartment PC. Chats do
not need to synchronize if both tasks read `docs/CURRENT_HANDOFF.md` and the
committed plan before editing.

## Normal Workflow

Work on only one computer at a time. Before leaving that computer, finish at a
buildable commit and push it. On the other computer, pull before editing.

### Start a Session

Run these commands before starting development work:

```powershell
git switch Testing
git status --short --branch
git fetch origin
git pull --ff-only origin Testing
git status --short --branch
git log -5 --oneline
```

Then tell the new task:

> Read `docs/CURRENT_HANDOFF.md`, the linked migration specification and plan,
> and the last five commits. Confirm the branch is clean and synchronized
> before making changes. Keep the phase gates and checkpoint workflow.

If `git status` shows modified or untracked files, stop and identify who made
them. Do not pull, reset, or clean until that work is understood.

### End a Session

1. Run the relevant checks and clean builds.
2. Review `git diff` and `git diff --check`.
3. Update `docs/CURRENT_HANDOFF.md` with the new checkpoint and next gate.
4. Stage only the intended files rather than using `git add .`.
5. Commit with a specific subject and explanatory body.
6. Push and confirm the branch is synchronized.

```powershell
git push origin Testing
git status --short --branch
git log -3 --oneline
```

Do not change computers while a commit exists only locally.

## If Both Computers Have Changes

Do not force-push and do not use `git reset --hard`.

1. Commit the work on each computer to a separate temporary branch, such as
   `pc/<topic>` and `laptop/<topic>`.
2. Push both branches.
3. Compare their changes against `Testing`.
4. Merge or cherry-pick one checkpoint at a time, resolving conflicts with the
   migration plan and physical-test results in view.
5. Build and test the combined result before updating `Testing`.

This is slower than sequential work, but it preserves both versions and makes
the integration reviewable.

## Files That Carry Context

- `docs/CURRENT_HANDOFF.md`: current checkpoint, physical gate, and resume state.
- `docs/superpowers/specs/`: approved architectural decisions.
- `docs/superpowers/plans/`: ordered implementation tasks and progress.
- Git commits: exact, reversible code checkpoints.
- GitHub `Testing`: shared integration state for both computers.

Never rely on a chat transcript as the only record of a code or robot-testing
decision.

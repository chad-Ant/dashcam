# Knowledge folder — user-supplied guidelines

This folder is for **additional guideline documents you add over time** — house
coding standards, project conventions, client requirements, datasheets, internal
best-practice notes, lessons learned from past deployments, etc. Drop files in
here (`.md` is ideal; `.pdf`/`.txt` are fine) and they become part of the skill's
working knowledge.

There is no fixed file list. Claude discovers whatever is present at use time, so
you can keep adding documents without editing any other file. Optionally group
them in subfolders (e.g. `knowledge/company-style/`, `knowledge/project-buoy/`) —
Claude will read whatever is relevant to the task.

## How Claude uses this folder

When generating or reviewing MKR Zero code, Claude treats documents here as
authoritative project guidance **on top of** the built-in references — applying
naming conventions, preferred libraries, logging formats, review checklists, and
so on that you've supplied.

## Precedence — read this carefully

The safety standard in `../references/mission-critical.md` is an adaptation of
NASA/JPL's *Power of Ten* rules for safety-critical code. **Those rules take
precedence.** If a document in this folder conflicts with them, the NASA-derived
rule wins, every time. The order of authority is:

1. **`references/mission-critical.md` (the NASA *Power of Ten* adaptation).**
   Non-negotiable. Highest authority on anything touching safety, reliability,
   control flow, memory, error handling, or bounded execution.
2. **Hardware reality** (`references/`, verified against the SAMD21 datasheet and
   the MKR Zero pinout). A guideline cannot override a physical constraint — e.g.
   nothing here can authorize feeding 5 V to an I/O pin, conjuring a second DAC,
   or driving two pins from one EXTINT line, because the silicon and the board
   won't do it.
3. **User guideline documents in this folder.** Authoritative for everything the
   higher tiers don't constrain: style, structure, library choices, naming,
   project-specific requirements, documentation format, etc.

### What a conflict looks like, and how to resolve it

A house guide here might say, for example, "retry SD init until it succeeds" or
"use `String` for log lines for readability" or "save the calibration to flash
every loop so it's always current." Each of those collides with a NASA rule
(bounded loops / no heap churn after init / flash-write endurance). In every such
case:

- **Follow the NASA-derived rule**, not the conflicting guideline.
- **Tell the user there was a conflict**, name both sides, and say briefly why
  the safety rule wins (e.g. "your style guide prefers retrying SD init until it
  works, but Rule 2 requires a bounded wait or the node hangs unattended, so I
  used a bounded retry with a run-without-logging fallback").

Do not silently discard a user guideline — surface the conflict so the user can
decide whether to revise their document. Where a guideline merely *adds* detail
without contradicting the safety rules (the common case), apply it directly and
without fanfare.

## Note on contradictions *between* user documents

If two documents in this folder disagree and neither is overridden by a higher
tier, prefer the more specific/most-recent one if that's discernible, and
otherwise ask the user which should govern rather than guessing.

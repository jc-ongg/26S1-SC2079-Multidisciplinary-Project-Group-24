# Stash audit — 2026-10-02

The two GitHub Desktop stashes belong to the entire repository. Their branch
labels record where each was created; they are not independent pending edits
on every current branch. All five retained GitHub branch tips were synchronized
before this audit. The normal working tree has no tracked environments/output.

| Saved work | Total changed paths | Environment/build/cache | Generated outputs | Review candidates |
| --- | ---: | ---: | ---: | ---: |
| Task1 services stash | 108 | 20 | 83 | 5 |
| Main stash | 28,221 | 28,008 | 202 | 11 |

The counts compare each stash with its original base, not with today's main.
Ignore rules added later do not rewrite old stash snapshots. Bulk applying or
committing the stashes would import old environments, image outputs and older
code. It is unnecessary and could overwrite already integrated improvements.

## Source reconciliation

- Services stash: temporary CV source, requirements, docs and tests are already
  integrated. Current CV adds output-directory recovery and its regression test.
  The old environment ignore rule is covered by current root/local ignores.
- Main stash: Android Save/Use files exactly match the integrated files. Its
  straight-scale10 change is already in main, with newer opt-in tuning profiles.
- Machine-local deployment-target IDE state is not merged.
- Five missing reference documents were restored without overwriting existing
  files. Recovered Git content hashes initially matched the saved blobs; PDFs
  remain unchanged:
  `MDP Theme.txt`, `MDP assessment and system checklist (2).pdf`, `MDP briefing.pdf`,
  `MDP student component list_ Sem1_2627.pdf`, `Rules for the Task 1 and Task 2.pdf`.
  The text document follows normal newline rules and three trailing spaces were
  removed for a clean diff. The full original remains in the recovery tag/archive.

## Recovery and stash-list cleanup

Complete stash snapshots, including all their parents/index objects, are retained
by annotated **local-only** recovery tags:

- `archive/2026-10-02/stash-task1-services` ->
  `426475c94c966e779e0734ee842b174acd0bd3aa`.
- `archive/2026-10-02/stash-main` ->
  `f8fd5f9f02f9433e6c7972d322f935c11f9c601c`.

These archive tags are deliberately NOT pushed, to avoid uploading thousands
of environment/output files to GitHub. They protect saved objects from local
Git garbage collection; they are not a separate-machine backup of the whole repo.
Selective document recovery archive: `output/stash-review-20261002/reference-documents.tar`.

The active stash entries remain until the user approves removing them from that
list. Keeping them visible therefore still shows large old counts in GitHub
Desktop even though all meaningful source is integrated and recoverable. To
fully clear the display, approve removing those two active stash entries after
verifying the recovery tags. This must not delete files on disk or archive tags.

Calibration lives in `Algo/consts.py` (plural). Its selectable profiles were
pushed in commit `ebf3f0bc`, then integrated into all retained branches. Base
defaults remain original; speed50 overlays must be explicitly enabled. See
[calibration status](../Algo/CALIBRATION.md).

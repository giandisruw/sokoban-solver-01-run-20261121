# Sokoban Solver — make.fun `sokoban-solver-01-run-20261121`
[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/giandisruw/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/giandisruw/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver for the make.fun bounty "Build the best
Sokoban solver, 21 November 2026". Entry point: `main.ts` (Node 22,
erasable TypeScript, Node built-ins only, no network, no install).

## How it works

The generator lays a 1-wide hallway with 1–4 small rooms (6×6 slots) off
it, each room holding 3–4 boxes and as many goals. A box pushed out of a
room through its door can never be pushed back (the hallway square it lands
on reaches no goal), so a level's push optimum P\* is the **sum of its
rooms' optima** — the problem splits per room.

1. **Decompose.** Cut the dead squares of the player's (hallway) row out of
   the floor; each remaining connected part holding boxes is a room.
2. **Solve each room push-optimally** with a push-based A\* search:
   - a state is the room's boxes (sorted) plus the player's reachable
     region (named by its lowest square), so walking is free and only
     pushes cost;
   - the heuristic is the cheapest assignment of boxes to goals, where one
     box-to-goal cost is its shortest grid distance ignoring the other
     boxes (each push moves one box exactly one square, so the sum is
     admissible — A\* is push-optimal);
   - two safe deadlock prunes: boxes on dead squares (no push, ignoring
     the other boxes, can ever bring them to a goal) and frozen 2×2 blocks
     of walls and boxes with a box off its goal.
   - if A\* runs out of its work budget on a room, a plain push BFS gets a
     second try (the generator keeps a room only when that BFS solves it).
3. **Stitch.** Rooms are solved in scan order (reversed order is tried if
   that fails), the other rooms' boxes standing as walls, the player
   walking on from where the last room left it. If the level is no set of
   rooms at all, the whole level is searched the same way as a last resort.

## Budgets and determinism

Everything is budgeted by **work** (node counts), never by the clock:
250,000 A\* expansions per room search, 250,000 states for the BFS
fallback, 600,000 expansions per instance (a fair share, reset per
instance, so one pathological level cannot starve the others). All
tie-breaking is fixed, so two runs produce byte-identical output. An
unsolved level gets an empty (valid, 0-point) solution rather than a crash
or a missing file.

## Practice results (kit's `grade-local`, `--driver process`)

- **Score 1,000,000 / 1,000,000** — every instance solved in exactly P\*
  pushes: i001 13/13, i002 16/16, i003 29/29, i004 47/47, i005 60/60.
- Full 5-instance run: ~0.6 s (limit 60 s), ~9,600 search expansions total.
- Two runs produce byte-identical outputs (determinism check passed).
- House baseline `plain-astar 1.0.0` on the same practice set: see
  `grade-local` comparison in the build report (it scores far below, as it
  minimises moves, not pushes, and knows no deadlock).

## Usage

Run by the grader as `node /bot/main.ts`; it reads `/input/<id>/level.txt`
for `i001`–`i005` and writes `/output/<id>/solution.txt` for each. Locally:

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see `LICENSE`.

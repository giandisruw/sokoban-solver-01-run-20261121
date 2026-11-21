// Sokoban push-optimal solver.
//
// Idea: the generator lays a 1-wide hallway with 1-4 small rooms (6x6 slots)
// off it, each room holding 3-4 boxes and as many goals, and a box pushed out
// of a room through its door can never be pushed back (the hallway square it
// lands on reaches no goal). So a level's push optimum P* is the SUM of its
// rooms' optima, and the whole problem splits per room.
//
// Each room is solved push-optimally with a push-based A* search:
//   - a state is the rooms' boxes (sorted) plus the player's reachable region
//     (named by its lowest square), so walking is free and only pushes cost;
//   - the heuristic is the cheapest assignment of boxes to goals, where the
//     cost of one box to one goal is its shortest grid distance ignoring the
//     other boxes (each push moves one box exactly one square, so the sum is
//     admissible, hence A* is push-optimal);
//   - two deadlocks are pruned, both safe (they never remove a solvable
//     state): a box on a dead square (no push, ignoring the other boxes, can
//     ever bring it to a goal) and a frozen 2x2 block of walls and boxes with
//     a box off its goal.
// If A* runs out of its work budget on a room, a plain push BFS (same state
// space, expansions in push order) gets a second try; the generator keeps a
// room only when that BFS solves it, so the fallback is a real safety net.
// Rooms are stitched in scan order (reversed order is tried if that fails),
// the other rooms' boxes standing as walls; the player walks on from where
// the last room left it. If the level is no set of rooms at all, the whole
// level is searched the same way as a last resort.
//
// Everything is budgeted by WORK (node counts), never by the clock, and all
// tie-breaking is fixed, so two runs produce byte-identical output. If a
// level cannot be solved within budget, an empty (valid, 0-point) solution is
// written for it rather than crashing or leaving a file missing.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions: [dx, dy, letter], in a fixed order used everywhere (walk BFS,
// push expansion, reconstruction), so ties always break the same way.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const LETTER = ['u', 'd', 'l', 'r'];

// Work budgets (node counts, never the clock).
const ASTAR_EXPANSIONS = 250_000; // A* expansions per room search
const BFS_STATES = 250_000; // states for the plain push-BFS fallback
const WHOLE_EXPANSIONS = 150_000; // expansions for the whole-level fallback
const MAX_STATES = 300_000; // states stored per search (memory bound)
// Expansions for one instance: a fair share of the run (5 instances share a
// 60 s wall). It is reset per instance, so one pathological level can never
// starve the other four; the run-wide worst case stays 5 x 600,000.
const INSTANCE_EXPANSIONS = 600_000;

let globalLeft = INSTANCE_EXPANSIONS;

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
};

type Room = { squares: number[]; boxes: number[]; goals: number[]; done: boolean };
type SearchOut = { pushes: number; moves: string } | null;

/** Parse XSB text. Short rows pad as walls; the outer frame is wall. */
function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
    }
  }
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/**
 * The squares from which a box alone can be pushed onto some goal: pull back
 * from the goals over open floor. A square y is live when some live square x
 * can be reached by pushing a box from y (the player standing at y - d).
 * A superset of the truly live squares, so pruning on it is safe.
 */
function liveSquares(w: number, wall: Uint8Array, goals: number[]): Uint8Array {
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (let k = 0; k < 4; k++) {
      const d = DY[k]! * w + DX[k]!;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** Shortest grid distances from `from` over non-wall squares (-1: unreachable). */
function bfsDist(w: number, wall: Uint8Array, from: number): Int32Array {
  const dist = new Int32Array(wall.length).fill(-1);
  const queue = [from];
  dist[from] = 0;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k]! * w + DX[k]!;
      if (!wall[q] && dist[q] < 0) {
        dist[q] = dist[p]! + 1;
        queue.push(q);
      }
    }
  }
  return dist;
}

/** All orderings of 0..n-1 (n <= 5, so at most 120). */
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const cur: number[] = [];
  const used = new Array<boolean>(n).fill(false);
  const rec = (): void => {
    if (cur.length === n) {
      out.push([...cur]);
      return;
    }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = true;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = false;
    }
  };
  rec();
  return out;
}

/** The shortest walk from `from` to `to` around walls and boxes (lower LURD). */
function walkTo(
  w: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  from: number,
  to: number,
): string {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k]! * w + DX[k]!;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = LETTER[k]!;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** The square the player stands on after `moves` (letters only) from `from`. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves) p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

type SearchOpts = {
  w: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
  expansions: number;
  heuristic: boolean; // true: A* with the assignment heuristic; false: plain push BFS
};

/**
 * Push-optimal search over one (sub-)level. A state is the sorted box
 * squares plus the player's reachable region (its lowest square). Returns
 * {pushes, moves} with moves in lower LURD (walks included), or null when
 * the work budget runs out or the level has no solution.
 */
function searchPush(o: SearchOpts): SearchOut {
  const { w, wall, goals, player } = o;
  const S = wall.length;
  const n = goals.length;
  const startBoxes = o.boxes.slice().sort((a, b) => a - b);
  const goalSet = new Uint8Array(S);
  for (const g of goals) goalSet[g] = 1;
  const live = liveSquares(w, wall, goals);
  for (const b of startBoxes) if (!live[b]) return null;
  const solved = (bs: number[]): boolean => bs.every((b) => goalSet[b]);
  if (solved(startBoxes)) return { pushes: 0, moves: '' };

  // Heuristic: cheapest box-to-goal assignment on grid distances.
  const dists = goals.map((g) => bfsDist(w, wall, g));
  const perms = n <= 5 ? permutations(n) : [];
  const hOf = (bs: number[]): number => {
    if (n > 5) {
      // Too many boxes for exact assignment: sum of individual minima.
      let s = 0;
      for (const b of bs) {
        let m = Infinity;
        for (let gi = 0; gi < n; gi++) {
          const d = dists[gi]![b]!;
          if (d >= 0 && d < m) m = d;
        }
        if (m === Infinity) return Infinity;
        s += m;
      }
      return s;
    }
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = dists[p[i]!]![bs[i]!]!;
        if (d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };

  // Flood-fill scratch: `seen` marks the region of the state being expanded,
  // `mark` the region of each child state a push makes. One stamp counter
  // serves both arrays (they are different arrays).
  const seen = new Int32Array(S);
  const mark = new Int32Array(S);
  const stack = new Int32Array(S);
  let stamp = 0;
  const occupied = new Uint8Array(S);
  const reach = (from: number, into: Int32Array, st: number): number => {
    let low = from;
    let sp = 0;
    stack[sp++] = from;
    into[from] = st;
    while (sp > 0) {
      const p = stack[--sp]!;
      if (p < low) low = p;
      for (let k = 0; k < 4; k++) {
        const q = p + DY[k]! * w + DX[k]!;
        if (wall[q] === 0 && occupied[q] === 0 && into[q] !== st) {
          into[q] = st;
          stack[sp++] = q;
        }
      }
    }
    return low;
  };

  // A state's key: the region's lowest square and the boxes' squares, as
  // digits in base S (exact while S ** (boxes + 1) stays under 2 ** 53).
  const numericKey = n <= 5;
  const keyOf = (bs: number[], low: number): number | string => {
    if (!numericKey) return `${low}:${bs.join(',')}`;
    let k = low;
    for (const b of bs) k = k * S + b;
    return k;
  };

  /** Whether the box just pushed to `t` sits in a frozen 2x2 block. */
  const frozenBlock = (t: number): boolean => {
    for (const a of [t, t - 1, t - w, t - w - 1]) {
      const b = a + 1;
      const c = a + w;
      const d = c + 1;
      if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b])) continue;
      if (!(wall[c] || occupied[c]) || !(wall[d] || occupied[d])) continue;
      if (
        (occupied[a] && !goalSet[a]) ||
        (occupied[b] && !goalSet[b]) ||
        (occupied[c] && !goalSet[c]) ||
        (occupied[d] && !goalSet[d])
      ) {
        return true;
      }
    }
    return false;
  };

  // The states: boxes, the player's square, parent, the push that made it,
  // its key and its push count. The open list is a heap on [f, seq, index];
  // seq makes ties break in insertion order, deterministically.
  const sBoxes: number[][] = [startBoxes];
  const sPlayer: number[] = [player];
  const sParent: number[] = [-1];
  const sFrom: number[] = [-1];
  const sTo: number[] = [-1];
  const sKey: (number | string)[] = [];
  const sG: number[] = [0];
  const bestG = new Map<number | string, number>();
  for (const b of startBoxes) occupied[b] = 1;
  const low0 = reach(player, seen, ++stamp);
  for (const b of startBoxes) occupied[b] = 0;
  sKey.push(keyOf(startBoxes, low0));
  bestG.set(sKey[0]!, 0);

  const heap: [number, number, number][] = [];
  const less = (a: [number, number, number], b: [number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && a[1] < b[1]);
  const heapPush = (e: [number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      const tmp = heap[i]!;
      heap[i] = heap[p]!;
      heap[p] = tmp;
      i = p;
    }
  };
  const heapPop = (): [number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        const tmp = heap[i]!;
        heap[i] = heap[c]!;
        heap[c] = tmp;
        i = c;
      }
    }
    return top;
  };

  const h0 = o.heuristic ? hOf(startBoxes) : 0;
  if (h0 === Infinity) return null;
  let seq = 1;
  heapPush([h0, 0, 0]);

  let expansions = 0;
  while (heap.length > 0) {
    const [, , s] = heapPop();
    if (bestG.get(sKey[s]!) !== sG[s]) continue; // superseded by a cheaper twin
    const bs = sBoxes[s]!;
    const g = sG[s]!;
    if (solved(bs)) {
      // Rebuild the moves: walk to each push's square, then push.
      const path: number[] = [];
      for (let t = s; t > 0; t = sParent[t]!) path.push(t);
      path.reverse();
      let moves = '';
      let playerPos = player;
      let curBoxes = startBoxes;
      for (const t of path) {
        const from = sFrom[t]!;
        const to = sTo[t]!;
        const d = to - from;
        const k = d === -w ? 0 : d === w ? 1 : d === -1 ? 2 : 3;
        for (const b of curBoxes) occupied[b] = 1;
        moves += walkTo(w, wall, occupied, playerPos, from - d);
        for (const b of curBoxes) occupied[b] = 0;
        moves += LETTER[k]!;
        playerPos = from;
        curBoxes = sBoxes[t]!;
      }
      return { pushes: path.length, moves };
    }
    if (expansions >= o.expansions || globalLeft <= 0) return null;
    expansions++;
    globalLeft--;
    for (const b of bs) occupied[b] = 1;
    const here = ++stamp;
    reach(sPlayer[s]!, seen, here);
    for (let bi = 0; bi < bs.length; bi++) {
      const b = bs[bi]!;
      for (let k = 0; k < 4; k++) {
        const d = DY[k]! * w + DX[k]!;
        const behind = b - d;
        const t = b + d;
        if (seen[behind] !== here) continue; // player cannot reach the push square
        if (wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(t);
        let key: number | string = -1;
        let nb: number[] | null = null;
        let nLow = -1;
        if (!frozen) {
          nb = bs.slice();
          nb[bi] = t;
          nb.sort((a, b2) => a - b2);
          nLow = reach(b, mark, ++stamp);
          key = keyOf(nb, nLow);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || nb === null) continue;
        const g2 = g + 1;
        const prevG = bestG.get(key);
        if (prevG !== undefined && prevG <= g2) continue;
        if (sBoxes.length >= MAX_STATES) return null;
        const h = o.heuristic ? hOf(nb) : 0;
        if (h === Infinity) continue;
        bestG.set(key, g2);
        const idx = sBoxes.length;
        sBoxes.push(nb);
        sPlayer.push(b);
        sParent.push(s);
        sFrom.push(b);
        sTo.push(t);
        sKey.push(key);
        sG.push(g2);
        heapPush([g2 + h, seq++, idx]);
      }
    }
    for (const b of bs) occupied[b] = 0;
  }
  return null;
}

/**
 * The level's rooms: cut the dead squares of the player's row (the hallway:
 * a box there reaches no goal) out of the floor, and each part left that
 * holds a box is a room. Null when some part holds more or fewer boxes than
 * goals, or a box starts in the hallway: then the level is no set of rooms.
 */
function roomsOf(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(w, wall, goals);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      const p = squares[i]!;
      for (let k = 0; k < 4; k++) {
        const q = p + DY[k]! * w + DX[k]!;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (
    goals.some((g) => part[g]! < 0) ||
    rooms.some((r) => r.boxes.length !== r.goals.length)
  ) {
    return null;
  }
  return rooms.filter((r) => r.boxes.length > 0);
}

/**
 * The level solved room by room: each room's push-optimal solution in turn,
 * the other rooms' boxes standing as walls (on their goals once their room
 * is done), the player walking on from where the last room left it.
 */
function solveByRooms(level: Level, roomsIn: Room[], reverse: boolean): SearchOut {
  const rooms = (reverse ? [...roomsIn].reverse() : roomsIn).map((r) => ({
    squares: r.squares,
    boxes: r.boxes,
    goals: r.goals,
    done: false,
  }));
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms) {
      if (other === room) continue;
      for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    }
    const sub: SearchOpts = {
      w: level.w,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
      expansions: ASTAR_EXPANSIONS,
      heuristic: true,
    };
    let solved = searchPush(sub);
    if (solved === null) {
      solved = searchPush({ ...sub, expansions: BFS_STATES, heuristic: false });
    }
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return { pushes, moves };
}

/** The moves for one level in XSB: '' when it cannot be solved in budget. */
function solveLevel(text: string): string {
  globalLeft = INSTANCE_EXPANSIONS; // fair share: this instance's budget only
  try {
    const level = parseLevel(text);
    if (level.boxes.every((b) => level.goals.includes(b))) return '';
    const rooms = roomsOf(level);
    if (rooms !== null) {
      const forward = solveByRooms(level, rooms, false);
      if (forward !== null) return forward.moves;
      const backward = solveByRooms(level, rooms, true);
      if (backward !== null) return backward.moves;
    }
    const whole = searchPush({
      w: level.w,
      wall: level.wall,
      goals: level.goals,
      boxes: level.boxes,
      player: level.player,
      expansions: WHOLE_EXPANSIONS,
      heuristic: true,
    });
    if (whole !== null) return whole.moves;
    const wholeBfs = searchPush({
      w: level.w,
      wall: level.wall,
      goals: level.goals,
      boxes: level.boxes,
      player: level.player,
      expansions: BFS_STATES,
      heuristic: false,
    });
    if (wholeBfs !== null) return wholeBfs.moves;
    return '';
  } catch {
    return '';
  }
}

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}

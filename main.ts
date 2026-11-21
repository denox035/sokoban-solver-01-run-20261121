// Room-decomposed push-optimal Sokoban solver.
// A level is one hallway row with 1-4 rooms off it; a box pushed out of a
// room can never come back, so each room is solved on its own by push A*
// (state = sorted boxes + canonical player cell) and the rooms are stitched
// with shortest walks. Deterministic: fixed direction order, insertion
// counters in the heap, expansion caps instead of any clock.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];
const CAP_OPT = 600000;
const CAP_W = 600000;
const CAP_ALL = 150000;

type Push = { b: number; d: number };
type Board = { W: number; H: number; wall: Uint8Array; goal: Uint8Array; boxes: number[]; player: number };

// Parse XSB into a board with a 1-cell wall border, so index arithmetic
// never leaves the array and never wraps across rows.
function parse(text: string): Board {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h0 = rows.length;
  const w0 = Math.max(...rows.map((r) => r.length));
  const W = w0 + 2;
  const H = h0 + 2;
  const N = W * H;
  const wall = new Uint8Array(N).fill(1);
  const goal = new Uint8Array(N);
  const boxes: number[] = [];
  let player = 0;
  for (let y = 0; y < h0; y++) {
    const row = rows[y];
    for (let x = 0; x < w0; x++) {
      const c = row[x] ?? '#';
      const i = (y + 1) * W + (x + 1);
      if (c === '#') continue;
      wall[i] = 0;
      if (c === '.' || c === '*' || c === '+') goal[i] = 1;
      if (c === '$' || c === '*') boxes.push(i);
      if (c === '@' || c === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { W, H, wall, goal, boxes, player };
}

const bitcount = (x: number): number => {
  let c = 0;
  while (x) {
    c += x & 1;
    x >>= 1;
  }
  return c;
};

// Push-optimal A* over one box set. floor = cells boxes and the player may
// occupy; exit = cell the player must be able to reach once solved (-1: none).
// weight 1 is optimal; higher weights trade pushes for speed. Returns the
// push list, or null when the expansion cap is hit.
function pushSearch(
  W: number, H: number, wall: Uint8Array, floor: Uint8Array,
  goals: number[], boxes0: number[], player0: number,
  exit: number, cap: number, weight: number
): Push[] | null {
  const N = W * H;
  const n = goals.length;
  const D = [-W, W, -1, 1];
  const goalSet = new Uint8Array(N);
  for (const g of goals) goalSet[g] = 1;
  // Push-distance maps: dist[g][c] = fewest pushes to move one box from c
  // onto goal g, ignoring other boxes (reverse BFS in push space).
  const dist: Int16Array[] = [];
  for (let gi = 0; gi < n; gi++) {
    const g = goals[gi];
    const d = new Int16Array(N).fill(-1);
    d[g] = 0;
    const q: number[] = [g];
    for (let qi = 0; qi < q.length; qi++) {
      const b = q[qi];
      for (let k = 0; k < 4; k++) {
        const b2 = b - D[k];
        const p = b - 2 * D[k];
        if (!floor[b2] || !floor[p] || d[b2] >= 0) continue;
        d[b2] = d[b] + 1;
        q.push(b2);
      }
    }
    dist.push(d);
  }
  const live = new Uint8Array(N);
  for (let i = 0; i < N; i++) {
    for (let gi = 0; gi < n; gi++) {
      if (dist[gi][i] >= 0) { live[i] = 1; break; }
    }
  }
  const useNum = n <= 4 && N <= 1024;
  const pack = (bs: number[]): number | string => {
    if (!useNum) return bs.join(',');
    let v = 0;
    for (let i = n - 1; i >= 0; i--) v = v * 1024 + bs[i];
    return v;
  };
  const unpack = (k: number | string): number[] => {
    if (typeof k === 'string') return k.split(',').map(Number);
    const bs: number[] = [];
    let v = k;
    for (let i = 0; i < n; i++) {
      bs.push(v % 1024);
      v = Math.floor(v / 1024);
    }
    return bs;
  };
  const keyOf = (pk: number | string, p: number): number | string =>
    useNum ? (pk as number) * 1024 + p : (pk as string) + '#' + p;
  // Admissible heuristic: min-cost assignment of boxes to goals; for many
  // boxes the cheaper sum of nearest-goal distances. Cached per box set.
  const hcache = new Map<number | string, number>();
  const heur = (pk: number | string, bs: number[]): number => {
    const c = hcache.get(pk);
    if (c !== undefined) return c;
    let h: number;
    if (useNum) {
      const full = (1 << n) - 1;
      const dp = new Float64Array(1 << n).fill(Infinity);
      dp[0] = 0;
      for (let mask = 0; mask <= full; mask++) {
        if (dp[mask] === Infinity) continue;
        const k = bitcount(mask);
        if (k >= n) continue;
        for (let j = 0; j < n; j++) {
          if (mask & (1 << j)) continue;
          const v = dist[j][bs[k]];
          if (v < 0) continue;
          const nm = mask | (1 << j);
          const nv = dp[mask] + v;
          if (nv < dp[nm]) dp[nm] = nv;
        }
      }
      h = dp[full];
    } else {
      h = 0;
      for (let i = 0; i < n; i++) {
        let m = Infinity;
        for (let j = 0; j < n; j++) {
          const v = dist[j][bs[i]];
          if (v >= 0 && v < m) m = v;
        }
        if (m === Infinity) { h = Infinity; break; }
        h += m;
      }
    }
    hcache.set(pk, h);
    return h;
  };
  // Reachability BFS. Two stamp arrays: rSt for the state being expanded
  // (kept across successor canonicalisation), cSt for canonicalisation.
  const rSt = new Int32Array(N);
  const cSt = new Int32Array(N);
  const bSt = new Int32Array(N);
  let rTag = 0;
  let cTag = 0;
  let bTag = 0;
  const reach = (from: number, bs: number[], st: Int32Array, tag: number): number[] => {
    bTag++;
    for (const b of bs) bSt[b] = bTag;
    const list: number[] = [from];
    st[from] = tag;
    for (let qi = 0; qi < list.length; qi++) {
      const c = list[qi];
      for (let k = 0; k < 4; k++) {
        const q = c + D[k];
        if (!floor[q] || st[q] === tag || bSt[q] === bTag) continue;
        st[q] = tag;
        list.push(q);
      }
    }
    return list;
  };
  // A 2x2 block is dead when every cell is a box or a wall and at least one
  // box in it is off its goal: every box there is frozen for good.
  const dead22 = (dest: number, nb: number[]): boolean => {
    const x = dest % W;
    const y = (dest / W) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        const cx = x + ox;
        const cy = y + oy;
        if (cx < 0 || cy < 0 || cx + 1 >= W || cy + 1 >= H) continue;
        const c0 = cy * W + cx;
        const cells = [c0, c0 + 1, c0 + W, c0 + W + 1];
        let bw = 0;
        let off = 0;
        for (const c of cells) {
          if (wall[c]) bw++;
          else if (nb.includes(c)) {
            bw++;
            if (!goalSet[c]) off++;
          }
        }
        if (bw === 4 && off >= 1) return true;
      }
    }
    return false;
  };
  // State storage: parallel arrays, boxes packed into one number when small.
  const pl: number[] = [];
  const bx: (number | string)[] = [];
  const gg: number[] = [];
  const ff: number[] = [];
  const sq: number[] = [];
  const par: number[] = [];
  const pd: number[] = [];
  const pb: number[] = [];
  const heap: number[] = [];
  let seq = 0;
  const best = new Map<number | string, number>();
  const less = (a: number, b: number): boolean =>
    ff[a] !== ff[b] ? ff[a] < ff[b] : gg[a] !== gg[b] ? gg[a] > gg[b] : sq[a] < sq[b];
  const hpush = (t: number): void => {
    let i = heap.length;
    heap.push(t);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i], heap[p])) break;
      const tmp = heap[i];
      heap[i] = heap[p];
      heap[p] = tmp;
      i = p;
    }
  };
  const hpop = (): number => {
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1], heap[c])) c++;
        if (!less(heap[c], heap[i])) break;
        const tmp = heap[i];
        heap[i] = heap[c];
        heap[c] = tmp;
        i = c;
      }
    }
    return top;
  };
  const reconstruct = (s: number): Push[] => {
    const out: Push[] = [];
    for (let t = s; t > 0; t = par[t]) out.push({ b: pb[t], d: pd[t] });
    out.reverse();
    return out;
  };
  // Start state, player canonicalised to the smallest reachable cell.
  const startRegion = reach(player0, boxes0, rSt, ++rTag);
  let minP = player0;
  for (const c of startRegion) if (c < minP) minP = c;
  const startPacked = pack(boxes0);
  const startH = heur(startPacked, boxes0);
  if (startH === Infinity) return null;
  best.set(keyOf(startPacked, minP), 0);
  pl.push(minP);
  bx.push(startPacked);
  gg.push(0);
  ff.push(weight * startH);
  sq.push(seq++);
  par.push(-1);
  pd.push(-1);
  pb.push(-1);
  hpush(0);
  let expanded = 0;
  while (heap.length > 0) {
    const s = hpop();
    const p = pl[s];
    const g = gg[s];
    const pk = bx[s];
    if (best.get(keyOf(pk, p)) !== g) continue;
    if (expanded >= cap) return null;
    expanded++;
    const bs = unpack(pk);
    const region = reach(p, bs, rSt, ++rTag);
    let solved = true;
    for (const b of bs) {
      if (!goalSet[b]) { solved = false; break; }
    }
    if (solved) {
      if (exit < 0 || rSt[exit] === rTag) return reconstruct(s);
      continue;
    }
    for (let i = 0; i < n; i++) {
      for (let k = 0; k < 4; k++) {
        const dest = bs[i] + D[k];
        if (!floor[dest] || !live[dest]) continue;
        if (bs.includes(dest)) continue;
        const from = bs[i] - D[k];
        if (rSt[from] !== rTag) continue;
        const nb = bs.slice();
        nb[i] = dest;
        nb.sort((a, b) => a - b);
        if (dead22(dest, nb)) continue;
        const npk = pack(nb);
        const h = heur(npk, nb);
        if (h === Infinity) continue;
        const nreg = reach(bs[i], nb, cSt, ++cTag);
        let np = bs[i];
        for (const c of nreg) if (c < np) np = c;
        const ng = g + 1;
        const nkey = keyOf(npk, np);
        const old = best.get(nkey);
        if (old !== undefined && old <= ng) continue;
        best.set(nkey, ng);
        const t = pl.length;
        pl.push(np);
        bx.push(npk);
        gg.push(ng);
        ff.push(ng + weight * h);
        sq.push(seq++);
        par.push(s);
        pd.push(k);
        pb.push(bs[i]);
        hpush(t);
      }
    }
  }
  return null;
}

// Turn a push list into moves: walk to each pushing cell, then push.
// Returns null when a walk is impossible.
function stitch(bd: Board, pushes: Push[]): string | null {
  const { W, H, wall, goal } = bd;
  const N = W * H;
  const D = [-W, W, -1, 1];
  let player = bd.player;
  const boxAt = new Uint8Array(N);
  for (const b of bd.boxes) boxAt[b] = 1;
  let moves = '';
  const walk = (from: number, to: number): string | null => {
    if (from === to) return '';
    const prev = new Int32Array(N).fill(-1);
    const q: number[] = [from];
    prev[from] = from;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      for (let k = 0; k < 4; k++) {
        const n2 = c + D[k];
        if (wall[n2] || boxAt[n2] || prev[n2] >= 0) continue;
        prev[n2] = c;
        if (n2 === to) {
          let s = '';
          for (let t = to; t !== from; t = prev[t]) {
            const d = t - prev[t];
            s = (d === -W ? 'u' : d === W ? 'd' : d === -1 ? 'l' : 'r') + s;
          }
          return s;
        }
        q.push(n2);
      }
    }
    return null;
  };
  for (const p of pushes) {
    const w = walk(player, p.b - D[p.d]);
    if (w === null) return null;
    moves += w;
    moves += CH[p.d].toUpperCase();
    boxAt[p.b] = 0;
    boxAt[p.b + D[p.d]] = 1;
    player = p.b;
  }
  for (let i = 0; i < N; i++) if (boxAt[i] && !goal[i]) return null;
  return moves;
}

// Independent replay of a move string on the level.
function replay(bd: Board, moves: string): boolean {
  const { W, H, wall, goal } = bd;
  const N = W * H;
  const D = [-W, W, -1, 1];
  let player = bd.player;
  const boxAt = new Uint8Array(N);
  for (const b of bd.boxes) boxAt[b] = 1;
  for (const ch of moves) {
    const k = 'udlr'.indexOf(ch.toLowerCase());
    if (k < 0) return false;
    const n2 = player + D[k];
    if (wall[n2]) return false;
    if (boxAt[n2]) {
      const b2 = n2 + D[k];
      if (wall[b2] || boxAt[b2]) return false;
      boxAt[n2] = 0;
      boxAt[b2] = 1;
    }
    player = n2;
  }
  for (let i = 0; i < N; i++) if (boxAt[i] && !goal[i]) return false;
  return true;
}

function solveLevel(text: string): string {
  const bd = parse(text);
  const { W, H, wall, goal } = bd;
  const N = W * H;
  const hy = Math.floor(bd.player / W);
  const DIRS = [-W, W, -1, 1];
  // Rooms: connected components of floor cells off the hallway row.
  const comp = new Int32Array(N).fill(-1);
  type Room = { cells: number[]; door: number; entry: number; boxes: number[]; goals: number[] };
  const rooms: Room[] = [];
  for (let i = 0; i < N; i++) {
    if (wall[i] || Math.floor(i / W) === hy || comp[i] >= 0) continue;
    const cells: number[] = [i];
    comp[i] = rooms.length;
    for (let q = 0; q < cells.length; q++) {
      const c = cells[q];
      for (const d of DIRS) {
        const n2 = c + d;
        if (!wall[n2] && Math.floor(n2 / W) !== hy && comp[n2] < 0) {
          comp[n2] = rooms.length;
          cells.push(n2);
        }
      }
    }
    rooms.push({ cells, door: -1, entry: -1, boxes: [], goals: [] });
  }
  let ok = rooms.length > 0;
  for (const r of rooms) {
    const doors: number[] = [];
    for (const c of r.cells) {
      if (Math.floor((c + W) / W) === hy && !wall[c + W]) doors.push(c);
      else if (Math.floor((c - W) / W) === hy && !wall[c - W]) doors.push(c);
    }
    if (doors.length !== 1) { ok = false; break; }
    r.door = doors[0];
    r.entry = Math.floor(r.door / W) < hy ? r.door + W : r.door - W;
  }
  if (ok) {
    for (const b of bd.boxes) {
      if (comp[b] < 0) { ok = false; break; }
      rooms[comp[b]].boxes.push(b);
    }
  }
  if (ok) {
    for (let i = 0; i < N; i++) {
      if (goal[i] && comp[i] >= 0) rooms[comp[i]].goals.push(i);
    }
    for (const r of rooms) {
      if (r.boxes.length === 0 || r.boxes.length !== r.goals.length) { ok = false; break; }
    }
  }
  if (ok) {
    rooms.sort((a, b) => a.door - b.door);
    const all: Push[] = [];
    for (const r of rooms) {
      const floor = new Uint8Array(N);
      for (const c of r.cells) floor[c] = 1;
      floor[r.entry] = 1;
      let sol = pushSearch(W, H, wall, floor, r.goals, r.boxes, r.entry, r.door, CAP_OPT, 1);
      if (!sol) sol = pushSearch(W, H, wall, floor, r.goals, r.boxes, r.entry, r.door, CAP_W, 3);
      if (!sol) sol = pushSearch(W, H, wall, floor, r.goals, r.boxes, r.entry, r.door, CAP_W, 30);
      if (!sol) { ok = false; break; }
      all.push(...sol);
    }
    if (ok) {
      const moves = stitch(bd, all);
      if (moves !== null && replay(bd, moves)) return moves;
    }
  }
  // Fallback: one push search over the whole level.
  const allFloor = new Uint8Array(N);
  for (let i = 0; i < N; i++) if (!wall[i]) allFloor[i] = 1;
  const allGoals: number[] = [];
  for (let i = 0; i < N; i++) if (goal[i]) allGoals.push(i);
  let sol = pushSearch(W, H, wall, allFloor, allGoals, bd.boxes, bd.player, -1, CAP_ALL, 1);
  if (!sol) sol = pushSearch(W, H, wall, allFloor, allGoals, bd.boxes, bd.player, -1, CAP_ALL, 3);
  if (sol) {
    const moves = stitch(bd, sol);
    if (moves !== null && replay(bd, moves)) return moves;
  }
  return '';
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}

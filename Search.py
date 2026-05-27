"""
COS30019 Assignment 2A — Route Finding Search Visualiser
=========================================================
Usage:
  GUI mode  : python search.py
  CLI mode  : python search.py <filename> <method>
              methods: DFS  BFS  GBFS  AS  CUS1  CUS2

Algorithms
----------
  DFS   — Depth-First Search (uninformed)
  BFS   — Breadth-First Search (uninformed)
  GBFS  — Greedy Best-First Search (informed, heuristic only)
  AS    — A* Search (informed, g + h)
  CUS1  — Uniform Cost Search (uninformed, optimal-cost path)
  CUS2  — Iterative Deepening A* / IDA* (informed, optimal path,
           least moves to goal)

Heuristic (GBFS / AS / CUS2): straight-line (Euclidean) distance
to the nearest destination node.
"""

import sys, os, math, time, threading, heapq
from collections import defaultdict, deque

# Tkinter is only needed for GUI mode; guard the import so that CLI
# mode works on headless systems or when tkinter is unavailable.
try:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk
    _HAS_TK = True
except ImportError:
    _HAS_TK = False
    # Provide minimal stubs so class definitions below don't raise NameError.
    # These are never instantiated in CLI mode.
    class _Stub:
        Canvas = object
        Tk     = object
        NORMAL = DISABLED = HORIZONTAL = LAST = BOTH = X = Y = LEFT = RIGHT \
               = TOP = BOTTOM = END = WORD = FLAT = DISABLED = NORMAL \
               = "stub"
        def __getattr__(self, _): return "stub"
    tk = _Stub()
    class _TtkStub:
        def __getattr__(self, _): return lambda *a, **k: None
    ttk = _TtkStub()
    filedialog = messagebox = _TtkStub()

# ══════════════════════════════════════════════════════════════════
#  PARSER
# ══════════════════════════════════════════════════════════════════
def parse_map(filepath):
    with open(filepath) as f:
        lines = [l.split("//")[0].strip() for l in f]
    lines = [l for l in lines if l]

    origin       = int(lines[0])
    destinations = list(map(int, lines[1].split(";")))
    nodes, edges = {}, []

    for line in lines[2:]:
        if ":" in line:
            n, coord = line.split(":")
            x, y = coord.strip("()").split(",")
            nodes[int(n)] = (float(x), float(y))
        elif "," in line:
            parts = line.split(",")
            edges.append((int(parts[0]), int(parts[1]), float(parts[2])))

    adj = defaultdict(list)
    for n1, n2, d in edges:
        adj[n1].append((n2, d))
    for k in adj:
        adj[k].sort(key=lambda x: x[0])

    return origin, destinations, nodes, dict(adj), edges


# ══════════════════════════════════════════════════════════════════
#  HEURISTIC — straight-line distance to nearest destination
# ══════════════════════════════════════════════════════════════════
def make_heuristic(nodes, destinations):
    def h(node):
        if node not in nodes:
            return float("inf")
        nx, ny = nodes[node]
        return min(
            math.hypot(nx - nodes[d][0], ny - nodes[d][1])
            for d in destinations if d in nodes
        )
    return h


def all_heuristics(nodes, destinations):
    h = make_heuristic(nodes, destinations)
    return {n: h(n) for n in nodes}


# ══════════════════════════════════════════════════════════════════
#  SEARCH ALGORITHMS
#  Each yields (frontier_list, visited_set, current_node, path, cost)
#  at every step, then finally returns via the callback mechanism.
#  step_callback(frontier, visited, current, path, cost) -> bool
#    return False to abort
# ══════════════════════════════════════════════════════════════════

def _neighbours(node, adj):
    return adj.get(node, [])


# ── DFS ──────────────────────────────────────────────────────────
def dfs(origin, destinations, adj, step_callback=None):
    dest_set = set(destinations)
    stack    = [(origin, [origin], 0.0)]
    visited  = set()
    created  = 1

    while stack:
        node, path, cost = stack.pop()
        if node in visited:
            continue
        visited.add(node)

        if step_callback:
            front = [s[0] for s in stack]
            if step_callback(front, visited, node, path, cost) is False:
                return None, [], 0.0, created

        if node in dest_set:
            return node, path, cost, created

        for nb, ec in reversed(_neighbours(node, adj)):
            if nb not in visited:
                stack.append((nb, path + [nb], cost + ec))
                created += 1

    return None, [], 0.0, created


# ── BFS ──────────────────────────────────────────────────────────
def bfs(origin, destinations, adj, step_callback=None):
    dest_set = set(destinations)
    queue    = deque([(origin, [origin], 0.0)])
    visited  = set([origin])
    created  = 1

    while queue:
        node, path, cost = queue.popleft()

        if step_callback:
            front = [q[0] for q in queue]
            if step_callback(front, set(visited), node, path, cost) is False:
                return None, [], 0.0, created

        if node in dest_set:
            return node, path, cost, created

        for nb, ec in _neighbours(node, adj):
            if nb not in visited:
                visited.add(nb)
                queue.append((nb, path + [nb], cost + ec))
                created += 1

    return None, [], 0.0, created


# ── GBFS ─────────────────────────────────────────────────────────
def gbfs(origin, destinations, nodes_coords, adj, step_callback=None):
    dest_set = set(destinations)
    h        = make_heuristic(nodes_coords, destinations)
    heap     = [(h(origin), 0, origin, [origin], 0.0)]
    visited  = set()
    counter  = 1
    created  = 1

    while heap:
        _, _, node, path, cost = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)

        if step_callback:
            front = [x[2] for x in heap]
            if step_callback(front, visited, node, path, cost) is False:
                return None, [], 0.0, created

        if node in dest_set:
            return node, path, cost, created

        for nb, ec in _neighbours(node, adj):
            if nb not in visited:
                heapq.heappush(heap, (h(nb), counter, nb, path + [nb], cost + ec))
                counter += 1
                created += 1

    return None, [], 0.0, created


# ── A* ───────────────────────────────────────────────────────────
def astar(origin, destinations, nodes_coords, adj, step_callback=None):
    dest_set = set(destinations)
    h        = make_heuristic(nodes_coords, destinations)
    heap     = [(h(origin), 0, origin, [origin], 0.0)]
    visited  = set()
    counter  = 1
    created  = 1

    while heap:
        f, _, node, path, cost = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)

        if step_callback:
            front = [x[2] for x in heap]
            if step_callback(front, visited, node, path, cost) is False:
                return None, [], 0.0, created

        if node in dest_set:
            return node, path, cost, created

        for nb, ec in _neighbours(node, adj):
            if nb not in visited:
                g = cost + ec
                heapq.heappush(heap, (g + h(nb), counter, nb, path + [nb], g))
                counter += 1
                created += 1

    return None, [], 0.0, created


# ── CUS1 — Uniform Cost Search (uninformed, optimal cost) ────────
def cus1(origin, destinations, adj, step_callback=None):
    """Uniform Cost Search: uninformed method that finds the
    minimum-cost path by always expanding the cheapest frontier node.
    Satisfies the CUS1 requirement (uninformed custom strategy)."""
    dest_set = set(destinations)
    heap     = [(0.0, 0, origin, [origin])]
    visited  = set()
    counter  = 1
    created  = 1

    while heap:
        cost, _, node, path = heapq.heappop(heap)
        if node in visited:
            continue
        visited.add(node)

        if step_callback:
            front = [x[2] for x in heap]
            if step_callback(front, visited, node, path, cost) is False:
                return None, [], 0.0, created

        if node in dest_set:
            return node, path, cost, created

        for nb, ec in _neighbours(node, adj):
            if nb not in visited:
                heapq.heappush(heap, (cost + ec, counter, nb, path + [nb]))
                counter += 1
                created += 1

    return None, [], 0.0, created


# ── CUS2 — Iterative Deepening A* (IDA*) (informed, least moves) ─
def cus2(origin, destinations, nodes_coords, adj, step_callback=None):
    """IDA* (Iterative Deepening A*): informed custom strategy.
    Performs DFS bounded by an f = g + h threshold, increasing the
    threshold each iteration.  Finds the optimal-cost path while
    using O(path-length) memory.  Satisfies the CUS2 requirement
    (informed custom strategy, finds shortest / least-moves path)."""
    dest_set = set(destinations)
    h        = make_heuristic(nodes_coords, destinations)
    created  = [1]          # list so inner functions can mutate it

    def _dfs(node, g, threshold, path_set, path, cost):
        """Return (next_threshold | -1_if_found, goal, result_path, result_cost)."""
        f = g + h(node)
        if f > threshold:
            return f, None, [], 0.0

        if node in dest_set:
            return -1, node, path[:], g

        minimum = float("inf")
        for nb, ec in _neighbours(node, adj):
            if nb not in path_set:          # avoid cycles within this path
                path.append(nb)
                path_set.add(nb)
                created[0] += 1

                if step_callback:
                    if step_callback([], set(path_set), nb, path[:], g + ec) is False:
                        # abort requested
                        path.pop(); path_set.discard(nb)
                        return -2, None, [], 0.0

                t, goal, rpath, rcost = _dfs(nb, g + ec, threshold,
                                             path_set, path, g + ec)
                path.pop()
                path_set.discard(nb)

                if t == -1:               # solution found
                    return -1, goal, rpath, rcost
                if t == -2:               # abort
                    return -2, None, [], 0.0
                if t < minimum:
                    minimum = t

        return minimum, None, [], 0.0

    threshold = h(origin)
    while True:
        path     = [origin]
        path_set = {origin}
        t, goal, rpath, rcost = _dfs(origin, 0.0, threshold, path_set, path, 0.0)

        if t == -1:
            return goal, rpath, rcost, created[0]
        if t == -2 or t == float("inf"):
            return None, [], 0.0, created[0]
        threshold = t


# ══════════════════════════════════════════════════════════════════
#  CLI RUNNER
# ══════════════════════════════════════════════════════════════════
METHODS = ["DFS", "BFS", "GBFS", "AS", "CUS1", "CUS2"]

def run_search(method, origin, destinations, nodes, adj):
    m = method.upper()
    if   m == "DFS":  return dfs(origin, destinations, adj)
    elif m == "BFS":  return bfs(origin, destinations, adj)
    elif m == "GBFS": return gbfs(origin, destinations, nodes, adj)
    elif m == "AS":   return astar(origin, destinations, nodes, adj)
    elif m == "CUS1": return cus1(origin, destinations, adj)
    elif m == "CUS2": return cus2(origin, destinations, nodes, adj)
    else:
        print(f"Unknown method '{method}'. Choose from: {', '.join(METHODS)}")
        return None, [], 0.0, 0


def _format_cost(cost):
    """Return cost as an integer string if whole, otherwise as a float string."""
    if cost == int(cost):
        return str(int(cost))
    return str(cost)


def cli_mode(filepath, method):
    # ── validate method ──────────────────────────────────────────
    m = method.upper()
    if m not in METHODS:
        print(f"Error: Unknown method '{method}'.")
        print(f"Valid methods: {', '.join(METHODS)}")
        print("Usage: python search.py <filename> <method>")
        sys.exit(1)

    # ── parse map file ───────────────────────────────────────────
    try:
        origin, destinations, nodes, adj, edges = parse_map(filepath)
    except FileNotFoundError:
        print(f"Error: File '{filepath}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error reading map: {e}")
        sys.exit(1)

    goal, path, cost, created = run_search(method, origin, destinations, nodes, adj)

    # ── output format required by the assignment spec ──
    print(f"{filepath} {m}")
    print(f"Goal State: {', '.join(map(str, destinations))}")
    print(f"Starting Node: {origin}")
    if goal is not None:
        print(f"Destination Node: {goal}")
        print(f"Number of nodes created: {created}")
        print(f"Path: {' -> '.join(map(str, path))}")
        print(f"Path Cost: {_format_cost(cost)}")
    else:
        print("No solution found.")


# ══════════════════════════════════════════════════════════════════
#  THEME / CONSTANTS
# ══════════════════════════════════════════════════════════════════
BG        = "#f5f6fa"
PANEL     = "#ffffff"
PANEL2    = "#eef0f7"
BORDER    = "#c8cce0"
ACCENT    = "#3a5fd9"       # blue
ACCENT2   = "#e03c3c"       # red/orange  — current node
SUCCESS   = "#1a9e72"       # teal        — solution
WARN      = "#c08800"       # yellow      — frontier
VISITED_C = "#8fa3e8"       # medium blue-purple — visited
NODE_BG   = "#c8cfe8"
NODE_FG   = "#1a1d2e"
MUTED     = "#7a82aa"
GRID_MAIN = "#dce0ee"       # graph-paper major lines
GRID_MINOR= "#eaedf6"       # graph-paper minor lines
AXIS_CLR  = "#b0b8d8"

FONT_HEAD = ("Segoe UI", 11, "bold")
FONT_UI   = ("Segoe UI", 10)
FONT_MONO = ("Consolas", 9)
FONT_TINY = ("Consolas", 8)

METHOD_COLORS = {
    "DFS":  "#6c8eff",
    "BFS":  "#4ecca3",
    "GBFS": "#ffd166",
    "AS":   "#ff9f43",
    "CUS1": "#ee5a94",
    "CUS2": "#a78bfa",
}

PADDING   = 55
NODE_R    = 16



# ══════════════════════════════════════════════════════════════════
#  GRAPH CANVAS  (graph-paper style)
# ══════════════════════════════════════════════════════════════════
class GraphCanvas(tk.Canvas):
    def __init__(self, master, **kw):
        super().__init__(master, bg=BG, highlightthickness=0, **kw)
        self.reset_state()
        self.bind("<Configure>", lambda e: self.redraw())

    def reset_state(self):
        self.nodes         = {}
        self.adj           = {}
        self.edges_raw     = []
        self.origin        = None
        self.dests         = []
        self.visited       = set()
        self.frontier      = set()
        self.solution_path = []
        self.current_node  = None
        self.heuristics    = {}   # node -> float
        self._min_x = self._min_y = 0
        self._scale_x = self._scale_y = 1
        self._canvas_h = 500

    def load(self, nodes, adj, edges_raw, origin, dests, heuristics):
        self.reset_state()
        self.nodes      = nodes
        self.adj        = adj
        self.edges_raw  = edges_raw
        self.origin     = origin
        self.dests      = dests
        self.heuristics = heuristics
        self._compute_transform()
        self.redraw()

    def update_state(self, visited, frontier, current, solution_path=None):
        self.visited      = set(visited)
        self.frontier     = set(frontier)
        self.current_node = current
        if solution_path is not None:
            self.solution_path = solution_path
        self.redraw()

    # ── coordinate transform ─────────────────────────────────────
    def _compute_transform(self):
        if not self.nodes: return
        xs = [v[0] for v in self.nodes.values()]
        ys = [v[1] for v in self.nodes.values()]
        mn_x, mx_x = 0, max(xs)   # always start from 0
        mn_y, mx_y = 0, max(ys)   # always start from 0
        w = max(self.winfo_width(),  400)
        h = max(self.winfo_height(), 300)
        sx = (w - 2*PADDING) / max(mx_x - mn_x, 1)
        sy = (h - 2*PADDING) / max(mx_y - mn_y, 1)
        self._scale_x  = sx
        self._scale_y  = sy
        self._min_x    = mn_x
        self._min_y    = mn_y
        self._canvas_h = h
        self._canvas_w = w
        # graph coordinate range for grid
        self._gx_min = mn_x
        self._gx_max = mx_x
        self._gy_min = mn_y
        self._gy_max = mx_y

    def to_canvas(self, x, y):
        cx = (x - self._min_x) * self._scale_x + PADDING
        cy = self._canvas_h - ((y - self._min_y) * self._scale_y + PADDING)
        return cx, cy

    # ── full redraw ───────────────────────────────────────────────
    def redraw(self, *_):
        self._compute_transform()
        self.delete("all")
        if not self.nodes: return
        self._draw_grid()
        self._draw_axes()
        self._draw_edges()
        self._draw_nodes()

    # ── graph-paper grid ─────────────────────────────────────────
    def _draw_grid(self):
        w, h = self._canvas_w, self._canvas_h
        # Work out a nice grid step in graph units
        span_x = max(self._gx_max - self._gx_min, 1)
        span_y = max(self._gy_max - self._gy_min, 1)

        def nice_step(span):
            raw = span / 10
            mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
            for m in [1, 2, 5, 10]:
                if raw <= m * mag:
                    return m * mag
            return mag * 10

        step = nice_step(min(span_x, span_y))

        # minor lines (step / 5)
        minor = step / 5
        x = math.floor(self._gx_min / minor) * minor
        while x <= self._gx_max + minor:
            cx, _ = self.to_canvas(x, 0)
            self.create_line(cx, 0, cx, h, fill=GRID_MINOR, width=1)
            x += minor
        y = math.floor(self._gy_min / minor) * minor
        while y <= self._gy_max + minor:
            _, cy = self.to_canvas(0, y)
            self.create_line(0, cy, w, cy, fill=GRID_MINOR, width=1)
            y += minor

        # major lines
        x = math.floor(self._gx_min / step) * step
        while x <= self._gx_max + step:
            cx, _ = self.to_canvas(x, 0)
            self.create_line(cx, 0, cx, h, fill=GRID_MAIN, width=1)
            self.create_text(cx, h - 12, text=f"{x:.0f}",
                             fill=MUTED, font=FONT_TINY, anchor="s")
            x += step
        y = math.floor(self._gy_min / step) * step
        while y <= self._gy_max + step:
            _, cy = self.to_canvas(0, y)
            self.create_line(0, cy, w, cy, fill=GRID_MAIN, width=1)
            self.create_text(14, cy, text=f"{y:.0f}",
                             fill=MUTED, font=FONT_TINY, anchor="w")
            y += step

    def _draw_axes(self):
        w, h = self._canvas_w, self._canvas_h
        # x-axis along bottom of graph area
        _, ay = self.to_canvas(0, self._gy_min)
        ax, _  = self.to_canvas(self._gx_min, 0)
        self.create_line(PADDING-10, ay, w-10, ay, fill=AXIS_CLR, width=2)
        self.create_line(ax, h-PADDING+10, ax, 10, fill=AXIS_CLR, width=2)
        # labels
        self.create_text(w-6, ay, text="x", fill=MUTED, font=FONT_UI, anchor="e")
        self.create_text(ax, 6,  text="y", fill=MUTED, font=FONT_UI, anchor="n")

    # ── edges ─────────────────────────────────────────────────────
    def _draw_edges(self):
        sol_set = set()
        for i in range(len(self.solution_path) - 1):
            sol_set.add((self.solution_path[i], self.solution_path[i+1]))

        # Find which pairs are bidirectional so we can offset them
        edge_pairs = set((n1, n2) for n1, n2, _ in self.edges_raw)
        drawn = set()

        for n1, n2, cost in self.edges_raw:
            if n1 not in self.nodes or n2 not in self.nodes: continue
            x1, y1 = self.to_canvas(*self.nodes[n1])
            x2, y2 = self.to_canvas(*self.nodes[n2])

            in_sol = (n1, n2) in sol_set
            clr    = SUCCESS if in_sol else BORDER
            w      = 3 if in_sol else 1.5

            # Only offset if the reverse direction also exists (bidirectional)
            is_bidir = (n2, n1) in edge_pairs
            dx, dy = x2-x1, y2-y1
            length = math.hypot(dx, dy) or 1
            if is_bidir:
                px, py = -dy/length * 4, dx/length * 4
            else:
                px, py = 0, 0

            self.create_line(x1+px, y1+py, x2+px, y2+py,
                             fill=clr, width=w,
                             arrow=tk.LAST, arrowshape=(10, 12, 4),
                             smooth=False)

            # Only draw cost label once per pair
            pair = tuple(sorted([n1, n2]))
            if pair not in drawn:
                mx, my = (x1+x2)/2 + px, (y1+y2)/2 + py - 9
                self.create_text(mx, my, text=f"{cost:.0f}",
                                 fill=MUTED, font=FONT_TINY)
                drawn.add(pair)

    # ── nodes ─────────────────────────────────────────────────────
    def _draw_nodes(self):
        for nid, (nx, ny) in self.nodes.items():
            cx, cy = self.to_canvas(nx, ny)
            r = NODE_R

            # pick colours
            if nid == self.current_node:
                fill, ring, rw = ACCENT2, "#ffffff", 3
            elif self.solution_path and nid in self.solution_path:
                fill, ring, rw = SUCCESS,  "#ffffff", 2
            elif nid in self.visited:
                fill, ring, rw = VISITED_C, "#5570cc", 2
            elif nid in self.frontier:
                fill, ring, rw = "#ffe066", WARN,    2
            elif nid == self.origin:
                fill, ring, rw = "#4cd6a8", SUCCESS, 2
            elif nid in self.dests:
                fill, ring, rw = "#ff8080", ACCENT2, 2
            else:
                fill, ring, rw = NODE_BG,  BORDER,   1

            # glow for current
            if nid == self.current_node:
                self.create_oval(cx-r-5, cy-r-5, cx+r+5, cy+r+5,
                                 fill="", outline=ACCENT2, width=1,
                                 stipple="gray25")

            self.create_oval(cx-r, cy-r, cx+r, cy+r,
                             fill=fill, outline=ring, width=rw)
            # use white text on vivid fills, dark on light fills
            text_clr = "#ffffff" if nid in (self.current_node,) or \
                       (self.solution_path and nid in self.solution_path) or \
                       nid in self.visited else NODE_FG
            self.create_text(cx, cy, text=str(nid),
                             fill=text_clr, font=("Consolas", 9, "bold"))

            # heuristic label above node
            if nid in self.heuristics:
                hv = self.heuristics[nid]
                self.create_text(cx, cy - r - 7,
                                 text=f"h={hv:.1f}",
                                 fill=WARN, font=FONT_TINY)

            # special badges
            if nid == self.origin:
                self.create_text(cx + r + 2, cy - r,
                                 text="S", fill=SUCCESS, font=FONT_TINY)
            if nid in self.dests:
                self.create_text(cx + r + 2, cy - r,
                                 text="G", fill=ACCENT2, font=FONT_TINY)

        self._draw_legend()

    def _draw_legend(self):
        items = [
            (ACCENT2,  "Current"),
            (SUCCESS,  "Solution"),
            (VISITED_C,"Visited"),
            (WARN,     "Frontier"),
        ]
        x0, y0 = 8, 8
        for clr, lbl in items:
            self.create_rectangle(x0, y0, x0+11, y0+11,
                                  fill=clr, outline="")
            self.create_text(x0+15, y0+5, text=lbl,
                             fill=MUTED, font=FONT_TINY, anchor="w")
            y0 += 17


# ══════════════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ══════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("COS30019 — Route Finding Visualiser")
        self.configure(bg=BG)
        self.geometry("1280x760")
        self.minsize(900, 600)

        self._map_data  = None
        self._filename  = ""
        self._filepath  = ""
        self._running   = False
        self._stop_flag = threading.Event()
        self._step_event= threading.Event()
        self._delay     = 600

        self._build_ui()
        self._apply_styles()

    # ── ttk styles ───────────────────────────────────────────────
    def _apply_styles(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure("TCombobox",
                    fieldbackground=PANEL2, background=PANEL2,
                    foreground=NODE_FG, selectbackground=BORDER,
                    selectforeground=NODE_FG, arrowcolor=ACCENT)
        s.map("TCombobox",
              fieldbackground=[("readonly", PANEL2)],
              background=[("readonly", PANEL2)])
        s.configure("TScale",
                    background=PANEL, troughcolor=BORDER,
                    sliderthickness=14)
    # ── UI layout ────────────────────────────────────────────────
    def _build_ui(self):
        self._build_topbar()

        pane = tk.PanedWindow(self, orient=tk.HORIZONTAL,
                              bg=BG, sashwidth=4, sashrelief=tk.FLAT)
        pane.pack(fill=tk.BOTH, expand=True, padx=6, pady=(0,6))

        # Graph canvas
        self._canvas = GraphCanvas(pane)
        pane.add(self._canvas, minsize=500)

        # Right sidebar
        sidebar = tk.Frame(pane, bg=PANEL, width=300)
        sidebar.pack_propagate(False)
        pane.add(sidebar, minsize=260)
        self._build_sidebar(sidebar)

    def _build_topbar(self):
        bar = tk.Frame(self, bg=PANEL, pady=0)
        bar.pack(fill=tk.X, side=tk.TOP)

        # Left: logo area
        logo = tk.Frame(bar, bg=PANEL, padx=14, pady=10)
        logo.pack(side=tk.LEFT)
        tk.Label(logo, text="ROUTE FINDER",
                 bg=PANEL, fg=ACCENT,
                 font=("Segoe UI", 13, "bold")).pack(side=tk.LEFT)
        tk.Label(logo, text=" COS30019",
                 bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 10)).pack(side=tk.LEFT, pady=1)

        # Separator
        tk.Frame(bar, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=6, padx=4)

        # File load
        self._file_label = tk.Label(bar, text="No file loaded",
                                    bg=PANEL, fg=MUTED, font=FONT_UI)
        self._file_label.pack(side=tk.LEFT, padx=(10,4))
        self._btn(bar, "📂 Open Map", self._load_file, ACCENT).pack(side=tk.LEFT, padx=4)

        # Method selector
        tk.Frame(bar, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=6, padx=6)
        tk.Label(bar, text="Method:", bg=PANEL, fg=MUTED,
                 font=FONT_UI).pack(side=tk.LEFT)
        self._method_var = tk.StringVar(value="DFS")
        self._method_combo = ttk.Combobox(
            bar, textvariable=self._method_var,
            values=METHODS, state="readonly", width=6,
            font=("Consolas", 10, "bold"))
        self._method_combo.pack(side=tk.LEFT, padx=(4,10))
        self._method_combo.bind("<<ComboboxSelected>>", self._on_method_change)

        # Action buttons
        self._btn_run  = self._btn(bar, "▶  Run",  self._run_search,
                                   SUCCESS, state=tk.DISABLED)
        self._btn_run.pack(side=tk.LEFT, padx=3)
        self._btn_step = self._btn(bar, "⏭  Step", self._step,
                                   WARN, state=tk.DISABLED)
        self._btn_step.pack(side=tk.LEFT, padx=3)
        self._btn_reset= self._btn(bar, "↺  Reset", self._reset, MUTED)
        self._btn_reset.pack(side=tk.LEFT, padx=3)

        # Speed
        tk.Frame(bar, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=6, padx=8)
        tk.Label(bar, text="Speed:", bg=PANEL, fg=MUTED,
                 font=FONT_UI).pack(side=tk.LEFT)
        self._speed = tk.IntVar(value=600)
        ttk.Scale(bar, from_=100, to=2000, orient=tk.HORIZONTAL,
                     variable=self._speed, length=110,
                     command=lambda v: setattr(self, "_delay", int(float(v)))
                     ).pack(side=tk.LEFT, padx=(4, 0))
        tk.Label(bar, text="slow", bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=(2,8))

        # CLI runner on right
        tk.Frame(bar, bg=BORDER, width=1).pack(side=tk.RIGHT, fill=tk.Y, pady=6, padx=4)
        self._btn(bar, "⌨  CLI Runner", self._open_cli_window,
                  "#a78bfa").pack(side=tk.RIGHT, padx=6)

        # Bottom divider
        tk.Frame(self, bg=BORDER, height=1).pack(fill=tk.X)

    def _btn(self, parent, text, cmd, color=ACCENT, state=tk.NORMAL):
        b = tk.Button(parent, text=text, command=cmd,
                      bg=PANEL2, fg=color,
                      activebackground=BORDER, activeforeground=color,
                      relief=tk.FLAT, font=FONT_UI,
                      padx=10, pady=5, state=state,
                      cursor="hand2", bd=0)
        return b

    def _build_sidebar(self, parent):
        def section(title):
            h = tk.Frame(parent, bg=PANEL)
            h.pack(fill=tk.X, padx=10, pady=(10,0))
            tk.Label(h, text=title, bg=PANEL, fg=ACCENT,
                     font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
            tk.Frame(parent, bg=BORDER, height=1).pack(fill=tk.X, padx=10, pady=(2,0))

        def text_box(height, fg=NODE_FG):
            t = tk.Text(parent, height=height, bg=PANEL2, fg=fg,
                        font=FONT_MONO, relief=tk.FLAT,
                        state=tk.DISABLED, padx=6, pady=4,
                        insertbackground=ACCENT, wrap=tk.WORD,
                        highlightthickness=1, highlightbackground=BORDER)
            t.pack(fill=tk.X, padx=10, pady=4)
            return t

        section("MAP INFO")
        self._info_box = text_box(5, MUTED)

        section("STATUS")
        self._status_box = text_box(5)

        section("FRONTIER  (next to expand)")
        self._frontier_box = tk.Listbox(
            parent, bg=PANEL2, fg=WARN, font=FONT_MONO,
            relief=tk.FLAT, selectbackground=BORDER,
            height=6, activestyle="none",
            highlightthickness=1, highlightbackground=BORDER)
        self._frontier_box.pack(fill=tk.X, padx=10, pady=4)

        section("HEURISTICS  (h = Straight-Line Distance to goal)")
        self._h_box = tk.Listbox(
            parent, bg=PANEL2, fg="#a78bfa", font=FONT_MONO,
            relief=tk.FLAT, selectbackground=BORDER,
            height=6, activestyle="none",
            highlightthickness=1, highlightbackground=BORDER)
        self._h_box.pack(fill=tk.X, padx=10, pady=4)

        section("RESULT")
        self._result_box = text_box(7, SUCCESS)

    # ── helpers ──────────────────────────────────────────────────
    def _set_text(self, w, txt):
        w.config(state=tk.NORMAL)
        w.delete("1.0", tk.END)
        w.insert(tk.END, txt)
        w.config(state=tk.DISABLED)

    def _on_method_change(self, *_):
        m = self._method_var.get()
        clr = METHOD_COLORS.get(m, ACCENT)
        self._btn_run.config(fg=clr)
        self._set_text(self._status_box,
            f"Method set to {m}.\nPress ▶ Run to start.")

    def _update_info(self):
        if not self._map_data: return
        o, d, nodes, adj, _ = self._map_data
        self._set_text(self._info_box,
            f"File    : {self._filename}\n"
            f"Origin  : {o}\n"
            f"Goals   : {', '.join(map(str, d))}\n"
            f"Nodes   : {len(nodes)}\n"
            f"Edges   : {sum(len(v) for v in adj.values())}"
        )

    def _update_h_box(self, heuristics):
        self._h_box.delete(0, tk.END)
        for n, hv in sorted(heuristics.items()):
            self._h_box.insert(tk.END, f"  Node {n:>3}  →  h = {hv:6.2f}")

    def _update_frontier(self, frontier):
        self._frontier_box.delete(0, tk.END)
        for n in frontier:
            hv = self._heuristics.get(n, 0)
            self._frontier_box.insert(tk.END, f"  Node {n}   h={hv:.1f}")

    def _update_status(self, visited, frontier, current, cost):
        m = self._method_var.get()
        self._set_text(self._status_box,
            f"Method   : {m}\n"
            f"Current  : {current}\n"
            f"Visited  : {sorted(visited)}\n"
            f"Frontier : {len(frontier)}\n"
            f"Cost     : {cost:.2f}"
        )
        self._update_frontier(frontier)

    # ── load file ────────────────────────────────────────────────
    def _load_file(self):
        path = filedialog.askopenfilename(
            title="Open Map File",
            filetypes=[("Text files","*.txt"),("All files","*.*")])
        if not path: return
        try:
            origin, dests, nodes, adj, edges = parse_map(path)
            self._heuristics = all_heuristics(nodes, dests)
            self._map_data   = (origin, dests, nodes, adj, edges)
            self._filename   = os.path.basename(path)
            self._filepath   = path
            self._file_label.config(text=self._filename, fg=NODE_FG)
            self._canvas.load(nodes, adj, edges, origin, dests, self._heuristics)
            self._btn_run.config(state=tk.NORMAL)
            self._btn_step.config(state=tk.NORMAL)
            self._update_info()
            self._update_h_box(self._heuristics)
            self._set_text(self._status_box, "Map loaded.\nChoose a method and press ▶ Run.")
            self._set_text(self._result_box, "—")
        except Exception as e:
            messagebox.showerror("Parse Error", str(e))

    # ── reset ────────────────────────────────────────────────────
    def _reset(self):
        self._running = False
        self._stop_flag.set()
        if self._map_data:
            o, d, nodes, adj, edges = self._map_data
            self._canvas.load(nodes, adj, edges, o, d, self._heuristics)
            self._set_text(self._status_box, "Reset. Press ▶ Run.")
            self._set_text(self._result_box, "—")
            self._frontier_box.delete(0, tk.END)
        self._btn_run.config(state=tk.NORMAL if self._map_data else tk.DISABLED)

    # ── step ─────────────────────────────────────────────────────
    def _step(self):
        self._step_event.set()

    # ── run search ───────────────────────────────────────────────
    def _run_search(self):
        if not self._map_data or self._running:
            return
        self._reset()
        self._running   = True
        self._stop_flag = threading.Event()
        self._step_event= threading.Event()
        self._btn_run.config(state=tk.DISABLED)

        origin, destinations, nodes, adj, edges = self._map_data
        method  = self._method_var.get()
        results = {}

        def step_cb(frontier, visited, current, path, cost):
            if self._stop_flag.is_set():
                return False
            self._step_event.clear()
            self.after(0, self._canvas.update_state, visited, frontier, current)
            self.after(0, self._update_status, visited, frontier, current, cost)
            deadline = time.time() + self._delay / 1000.0
            while time.time() < deadline:
                if self._step_event.is_set() or self._stop_flag.is_set():
                    break
                time.sleep(0.02)
            return True

        def run():
            try:
                m = method.upper()
                if   m == "DFS":
                    r = dfs(origin, destinations, adj, step_cb)
                elif m == "BFS":
                    r = bfs(origin, destinations, adj, step_cb)
                elif m == "GBFS":
                    r = gbfs(origin, destinations, nodes, adj, step_cb)
                elif m == "AS":
                    r = astar(origin, destinations, nodes, adj, step_cb)
                elif m == "CUS1":
                    r = cus1(origin, destinations, adj, step_cb)
                elif m == "CUS2":
                    r = cus2(origin, destinations, nodes, adj, step_cb)
                else:
                    r = (None, [], 0.0, 0)

                if not self._stop_flag.is_set():
                    results["goal"], results["path"], \
                    results["cost"], results["created"] = r
                else:
                    results["goal"] = None
            except Exception as e:
                results["goal"] = None
                results["error"] = str(e)
            finally:
                self.after(0, self._on_done, results)

        threading.Thread(target=run, daemon=True).start()

    def _on_done(self, results):
        self._running = False
        self._btn_run.config(state=tk.NORMAL)
        goal    = results.get("goal")
        path    = results.get("path", [])
        cost    = results.get("cost", 0)
        created = results.get("created", 0)
        method  = self._method_var.get()

        if goal is not None:
            self._canvas.update_state(
                self._canvas.visited, [], None, path)
            route = " → ".join(map(str, path))
            self._set_text(self._result_box,
                f"✓ Goal reached: {goal}\n"
                f"Method       : {method}\n"
                f"Nodes created: {created}\n"
                f"Path cost    : {cost:.2f}\n\n"
                f"Path:\n{route}"
            )
            self._set_text(self._status_box,
                f"Search complete!\nGoal: {goal}  Cost: {cost:.2f}")
        else:
            self._set_text(self._result_box,
                "✗ No solution found." if not results.get("error")
                else f"Error: {results['error']}")
            self._set_text(self._status_box, "Search complete.\nNo solution found.")

    # ── CLI runner window ─────────────────────────────────────────
    def _open_cli_window(self):
        win = tk.Toplevel(self)
        win.title("CLI Runner")
        win.configure(bg=PANEL)
        win.geometry("680x480")
        win.resizable(True, True)

        # Title
        tk.Label(win, text="CLI Runner", bg=PANEL, fg=ACCENT,
                 font=("Segoe UI", 12, "bold")).pack(pady=(14,4), padx=16, anchor="w")
        tk.Frame(win, bg=BORDER, height=1).pack(fill=tk.X, padx=16)

        # File row
        row1 = tk.Frame(win, bg=PANEL)
        row1.pack(fill=tk.X, padx=16, pady=(10,2))
        tk.Label(row1, text="Map file:", bg=PANEL, fg=MUTED,
                 font=FONT_UI, width=9, anchor="w").pack(side=tk.LEFT)
        cli_file_var = tk.StringVar(value=self._filepath or "")
        tk.Entry(row1, textvariable=cli_file_var, bg=PANEL2, fg=NODE_FG,
                 font=FONT_MONO, relief=tk.FLAT, insertbackground=ACCENT,
                 highlightthickness=1, highlightbackground=BORDER
                 ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(4,4))

        def browse():
            p = filedialog.askopenfilename(
                title="Select Map",
                filetypes=[("Text","*.txt"),("All","*.*")])
            if p: cli_file_var.set(p)
        tk.Button(row1, text="Browse", command=browse,
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                  font=FONT_UI, padx=8).pack(side=tk.LEFT)

        # Method row
        row2 = tk.Frame(win, bg=PANEL)
        row2.pack(fill=tk.X, padx=16, pady=4)
        tk.Label(row2, text="Method:", bg=PANEL, fg=MUTED,
                 font=FONT_UI, width=9, anchor="w").pack(side=tk.LEFT)
        cli_method_var = tk.StringVar(value=self._method_var.get())
        for m in METHODS:
            rb = tk.Radiobutton(row2, text=m, variable=cli_method_var,
                                value=m, bg=PANEL, fg=METHOD_COLORS.get(m, ACCENT),
                                selectcolor=PANEL2, activebackground=PANEL,
                                font=("Consolas", 10, "bold"),
                                relief=tk.FLAT)
            rb.pack(side=tk.LEFT, padx=6)

        # Run button
        def do_run():
            fp = cli_file_var.get().strip()
            m  = cli_method_var.get()
            if not fp:
                messagebox.showwarning("Missing file", "Please enter a map file path.",
                                       parent=win)
                return
            out_box.config(state=tk.NORMAL)
            out_box.delete("1.0", tk.END)
            out_box.insert(tk.END, f"$ python search.py \"{fp}\" {m}\n\n")
            try:
                origin, dests, nodes, adj, edges = parse_map(fp)
                goal, path, cost, created = run_search(m, origin, dests, nodes, adj)
                lines = [
                    f"Starting Node   : {origin}",
                    f"Method          : {m}",
                ]
                if goal is not None:
                    lines += [
                        f"Destination Node: {goal}",
                        f"Nodes created   : {created}",
                        f"Path            : {' -> '.join(map(str, path))}",
                        f"Path Cost       : {cost:.2f}",
                    ]
                else:
                    lines.append("No solution found.")
                out_box.insert(tk.END, "\n".join(lines))
                # Also load into GUI if this file differs
                if fp != self._filepath:
                    try:
                        h = all_heuristics(nodes, dests)
                        self._heuristics = h
                        self._map_data   = (origin, dests, nodes, adj, edges)
                        self._filename   = os.path.basename(fp)
                        self._filepath   = fp
                        self._file_label.config(text=self._filename, fg=NODE_FG)
                        self._canvas.load(nodes, adj, edges, origin, dests, h)
                        self._btn_run.config(state=tk.NORMAL)
                        self._btn_step.config(state=tk.NORMAL)
                        self._update_info()
                        self._update_h_box(h)
                        self._method_var.set(m)
                    except Exception:
                        pass
            except Exception as e:
                out_box.insert(tk.END, f"Error: {e}")
            out_box.config(state=tk.DISABLED)

        btn_row = tk.Frame(win, bg=PANEL)
        btn_row.pack(fill=tk.X, padx=16, pady=6)
        tk.Button(btn_row, text="▶  Execute", command=do_run,
                  bg=PANEL2, fg=SUCCESS, relief=tk.FLAT,
                  font=("Segoe UI", 10, "bold"),
                  padx=14, pady=6, cursor="hand2"
                  ).pack(side=tk.LEFT)

        # Output box
        tk.Frame(win, bg=BORDER, height=1).pack(fill=tk.X, padx=16)
        out_box = tk.Text(win, bg="#f5f6fa", fg="#1a7a50",
                          font=("Consolas", 10), relief=tk.FLAT,
                          state=tk.DISABLED, padx=10, pady=8,
                          highlightthickness=0, wrap=tk.WORD)
        out_box.pack(fill=tk.BOTH, expand=True, padx=16, pady=10)
        out_box.config(state=tk.NORMAL)
        out_box.insert(tk.END, "Select a file and method, then click ▶ Execute.\n")
        out_box.config(state=tk.DISABLED)


# ══════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════
_USAGE = (
    "Usage: python search.py <filename> <method>\n"
    f"       methods: {', '.join(METHODS)}"
)

if __name__ == "__main__":
    if len(sys.argv) >= 3:
        cli_mode(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        print(_USAGE)
        sys.exit(1)
    else:
        if not _HAS_TK:
            print("tkinter is not available — cannot launch GUI.")
            print(_USAGE)
            sys.exit(1)
        App().mainloop()
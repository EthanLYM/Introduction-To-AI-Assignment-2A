import sys, os, math, time, threading, heapq, ctypes
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

try:
    # Tell Windows that this app is explicitly High-DPI aware
    ctypes.windll.shcore.SetProcessDpiAwareness(2) # 2 = Per Monitor DPI Aware
except Exception:
    try:
        # Fallback for older Windows versions (Windows 7/8)
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass # Fallback for non-Windows platforms

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
    created  = [1]

    def _dfs(node, g, threshold, path_set, path, cost):
        """Return (next_threshold | -1_if_found, goal, result_path, result_cost)."""
        f = g + h(node)
        if f > threshold:
            return f, None, [], 0.0

        if node in dest_set:
            return -1, node, path[:], g

        minimum = float("inf")
        for nb, ec in _neighbours(node, adj):
            if nb not in path_set:
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

                if t == -1:
                    return -1, goal, rpath, rcost
                if t == -2:
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
    print(f"Starting Node: {origin}")
    if goal is not None:
        print(f"Destination Node: {goal}")
        print(f"Number of nodes created: {created}")
        print(f"Path: {' -> '.join(map(str, path))}")
        print(f"Path Cost: {_format_cost(cost)}")
    else:
        print("No solution found.")


# ══════════════════════════════════════════════════════════════════
#  UI COLORS & THEME CONSTANTS 
# ══════════════════════════════════════════════════════════════════
# Default global fallbacks representing Dark Mode
BG        = "#0B0F19"      
PANEL     = "#161B26"       
PANEL2    = "#1F2633"       
BORDER    = "#2F374A"       
NODE_FG   = "#F0F4F8"       
MUTED     = "#6B7A99"       

# Color indicators 
ACCENT    = "#00E5FF"       # Start/Origin highlights (Cyan)
ACCENT2   = "#FF2A85"       # Active/Current Node (Magenta)
SUCCESS   = "#00E676"       # Solution Path color (Green)
WARN      = "#FFD600"       # Frontier list (Yellow)
VISITED_C = "#7C4DFF"       # Visited Node color (Purple)
GOAL_CLR  = "#FF8F00"       # Destination Node color (Orange)

METHOD_COLORS = {
    "DFS":  "#00E5FF",
    "BFS":  "#00E676",
    "GBFS": "#FFD600",
    "AS":   "#FF2A85",
    "CUS1": "#7C4DFF",
    "CUS2": "#FF8F00",
}

PADDING   = 60
NODE_R    = 18

FONT_HEAD = ("Segoe UI", 11, "bold")
FONT_UI   = ("Segoe UI", 10)
FONT_MONO = ("Consolas", 10)
FONT_TINY = ("Consolas", 8)


# ══════════════════════════════════════════════════════════════════
#  VECTOR BLUEPRINT GRAPH CANVAS (Supports Drag to Pan and Zoom)
# ══════════════════════════════════════════════════════════════════
class GraphCanvas(tk.Canvas):
    def __init__(self, master, **kw):
        super().__init__(master, bg=BG, highlightthickness=0, **kw)
        self.dark_mode = True
        self.reset_state()
        
        # Interactive Zoom & Pan Parameters
        self.zoom_factor = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.hovered_node = None
        
        self.bind("<Configure>", self._on_canvas_configure)
        
        # Drag to Pan (Bind to Left, Middle, and Right Clicks)
        self.bind("<ButtonPress-1>", self._start_pan)
        self.bind("<B1-Motion>", self._pan)
        self.bind("<ButtonPress-2>", self._start_pan)
        self.bind("<B2-Motion>", self._pan)
        self.bind("<ButtonPress-3>", self._start_pan)
        self.bind("<B3-Motion>", self._pan)
        
        self.bind("<MouseWheel>", self._zoom)          # Scroll Wheel Zoom
        self.bind("<Motion>", self._on_mouse_move)    # Hover Detector
        
        self.hover_callback = None

        self._configure_job = None

    def _on_canvas_configure(self, event=None):
        if self._configure_job is not None:
            self.after_cancel(self._configure_job)
        self._configure_job = self.after(30, self.redraw)

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
        self.heuristics    = {}
        self._min_x = self._min_y = 0
        self._scale_x = self._scale_y = 1
        self._canvas_h = 500
        self.zoom_factor = 1.0
        self.pan_x = 0
        self.pan_y = 0

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

    # ── Pan & Drag ────────────────────────────────────────────────
    def _start_pan(self, event):
        self._drag_start_x = event.x
        self._drag_start_y = event.y

    def _pan(self, event):
        dx = event.x - self._drag_start_x
        dy = event.y - self._drag_start_y
        self.pan_x += dx
        self.pan_y += dy
        self._drag_start_x = event.x
        self._drag_start_y = event.y
        self.redraw()

    # ── Zoom ──────────────────────────────────────────────────────
    def _zoom(self, event):
        factor = 1.1 if event.delta > 0 else 0.9
        self.zoom_factor = max(0.3, min(self.zoom_factor * factor, 8.0))
        self.redraw()

    # ── Hover Detector ────────────────────────────────────────────
    def _on_mouse_move(self, event):
        if not self.nodes: return
        found_node = None
        for nid, (nx, ny) in self.nodes.items():
            cx, cy = self.to_canvas(nx, ny)
            dist = math.hypot(event.x - cx, event.y - cy)
            if dist < NODE_R + 6:
                found_node = nid
                break
        
        if found_node is not None:
            self.config(cursor="hand2")
        else:
            self.config(cursor="fleur")

        if found_node != self.hovered_node:
            self.hovered_node = found_node
            self.redraw()
            if self.hover_callback:
                self.hover_callback(found_node)

    def _compute_transform(self):
        if not self.nodes: return
        xs = [v[0] for v in self.nodes.values()]
        ys = [v[1] for v in self.nodes.values()]
        mn_x, mx_x = 0, max(xs)
        mn_y, mx_y = 0, max(ys)
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
        self._gx_min = mn_x
        self._gx_max = mx_x
        self._gy_min = mn_y
        self._gy_max = mx_y

    def to_canvas(self, x, y):
        cx = (x - self._min_x) * self._scale_x + PADDING
        cy = self._canvas_h - ((y - self._min_y) * self._scale_y + PADDING)
        
        center_x = self._canvas_w / 2
        center_y = self._canvas_h / 2
        cx = center_x + (cx - center_x) * self.zoom_factor + self.pan_x
        cy = center_y + (cy - center_y) * self.zoom_factor + self.pan_y
        return cx, cy

    def redraw(self, *_):
        self._compute_transform()
        self.delete("all")
        if not self.nodes: return
        
        # Color palettes configured depending on Theme state
        self.bg_color         = "#0B0F19" if self.dark_mode else "#ffffff"
        self.grid_main_color  = "#121926" if self.dark_mode else "#e5e7eb"
        self.grid_minor_color = "#0F1420" if self.dark_mode else "#f3f4f6"
        self.text_color       = "#F0F4F8" if self.dark_mode else "#1a1d2e"
        self.muted_color      = "#6B7A99" if self.dark_mode else "#7a82aa"
        self.border_color     = "#2F374A" if self.dark_mode else "#c8cce0"
        self.node_bg_color    = "#2C3448" if self.dark_mode else "#c8cfe8"

        self._draw_radar_grid()
        self._draw_interactive_edges()
        self._draw_interactive_nodes()

    def _draw_radar_grid(self):
        w, h = self._canvas_w, self._canvas_h

        # Target ~60-80 px between major gridlines regardless of zoom/pan.
        # pixels_per_unit already embeds zoom_factor via to_canvas(), so we
        # compute it by measuring two world points that are 1 unit apart.
        cx0, cy0 = self.to_canvas(0, 0)
        cx1, _   = self.to_canvas(1, 0)
        _,   cy1 = self.to_canvas(0, 1)
        px_per_unit_x = abs(cx1 - cx0) if abs(cx1 - cx0) > 0 else 1
        px_per_unit_y = abs(cy1 - cy0) if abs(cy1 - cy0) > 0 else 1

        TARGET_PX = 70  # desired pixel gap between major lines

        def nice_step(px_per_unit):
            raw = TARGET_PX / px_per_unit   # world-units that fill TARGET_PX px
            if raw <= 0:
                return 1
            mag = 10 ** math.floor(math.log10(raw))
            for m in [1, 2, 5, 10]:
                if raw <= m * mag:
                    return m * mag
            return mag * 10

        step_x = nice_step(px_per_unit_x)
        step_y = nice_step(px_per_unit_y)

        # Determine visible world-space extents (inverse of to_canvas)
        # so we only draw lines that are actually on screen.
        def world_x_range():
            center_x = w / 2
            # cx = center_x + (raw_cx - center_x)*zoom + pan_x  →  solve for data-x
            # raw_cx = (x - min_x)*scale_x + PADDING
            # Rearranging: x = (raw_cx - PADDING)/scale_x + min_x
            def canvas_to_world_x(cx):
                raw_cx = (cx - self.pan_x - center_x) / self.zoom_factor + center_x
                return (raw_cx - PADDING) / self._scale_x + self._min_x
            return canvas_to_world_x(0), canvas_to_world_x(w)

        def world_y_range():
            center_y = h / 2
            def canvas_to_world_y(cy):
                raw_cy = (cy - self.pan_y - center_y) / self.zoom_factor + center_y
                return (self._canvas_h - raw_cy - PADDING) / self._scale_y + self._min_y
            return canvas_to_world_y(h), canvas_to_world_y(0)   # note: y flipped

        wx_min, wx_max = world_x_range()
        wy_min, wy_max = world_y_range()

        minor_x = step_x / 5
        minor_y = step_y / 5

        x = math.floor(wx_min / minor_x) * minor_x
        while x <= wx_max + minor_x:
            cx, _ = self.to_canvas(x, 0)
            if 0 <= cx <= w:
                self.create_line(cx, 0, cx, h, fill=self.grid_minor_color, width=1)
            x += minor_x

        y = math.floor(wy_min / minor_y) * minor_y
        while y <= wy_max + minor_y:
            _, cy = self.to_canvas(0, y)
            if 0 <= cy <= h:
                self.create_line(0, cy, w, cy, fill=self.grid_minor_color, width=1)
            y += minor_y

        x = math.floor(wx_min / step_x) * step_x
        while x <= wx_max + step_x:
            cx, _ = self.to_canvas(x, 0)
            if 0 <= cx <= w:
                self.create_line(cx, 0, cx, h, fill=self.grid_main_color, width=1)
                self.create_text(cx, h - 14, text=f"{x:.0f}",
                                 fill=self.muted_color, font=FONT_TINY, anchor="s")
            x += step_x

        y = math.floor(wy_min / step_y) * step_y
        while y <= wy_max + step_y:
            _, cy = self.to_canvas(0, y)
            if 0 <= cy <= h:
                self.create_line(0, cy, w, cy, fill=self.grid_main_color, width=1)
                self.create_text(16, cy, text=f"{y:.0f}",
                                 fill=self.muted_color, font=FONT_TINY, anchor="w")
            y += step_y

    def _draw_interactive_edges(self):
        sol_set = set()
        for i in range(len(self.solution_path) - 1):
            sol_set.add((self.solution_path[i], self.solution_path[i+1]))

        edge_pairs = set((n1, n2) for n1, n2, _ in self.edges_raw)
        processed_pairs = set()

        for n1, n2, cost in self.edges_raw:
            if n1 not in self.nodes or n2 not in self.nodes: 
                continue
            
            pair = tuple(sorted([n1, n2]))
            if pair in processed_pairs:
                continue
            processed_pairs.add(pair)

            x1, y1 = self.to_canvas(*self.nodes[n1])
            x2, y2 = self.to_canvas(*self.nodes[n2])

            in_sol = (n1, n2) in sol_set or (n2, n1) in sol_set
            
            # Checks if any valid outgoing directed traversable connection exists between the nodes
            is_hover_related = (
                (self.hovered_node == n1 and (n1, n2) in edge_pairs) or
                (self.hovered_node == n2 and (n2, n1) in edge_pairs)
            )
            
            if in_sol:
                clr, shadow_clr, w = SUCCESS, "#00331A" if self.dark_mode else "#D1FAE5", 4
            elif is_hover_related:
                clr, shadow_clr, w = ACCENT, "#002B30" if self.dark_mode else "#E0F7FA", 3
            else:
                clr, shadow_clr, w = self.border_color, self.grid_minor_color, 1.5

            is_bidir = (n2, n1) in edge_pairs and (n1, n2) in edge_pairs
            arrow_setting = tk.BOTH if is_bidir else tk.LAST

            if in_sol or is_hover_related:
                self.create_line(x1, y1, x2, y2,
                                 fill=shadow_clr, width=w+6,
                                 arrow=arrow_setting, arrowshape=(12, 14, 5))

            self.create_line(x1, y1, x2, y2,
                             fill=clr, width=w,
                             arrow=arrow_setting, arrowshape=(10, 12, 4))

            mx, my = (x1 + x2) / 2, (y1 + y2) / 2 - 8
            self.create_text(mx, my, text=f"{cost:.0f}",
                             fill=ACCENT if is_hover_related else self.muted_color, 
                             font=FONT_TINY)

    def _draw_interactive_nodes(self):
        for nid, (nx, ny) in self.nodes.items():
            cx, cy = self.to_canvas(nx, ny)
            r = NODE_R * (1.15 if nid == self.hovered_node else 1.0)

            # Color assign keys 
            if nid == self.current_node:
                fill, ring, rw = ACCENT2, "#FFFFFF", 3
            elif self.solution_path and nid in self.solution_path:
                fill, ring, rw = SUCCESS,  "#FFFFFF", 2
            elif nid in self.visited:
                fill, ring, rw = VISITED_C, "#7C4DFF", 2
            elif nid in self.frontier:
                fill, ring, rw = "#FFE066", WARN,    2
            elif nid == self.origin:
                fill, ring, rw = "#00E5FF", ACCENT, 2
            elif nid in self.dests:
                fill, ring, rw = GOAL_CLR, "#FF6F00", 2
            else:
                fill, ring, rw = self.node_bg_color, self.border_color, 1.5

            if nid == self.current_node or nid == self.hovered_node:
                self.create_oval(cx-r-6, cy-r-6, cx+r+6, cy+r+6,
                                 fill="", outline=ACCENT if nid == self.hovered_node else ACCENT2, 
                                 width=1.5)

            self.create_oval(cx-r, cy-r, cx+r, cy+r, fill=fill, outline=ring, width=rw)
            
            text_clr = "#0B0F19" if nid in (self.current_node, self.origin) or \
                       (self.solution_path and nid in self.solution_path) else self.text_color
                       
            self.create_text(cx, cy, text=str(nid),
                             fill=text_clr, font=("Consolas", 10, "bold"))

            if nid in self.heuristics:
                hv = self.heuristics[nid]
                self.create_text(cx, cy - r - 9,
                                 text=f"h={hv:.1f}",
                                 fill=WARN if self.dark_mode else "#B8860B", font=FONT_TINY)

        self._draw_legend_panel()

    def _draw_legend_panel(self):
        items = [
            ("#00E5FF", "Origin"),
            (GOAL_CLR,  "Destination"),
            (ACCENT2,   "Current Node"),
            (SUCCESS,   "Solution Path"),
            (VISITED_C, "Visited Node"),
            (WARN,      "Frontier"),
        ]
        x0, y0 = 12, 12
        for clr, lbl in items:
            self.create_rectangle(x0, y0, x0+12, y0+12, fill=clr, outline="")
            self.create_text(x0+18, y0+6, text=lbl,
                             fill=self.text_color, font=FONT_TINY, anchor="w")
            y0 += 18


# ══════════════════════════════════════════════════════════════════
#  HOVER REACTIVE SYSTEM BUTTONS
# ══════════════════════════════════════════════════════════════════
class InteractiveButton(tk.Button):
    def __init__(self, master, text, command, color, **kwargs):
        super().__init__(master, text=text, command=command,
                         bg="#1A2130", fg=color,
                         activebackground=color, activeforeground="#0B0F19",
                         font=("Segoe UI", 9, "bold"), bd=0, relief="flat",
                         highlightthickness=1, highlightbackground=color,
                         padx=12, pady=6, cursor="hand2", **kwargs)
        self.color = color
        self.bind("<Enter>", lambda e: self.config(bg=color, fg="#0B0F19"))
        self.bind("<Leave>", lambda e: self.config(bg="#1A2130" if self.master.cget("bg") == "#161B26" else "#eef0f7", fg=color))


# ══════════════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ══════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("COS30019 — Route Finding Visualizer")
        self.dark_mode = True 
        self.geometry("1400x820")
        self.minsize(1000, 700)

        self._map_data  = None
        self._filename  = ""
        self._filepath  = ""
        self._running   = False
        self._stop_flag = threading.Event()
        self._step_event= threading.Event()
        self._delay     = 600

        self._build_dashboard_ui()
        self._apply_ttk_styles()
        self._on_method_change()
        self._apply_theme() 

    def _apply_ttk_styles(self):
        self._style = ttk.Style(self)
        self._style.theme_use("clam")

    def _build_dashboard_ui(self):
        self._topbar_ref = tk.Frame(self, bg=PANEL, pady=6)
        self._topbar_ref.pack(fill=tk.X, side=tk.TOP)

        logo = tk.Frame(self._topbar_ref, bg=PANEL, padx=14)
        logo.pack(side=tk.LEFT)
        self._logo_frame = logo # Keep track of parent logo Frame
        self._logo_label = tk.Label(logo, text="ROUTE FINDER", bg=PANEL, fg=ACCENT, font=("Segoe UI", 12, "bold"), bd=0, highlightthickness=0)
        self._logo_label.pack(side=tk.LEFT)
        self._logo_sub = tk.Label(logo, text=" [COS30019]", bg=PANEL, fg=MUTED, font=("Segoe UI", 9, "bold"), bd=0, highlightthickness=0)
        self._logo_sub.pack(side=tk.LEFT, pady=1, padx=(6,0))

        tk.Frame(self._topbar_ref, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=6, padx=8)

        self._file_label = tk.Label(self._topbar_ref, text="No map loaded", bg=PANEL, fg=MUTED, font=FONT_UI, bd=0, highlightthickness=0)
        self._file_label.pack(side=tk.LEFT, padx=(10,4))
        self._btn_open = InteractiveButton(self._topbar_ref, "📂 Open Map", self._load_file, ACCENT)
        self._btn_open.pack(side=tk.LEFT, padx=4)

        tk.Frame(self._topbar_ref, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=6, padx=8)

        self._method_label = tk.Label(self._topbar_ref, text="Method:", bg=PANEL, fg=NODE_FG, font=FONT_UI, bd=0, highlightthickness=0)
        self._method_label.pack(side=tk.LEFT)
        self._method_var = tk.StringVar(value="DFS")
        self._method_combo = ttk.Combobox(
            self._topbar_ref, textvariable=self._method_var,
            values=METHODS, state="readonly", width=8,
            font=("Consolas", 10, "bold"))
        self._method_combo.pack(side=tk.LEFT, padx=(6,10))
        self._method_combo.bind("<<ComboboxSelected>>", self._on_method_change)

        self._btn_run  = InteractiveButton(self._topbar_ref, "▶ Run",  self._run_search, SUCCESS, state=tk.DISABLED)
        self._btn_run.pack(side=tk.LEFT, padx=3)
        self._btn_step = InteractiveButton(self._topbar_ref, "⏭ Step", self._step, WARN, state=tk.DISABLED)
        self._btn_step.pack(side=tk.LEFT, padx=3)
        self._btn_reset = InteractiveButton(self._topbar_ref, "↺ Reset", self._reset, MUTED)
        self._btn_reset.pack(side=tk.LEFT, padx=3)

        tk.Frame(self._topbar_ref, bg=BORDER, width=1).pack(side=tk.LEFT, fill=tk.Y, pady=6, padx=8)

        self._speed_label = tk.Label(self._topbar_ref, text="Speed:", bg=PANEL, fg=MUTED, font=FONT_UI, bd=0, highlightthickness=0)
        self._speed_label.pack(side=tk.LEFT)
        self._speed = tk.IntVar(value=600)
        self._speed_scale = ttk.Scale(self._topbar_ref, from_=100, to=2000, orient=tk.HORIZONTAL,
                     variable=self._speed, length=100,
                     command=lambda v: setattr(self, "_delay", int(float(v))))
        self._speed_scale.pack(side=tk.LEFT, padx=(4, 0))
        
        # Explicitly added label to define the slow side of speed settings
        self._speed_slow_label = tk.Label(self._topbar_ref, text="slow", bg=PANEL, fg=MUTED, font=("Segoe UI", 8), bd=0, highlightthickness=0)
        self._speed_slow_label.pack(side=tk.LEFT, padx=(2,8))

        self._btn_theme = InteractiveButton(self._topbar_ref, "🌓 Theme", self._toggle_theme, ACCENT)
        self._btn_theme.pack(side=tk.RIGHT, padx=14)

        self._div_line = tk.Frame(self, bg=BORDER, height=1)
        self._div_line.pack(fill=tk.X)

        self._pane = tk.PanedWindow(self, orient=tk.HORIZONTAL, bg=BORDER, sashwidth=4, sashrelief=tk.FLAT)
        self._pane.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0,8))
        self._pane_resize_job = None
        self._pane.bind("<Configure>", self._on_pane_configure)
        self._pane.bind("<ButtonRelease-1>", lambda e: self._canvas.redraw())
        self._pane.bind("<B1-Motion>", self._on_sash_drag)

        self._canvas = GraphCanvas(self._pane)
        self._canvas.hover_callback = self._on_node_hover_update
        self._pane.add(self._canvas, minsize=650)

        self._sidebar_ref = tk.Frame(self._pane, bg=PANEL, width=320)
        self._pane.add(self._sidebar_ref, minsize=320)
        self._build_sidebar_hud(self._sidebar_ref)

    def _build_sidebar_hud(self, parent):
        self._headers = []
        self._header_frames = []
        self._div_lines = []
        self._text_widgets = []

        def hud_panel(title):
            h = tk.Frame(parent, bg=PANEL)
            h.pack(fill=tk.X, padx=12, pady=(6,0)) # Compact top padding
            self._header_frames.append(h) # Track parent header frame!
            
            lbl = tk.Label(h, text=title, bg=PANEL, fg=ACCENT, font=("Segoe UI", 9, "bold"), bd=0, highlightthickness=0)
            lbl.pack(side=tk.LEFT)
            self._headers.append(lbl)

            line = tk.Frame(parent, bg=BORDER, height=1)
            line.pack(fill=tk.X, padx=12, pady=(2,0))
            self._div_lines.append(line)

        def output_screen(height, fg=NODE_FG):
            t = tk.Text(parent, height=height, bg=PANEL2, fg=fg,
                        font=FONT_MONO, relief=tk.FLAT,
                        state=tk.DISABLED, padx=8, pady=4,
                        highlightthickness=1, highlightbackground=BORDER)
            t.pack(fill=tk.X, padx=12, pady=2) # Compact spacing
            # Store the intended fg so _apply_theme can restore it per-widget
            t._fixed_fg = fg
            self._text_widgets.append(t)
            return t

        hud_panel("MAP INFO")
        self._info_box = output_screen(4, MUTED)

        # Height of 4 displays hover connection details with zero vertical scrolling
        hud_panel("NODE INSPECTOR")
        self._inspect_box = output_screen(4, ACCENT)
        self._set_text(self._inspect_box, "Hover over a node...")

        hud_panel("STATUS")
        self._status_box = output_screen(3)

        hud_panel("FRONTIER (next to expand)")
        self._frontier_box = tk.Listbox(
            parent, bg=PANEL2, fg=WARN, font=FONT_MONO,
            relief=tk.FLAT, selectbackground=BORDER,
            height=4, activestyle="none",
            highlightthickness=1, highlightbackground=BORDER)
        self._frontier_box.pack(fill=tk.X, padx=12, pady=2)

        # Ensure all heuristics on default map render with zero scrolling
        hud_panel("HEURISTICS (h = Straight-Line Distance to goal)")
        self._h_box = tk.Listbox(
            parent, bg=PANEL2, fg="#A78BFA", font=FONT_MONO,
            relief=tk.FLAT, selectbackground=BORDER,
            height=7, activestyle="none",
            highlightthickness=1, highlightbackground=BORDER)
        self._h_box.pack(fill=tk.X, padx=12, pady=2)

        hud_panel("RESULT")
        self._result_box = output_screen(5, SUCCESS)
        # Reconfigured to use fill=BOTH and expand=True so the Result box dynamically occupies all remaining space
        self._result_box.pack_configure(fill=tk.BOTH, expand=True, pady=(2, 12))

    def _toggle_theme(self):
        self.dark_mode = not self.dark_mode
        self._apply_theme()

    def _apply_theme(self):
        # Base colors ( Obsidian dark mode vs clean white layout)
        bg     = "#0B0F19" if self.dark_mode else "#ffffff"
        panel  = "#161B26" if self.dark_mode else "#ffffff"
        panel2 = "#1F2633" if self.dark_mode else "#f8f9fa"
        border = "#2F374A" if self.dark_mode else "#cbd5e1"
        text   = "#F0F4F8" if self.dark_mode else "#1e293b"
        muted  = "#6B7A99" if self.dark_mode else "#64748b"

        self.configure(bg=bg)
        self._canvas.dark_mode = self.dark_mode
        self._canvas.configure(bg=bg)
        self._topbar_ref.configure(bg=panel)
        self._sidebar_ref.configure(bg=panel)
        self._pane.configure(bg=border)
        self._div_line.configure(bg=border)

        # Clean dynamic styles for headers & logo container frame
        self._logo_frame.configure(bg=panel)
        self._logo_label.configure(bg=panel, bd=0, highlightthickness=0)
        self._logo_sub.configure(bg=panel, fg=muted, bd=0, highlightthickness=0)
        self._file_label.configure(bg=panel, fg=text if self._map_data else muted, bd=0, highlightthickness=0)
        self._method_label.configure(bg=panel, fg=text, bd=0, highlightthickness=0)
        self._speed_label.configure(bg=panel, fg=muted, bd=0, highlightthickness=0)
        self._speed_slow_label.configure(bg=panel, fg=muted, bd=0, highlightthickness=0)

        # Dynamic TTK Styling state maps (Dropdown and Slider backgrounds respect active theme)
        self._style.map("TCombobox",
            fieldbackground=[("readonly", panel2)],
            background=[("readonly", panel2)],
            foreground=[("readonly", text)],
            selectbackground=[("readonly", border)],
            selectforeground=[("readonly", text)]
        )
        self._style.configure("TScale",
                            background=panel, troughcolor=border,
                            sliderthickness=14)

        for b in [self._btn_open, self._btn_run, self._btn_step, self._btn_reset, self._btn_theme]:
            b.configure(bg="#1A2130" if self.dark_mode else "#eef0f7")

        # Set clean flat textboxes highlights — preserve each widget's own fg color
        for t in self._text_widgets:
            # Widgets with a fixed accent color (result=green, inspect=cyan, info=muted)
            # keep that color in both themes; only plain NODE_FG boxes follow the theme text color.
            widget_fg = getattr(t, '_fixed_fg', None)
            if widget_fg in (None, NODE_FG):
                widget_fg = text
            t.configure(bg=panel2, fg=widget_fg, highlightbackground=border, highlightcolor=border)

        # Update Listbox styles 
        self._frontier_box.configure(bg=panel2, fg=WARN if self.dark_mode else "#B8860B", highlightbackground=border, highlightcolor=border)
        self._h_box.configure(bg=panel2, fg="#A78BFA" if self.dark_mode else "#6D28D9", highlightbackground=border, highlightcolor=border)

        # Dynamically updates parent container frames of each header 
        for h_frame in self._header_frames:
            h_frame.configure(bg=panel)

        # Update dynamic blue titles in Light Theme 
        header_color = ACCENT if self.dark_mode else "#0056b3"
        for lbl in self._headers:
            lbl.configure(bg=panel, fg=header_color, bd=0, highlightthickness=0)

        for line in self._div_lines:
            line.configure(bg=border)

        self._update_info_hud()
        if self._map_data:
            self._update_h_box(self._heuristics)
        self._canvas.redraw()

    def _set_text(self, w, txt):
        w.config(state=tk.NORMAL)
        w.delete("1.0", tk.END)
        w.insert(tk.END, txt)
        w.config(state=tk.DISABLED)

    def _on_method_change(self, *_):
        m = self._method_var.get()
        clr = METHOD_COLORS.get(m, ACCENT)
        self._btn_run.config(fg=clr)
        self._set_text(self._status_box, f"Method set to: {m}.\nSelect a run option to begin.")

    def _update_info_hud(self):
        if not self._map_data: return
        o, d, nodes, adj, _ = self._map_data
        self._set_text(self._info_box,
            f"File: {self._filename}\n"
            f"Origin: Node {o}\n"
            f"Destinations: {', '.join(map(str, d))}\n"
            f"Number of nodes: {len(nodes)}\n"
            f"Number of edges: {sum(len(v) for v in adj.values())}"
        )

    def _on_node_hover_update(self, hovered_nid):
        if not self._map_data or hovered_nid is None:
            self._set_text(self._inspect_box, "Hover over a node...")
            return
            
        o, d, nodes, adj, _ = self._map_data
        coords = nodes.get(hovered_nid, (0.0, 0.0))
        h_val = self._heuristics.get(hovered_nid, float("inf"))
        connections = [f"{nb} (distance: {c})" for nb, c in adj.get(hovered_nid, [])]
        conn_str = ", ".join(connections) if connections else "None"
        
        self._set_text(self._inspect_box,
            f"Node: {hovered_nid}\n"
            f"Coordinates: ({coords[0]:.1f}, {coords[1]:.1f})\n"
            f"Heuristic h: {h_val:.2f}\n"
            f"Connections: {conn_str}"
        )

    def _update_h_box(self, heuristics):
        self._h_box.delete(0, tk.END)
        for n, hv in sorted(heuristics.items()):
            self._h_box.insert(tk.END, f"  Node {n:>3}  →  h = {hv:6.2f}")

    def _update_frontier(self, frontier):
        self._frontier_box.delete(0, tk.END)
        for n in frontier:
            hv = self._heuristics.get(n, 0)
            self._frontier_box.insert(tk.END, f"  Node {n}  →  h={hv:.1f}")

    def _update_status(self, visited, frontier, current, cost):
        m = self._method_var.get()
        self._set_text(self._status_box,
            f"Method: {m}\n"
            f"Current Node: {current}\n"
            f"Visited Nodes: {len(visited)}\n"
            f"Frontier Nodes: {len(frontier)}\n"
            f"Path Cost: {cost:.2f}"
        )
        self._update_frontier(frontier)

    def _on_pane_configure(self, event=None):
        # Only redraw the canvas on configure; sidebar widgets are native Tk
        # frames so they reflow automatically — no extra redraw needed.
        # Use a short debounce so rapid events during window resize are coalesced.
        if self._pane_resize_job is not None:
            self.after_cancel(self._pane_resize_job)
        self._pane_resize_job = self.after(16, self._canvas.redraw)

    def _on_sash_drag(self, event=None):
        # During sash drag, skip mid-motion canvas redraws entirely.
        # _on_pane_configure fires but we cancel it immediately; the
        # ButtonRelease bind does the one final clean redraw.
        if self._pane_resize_job is not None:
            self.after_cancel(self._pane_resize_job)
        self._pane_resize_job = None

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
            
            text_color = "#F0F4F8" if self.dark_mode else "#1a1d2e"
            self._file_label.config(text=f"Loaded: {self._filename}", fg=text_color)
            
            self._canvas.load(nodes, adj, edges, origin, dests, self._heuristics)
            self._btn_run.config(state=tk.NORMAL)
            self._btn_step.config(state=tk.NORMAL)
            self._update_info_hud()
            self._update_h_box(self._heuristics)
            self._set_text(self._status_box, "Map loaded.\nSelect a method to begin.")
            self._set_text(self._result_box, "—")
        except Exception as e:
            messagebox.showerror("Parse Error", str(e))

    def _reset(self):
        self._running = False
        self._is_stepping = False
        self._stop_flag.set()
        self._step_event.set()
        
        if self._map_data:
            o, d, nodes, adj, edges = self._map_data
            self._canvas.load(nodes, adj, edges, o, d, self._heuristics)
            self._set_text(self._status_box, "Reset complete.\nSelect a run option to begin.")
            self._set_text(self._result_box, "—")
            self._frontier_box.delete(0, tk.END)
            
        self._btn_run.config(state=tk.NORMAL if self._map_data else tk.DISABLED)
        self._btn_step.config(state=tk.NORMAL if self._map_data else tk.DISABLED)

    def _step(self):
        if not self._running:
            self._run_search(start_in_step_mode=True)
            return
            
        self._is_stepping = True
        self._step_event.set()

    def _run_search(self, start_in_step_mode=False):
        if self._running:
            if self._is_stepping:
                self._is_stepping = False
                self._step_event.set()
            return

        # Clear any leftover solution/visited state from a previous run
        self._canvas.solution_path = []
        self._canvas.visited       = set()
        self._canvas.frontier      = set()
        self._canvas.current_node  = None
        self._canvas.redraw()
        self._set_text(self._result_box, "—")
        self._frontier_box.delete(0, tk.END)

        self._running   = True
        self._is_stepping = start_in_step_mode
        self._stop_flag = threading.Event()
        self._step_event= threading.Event()
        
        self._btn_run.config(state=tk.NORMAL)
        self._btn_step.config(state=tk.NORMAL)

        origin, destinations, nodes, adj, edges = self._map_data
        method  = self._method_var.get()
        results = {}

        def step_cb(frontier, visited, current, path, cost):
            if self._stop_flag.is_set():
                return False
                
            self.after(0, self._canvas.update_state, visited, frontier, current)
            self.after(0, self._update_status, visited, frontier, current, cost)
            
            if self._is_stepping:
                self._step_event.clear()
                while self._is_stepping and not self._step_event.is_set() and not self._stop_flag.is_set():
                    time.sleep(0.01)
            else:
                deadline = time.time() + self._delay / 1000.0
                while time.time() < deadline:
                    if self._is_stepping or self._stop_flag.is_set():
                        break
                    time.sleep(0.01)
                    
            return not self._stop_flag.is_set()

        def run():
            try:
                m = method.upper()
                if   m == "DFS":  r = dfs(origin, destinations, adj, step_cb)
                elif m == "BFS":  r = bfs(origin, destinations, adj, step_cb)
                elif m == "GBFS": r = gbfs(origin, destinations, nodes, adj, step_cb)
                elif m == "AS":   r = astar(origin, destinations, nodes, adj, step_cb)
                elif m == "CUS1": r = cus1(origin, destinations, adj, step_cb)
                elif m == "CUS2": r = cus2(origin, destinations, nodes, adj, step_cb)
                else:             r = (None, [], 0.0, 0)

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
        self._is_stepping = False
        self._btn_run.config(state=tk.NORMAL)
        self._btn_step.config(state=tk.NORMAL)
        
        goal    = results.get("goal")
        path    = results.get("path", [])
        cost    = results.get("cost", 0)
        created = results.get("created", 0)
        method  = self._method_var.get()

        if goal is not None:
            self._canvas.update_state(self._canvas.visited, [], None, path)
            route = " -> ".join(map(str, path))
            self._set_text(self._result_box,
                f"✓ Goal reached: {goal}\n"
                f"Method: {method}\n"
                f"Number of nodes created: {created}\n"
                f"Path Cost: {cost:.2f}\n\n"
                f"Path:\n{route}"
            )
            self._set_text(self._status_box, f"Search complete.\nGoal: {goal}  Cost: {cost:.2f}")
        else:
            self._set_text(self._result_box,
                "✗ No solution found." if not results.get("error")
                else f"Error: {results['error']}")
            self._set_text(self._status_box, "Search finished.\nNo solution found.")


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
            print("tkinter is not available — running CLI mode is required.")
            print(_USAGE)
            sys.exit(1)
        App().mainloop()
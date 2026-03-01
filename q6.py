import tkinter as tk
from tkinter import messagebox
import heapq
import random
import math
import time
from collections import defaultdict

# ─────────────────────────── COLOUR PALETTE ───────────────────────────
C = {
    "bg":       "#0D1117",
    "panel":    "#161B22",
    "border":   "#30363D",
    "start":    "#00D26A",
    "goal":     "#FF6B6B",
    "wall":     "#2D3748",
    "frontier": "#F6C90E",
    "visited":  "#3D5A80",
    "path":     "#00BFA5",
    "agent":    "#FF9F1C",
    "empty":    "#1C2333",
    "text":     "#E6EDF3",
    "muted":    "#8B949E",
    "accent":   "#58A6FF",
    "new_wall": "#FF4444",
}

ANIM_MS = 30
MOVE_MS = 120
SPAWN_P = 0.03
GAP     = 1

# ─────────────────────────── HEURISTICS ───────────────────────────────
def manhattan(a, b):  return abs(a[0]-b[0]) + abs(a[1]-b[1])
def euclidean(a, b):  return math.hypot(a[0]-b[0], a[1]-b[1])
def chebyshev(a, b):  return max(abs(a[0]-b[0]), abs(a[1]-b[1]))
HEURISTICS = {"Manhattan": manhattan, "Euclidean": euclidean, "Chebyshev": chebyshev}

DIRS_4 = [(-1,0),(1,0),(0,-1),(0,1)]
DIRS_8 = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

# ─────────────────────── SEARCH ALGORITHMS ────────────────────────────
def gbfs(grid, rows, cols, start, goal, h_fn, dirs):
    open_heap = [(h_fn(start,goal), 0, start)]
    came_from = {start: None}
    visited, frontier_history, counter = [], [], 0
    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        visited.append(cur)
        frontier_history.append(set(n for _,_,n in open_heap))
        if cur == goal:
            return _reconstruct(came_from, goal), visited, frontier_history
        for dr, dc in dirs:
            nb = (cur[0]+dr, cur[1]+dc)
            if (0<=nb[0]<rows and 0<=nb[1]<cols
                    and grid[nb[0]][nb[1]] not in (1,2)
                    and nb not in came_from):
                came_from[nb] = cur
                counter += 1
                heapq.heappush(open_heap, (h_fn(nb,goal), counter, nb))
    return [], visited, frontier_history

def astar(grid, rows, cols, start, goal, h_fn, dirs):
    g = defaultdict(lambda: float('inf'))
    g[start] = 0
    open_heap = [(h_fn(start,goal), 0, start)]
    came_from = {start: None}
    closed, visited, frontier_history, counter = set(), [], [], 0
    while open_heap:
        _, _, cur = heapq.heappop(open_heap)
        if cur in closed: continue
        closed.add(cur)
        visited.append(cur)
        frontier_history.append(set(n for _,_,n in open_heap) - closed)
        if cur == goal:
            return _reconstruct(came_from, goal), visited, frontier_history
        for dr, dc in dirs:
            nb = (cur[0]+dr, cur[1]+dc)
            if (0<=nb[0]<rows and 0<=nb[1]<cols
                    and grid[nb[0]][nb[1]] not in (1,2)
                    and nb not in closed):
                step = math.hypot(dr,dc) if abs(dr)+abs(dc)==2 else 1.0
                t = g[cur] + step
                if t < g[nb]:
                    g[nb] = t; came_from[nb] = cur; counter += 1
                    heapq.heappush(open_heap, (t+h_fn(nb,goal), counter, nb))
    return [], visited, frontier_history

def _reconstruct(came_from, goal):
    path, cur = [], goal
    while cur is not None:
        path.append(cur); cur = came_from[cur]
    path.reverse(); return path

# ──────────────────────── SCROLLABLE FRAME ────────────────────────────
class ScrollableFrame(tk.Frame):
    """Left panel with vertical scrollbar so nothing is cut off."""
    def __init__(self, parent, bg, width, **kw):
        super().__init__(parent, bg=bg, **kw)
        self._c = tk.Canvas(self, bg=bg, width=width,
                            highlightthickness=0, bd=0)
        self._sb = tk.Scrollbar(self, orient=tk.VERTICAL, command=self._c.yview)
        self._c.configure(yscrollcommand=self._sb.set)
        self._sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._c.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.inner = tk.Frame(self._c, bg=bg)
        self._win  = self._c.create_window((0,0), window=self.inner, anchor="nw")
        self.inner.bind("<Configure>", lambda e: self._c.configure(
            scrollregion=self._c.bbox("all")))
        self._c.bind("<Configure>", lambda e: self._c.itemconfig(
            self._win, width=e.width))
        # Bind mouse-wheel on both canvas and inner frame
        for w in (self._c, self.inner):
            w.bind("<MouseWheel>",
                   lambda e: self._c.yview_scroll(int(-1*(e.delta/120)), "units"))

# ──────────────────────── MAIN APPLICATION ────────────────────────────
class PathfinderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dynamic Pathfinding Agent")
        self.root.configure(bg=C["bg"])
        self.root.resizable(True, True)

        # Fit window to screen
        sw = root.winfo_screenwidth()
        sh = root.winfo_screenheight()
        win_w = min(sw - 40, 1280)
        win_h = min(sh - 80, 750)
        root.geometry(f"{win_w}x{win_h}+10+10")

        self.rows = 18;  self.cols = 24
        self.grid  = [[0]*self.cols for _ in range(self.rows)]
        self.start = (1, 1)
        self.goal  = (self.rows-2, self.cols-2)

        self.mode          = "wall"
        self.running       = False
        self.dynamic_mode  = tk.BooleanVar(value=False)
        self.diagonal_mode = tk.BooleanVar(value=False)
        self.algo_var      = tk.StringVar(value="A*")
        self.heur_var      = tk.StringVar(value="Manhattan")
        self.density_var   = tk.DoubleVar(value=0.30)
        self.rows_var      = tk.IntVar(value=self.rows)
        self.cols_var      = tk.IntVar(value=self.cols)

        self.visited_cells  = set()
        self.frontier_cells = set()
        self.path_cells     = set()
        self.agent_pos      = None
        self.agent_path     = []
        self.agent_idx      = 0
        self.anim_ids       = []

        self.stat_visited = tk.StringVar(value="0")
        self.stat_cost    = tk.StringVar(value="0")
        self.stat_time    = tk.StringVar(value="0 ms")
        self.stat_status  = tk.StringVar(value="Ready")

        self._build_ui()
        self._draw_grid()

    # ─────────────────────── UI CONSTRUCTION ──────────────────────────
    def _build_ui(self):
        PANEL_W = 195
        F  = ("Consolas", 8)
        FB = ("Consolas", 8, "bold")

        # Scrollable panel
        sf = ScrollableFrame(self.root, bg=C["panel"], width=PANEL_W)
        sf.pack(side=tk.LEFT, fill=tk.Y)
        p = sf.inner

        def section(txt):
            tk.Label(p, text=txt, bg=C["panel"], fg=C["muted"],
                     font=FB).pack(anchor=tk.W, padx=8, pady=(8,1))

        def btn(txt, cmd, color):
            fg = C["bg"] if color != C["border"] else C["text"]
            tk.Button(p, text=txt, command=cmd, bg=color, fg=fg,
                      activebackground=C["text"], activeforeground=C["bg"],
                      relief=tk.FLAT, bd=0, pady=4, font=FB,
                      cursor="hand2").pack(fill=tk.X, padx=8, pady=2)

        def spin(lbl, var, mn, mx):
            row = tk.Frame(p, bg=C["panel"]); row.pack(fill=tk.X, padx=8, pady=1)
            tk.Label(row, text=lbl, bg=C["panel"], fg=C["text"],
                     font=F, width=5).pack(side=tk.LEFT)
            tk.Spinbox(row, from_=mn, to=mx, textvariable=var, width=4,
                       bg=C["bg"], fg=C["text"], insertbackground=C["text"],
                       relief=tk.FLAT, font=F).pack(side=tk.RIGHT)

        def slider(lbl, var, mn, mx):
            tk.Label(p, text=lbl, bg=C["panel"], fg=C["text"],
                     font=F).pack(anchor=tk.W, padx=8)
            tk.Scale(p, variable=var, from_=mn, to=mx, resolution=0.05,
                     orient=tk.HORIZONTAL, bg=C["panel"], fg=C["text"],
                     troughcolor=C["bg"], highlightthickness=0,
                     showvalue=True, font=F).pack(fill=tk.X, padx=8)

        # Grid Settings
        section("⚙ GRID SETTINGS")
        spin("Rows", self.rows_var, 5, 40)
        spin("Cols", self.cols_var, 5, 60)
        slider("Density", self.density_var, 0.05, 0.70)
        btn("Generate Map", self._generate_map, C["accent"])
        btn("Clear Map",    self._clear_map,    C["border"])

        # Edit Mode
        section("✏ EDIT MODE")
        for lbl, val in [("🧱 Wall","wall"),("🟢 Start","start"),("🔴 Goal","goal")]:
            tk.Button(p, text=lbl, bg=C["border"], fg=C["text"],
                      activebackground=C["accent"], activeforeground=C["bg"],
                      relief=tk.FLAT, bd=0, pady=4, cursor="hand2", font=F,
                      command=lambda v=val: self._set_mode(v)
                      ).pack(fill=tk.X, padx=8, pady=1)

        # Algorithm
        section("🔍 ALGORITHM")
        for a in ["A*","GBFS"]:
            tk.Radiobutton(p, text=a, variable=self.algo_var, value=a,
                           bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                           activebackground=C["panel"], activeforeground=C["accent"],
                           font=F).pack(anchor=tk.W, padx=16)

        # Heuristic
        section("📐 HEURISTIC")
        for h in HEURISTICS:
            tk.Radiobutton(p, text=h, variable=self.heur_var, value=h,
                           bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                           activebackground=C["panel"], activeforeground=C["accent"],
                           font=F).pack(anchor=tk.W, padx=16)

        # Options
        section("🎛 OPTIONS")
        for txt, var in [("Dynamic Obstacles",self.dynamic_mode),
                          ("8-Dir Movement",   self.diagonal_mode)]:
            tk.Checkbutton(p, text=txt, variable=var,
                           bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                           activebackground=C["panel"], font=F
                           ).pack(anchor=tk.W, padx=16)

        # Control
        section("🚀 CONTROL")
        btn("▶ Find Path",   self._start_search, C["start"])
        btn("⟳ Reset Vis",  self._reset_vis,    "#6B48FF")
        btn("⏹ Stop",        self._stop,         C["goal"])

        # Metrics
        section("📊 METRICS")
        for lbl, var, col in [
            ("Visited", self.stat_visited, C["visited"]),
            ("Cost",    self.stat_cost,    C["path"]),
            ("Time",    self.stat_time,    C["frontier"]),
        ]:
            row = tk.Frame(p, bg=C["panel"]); row.pack(fill=tk.X, padx=8, pady=1)
            tk.Label(row, text=lbl+":", bg=C["panel"], fg=C["muted"], font=F).pack(side=tk.LEFT)
            tk.Label(row, textvariable=var, bg=C["panel"], fg=col, font=FB).pack(side=tk.RIGHT)
        tk.Label(p, textvariable=self.stat_status, bg=C["panel"],
                 fg=C["accent"], font=FB, wraplength=PANEL_W-16).pack(pady=4)

        # Legend
        section("🎨 LEGEND")
        for lbl, col in [
            ("Start",C["start"]),("Goal",C["goal"]),("Wall",C["wall"]),
            ("Frontier",C["frontier"]),("Visited",C["visited"]),
            ("Path",C["path"]),("Agent",C["agent"]),("New Wall",C["new_wall"]),
        ]:
            row = tk.Frame(p, bg=C["panel"]); row.pack(fill=tk.X, padx=8, pady=1)
            tk.Frame(row, bg=col, width=12, height=12).pack(side=tk.LEFT, padx=(0,4))
            tk.Label(row, text=lbl, bg=C["panel"], fg=C["text"], font=F).pack(side=tk.LEFT)

        # Canvas area (right side)
        right = tk.Frame(self.root, bg=C["bg"])
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        hdr = tk.Frame(right, bg=C["bg"])
        hdr.pack(fill=tk.X, padx=6, pady=(4,0))
        tk.Label(hdr, text="DYNAMIC PATHFINDING AGENT",
                 bg=C["bg"], fg=C["accent"],
                 font=("Consolas",11,"bold")).pack(side=tk.LEFT)
        tk.Label(hdr, text="Scroll left panel ↕ for all options",
                 bg=C["bg"], fg=C["muted"],
                 font=("Consolas",8)).pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(right, bg=C["bg"],
                                highlightthickness=0, cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.canvas.bind("<Button-1>",  self._on_click)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<Configure>", lambda e: self._draw_grid())

    # ─────────── DYNAMIC CELL SIZE — auto-fits canvas ─────────────────
    def _cell_size(self):
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 10 or ch < 10:
            return 20
        cw_cell = max(4, (cw - (self.cols+1)*GAP) // self.cols)
        ch_cell = max(4, (ch - (self.rows+1)*GAP) // self.rows)
        return min(cw_cell, ch_cell, 40)

    def _cell_coords(self, r, c):
        cs = self._cell_size()
        x0 = c*(cs+GAP);  y0 = r*(cs+GAP)
        return x0, y0, x0+cs, y0+cs

    # ─────────────────────── GRID DRAWING ─────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("all")
        self.rects = {}
        for r in range(self.rows):
            for c in range(self.cols):
                x0,y0,x1,y1 = self._cell_coords(r,c)
                rect = self.canvas.create_rectangle(
                    x0,y0,x1,y1, fill=self._cell_color(r,c),
                    outline=C["bg"], width=GAP)
                self.rects[(r,c)] = rect

    def _cell_color(self, r, c):
        pos = (r,c)
        if pos == self.start:          return C["start"]
        if pos == self.goal:           return C["goal"]
        if pos == self.agent_pos:      return C["agent"]
        if self.grid[r][c] == 2:       return C["new_wall"]
        if self.grid[r][c] == 1:       return C["wall"]
        if pos in self.path_cells:     return C["path"]
        if pos in self.frontier_cells: return C["frontier"]
        if pos in self.visited_cells:  return C["visited"]
        return C["empty"]

    def _update_cell(self, r, c):
        if (r,c) in self.rects:
            self.canvas.itemconfig(self.rects[(r,c)], fill=self._cell_color(r,c))

    def _refresh_all(self):
        for r in range(self.rows):
            for c in range(self.cols):
                self._update_cell(r,c)

    # ─────────────────────── USER INTERACTION ─────────────────────────
    def _set_mode(self, m): self.mode = m

    def _canvas_to_cell(self, x, y):
        cs = self._cell_size()
        c = x//(cs+GAP);  r = y//(cs+GAP)
        if 0<=r<self.rows and 0<=c<self.cols: return r,c
        return None,None

    def _on_click(self, e):
        if self.running: return
        r,c = self._canvas_to_cell(e.x, e.y)
        if r is not None: self._apply_edit(r,c)

    def _on_drag(self, e):
        if self.running or self.mode!="wall": return
        r,c = self._canvas_to_cell(e.x, e.y)
        if r is not None: self._apply_edit(r,c)

    def _apply_edit(self, r, c):
        pos = (r,c)
        if self.mode == "wall":
            if pos in (self.start, self.goal): return
            self.grid[r][c] = 0 if self.grid[r][c] else 1
            self._update_cell(r,c)
        elif self.mode == "start":
            old=self.start; self.start=pos; self.grid[r][c]=0
            self._update_cell(*old); self._update_cell(r,c)
        elif self.mode == "goal":
            old=self.goal; self.goal=pos; self.grid[r][c]=0
            self._update_cell(*old); self._update_cell(r,c)

    # ─────────────────────── MAP MANAGEMENT ───────────────────────────
    def _generate_map(self):
        if self.running: return
        self._stop()
        try:
            self.rows=int(self.rows_var.get()); self.cols=int(self.cols_var.get())
        except ValueError: pass
        self.grid=[[0]*self.cols for _ in range(self.rows)]
        d=self.density_var.get()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) not in (self.start,self.goal):
                    self.grid[r][c]=1 if random.random()<d else 0
        self.start=(min(self.start[0],self.rows-1),min(self.start[1],self.cols-1))
        self.goal =(min(self.goal[0], self.rows-1),min(self.goal[1], self.cols-1))
        self.grid[self.start[0]][self.start[1]]=0
        self.grid[self.goal[0]][self.goal[1]]=0
        self._reset_vis(); self._draw_grid()

    def _clear_map(self):
        if self.running: return
        self._stop()
        self.grid=[[0]*self.cols for _ in range(self.rows)]
        self._reset_vis(); self._draw_grid()

    def _reset_vis(self):
        self._cancel_anims()
        self.visited_cells=set(); self.frontier_cells=set()
        self.path_cells=set(); self.agent_pos=None
        self.agent_path=[]; self.agent_idx=0
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c]==2: self.grid[r][c]=1
        self.stat_visited.set("0"); self.stat_cost.set("0")
        self.stat_time.set("0 ms"); self.stat_status.set("Ready")
        self._refresh_all()

    def _cancel_anims(self):
        for aid in self.anim_ids: self.root.after_cancel(aid)
        self.anim_ids=[]; self.running=False

    def _stop(self):
        self._cancel_anims(); self.stat_status.set("Stopped")

    # ─────────────────────── SEARCH & ANIMATION ───────────────────────
    def _start_search(self):
        if self.running: return
        self._reset_vis(); self._run_search(self.start)

    def _run_search(self, from_node):
        algo=self.algo_var.get()
        h_fn=HEURISTICS[self.heur_var.get()]
        dirs=DIRS_8 if self.diagonal_mode.get() else DIRS_4
        t0=time.perf_counter()
        if algo=="A*":
            path,visited,frontiers=astar(self.grid,self.rows,self.cols,from_node,self.goal,h_fn,dirs)
        else:
            path,visited,frontiers=gbfs(self.grid,self.rows,self.cols,from_node,self.goal,h_fn,dirs)
        elapsed=(time.perf_counter()-t0)*1000
        self.stat_time.set(f"{elapsed:.1f} ms")
        self.stat_visited.set(str(len(visited)))
        if not path:
            self.stat_status.set("No path found!")
            messagebox.showwarning("No Path","No path exists between Start and Goal.")
            return
        cost=sum(
            math.hypot(path[i][0]-path[i-1][0],path[i][1]-path[i-1][1])
            if abs(path[i][0]-path[i-1][0])+abs(path[i][1]-path[i-1][1])==2
            else 1.0 for i in range(1,len(path)))
        self.stat_cost.set(f"{cost:.1f}")
        self.stat_status.set(f"Animating ({algo})…")
        self.running=True
        self._animate_search(visited,frontiers,path)

    def _animate_search(self, visited, frontiers, path):
        step=[0]
        def tick():
            if not self.running: return
            if step[0]<len(visited):
                v=visited[step[0]]
                if v not in (self.start,self.goal):
                    self.visited_cells.add(v); self._update_cell(*v)
                if step[0]<len(frontiers):
                    old_f=self.frontier_cells-{self.start,self.goal}
                    self.frontier_cells=frontiers[step[0]]-self.visited_cells-{self.start,self.goal}
                    for cell in old_f|self.frontier_cells: self._update_cell(*cell)
                step[0]+=1
                self.anim_ids.append(self.root.after(ANIM_MS,tick))
            else:
                self.frontier_cells=set(); self.path_cells=set(path)
                self._refresh_all()
                self.stat_status.set("Path found! Moving agent…")
                self.agent_path=path; self.agent_idx=0
                self.anim_ids.append(self.root.after(300,self._move_agent))
        tick()

    # ─────────────────────── AGENT MOVEMENT ───────────────────────────
    def _move_agent(self):
        if not self.running: return
        if self.agent_idx>=len(self.agent_path):
            self.agent_pos=None; self.stat_status.set("✓ Goal reached!")
            self.running=False; self._refresh_all(); return
        old=self.agent_pos
        self.agent_pos=self.agent_path[self.agent_idx]; self.agent_idx+=1
        if old: self._update_cell(*old)
        self._update_cell(*self.agent_pos)
        if self.dynamic_mode.get() and self.agent_idx<len(self.agent_path):
            self._maybe_spawn_wall()
        self.anim_ids.append(self.root.after(MOVE_MS,self._move_agent))

    # ─────────────────── DYNAMIC OBSTACLE LOGIC ───────────────────────
    def _maybe_spawn_wall(self):
        if random.random()>SPAWN_P: return
        candidates=[(r,c) for r in range(self.rows) for c in range(self.cols)
                    if self.grid[r][c]==0
                    and (r,c) not in (self.start,self.goal,self.agent_pos)]
        remaining=self.agent_path[self.agent_idx:]
        path_cands=[p for p in remaining
                    if p not in (self.start,self.goal,self.agent_pos)
                    and self.grid[p[0]][p[1]]==0]
        if path_cands and random.random()<0.6:
            pos=random.choice(path_cands)
        elif candidates:
            pos=random.choice(candidates)
        else: return
        self.grid[pos[0]][pos[1]]=2
        self._update_cell(*pos)
        if pos in set(self.agent_path[self.agent_idx:]):
            self._replan()

    def _replan(self):
        self._cancel_anims(); self.running=True
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c]==2: self.grid[r][c]=1
        self.stat_status.set("⚡ Re-planning…")
        self.visited_cells=set(); self.frontier_cells=set(); self.path_cells=set()
        self._refresh_all()
        self.anim_ids.append(self.root.after(50,lambda: self._run_search(self.agent_pos)))

# ──────────────────────────── ENTRY POINT ─────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app  = PathfinderApp(root)
    root.mainloop()
# Dynamic Pathfinding Agent

A grid-based pathfinding visualizer built with Python and Tkinter.
Supports A* and Greedy Best-First Search (GBFS) with real-time dynamic obstacle spawning and re-planning.

---

## Requirements

- Python 3.8 or higher
- Tkinter (comes built-in with Python on Windows and macOS)

No external libraries needed — this project uses only Python's standard library.

---

## Installation

### Step 1 — Install Python

Download and install Python from the official website:
https://www.python.org/downloads/

Make sure to check "Add Python to PATH" during installation.

### Step 2 — Verify Python is installed

Open a terminal / command prompt and run:

```bash
python --version
```

You should see something like `Python 3.11.9`.

### Step 3 — Check Tkinter is available

```bash
python -m tkinter
```

A small test window should pop up. If it does, you are ready to go.

#### If Tkinter is missing (Linux only):

```bash
# Ubuntu / Debian
sudo apt-get install python3-tk

# Fedora
sudo dnf install python3-tkinter

# Arch
sudo pacman -S tk
```

On Windows and macOS, Tkinter is always included with Python — no extra install needed.

---

## How to Run

### Option 1 — Run directly

```bash
python dynamic_pathfinding_agent.py
```

### Option 2 — If you have multiple Python versions

```bash
python3 dynamic_pathfinding_agent.py
```

The GUI window will open immediately.

---

## How to Use

### Edit the Grid

| Action | How |
|---|---|
| Place / Remove Wall | Click Wall mode, then click or drag on the grid |
| Move Start Node | Click Start mode, then click any empty cell |
| Move Goal Node | Click Goal mode, then click any empty cell |

### Generate a Map

1. Set Rows and Cols using the spinboxes
2. Set Obstacle Density with the slider (e.g. 0.30 = 30% walls)
3. Click Generate Map

### Run Pathfinding

1. Choose an algorithm: A* or GBFS
2. Choose a heuristic: Manhattan, Euclidean, or Chebyshev
3. Optionally enable 8-Directional Movement or Dynamic Obstacles
4. Click Find Path

### Controls

| Button | Action |
|---|---|
| Find Path | Run the selected algorithm and animate |
| Reset Vis | Clear visited/path colours, keep the map |
| Stop | Stop the current animation |
| Generate Map | Create a new random map |
| Clear Map | Remove all walls |

### Dynamic Obstacles Mode

When enabled, new walls randomly spawn while the agent is moving.
If a wall blocks the current path, the agent automatically re-plans from its current position.

---

## Metrics Dashboard

After each search, the panel shows:

| Metric | Description |
|---|---|
| Nodes Visited | Total nodes expanded during search |
| Path Cost | Total length of the final path (diagonal = sqrt 2) |
| Time | Computation time in milliseconds |

---

## Colour Legend

| Colour | Meaning |
|---|---|
| Green | Start node |
| Red | Goal node |
| Dark grey | Wall |
| Yellow | Frontier (open list) |
| Blue | Visited (closed list) |
| Teal | Final path |
| Orange | Agent (moving) |
| Bright red | Dynamically spawned wall |

---

## Project Structure

```
project/
 |-- dynamic_pathfinding_agent.py   (main source code)
 |-- README.md                      (this file)
```

---

## Algorithms Implemented

### A* Search

- Evaluation function: f(n) = g(n) + h(n)
- g(n) = actual cost from start to node n
- h(n) = heuristic estimate to goal
- Guarantees the optimal (shortest) path

### Greedy Best-First Search (GBFS)

- Evaluation function: f(n) = h(n)
- Only uses the heuristic, ignores path cost
- Faster but does not guarantee the optimal path

### Heuristics

| Name | Formula | Best for |
|---|---|---|
| Manhattan | |dx| + |dy| | 4-directional grids |
| Euclidean | sqrt(dx^2 + dy^2) | 8-directional grids |
| Chebyshev | max(|dx|, |dy|) | 8-directional grids |

---

## Tips

- Use Manhattan heuristic with 4-dir movement for the most accurate A* results
- Use Euclidean or Chebyshev when 8-Directional Movement is enabled
- Enable Dynamic Obstacles to see real-time re-planning in action
- Scroll the left panel with your mouse wheel to access all options on smaller screens

---

## Author

Muhammad Taha Sajid
National University of Computer and Emerging Sciences
Artificial Intelligence — Assignment 2

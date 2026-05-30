# Introduction-To-AI-Assignment-2A
COS30019 Assignment 2A (2026 S1)

# COS30019 Assignment 2A — Route Finding Search Visualiser
=========================================================
```bash
Usage:
  GUI mode: python search.py
  CLI mode: python search.py <filename> <method>
              methods: DFS BFS GBFS AS CUS1(UCS) CUS2(IDA*)
```

# Algorithms
----------
```bash
  DFS   — Depth-First Search (uninformed)
  BFS   — Breadth-First Search (uninformed)
  GBFS  — Greedy Best-First Search (informed, heuristic only)
  AS    — A* Search (informed, g + h)
  CUS1  — Uniform Cost Search (uninformed, optimal-cost path)
  CUS2  — Iterative Deepening A* / IDA* (informed, optimal path,
           least moves to goal)
```

# Heuristic
----------
Heuristic (GBFS / AS / CUS2): straight-line (Euclidean) distance
to the nearest destination node.

# How to Run
----------

### 1. Graphical User Interface (GUI) Mode
Run the script without any arguments to launch the interactive GUI mode (requires `tkinter`).

**Command:**
```bash
python search.py
```

**Features in GUI:**
- Click **📂 Open Map** to select a map file (e.g., `Map1.txt`).
- Select a search algorithm from the **Method** dropdown.
- Click **▶ Run** to execute the search algorithm continuously.
- Click **⏭ Step** to step through the algorithm node by node.
- Click **↺ Reset** to clear the current search and start over.
- Use the **Speed** slider to adjust the visualization speed.
- Click **🌓 Theme** to toggle between light and dark modes.

### 2. Command-Line Interface (CLI) Mode
To run the search algorithm directly from the terminal without the GUI, provide the map filename and the search method as arguments.

**Command:**
```bash
python search.py <filename> <method>
```

**Examples:**
```bash
python search.py Map1.txt DFS
python search.py Map2.txt AS
```

# CLI Output
----------
    CLI output is as follows: 
    
        <map_file> <search_method> 
        Goal State(s): <goal(s)> 
        Starting Node: <starting_node> 
        Destination Node: <destination_node>  
        Number of Nodes Created: <number_of_nodes>  
        Path: <path>  
        Path Cost: <path_cost> 
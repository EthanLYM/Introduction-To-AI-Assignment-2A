# Introduction-To-AI-Assignment-2A
COS30019 Assignment 2A (2026 S1)

# COS30019 Assignment 2A — Route Finding Search Visualiser
=========================================================
Usage:
  GUI mode: python search.py
  CLI mode: python search.py <filename> <method>
              methods: DFS BFS GBFS AS CUS1(UCS) CUS2(IDA*)

# Algorithms
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
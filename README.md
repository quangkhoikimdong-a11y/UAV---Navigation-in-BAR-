# UAV Navigation and Blind Alley Region (BAR) Escape

A 2D Python simulation of an Unmanned Aerial Vehicle (UAV) navigating complex obstacle environments. The UAV uses a greedy local-search algorithm to explore unknown spaces and automatically detects and escapes from dead-ends (blind alleys) using a path-smoothing mechanism.

## Overview

This project implements two core algorithms:

1. **Algorithm 2: Greedy Explorer** - Local search that scans the environment and ranks available waypoints
2. **Algorithm 1: DAP (Dynamic Approximate Path) Escape** - Escapes blind alleys by smoothing and shortening paths

The robot can navigate toward a goal, avoid obstacles, detect when it's trapped, and backtrack to escape using memory.

## Quick Start

### Install Dependencies

```bash
pip install matplotlib shapely numpy
```

### Run the Simulation

```bash
# Visualize the environment
python Code_of_map.py

# Run the full navigation simulation
python codenav.py
```

The simulation will:
- Deploy the UAV from start (20, 126) to goal (200, 75)
- Show the path in real-time with matplotlib
- Save trajectory logs to `logs/` folder

## Repository Structure

```
.
├── codenav.py                  # Main simulation engine
├── Code_of_map.py              # Environment generation & visualization
├── logs/                       # Output directory (auto-created)
│   ├── historical_coords.csv   # Step-by-step coordinates
│   └── open_points_rankings.csv # Ranking scores for each waypoint
└── README.md
```

## How It Works

### Algorithm 2: Greedy Explorer

The UAV scans its surroundings within a vision radius (r=15) using 72 rays to find open directions. It ranks each candidate waypoint using:

```
ranking = -(α × distance_to_goal + β × angular_penalty)
```

Where:
- **α** (Distance penalty): Pulls the drone toward the goal
- **β** (Angular penalty): Encourages smooth turns, avoiding sharp zigzags

The robot always moves to the highest-ranked point and keeps track of all visited locations.

### Blind Alley Detection & Escape

If the robot hits a dead-end (no forward movement possible), **Algorithm 1** triggers:

1. **Memory Lookup**: Access the robot's stored points from previous exploration
2. **Path Extraction**: Retrieve the jagged path from memory leading back to an open area
3. **Path Smoothing**: Use line-of-sight checks to "pull the path tight" like a rubber band
4. **Shortcut**: Eliminate intermediate waypoints by connecting directly when possible

By the triangle inequality, a straight line is always shorter than a multi-segment detour—so each shortcut optimizes the escape route.

## Parameters

Adjust these in `codenav.py` to change behavior:

```python
self.r = 15.0          # Vision range radius
self.alpha = 1.0       # Weight for distance penalty
self.beta = 0.1        # Weight for angular penalty
self.start = Point(20, 126)   # Starting position
self.goal = Point(200, 75)    # Target position
```

**Tips:**
- Increase `beta` for smoother paths (fewer sharp turns)
- Increase `alpha` to prioritize reaching the goal faster
- Increase `r` for better map awareness (less realistic)

## Key Implementation Notes

### Simplifications & Tweaks

The implementation includes practical simplifications compared to the academic paper:

1. **DAP Algorithm (Algorithm 1)**
   - **Paper version**: Iterative coordinate-descent with convergence threshold δ
   - **Our version**: One-pass greedy line-of-sight shortcut
   - **Why**: Faster execution, deterministic output, ideal for real-time robotics
   - **Trade-off**: Still produces near-optimal solutions in practice

2. **Open Point Detection**
   - **Paper version**: Static front/side/back sight directions
   - **Our version**: Adaptive ray splitting based on open space width
   - **Why**: More flexible for various environment shapes

3. **Path Bundles**
   - **Paper version**: Formal sequences with disjoint vertex constraints
   - **Our version**: Simplified convex hull construction
   - **Why**: Computationally efficient while maintaining correctness

### Output Visualization

The generated plot shows:
- **Blue circle**: Robot's vision range at each step
- **Red line**: Main trajectory from start to goal
- **Gold line**: DAP escape paths (when blind alleys are encountered)
- **Green areas**: Funnel regions optimized during escape

## Output Files

### historical_coords.csv
Step-by-step coordinates of the UAV's trajectory:
```
step_index, x, y
0, 20.00, 126.00
1, 25.34, 120.10
...
```

### open_points_rankings.csv
Ranking scores for each selected waypoint:
```
step_index, target_x, target_y, cost
0, 25.34, 120.10, 45.23
1, 30.12, 118.50, 42.87
...
```

## File Descriptions

### codenav.py
**Core simulation engine**
- `UAVNavigator`: Main controller class
- `scan_neighbor_open_points()`: Ray-casting to find open directions
- `DAP_Algorithm_1()`: Path smoothing via line-of-sight
- `run_navigation()`: Main execution loop with BAR detection

### Code_of_map.py
**Environment setup**
- Generates the Gauntlet Canyon obstacle map
- `MapVisualizer`: Displays the map layout
- Can be customized with new obstacle definitions

## Advantages

✅ **Detects blind alleys automatically** - No need for global map knowledge  
✅ **Memory-efficient** - Only stores visited waypoints, not full map  
✅ **Real-time capable** - Greedy decisions avoid expensive computations  
✅ **Path-bounded** - Escape path never exceeds entry path length  
✅ **Smooth trajectories** - Angular penalty prevents aimless turns  
✅ **Proven optimization** - Based on triangle inequality principle  

## Example Usage

```python
from codenav import UAVNavigator

# Create navigator
nav = UAVNavigator("gauntlet_canyon_loose.csv")

# Run simulation
nav.run_navigation(max_steps=500)

# Check results
print(f"Waypoints: {len(nav.trajectory)}")
print(f"Blind alleys encountered: {len(nav.alg1_dap_paths)}")
print(f"Logs saved to: {nav.log_dir}/")
```

## Customization

### Using a Different Map

1. Edit `Code_of_map.py` with new polygon coordinates in CSV format
2. Update start/goal positions in `codenav.py`
3. Run the simulation

### Tuning Behavior

```python
nav = UAVNavigator("my_map.csv")
nav.alpha = 0.8   # Less goal-focused
nav.beta = 0.5    # Smoother paths
nav.r = 20.0      # Larger vision
nav.run_navigation()
```

## Troubleshooting

**Q: Robot gets stuck or doesn't find the goal?**  
A: Try increasing `max_steps` or adjusting `alpha`/`beta` weights.

**Q: Path looks choppy with sharp turns?**  
A: Increase `beta` to penalize angular changes more heavily.

**Q: Simulation runs too slow?**  
A: Reduce `max_steps` or the number of rays in `scan_neighbor_open_points()`.

## Based On

This implementation is based on academic research in autonomous robot navigation and path planning for unknown environments with limited sensor range.

## License

Academic Use - All copyrights acknowledged

---

**Created:** May 2026  
**Language:** Python 3.8+  
**Status:** ✅ Ready to use

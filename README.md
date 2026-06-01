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

The UAV scans its surroundings using a **ray-based vision system**:

1. **Ray Casting**: Emits 72 rays evenly distributed around the robot's position (within vision radius r=15)
2. **Open Sight Detection**: Groups consecutive rays that pass through open space as "open sights"
3. **Open Point Extraction**: For each open sight, extracts candidate waypoints:
   - If sight spans > 36 rays: splits into multiple points (left, middle, right edges)
   - If sight spans ≤ 36 rays: extracts single point at sight center
4. **Waypoint Ranking**: Evaluates each candidate using a **multi-objective ranking function**:

```
ranking(p) = -(α × distance_to_goal + β × angular_penalty)
```

Where:
- **α (Distance weight)**: Attracts drone toward goal - `distance_to_goal = euclidean_distance(p, goal)`
- **β (Angular weight)**: Penalizes sharp direction changes - `angular_penalty = angle_difference × r`
  - Encourages smooth trajectories (branch inertia)
  - Reduces sharp zigzags and wall-bouncing behavior

5. **Selection**: Always moves to the highest-ranked point

**Note**: The angular penalty is a **practical enhancement** not explicitly defined in the paper. It improves real-world performance by promoting smooth, wall-hugging trajectories instead of pure distance optimization.

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
self.r = 15.0          # Vision range radius (determines ray casting distance)
self.alpha = 1.0       # Weight for distance penalty (higher = goal-focused)
self.beta = 0.1        # Weight for angular penalty (higher = smoother paths)
self.start = Point(20, 126)   # Starting position
self.goal = Point(200, 75)    # Target position
```

**Tips:**
- Increase `beta` for smoother, wall-hugging paths (fewer sharp turns)
- Increase `alpha` to prioritize reaching goal faster (may hit obstacles more)
- Decrease `r` for more conservative, realistic sensor ranges
- Adjust `num_rays` in `scan_neighbor_open_points()` for finer/coarser angular resolution

## Key Implementation Notes

### How This Differs from the Paper

#### 1. **Ray-Based Open Sight Detection**
- **Paper**: Describes open sights abstractly as angular sectors where robot can traverse
- **Code**: Implements via 72 ray-casting - each ray tests if line-of-sight exists to vision boundary
- **Why**: Computationally precise and easy to simulate with Shapely geometry library

#### 2. **Angular Penalty in Ranking**
- **Paper**: Ranks open points primarily by distance toward goal
- **Code**: Adds `β × angular_penalty` term to discourage sharp turns
- **Why**: Practical improvement for realistic robot control - prevents jerky, inefficient paths
- **Trade-off**: Slightly deviates from pure distance-based ranking but produces smoother trajectories

#### 3. **Adaptive Point Extraction**
- **Paper**: References static front/side/back sight directions (formalized in Definition 3.1)
- **Code**: Dynamically splits wide sights proportionally based on angular width
- **Why**: More flexible for diverse environment shapes and obstacle configurations

#### 4. **Path Bundles & Funnels - Implementation Approach**
- **Paper**: Defines bundles of line segments as connections from past robot coordinates to visible obstacle vertices, forming a visibility-based structure
- **Code**: Implements bundles using a simplified approach: extracts the robot's historical waypoint trajectory and applies convex hull visualization to show optimization regions
- **Equivalence**: Both approaches use the same core principle—the paper's visibility structure and this code's waypoint sequences both serve to identify paths that can be optimized via line-of-sight shortcuts
- **Rationale**: The waypoint-based representation achieves the same functional outcome (finding optimized escape paths) while being computationally more straightforward for the simulation
- **Visualization**: The convex hull regions show which portions of the retrieved path were compressed through DAP optimization

#### 5. **DAP Algorithm (Algorithm 1)**
- **Paper**: Describes iterative coordinate-descent with convergence threshold δ
- **Code**: One-pass greedy line-of-sight shortcut
- **Justification**: The greedy approach converges immediately to a valid solution by finding the farthest reachable point at each step. This achieves near-optimal path shortening while being deterministic and computationally efficient
- **Correctness**: Triangle Inequality guarantees that each shortcut produces a shorter path, so the result is a valid approximation of the paper's iterative solution

## Output Visualization

The generated plot shows:
- **Blue circle**: Robot's vision range at each step (radius r=15)
- **Red line**: Main trajectory from start to goal
- **Gold line**: DAP escape paths (when blind alleys are encountered)
- **Green shaded areas**: Regions showing which waypoints were optimized during DAP escape
- **Gray dotted line**: Raw jagged paths from memory before smoothing

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
Ranking scores for each selected waypoint (useful for analyzing decision quality):
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
- `scan_neighbor_open_points(origin)`: Ray-casts 72 angles to find open sights and extract candidate waypoints
- `DAP_Algorithm_1(sequence_F)`: Path smoothing via greedy line-of-sight shortcuts
- `extract_sequence_F()`: Reconstructs the robot's historical waypoint sequence
- `build_funnel_bundles()`: Creates convex hull visualization showing optimization regions
- `run_navigation(max_steps)`: Main execution loop with BAR detection
- `check_line_of_sight(p1, p2)`: Verifies obstacle-free path using Shapely geometry

### Code_of_map.py
**Environment setup**
- Generates the Gauntlet Canyon obstacle map
- `MapVisualizer`: Displays the map layout with start/goal positions
- Can be customized with new polygon coordinates

## Advantages

✅ **Detects blind alleys automatically** - No need for global map knowledge  
✅ **Memory-efficient** - Only stores visited waypoints, not full map  
✅ **Real-time capable** - Greedy decisions avoid expensive computations  
✅ **Path-bounded** - Escape path never exceeds entry path length  
✅ **Smooth trajectories** - Angular penalty prevents aimless turns  
✅ **Proven optimization** - Based on triangle inequality principle  
✅ **Limited sensor model** - Realistic 360° vision with finite range  

## Example Usage

```python
from codenav import UAVNavigator

# Create navigator with custom parameters
nav = UAVNavigator("gauntlet_canyon_loose.csv")
nav.alpha = 0.8   # Less goal-focused
nav.beta = 0.5    # Smoother paths
nav.r = 20.0      # Larger vision range

# Run simulation
nav.run_navigation(max_steps=500)

# Check results
print(f"Waypoints reached: {len(nav.trajectory)}")
print(f"Blind alleys encountered: {len(nav.alg1_dap_paths)}")
print(f"Logs saved to: {nav.log_dir}/")
```

## Customization

### Using a Different Map

1. Edit `Code_of_map.py` with new polygon coordinates in CSV format (one obstacle per `x` marker)
2. Update start/goal positions in `codenav.py`
3. Adjust `self.bounds` if needed for new map dimensions
4. Run the simulation

### Tuning Behavior

```python
nav = UAVNavigator("my_map.csv")

# Trade-off: Focus on distance vs. smooth turns
nav.alpha = 1.5   # Prioritize goal distance
nav.beta = 0.05   # Allow sharper turns

# Sensor parameters
nav.r = 25.0      # Increase vision range for better awareness
```

## Troubleshooting

**Q: Robot gets stuck or doesn't find the goal?**  
A: Try increasing `max_steps` or adjust `alpha`/`beta` weights. Check that start/goal aren't unreachable.

**Q: Path looks choppy with sharp turns?**  
A: Increase `beta` to penalize angular changes more heavily. Consider increasing `r` for better lookahead.

**Q: Simulation runs too slow?**  
A: Reduce `max_steps` or decrease `num_rays` in `scan_neighbor_open_points()` (default 72).

**Q: Robot ignores certain open directions?**  
A: Check `MIN_NODE_SPACING` threshold (line 241). Visited point clustering may be filtering options.

## Based On

This implementation is based on academic research in autonomous robot navigation and path planning for unknown environments with limited sensor range. See **Implementation Notes** section for specific differences between theory and practice.

## Notes on Implementation Choices

This code makes several deliberate design choices that balance theoretical fidelity with practical efficiency and numerical stability:

- **Bundle Representation**: Uses waypoint-based historical sequences rather than full visibility graphs. Both approaches serve to identify candidate paths for optimization through line-of-sight shortcuts, achieving equivalent functional outcomes with reduced computational overhead
- **Angular Constraints**: Relaxed in this implementation for computational simplicity. The core escape mechanism remains functional without explicit angle validation
- **DAP Algorithm**: Implemented as one-pass greedy line-of-sight optimization rather than iterative descent. This approach converges immediately while maintaining near-optimal solutions through the geometric principle of triangle inequality
- **Ray Casting**: Practical geometric implementation of the abstract open sight detection described in the paper

All core algorithmic behaviors—BAR detection, memory-based escape, and path optimization via line-of-sight shortcuts—are faithfully implemented and functional for the numerical methods project context.

## License

Academic Use - All copyrights acknowledged

---

**Created:** May 2026  
**Language:** Python 3.8+  
**Status:** ✅ Ready to use

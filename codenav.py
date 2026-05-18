import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon
import math
import numpy as np
import csv
import os

# ==========================================
# 1. GENERATE THE SHIFTED MAP (+25X, +25Y)
# ==========================================
csv_content = """x
65, 200
185, 200
185, 50
65, 50
65, 95
100, 80
130, 95
160, 90
175, 100
175, 115
160, 105
130, 110
100, 95
65, 110"""

map_file_name = "gauntlet_canyon_loose.csv"
with open(map_file_name, "w") as f:
    f.write(csv_content)

# ==========================================
# 2. GRAPH NODE
# ==========================================
class GraphNode:
    def __init__(self, point, parent=None):
        self.point = point
        self.parent = parent

# ==========================================
# 3. UAV ALGORITHM 2 & 1 IMPLEMENTATION
# ==========================================
class UAVNavigator:
    def __init__(self, map_filename):
        self.map_filename = map_filename
        self.r = 15.0 
        self.bounds = (0, 220, 0, 220) 
        
        self.start = Point(20, 126)    
        self.goal = Point(200, 75)    
        
        self.alpha = 1.0  
        self.beta = 0.1   
        
        self.obstacles = self._load_csv_map()
        
        self.log_dir = "logs"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        self.O_e = [] 
        self.all_visited_points = [] 
        
        self.trajectory = [self.start] 
        self.alg1_dap_paths = []       
        
        self.funnel_polygons = [] 
        self.graph_paths_to_draw = [] 
        self.rankings_log = [] # <-- This was filling up, but not saving!
        
        self.fig, self.ax = plt.subplots(figsize=(10, 10)) 
        self._init_plot()

    def _load_csv_map(self):
        obstacles = []
        with open(self.map_filename, 'r') as file:
            reader = csv.reader(file)
            current_obs = []
            for row in reader:
                if not row: continue
                col1 = str(row[0]).strip().lower()
                if col1 == 'x':
                    if len(current_obs) > 2: obstacles.append(Polygon(current_obs))
                    current_obs = []
                else:
                    try: current_obs.append((float(row[0]), float(row[1])))
                    except ValueError: pass
            if len(current_obs) > 2: obstacles.append(Polygon(current_obs))
        return obstacles

    def _init_plot(self):
        self.ax.set_xlim(self.bounds[0], self.bounds[1])
        self.ax.set_ylim(self.bounds[2], self.bounds[3])
        self.ax.set_aspect('equal', adjustable='box') 
        self.ax.grid(True, linestyle='--', color='gray', alpha=0.3)
        for obs in self.obstacles:
            x, y = obs.exterior.xy
            self.ax.plot(x, y, color='black', linewidth=2, zorder=3)
            self.ax.fill(x, y, alpha=0.4, fc='dimgray', hatch='//', zorder=2)
        self.ax.plot(self.start.x, self.start.y, 'bo', markersize=10, zorder=10, label='Start')
        self.ax.plot(self.goal.x, self.goal.y, 'r*', markersize=15, zorder=10, label='Goal')

    def check_line_of_sight(self, p1, p2):
        line = LineString([(p1.x, p1.y), (p2.x, p2.y)])
        for obs in self.obstacles:
            if line.crosses(obs) or line.within(obs): return False
        return True

    def scan_neighbor_open_points(self, origin):
        num_rays = 72
        angle_to_goal = math.atan2(self.goal.y - origin.y, self.goal.x - origin.x)
        angles = np.linspace(angle_to_goal - math.pi/2, angle_to_goal + 3*math.pi/2, num_rays, endpoint=False) 
        
        ray_open_status = []
        for angle in angles:
            end_x = origin.x + (self.r - 0.1) * math.cos(angle)
            end_y = origin.y + (self.r - 0.1) * math.sin(angle)
            ray = LineString([(origin.x, origin.y), (end_x, end_y)])
            
            is_open = True
            for obs in self.obstacles:
                if ray.intersects(obs.boundary):
                    is_open = False
                    break
            ray_open_status.append(is_open)

        open_sights = []
        if all(ray_open_status):
            open_sights = [list(range(num_rays))] 
        elif any(ray_open_status):
            start_idx = ray_open_status.index(False)
            current_sight = []
            for i in range(start_idx + 1, start_idx + 1 + num_rays):
                idx = i % num_rays
                if ray_open_status[idx]:
                    current_sight.append(idx)
                else:
                    if current_sight:
                        open_sights.append(current_sight)
                        current_sight = []
            if current_sight:
                open_sights.append(current_sight)

        open_points = [] 
        max_rays = num_rays // 2 
        
        for sight in open_sights:
            if len(sight) > max_rays:
                mid_idx = len(sight) // 2
                sight1, sight2 = sight[:mid_idx], sight[mid_idx:]
                
                m1 = sight1[len(sight1) // 2]
                open_points.append(Point(origin.x + (self.r - 0.5) * math.cos(angles[m1]), origin.y + (self.r - 0.5) * math.sin(angles[m1])))
                
                m2 = sight2[len(sight2) // 2]
                open_points.append(Point(origin.x + (self.r - 0.5) * math.cos(angles[m2]), origin.y + (self.r - 0.5) * math.sin(angles[m2])))
                
                e1, e2 = sight[0], sight[-1]
                open_points.append(Point(origin.x + (self.r - 0.5) * math.cos(angles[e1]), origin.y + (self.r - 0.5) * math.sin(angles[e1])))
                open_points.append(Point(origin.x + (self.r - 0.5) * math.cos(angles[e2]), origin.y + (self.r - 0.5) * math.sin(angles[e2])))
            else:
                m = sight[len(sight) // 2]
                open_points.append(Point(origin.x + (self.r - 0.5) * math.cos(angles[m]), origin.y + (self.r - 0.5) * math.sin(angles[m])))
                
        return open_points

    def extract_sequence_F(self, current_node, target_node):
        path_from_current = []
        curr = current_node
        while curr is not None:
            path_from_current.append(curr)
            curr = curr.parent
            
        path_from_target = []
        curr = target_node
        while curr is not None:
            path_from_target.append(curr)
            curr = curr.parent
            
        lca = None
        for node in path_from_current:
            if node in path_from_target:
                lca = node
                break
                
        sequence_F = []
        for node in path_from_current:
            sequence_F.append(node.point)
            if node == lca: break
            
        path_to_target_rev = []
        for node in path_from_target:
            if node == lca: break
            path_to_target_rev.append(node.point)
            
        sequence_F.extend(path_to_target_rev[::-1])
        return sequence_F

    def DAP_Algorithm_1(self, sequence_F):
        if len(sequence_F) < 3: return sequence_F
        smoothed = [sequence_F[0]]
        current_idx = 0
        
        while current_idx < len(sequence_F) - 1:
            furthest = current_idx + 1
            for j in range(len(sequence_F) - 1, current_idx, -1):
                if self.check_line_of_sight(sequence_F[current_idx], sequence_F[j]):
                    furthest = j
                    break
            smoothed.append(sequence_F[furthest])
            current_idx = furthest
        return smoothed

    def build_funnel_bundles(self, sequence_F, dap_path):
        if len(sequence_F) < 3 or len(dap_path) < 2: return 
        
        try:
            indices = [sequence_F.index(p) for p in dap_path]
        except ValueError:
            return
            
        for i in range(len(indices) - 1):
            idx1 = indices[i]
            idx2 = indices[i+1]
            
            if abs(idx1 - idx2) > 1:
                start_idx = min(idx1, idx2)
                end_idx = max(idx1, idx2)
                
                cluster = sequence_F[start_idx : end_idx + 1]
                coords = [(p.x, p.y) for p in cluster]
                
                poly = Polygon(coords).convex_hull
                if not poly.is_empty:
                    self.funnel_polygons.append(poly)

    def run_navigation(self, max_steps=500):
        print(f"Deploying UAV (Alpha={self.alpha}, Beta={self.beta})...")
        
        current_node = GraphNode(self.start)
        self.all_visited_points.append(current_node.point)
        MIN_NODE_SPACING = self.r * 0.75  
        
        for step in range(max_steps):
            C_t = current_node.point
            
            vision = plt.Circle((C_t.x, C_t.y), self.r, color='blue', fill=False, linestyle=':', alpha=0.15, zorder=2)
            self.ax.add_patch(vision)

            if C_t.distance(self.goal) < self.r and self.check_line_of_sight(C_t, self.goal):
                print(f"*** TARGET REACHED in {step} steps! ***")
                self.trajectory.append(self.goal)
                break
                
            raw_open_points = self.scan_neighbor_open_points(C_t)
            
            if current_node.parent:
                curr_heading = math.atan2(C_t.y - current_node.parent.point.y, C_t.x - current_node.parent.point.x)
            else:
                curr_heading = math.atan2(self.goal.y - C_t.y, self.goal.x - C_t.x)

            for p in raw_open_points:
                is_novel = True
                
                for visited in self.all_visited_points:
                    if p.distance(visited) < MIN_NODE_SPACING: 
                        is_novel = False
                        break
                if is_novel:
                    for active_op in self.O_e:
                        if p.distance(active_op['point']) < MIN_NODE_SPACING:
                            is_novel = False
                            break
                            
                if is_novel:
                    dist_to_goal = p.distance(self.goal)
                    cand_heading = math.atan2(p.y - C_t.y, p.x - C_t.x)
                    
                    angle_diff = abs(cand_heading - curr_heading)
                    if angle_diff > math.pi: 
                        angle_diff = 2 * math.pi - angle_diff
                    
                    angular_penalty = angle_diff * self.r 
                    rank = -(self.alpha * dist_to_goal + self.beta * angular_penalty) 
                    
                    self.O_e.append({'point': p, 'parent_node': current_node, 'rank': rank})
            
            if not self.O_e:
                print("\nCRITICAL: Global Memory Empty. Map impossible.")
                break
                
            best_op_dict = max(self.O_e, key=lambda k: k['rank'])
            n = best_op_dict['point']
            n_parent = best_op_dict['parent_node']
            
            self.rankings_log.append([step, n.x, n.y, -best_op_dict['rank']])
            
            if n_parent == current_node:
                new_node = GraphNode(n, current_node)
                self.trajectory.append(n)
            else:
                print(f"--> Step {step}: BAR Detected! Retreating via Alg 1 (DAP)...")
                new_node = GraphNode(n, n_parent)
                
                sequence_F = self.extract_sequence_F(current_node, new_node)
                dap_path = self.DAP_Algorithm_1(sequence_F)
                
                self.build_funnel_bundles(sequence_F, dap_path)
                
                self.graph_paths_to_draw.append(([p.x for p in sequence_F], [p.y for p in sequence_F]))
                self.alg1_dap_paths.append(dap_path)
                
                self.trajectory.extend(dap_path[1:])
                for pt in dap_path:
                    self.all_visited_points.append(pt)
            
            self.O_e.remove(best_op_dict)
            current_node = new_node
            self.all_visited_points.append(current_node.point)
            
        # --- RENDERING ---
        for poly in self.funnel_polygons:
            x, y = poly.exterior.xy
            self.ax.fill(x, y, alpha=0.35, fc='lightgreen', ec='green', zorder=3)

        for (x_coords, y_coords) in self.graph_paths_to_draw:
            self.ax.plot(x_coords, y_coords, color='gray', linestyle=':', linewidth=2, alpha=0.8, zorder=4)

        traj_x, traj_y = [p.x for p in self.trajectory], [p.y for p in self.trajectory]
        self.ax.plot(traj_x, traj_y, color='red', linewidth=1.5, marker='.', markersize=4, zorder=6, label='Algorithm 2 Path')

        for dap in self.alg1_dap_paths:
            ex, ey = [p.x for p in dap], [p.y for p in dap]
            self.ax.plot(ex, ey, color='gold', linewidth=3, zorder=9, label='Algorithm 1 (DAP) Escape' if dap == self.alg1_dap_paths[0] else "")

        self.ax.set_title(f"UAV Navigation: Stable Bundle Regions", fontweight='bold')
        self.ax.legend(loc='lower left', fontsize=10)
        self.save_logs() 
        plt.show()

    def save_logs(self):
        # 1. Save Coordinates
        with open(os.path.join(self.log_dir, 'historical_coords.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['step_index', 'x', 'y'])
            for i, p in enumerate(self.trajectory): writer.writerow([i, round(p.x, 2), round(p.y, 2)])
            
        # 2. RESTORED: Save Rankings
        with open(os.path.join(self.log_dir, 'open_points_rankings.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['step_index', 'target_x', 'target_y', 'cost'])
            writer.writerows(self.rankings_log)
            
        print("Data Logs Saved Successfully.")

if __name__ == "__main__":
    nav = UAVNavigator(map_file_name)
    nav.run_navigation()

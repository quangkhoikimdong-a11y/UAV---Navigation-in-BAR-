import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import csv
import os

# ==========================================
# 1. GENERATE THE FLATTENED LOOSE ZIG-ZAG
# ==========================================
csv_content = """x
40, 175
160, 175
160, 25
40, 25
40, 70
75, 45
105, 75
135, 65
150, 80
150, 110
135, 95
105, 105
75, 75
40, 100"""

file_name = "gauntlet_canyon_loose.csv"

# Write the CSV file
with open(file_name, "w") as f:
    f.write(csv_content)
print(f"Successfully generated: {file_name}")

# ==========================================
# 2. VISUALIZE THE MAP
# ==========================================
class MapVisualizer:
    def __init__(self, map_filename):
        self.map_filename = map_filename
        self.obstacles = self._load_csv_map()
        self.start = (20, 100)
        self.goal = (180, 100)

    def _load_csv_map(self):
        obstacles = []
        if not os.path.exists(self.map_filename):
            print("Error: CSV not found.")
            return obstacles

        with open(self.map_filename, 'r') as file:
            reader = csv.reader(file)
            current_obs = []

            for row in reader:
                if not row:
                    continue
                col1 = str(row[0]).strip().lower()
                if col1 == 'x':
                    if len(current_obs) > 2:
                        obstacles.append(Polygon(current_obs))
                    current_obs = []
                else:
                    try:
                        x, y = float(row[0]), float(row[1])
                        current_obs.append((x, y))
                    except (ValueError, IndexError):
                        pass

            if len(current_obs) > 2:
                obstacles.append(Polygon(current_obs))

        return obstacles

    def plot(self):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlim(0, 200)
        ax.set_ylim(0, 200)
        ax.set_aspect('equal') # Keeps circles round!
        ax.grid(True, linestyle='--', color='gray', alpha=0.3)

        # Draw the loosened trap
        for obs in self.obstacles:
            x, y = obs.exterior.xy
            ax.plot(x, y, color='black', linewidth=2, zorder=2)
            ax.fill(x, y, alpha=0.4, fc='dimgray', hatch='//', zorder=1)

        # Plot Start and Goal
        ax.plot(self.start[0], self.start[1], 'bo', markersize=10, label='Start', zorder=5)
        ax.annotate('Start', (self.start[0]-5, self.start[1]-15), color='blue', fontweight='bold')

        ax.plot(self.goal[0], self.goal[1], 'r*', markersize=15, label='Goal', zorder=5)
        ax.annotate('Goal', (self.goal[0]-5, self.goal[1]-15), color='red', fontweight='bold')

        plt.title("Custom Map: 'Loose' Zig-Zag (Peak Flattened)", fontweight='bold')
        plt.show()

if __name__ == "__main__":
    vis = MapVisualizer(file_name)
    vis.plot()

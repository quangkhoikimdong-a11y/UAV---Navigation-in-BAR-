import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import csv
import os

# ==========================================
# 1. GENERATE THE SHIFTED ZIG-ZAG (+25 X, +25 Y)
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
        # Shifted Start and Goal Y up by 25 to match the new tunnel entrance
        # Goal X pushed out to 200 to clear the new wall at X=185
        self.start = (20, 125)
        self.goal = (200, 75)

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
        # Expanded bounds to 220 to fit the new shifted coordinates comfortably
        ax.set_xlim(0, 220)
        ax.set_ylim(0, 220)
        ax.set_aspect('equal') # Keeps circles round!
        ax.grid(True, linestyle='--', color='gray', alpha=0.3)

        # Draw the shifted trap
        for obs in self.obstacles:
            x, y = obs.exterior.xy
            ax.plot(x, y, color='black', linewidth=2, zorder=2)
            ax.fill(x, y, alpha=0.4, fc='dimgray', hatch='//', zorder=1)

        # Plot Start and Goal
        ax.plot(self.start[0], self.start[1], 'bo', markersize=10, label='Start', zorder=5)
        ax.annotate('Start', (self.start[0]-5, self.start[1]-15), color='blue', fontweight='bold')

        ax.plot(self.goal[0], self.goal[1], 'r*', markersize=15, label='Goal', zorder=5)
        ax.annotate('Goal', (self.goal[0]-5, self.goal[1]-15), color='red', fontweight='bold')

        plt.title("Custom Map: Shifted Zig-Zag (+25 Right, +25 Up)", fontweight='bold')
        plt.show()

if __name__ == "__main__":
    vis = MapVisualizer(file_name)
    vis.plot()

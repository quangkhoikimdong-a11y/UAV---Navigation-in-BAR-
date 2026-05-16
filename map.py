import pandas as pd
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import os
import math

class EnvironmentMap:
    def __init__(self, map_filename="_map_bugtrap.csv"):
        self.sensor_radius = 15.0
        self.map_filename = map_filename
        self.obstacles = self._load_csv_map()
        self.bounds = self._calculate_bounds()

        # Updated Start coordinate extracted from the new reference image (X=55, Y=50)
        # Goal remains behind the bugtrap to force the BAR trap
        self.start = Point(55, 50)
        self.goal = Point(190, 100)

    def _load_csv_map(self):
        obstacles = []
        if not os.path.exists(self.map_filename):
            print(f"CRITICAL ERROR: {self.map_filename} not found in this folder.")
            return obstacles

        try:
            df = pd.read_csv(self.map_filename)
        except Exception as e:
            print(f"Error reading CSV: {e}")
            return obstacles

        current_obs = []
        for index, row in df.iterrows():
            col1 = str(row.iloc[0]).strip().lower()

            if col1 == 'x':
                if len(current_obs) > 2:
                    obstacles.append(Polygon(current_obs))
                current_obs = []
            else:
                try:
                    x, y = float(row.iloc[0]), float(row.iloc[1])
                    current_obs.append((x, y))
                except ValueError:
                    pass

        if len(current_obs) > 2:
            obstacles.append(Polygon(current_obs))

        return obstacles

    def _calculate_bounds(self):
        if not self.obstacles:
            return (0, 200, 0, 200)

        min_x, min_y = float('inf'), float('inf')
        max_x, max_y = float('-inf'), float('-inf')

        for obs in self.obstacles:
            bounds = obs.bounds
            min_x, min_y = min(min_x, bounds[0]), min(min_y, bounds[1])
            max_x, max_y = max(max_x, bounds[2]), max(max_y, bounds[3])

        return (min_x - 10, max_x + 10, min_y - 10, max_y + 10)

    def plot(self):
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim(self.bounds[0], self.bounds[1])
        ax.set_ylim(self.bounds[2], self.bounds[3])
        ax.grid(True, linestyle='--', color='gray', alpha=0.5)

        # Plot the edges and hatch the interior
        for obs in self.obstacles:
            x, y = obs.exterior.xy
            ax.plot(x, y, color='black', linewidth=2, zorder=3)
            ax.fill(x, y, alpha=0.2, fc='gray', hatch='//', zorder=2)

        # Plot UAV
        ax.plot(self.start.x, self.start.y, 'ko', markersize=8, zorder=5)
        ax.annotate('UAV', (self.start.x - 5, self.start.y - 8), fontweight='bold')

        # Plot Goal
        ax.plot(self.goal.x, self.goal.y, 'r*', markersize=12, zorder=5)
        ax.annotate('Goal', (self.goal.x - 4, self.goal.y + 6), fontweight='bold')

        # Vision Circle
        vision_circle = plt.Circle((self.start.x, self.start.y), self.sensor_radius, color='blue', fill=False, linestyle=':')
        ax.add_patch(vision_circle)

        # Calculate greedy line-of-sight arrow pointing towards the goal
        dx = self.goal.x - self.start.x
        dy = self.goal.y - self.start.y
        magnitude = math.sqrt(dx**2 + dy**2)
        arrow_length = 20
        ax.arrow(self.start.x, self.start.y, (dx/magnitude)*arrow_length, (dy/magnitude)*arrow_length,
                 head_width=3, head_length=4, fc='k', ec='k', alpha=0.6, zorder=4)

        plt.title("UAV Environment: Blind Alley Region (BAR) Trap", fontsize=14, fontweight='bold')
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.show()

if __name__ == "__main__":
    env = EnvironmentMap("_map_bugtrap.csv")
    env.plot()

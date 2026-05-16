import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import math

class DeepSandboxMap:
    def __init__(self):
        self.sensor_radius = 30.0
        
        # 800x200 Grid. 
        # Deep U-Shape Trap. 
        # Outer block: X=200 to X=600. Y=50 to Y=150.
        # Inner hollow corridor: X=200 to X=550. Y=80 to Y=120.
        self.obstacles = [
            Polygon([
                (200, 150),  # Top-left outer
                (600, 150),  # Top-right outer
                (600, 50),   # Bottom-right outer
                (200, 50),   # Bottom-left outer
                (200, 80),   # Bottom mouth edge
                (550, 80),   # Inner bottom corner (Deep inside)
                (550, 120),  # Inner top corner (Deep inside)
                (200, 120)   # Top mouth edge
            ])
        ]
        
        self.bounds = (0, 800, 0, 200)
        
        # Start far left, Goal far right
        self.start = Point(50, 100)
        self.goal = Point(750, 100)

    def plot(self):
        fig, ax = plt.subplots(figsize=(15, 4)) # Wide aspect ratio for 800x200
        ax.set_xlim(self.bounds[0], self.bounds[1])
        ax.set_ylim(self.bounds[2], self.bounds[3])
        ax.grid(True, linestyle='--', color='gray', alpha=0.5)

        for obs in self.obstacles:
            x, y = obs.exterior.xy
            ax.plot(x, y, color='black', linewidth=2, zorder=3)
            ax.fill(x, y, alpha=0.2, fc='gray', hatch='//', zorder=2)

        ax.plot(self.start.x, self.start.y, 'ko', markersize=8, zorder=5)
        ax.annotate('UAV', (self.start.x - 15, self.start.y - 15), fontweight='bold')

        ax.plot(self.goal.x, self.goal.y, 'r*', markersize=12, zorder=5)
        ax.annotate('Goal', (self.goal.x - 15, self.goal.y + 15), fontweight='bold')

        vision_circle = plt.Circle((self.start.x, self.start.y), self.sensor_radius, color='blue', fill=False, linestyle=':')
        ax.add_patch(vision_circle)

        dx = self.goal.x - self.start.x
        dy = self.goal.y - self.start.y
        magnitude = math.sqrt(dx**2 + dy**2)
        arrow_length = 50
        ax.arrow(self.start.x, self.start.y, (dx/magnitude)*arrow_length, (dy/magnitude)*arrow_length,
                 head_width=8, head_length=10, fc='k', ec='k', alpha=0.6, zorder=4)

        plt.title("UAV Environment: Deep Sandbox Trap (200x800)", fontsize=14, fontweight='bold')
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.show()

if __name__ == "__main__":
    env = DeepSandboxMap()
    env.plot()

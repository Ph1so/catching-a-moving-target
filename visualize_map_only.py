import numpy as np
import matplotlib.pyplot as plt
import sys

def parse_map_only(filename):
    """Parses only the map dimensions and the costmap data."""
    try:
        with open(filename, 'r') as file:
            # Skip metadata headers we don't need for a simple map view
            # N, x_size/y_size, C, collision_thresh, R, robotX/robotY, T
            lines = file.readlines()
            
            # Find the line where the actual map (M) starts
            map_start_idx = 0
            for i, line in enumerate(lines):
                if line.strip() == 'M':
                    map_start_idx = i + 1
                    break
            
            costmap = []
            for line in lines[map_start_idx:]:
                if line.strip():
                    row = list(map(float, line.strip().split(',')))
                    costmap.append(row)
            
            # Transpose to match (x, y) coordinates to (column, row)
            return np.asarray(costmap).T
            
    except Exception as e:
        print(f"Error parsing map file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python map_visualizer.py <map_filename>")
        sys.exit(1)

    costmap = parse_map_only(sys.argv[1])

    plt.figure(figsize=(10, 8))
    
    # Using 'viridis' or 'magma' often provides better contrast for costmaps than 'jet'
    img = plt.imshow(costmap, cmap='viridis', origin='lower')
    
    plt.colorbar(img, label='Cost')
    plt.title(f"Map Visualization: {sys.argv[1]}")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    
    plt.show()
#!/usr/bin/env python3
"""
Script to generate test maps for path planning
"""

import random
import sys

def generate_random_obstacles(width, height, obstacle_density=0.3):
    """Generate a random obstacle map with given density"""
    grid = []
    for y in range(height):
        row = []
        for x in range(width):
            # Add obstacles based on density
            if random.random() < obstacle_density:
                row.append(1)
            else:
                row.append(1)
        grid.append(row)
    return grid

def generate_maze_pattern(width, height):
    """Generate a maze-like pattern with walls"""
    grid = [[1 for _ in range(width)] for _ in range(height)]

    # Create corridors
    for y in range(0, height, 10):
        for x in range(width):
            grid[y][x] = 1

    for x in range(0, width, 10):
        for y in range(height):
            grid[y][x] = 1

    # Add some openings
    for y in range(5, height, 10):
        for x in range(5, width, 10):
            # Create 5x5 open spaces
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < height and 0 <= nx < width:
                        grid[ny][nx] = 1

    return grid

def generate_clusters(width, height, num_clusters=20):
    """Generate random clusters of obstacles"""
    grid = [[1 for _ in range(width)] for _ in range(height)]

    for _ in range(num_clusters):
        # Random cluster center
        cy = random.randint(50, height - 50)
        cx = random.randint(50, width - 50)
        cluster_size = random.randint(20, 80)

        # Create circular obstacle
        for y in range(max(0, cy - cluster_size), min(height, cy + cluster_size)):
            for x in range(max(0, cx - cluster_size), min(width, cx + cluster_size)):
                dist_sq = (y - cy) ** 2 + (x - cx) ** 2
                if dist_sq <= cluster_size ** 2:
                    grid[y][x] = 1

    return grid

def generate_diagonal_walls(width, height):
    """Generate diagonal wall patterns"""
    grid = [[1 for _ in range(width)] for _ in range(height)]

    # Create diagonal walls
    for i in range(0, width, 50):
        for j in range(height):
            x = i + j // 10
            if 0 <= x < width:
                # Make walls thicker (3 pixels)
                for dx in range(-1, 2):
                    if 0 <= x + dx < width:
                        grid[j][x + dx] = 1

    return grid

def generate_open_space(width, height):
    """Generate mostly open space with border walls"""
    grid = [[1 for _ in range(width)] for _ in range(height)]
    return grid

def write_map(filename, width, height, grid, num_targets=200):
    """Write map to file in the expected format"""

    # Generate random robot position in open space
    robot_x = random.randint(width // 4, width * 3 // 4)
    robot_y = random.randint(height // 4, height * 3 // 4)

    # Count actual obstacles (cells with value 1 where it's not just border)
    obstacle_count = sum(sum(row) for row in grid)

    # Generate random target positions
    targets = []
    for _ in range(num_targets):
        target_x = random.randint(10, width - 10)
        target_y = random.randint(10, height - 10)
        targets.append((target_x, target_y))

    with open(filename, 'w') as f:
        # Write header
        f.write("N\n")
        f.write(f"{width},{height}\n")
        f.write("C\n")
        f.write(f"{obstacle_count}\n")
        f.write("R\n")
        f.write(f"{robot_x},{robot_y}\n")
        f.write("T\n")

        # Write targets
        for tx, ty in targets:
            f.write(f"{tx},{ty}\n")

        # Write map marker
        f.write("M\n")

        # Write obstacle grid
        for row in grid:
            line = ", ".join(str(cell) for cell in row)
            f.write(line + "\n")

def main():
    width = 2000
    height = 2000

    print(f"Generating 2000x2000 test maps...")

    # Map 1: Mostly open space (like original maps)
    print("Generating map13.txt - Open space...")
    grid1 = generate_open_space(width, height)
    write_map("grad/map13.txt", width, height, grid1, num_targets=300)

    # Map 2: Random clusters
    print("Generating map14.txt - Random clusters...")
    grid2 = generate_clusters(width, height, num_clusters=30)
    write_map("grad/map14.txt", width, height, grid2, num_targets=300)

    # Map 3: Diagonal walls
    print("Generating map15.txt - Diagonal walls...")
    grid3 = generate_diagonal_walls(width, height)
    write_map("grad/map15.txt", width, height, grid3, num_targets=300)

    # Map 4: Maze pattern
    print("Generating map16.txt - Maze pattern...")
    grid4 = generate_maze_pattern(width, height)
    write_map("grad/map16.txt", width, height, grid4, num_targets=300)

    # Map 5: Another open space variant
    print("Generating map17.txt - Open space variant...")
    grid5 = generate_open_space(width, height)
    write_map("grad/map17.txt", width, height, grid5, num_targets=400)

    print("Done! Generated 5 test maps in the grad directory.")
    print("Maps: map13.txt, map14.txt, map15.txt, map16.txt, map17.txt")

if __name__ == "__main__":
    main()

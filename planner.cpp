/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <stdio.h>
#include <limits>
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
#include <chrono>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#define GETXFROMINDEX(IDX, XSIZE) (((IDX) % (XSIZE)) + 1)
#define GETYFROMINDEX(IDX, XSIZE) (((IDX) / (XSIZE)) + 1)


#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

int num_goals_passsed = 1;
int huersitic_type = 1;

/*=================================================================
 * REVERSE A STRATEGY: iterate on each node on the traj that the target hasnt been to yet
 * and get the cost for the robot to travel to that node and intercept the target
 * choose the best node to intercept the target at
 * 
 * Constraint: node_i (node to intercept the target) subtracted by the current node
 * the target is at is the number of time steps that the robot has to plan for
 * 
 * Method: Perform one A search until all points on the target's trajectory has been
 * expanded. Filter out points that don't meet the above constraint. Choose the least
 * cost point based on the A search
 *=================================================================*/

std::vector<int> init_gvalues(int x_size, int y_size)
{
    const int INF = std::numeric_limits<int>::max();
    return std::vector<int>(x_size * y_size, INF);
}

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // auto start_time = std::chrono::high_resolution_clock::now();

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    auto is_map_index_valid = [&](int newx, int newy) -> bool {
        int idx = GETMAPINDEX(newx, newy, x_size, y_size);
        return (map[idx] >= 0) && (map[idx] < collision_thresh);
    };

    auto calc_cost = [&](int node_index) -> int {
        int x = GETXFROMINDEX(node_index, x_size);
        int y = GETYFROMINDEX(node_index, x_size);
        return (int)map[GETMAPINDEX(x,y,x_size,y_size)];
    };

    const int INITIAL_CAPACITY = 8000000; 
    const int INF = std::numeric_limits<int>::max();

    static std::vector<int> g_values(INITIAL_CAPACITY, INF);
    static std::vector<int> steps(INITIAL_CAPACITY, INF);
    static std::vector<int> parent(INITIAL_CAPACITY, -1);
    // Replace unordered_map with a static O(1) lookup array for speed
    static std::vector<int> traj_lookup(INITIAL_CAPACITY, -1); 

    int current_map_size = x_size * y_size;

    if (current_map_size > g_values.size()) {
        g_values.resize(current_map_size, INF);
        steps.resize(current_map_size, INF);
        parent.resize(current_map_size, -1);
        traj_lookup.resize(current_map_size, -1);
    }

    std::fill(g_values.begin(), g_values.begin() + current_map_size, INF);
    std::fill(steps.begin(), steps.begin() + current_map_size, INF);
    std::fill(parent.begin(), parent.begin() + current_map_size, -1);
    
    // Fill trajectory lookup and count unique nodes
    int traj_nodes_left = 0;
    for (int i = 0; i < target_steps; i++) {
        int idx = GETMAPINDEX(target_traj[i], target_traj[i + target_steps], x_size, y_size);
        if (traj_lookup[idx] == -1) traj_nodes_left++;
        traj_lookup[idx] = i; 
    }

    int S_start = GETMAPINDEX(robotposeX, robotposeY, x_size, y_size);
    using State = std::pair<int, int>;  
    std::priority_queue<State, std::vector<State>, std::greater<State>> open_list;
    std::vector<bool> closed_list(current_map_size, false);
    std::priority_queue<State, std::vector<State>, std::greater<State>> viable_trajectory_index_list;

    g_values[S_start] = 0;
    steps[S_start] = 0;
    open_list.push({0, S_start});

    while (!open_list.empty() && traj_nodes_left > 0)
    {
        int s = open_list.top().second;
        open_list.pop();

        if (closed_list[s]) continue;
        closed_list[s] = true;

        // O(1) trajectory check instead of O(log N) set search
        if (traj_lookup[s] != -1) {
            traj_nodes_left--;
            int t_target_at_s = traj_lookup[s];
            int t_robot_at_s = curr_time + steps[s];

            if (t_robot_at_s <= t_target_at_s) {
                int wait_steps = t_target_at_s - t_robot_at_s;
                int total_intercept_cost = g_values[s] + (wait_steps * calc_cost(s));
                viable_trajectory_index_list.push({total_intercept_cost, s});
            }
        }

        int x = GETXFROMINDEX(s, x_size);
        int y = GETYFROMINDEX(s, x_size);

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int nx = x + dX[dir];
            int ny = y + dY[dir];

            if (nx < 1 || nx > x_size || ny < 1 || ny > y_size) continue;
            int s_p = GETMAPINDEX(nx, ny, x_size, y_size);
            if (map[s_p] >= collision_thresh) continue;

            if (!closed_list[s_p]) {
                int cost = g_values[s] + map[s_p];
                if (g_values[s_p] > cost) {
                    g_values[s_p] = cost;
                    steps[s_p] = steps[s] + 1;
                    parent[s_p] = s;
                    open_list.push({cost, s_p});
                }
            }
        }
    }

    if (!viable_trajectory_index_list.empty()) {
        int cur = viable_trajectory_index_list.top().second;
        while (parent[cur] != -1 && parent[cur] != S_start) {
            cur = parent[cur];
        }
        action_ptr[0] = GETXFROMINDEX(cur, x_size);
        action_ptr[1] = GETYFROMINDEX(cur, x_size);
    } else {
        action_ptr[0] = robotposeX;
        action_ptr[1] = robotposeY;
    }

    // Reset trajectory lookup for next call
    for (int i = 0; i < target_steps; i++) {
        int idx = GETMAPINDEX(target_traj[i], target_traj[i + target_steps], x_size, y_size);
        traj_lookup[idx] = -1;
    }

    // auto end_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    // printf("time: %.4f ms\n", elapsed.count());
    return;
}
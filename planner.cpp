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
 * 
 * UPDATE: Now supports waiting at intermediate cells. The planner considers waiting
 * at any cell along the path if it reduces total cost.
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
    
    // fill trajectory lookup and count unique nodes
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
    
    // store best intercept option
    struct InterceptOption {
        int total_cost;
        int intercept_node;
        int wait_node;
        
        bool operator>(const InterceptOption& other) const {
            return total_cost > other.total_cost;
        }
    };
    std::priority_queue<InterceptOption, std::vector<InterceptOption>, std::greater<InterceptOption>> viable_options;

    g_values[S_start] = 0;
    steps[S_start] = 0;
    open_list.push({0, S_start});

    while (!open_list.empty() && traj_nodes_left > 0)
    {
        int s = open_list.top().second;
        open_list.pop();

        if (closed_list[s]) continue;
        closed_list[s] = true;

        // check if this is a trajectory node
        if (traj_lookup[s] != -1) {
            traj_nodes_left--;
            int t_target_at_s = traj_lookup[s];
            int t_robot_at_s = curr_time + steps[s];

            if (t_robot_at_s <= t_target_at_s) {
                int wait_steps = t_target_at_s - t_robot_at_s;
                
                // Option 1: wait at the intercept node itself
                int total_cost_wait_here = g_values[s] + (wait_steps * calc_cost(s));
                viable_options.push({total_cost_wait_here, s, s});
                
                // Option 2: or wait at any node along the path to s
                int cur = s;
                int accumulated_cost = g_values[s];
                int path_steps = steps[s];
                
                while (parent[cur] != -1) {
                    int prev = parent[cur];
                    accumulated_cost -= calc_cost(cur);
                    path_steps--;
                    
                    int t_robot_at_prev = curr_time + path_steps;
                    if (t_robot_at_prev <= t_target_at_s) {
                        int wait_at_prev = t_target_at_s - t_robot_at_prev;
                        int cost_to_prev = accumulated_cost;
                        int wait_cost = wait_at_prev * calc_cost(prev);
                        int move_cost_to_intercept = g_values[s] - g_values[prev];
                        int total_cost_wait_at_prev = cost_to_prev + wait_cost + move_cost_to_intercept;
                        
                        viable_options.push({total_cost_wait_at_prev, s, prev});
                    }
                    
                    cur = prev;
                }
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

    /*
        add check to see if there is any intercept option at all
        question: what to do if this is the case
            -   choose the option that minimizes distance and hope that the next step will have an intercept option?
    */

    if (!viable_options.empty()) {
        InterceptOption best = viable_options.top();
        int wait_node = best.wait_node;
        
        int cur = wait_node;
        while (parent[cur] != -1 && parent[cur] != S_start) {
            cur = parent[cur];
        }
        action_ptr[0] = GETXFROMINDEX(cur, x_size);
        action_ptr[1] = GETYFROMINDEX(cur, x_size);
    } else {
        int dx = (targetposeX > robotposeX) ? 1 : ((targetposeX < robotposeX) ? -1 : 0);
        int dy = (targetposeY > robotposeY) ? 1 : ((targetposeY < robotposeY) ? -1 : 0);
        action_ptr[0] = robotposeX + dx;
        action_ptr[1] = robotposeY + dy;
    }

    // reset trajectory lookup for next call
    for (int i = 0; i < target_steps; i++) {
        int idx = GETMAPINDEX(target_traj[i], target_traj[i + target_steps], x_size, y_size);
        traj_lookup[idx] = -1;
    }

    // auto end_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
    // printf("time: %.4f ms\n", elapsed.count());
    return;
}
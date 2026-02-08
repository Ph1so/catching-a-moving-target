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
 * NEW IDEA: iterate on each node on the traj that the target hasnt been to yet
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
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    auto is_map_index_valid = [&](int newx, int newy) -> bool {
        int idx = GETMAPINDEX(newx, newy, x_size, y_size);
        return (map[idx] >= 0) && (map[idx] < collision_thresh);
    };

    auto get_index = [&](int x, int y) -> int {
        return GETMAPINDEX(x, y, x_size, y_size);
    };

    auto get_latest_goal = [&](int* target_traj, int target_steps) -> int {
        for (int i = target_steps-1; i > 0; i--)
        {
            if (is_map_index_valid(target_traj[i], target_traj[i+target_steps])) return i;
        }
        return 0;
    };

    auto calc_heuristic = [&](int node_index, int node_goal, int type) -> int {
        int x_start = GETXFROMINDEX(node_index, x_size);
        int y_start = GETYFROMINDEX(node_index, x_size);
        int x_end   = GETXFROMINDEX(node_goal,  x_size);
        int y_end   = GETYFROMINDEX(node_goal,  x_size);
        int dx = x_start - x_end;
        int dy = y_start - y_end;
        // Euclidean Distance
        if (type == 1){
            return static_cast<int>(std::sqrt(dx*dx + dy*dy));
        }
        // Chebyshev 
        else if (type == 2) {
            return std::max(abs(dx), abs(dy));
        }
        else return 0;
    };

    auto get_neighbors = [&](int node_index) -> std::vector<int> {
        std::vector<int> neighbors;

        int x = GETXFROMINDEX(node_index, x_size);
        int y = GETYFROMINDEX(node_index, x_size); 

        for (int dir = 0; dir < NUMOFDIRS; dir++) {
            int nx = x + dX[dir];
            int ny = y + dY[dir];

            if (nx < 1 || nx > x_size || ny < 1 || ny > y_size) continue;
            if (!is_map_index_valid(nx, ny)) continue;

            neighbors.push_back(GETMAPINDEX(nx, ny, x_size, y_size));
        }

        return neighbors;
    };

    auto calc_cost = [&](int node_index) -> int {
        int x = GETXFROMINDEX(node_index, x_size);
        int y = GETYFROMINDEX(node_index, x_size);
        return (int)map[GETMAPINDEX(x,y,x_size,y_size)];
    };

    auto get_trajectory_indexs = [&](std::unordered_map<int, int> &traj_index_mapping) -> std::set<int> {
        std::set<int> list;
        for (int i = 0; i < target_steps; i++)
        {
            int index = GETMAPINDEX(target_traj[i], target_traj[i + target_steps], x_size, y_size);
            list.insert(index);
            traj_index_mapping[index] = i;
        }
        return list;
    };

    int latest_goal = get_latest_goal(target_traj, target_steps);
    int goalposeX = target_traj[latest_goal-num_goals_passsed];
    int goalposeY = target_traj[latest_goal+target_steps-num_goals_passsed];

    if (goalposeX == robotposeX && goalposeY == robotposeY)
    {
        num_goals_passsed++;
        goalposeX = target_traj[latest_goal-num_goals_passsed];
        goalposeY = target_traj[latest_goal+target_steps-num_goals_passsed];
        if (!is_map_index_valid(goalposeX, goalposeY)){
            num_goals_passsed--;
            goalposeX = target_traj[latest_goal-num_goals_passsed];
            goalposeY = target_traj[latest_goal+target_steps-num_goals_passsed];
        }
    }
    // printf("goal: %d %d\n", goalposeX, goalposeY);
    const int INITIAL_CAPACITY = 8000000; 
    const int INF = std::numeric_limits<int>::max();

    // Static variables persist across calls, so we only allocate once [cite: 17]
    static std::vector<int> g_values(INITIAL_CAPACITY, INF);
    static std::vector<int> steps(INITIAL_CAPACITY, INF);
    static std::vector<int> parent(INITIAL_CAPACITY, -1);

    int current_map_size = x_size * y_size;

    // If a custom map exceeds our initial guess, resize it once [cite: 101, 102]
    if (current_map_size > g_values.size()) {
        g_values.resize(current_map_size, INF);
        steps.resize(current_map_size, INF);
        parent.resize(current_map_size, -1);
    }

    // Reset ONLY the area used by the current map [cite: 39, 40]
    std::fill(g_values.begin(), g_values.begin() + current_map_size, INF);
    std::fill(steps.begin(), steps.begin() + current_map_size, INF);
    std::fill(parent.begin(), parent.begin() + current_map_size, -1);
    

    int S_goal = get_index(goalposeX, goalposeY);
    int S_start = get_index(robotposeX, robotposeY);

    using State = std::pair<int, int>;  // (cost, node)

    std::priority_queue<State, std::vector<State>, std::greater<State>> open_list;
    std::set<int> closed_list;
    std::unordered_map<int, int> traj_index_mapping;
    std::set<int> trajectory_index_list = get_trajectory_indexs(traj_index_mapping);
    std::priority_queue<State, std::vector<State>, std::greater<State>> viable_trajectory_index_list;

    // Perform A search throughout whole graph
    g_values[S_start] = 0;
    steps[S_start] = 0;
    open_list.push({calc_cost(S_start), S_start});

    while (!open_list.empty() && !trajectory_index_list.empty())
    {
        // remove s with smallest g value from OPEN
        int s = open_list.top().second;
        open_list.pop();

        if (trajectory_index_list.find(s) != trajectory_index_list.end()) 
            trajectory_index_list.erase(s);

        if (traj_index_mapping.count(s)) {
            int t_target_at_s = traj_index_mapping[s];
            int t_robot_at_s = curr_time + steps[s];

            // If robot arrives before or exactly when the target is there
            if (t_robot_at_s <= t_target_at_s) {
                int wait_steps = t_target_at_s - t_robot_at_s;
                int total_intercept_cost = g_values[s] + (wait_steps * calc_cost(s));
                
                viable_trajectory_index_list.push({total_intercept_cost, s});
            }
        }
        // add s to CLOSED
        if (closed_list.count(s)) continue;
        closed_list.insert(s);

        // for every neighbor s` of s such that s` is not in CLOSED
            //  if g(s’) > g(s) + c(s,s’)
            // g(s’) = g(s) + c(s,s’);
            // insert s’ into OPEN;

        // if (s == S_goal) break; [Do full search instead of stopping]

        std::vector<int> neighbors = get_neighbors(s);
        for (int s_p : neighbors)
        {
            if (closed_list.find(s_p) == closed_list.end() && 
                is_map_index_valid(GETXFROMINDEX(s_p, x_size), GETYFROMINDEX(s_p, x_size))) 
            {
                int cost = g_values[s] + calc_cost(s_p);
                if (g_values[s_p] > cost)
                {
                    g_values[s_p] = cost;
                    steps[s_p] = steps[s] + 1;
                    open_list.push({cost, s_p});
                }
            }
        }
    }

    // int cur = S_goal;
    // while (parent[cur] != -1 && parent[cur] != S_start) {
    //     cur = parent[cur];
    // }

    // printf("is valid move: %d\n", is_map_index_valid(GETXFROMINDEX(cur, x_size), GETYFROMINDEX(cur, x_size)));
    // action_ptr[0] = GETXFROMINDEX(cur, x_size);
    // action_ptr[1] = GETYFROMINDEX(cur, x_size);

    // printf("move: %d %d \n", GETXFROMINDEX(cur, x_size),  GETYFROMINDEX(cur, x_size));
    // printf("\n");
    return;
}
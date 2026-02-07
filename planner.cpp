/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include <vector>
#include <queue>
#include <set>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

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
    auto is_map_index_valid = [&](int newx, int newy) -> bool {
        int idx = GETMAPINDEX(newx, newy, x_size, y_size);
        return (map[idx] >= 0) && (map[idx] < collision_thresh);
    };

    auto get_index = [&](int y, int x) -> int {
        return y * x_size + x;
    };

    auto calc_heuristic = [&](int node_index) -> int {
        return node_index; // TODO
    };

    auto calc_cost = [&](int node_index) -> int {
        return calc_heuristic(node_index); // TODO
    };

    auto get_neighbors = [&](int node_index) -> std::vector<int> {
        return;
    };

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    int goalposeX = target_traj[curr_time+1];
    int goalposeY = target_traj[curr_time+target_steps+1];
    printf("goal: %d %d;\n", goalposeX, goalposeY);

    std::vector<int> g_values = init_gvalues(x_size, y_size);

    int S_goal = get_index(goalposeY, goalposeX);
    int S_start = get_index(robotposeY, robotposeX);

    using State = std::pair<int, int>;  // (cost, node)

    std::priority_queue<State, std::vector<State>, std::greater<State>> open_list;
    std::set<int> closed_list;

    open_list.push({calc_cost(S_start), S_start});

    while (!open_list.empty())
    {
        // remove s with smallest g value from OPEN
        auto [g, node] = open_list.top();
        open_list.pop();

        // add s to CLOSED
        closed_list.insert(node);

        // for every neighbor s` of s such that s` is not in CLOSED
            //  if g(s’) > g(s) + c(s,s’)
            // g(s’) = g(s) + c(s,s’);
            // insert s’ into OPEN;

        std::vector<int> neighbors = get_neighbors(node);
        for (int i : neighbors)
        {
            if (closed_list.find(i) == closed_list.end()) {
                continue;
            }
        }
    }


    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (is_map_index_valid(newx, newy))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }


    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    printf("\n");
    return;
}
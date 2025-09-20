//
// Created by ranais on 9/16/25.
//

#include "../include/planner.h"
#include "../include/State.h"
#include <climits>
#include <cmath>
#include <unordered_set>

/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <queue>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


// how many actions it took to traverse the array
// init to MAX_INT initially
State states[2000][2000] = {};

void initialize(const int *map, const int &xsize, const int &ysize) {
    for (unsigned int x = 0; x < xsize; x++) {
        for (unsigned int y = 0; y < ysize; y++) {
            map[GETMAPINDEX(x, y, xsize, ysize)]
        }
    }
}

/**
 *
 * @param map The actual cost map that is used to estimate where obstacles are
 * @param collision_thresh The value that defines what classifies as an obstacle
 * @param x_size Size of the horizontal dimension (total num_cols). Column iterators goes from 1 to x_size
 * @param y_size Size of the vertical dimension (total num_rows). Row iterator goes from 1 to y_size
 * @param robotposeX current pose x of the robot
 * @param robotposeY current y pose of the robot
 * @param target_steps the total number of steps that the robot takes before disappearing
 * @param target_traj the trajectory that should be indexed by (t+1)th time step to get the curr pose of the robot
 * @param targetposeX target_traj[curr_time + 1][0]
 * @param targetposeY target_traj[curr_time + 1][1]
 * @param curr_time the current time step
 * @param action_ptr the next step to take [x, y]
 */
void planner(
    int *map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int *target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int *action_ptr
) {
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    State goalState;
    goalState.x = target_traj[target_steps - 1],
            goalState.y = target_traj[target_steps - 1 + target_steps],
            goalState.t = curr_time;
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    for (int dir = 0; dir < NUMOFDIRS; dir++) {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
            /* Why is (map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) condition needed? */
            /* X, Y are 1 indexed.  */
            if ((map[GETMAPINDEX(newx, newy, x_size, y_size)] >= 0) && (
                    map[GETMAPINDEX(newx, newy, x_size, y_size)] < collision_thresh)) //if free
            {
            }
        }
    }
    robotposeX = robotposeX + x_move;
    robotposeY = robotposeY + y_move;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    return;
}

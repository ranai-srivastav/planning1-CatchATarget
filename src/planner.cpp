/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <cmath>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

/**
 *
 * @param map The actual cost map that is used to estimate where obstacles are
 * @param collision_thresh The value that defines what classifies as an obstacle
 * @param x_size Size of the horizontal dimension (total num_cols). Column iterators goes from 1 to x_size
 * @param y_size Size of the vertical dimension (total num_rows). Row iterator goes from 1 to y_size
 * @param robotposeX current pose x of the robot
 * @param robotposeY current y pose of the robot
 * @param target_steps the total number of steps that the robot takes before disappearing
 * @param target_traj the trajectory that should be indexed by (t+1)th time step to get the curr pose of the target, [x1, x2, x3, ... , y1, y2, y3]
 * @param targetposeX target_traj[curr_time + 1][0]
 * @param targetposeY target_traj[curr_time + 1][1]
 * @param curr_time the current time step. TODO I think starts at 1 not 0
 * @param action_ptr the next step to take [x, y]
 */
void def_planner(
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

    // for now greedily move towards the final target position,
    // but this is where you can put your planner
    int goalposeX = target_traj[target_steps-1];                // Shouldnt this be curr_time
    int goalposeY = target_traj[target_steps-1 + target_steps]; // Shouldn't this be target_steps - 1 + curr_time
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double oldDistToTarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double distToTarget;


    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            /* Why is (map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) condition needed? */
            if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                distToTarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(distToTarget < oldDistToTarget)
                {
                    oldDistToTarget = distToTarget;
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

    return;
}

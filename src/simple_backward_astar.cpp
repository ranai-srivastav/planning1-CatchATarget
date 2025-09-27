//
// Created by ranais on 9/16/25.
//

#define DEBUG 0

#include "../include/planner.h"
#include "../include/State.h"
#include <climits>
#include <cmath>
#include <vector>
#include <unordered_set>

/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <fstream>
#include <iostream>

#include "../include/planner.h"
#include <math.h>
#include <memory>
#include <queue>

// how many actions it took to traverse the array
// init to MAX_INT initially
// std::shared_ptr<State> states[2000][2000] = {};

SharedPtr_State_UnorderedSet djik_allStates;
SharedPtr_State_UnorderedSet allStates;
SharedPtr_State_UnorderedSet goalStates;
SharedPtr_State_UnorderedSet closedList;
std::priority_queue<std::shared_ptr<State>, // --------------------- Datatype of each element
    std::vector<std::shared_ptr<State> >, // ------------- Underlying data structure
    CompareFValues> openList; //------------------- Comparator being used

std::vector<std::shared_ptr<State> > pathToGoal;
std::shared_ptr<State> finalGoalState;

bool doneInit = false;
int traj_idx = -1;

void start_backtracking(const std::shared_ptr<State> &goal, std::vector<std::shared_ptr<State> > &ret_pathToGoal) {
    printf(" -- -- SUCCESS STARTING BACKTRACKING -- -- \n ");
    std::shared_ptr<State> currState = goal;
    while (currState->pred != nullptr) {
        ret_pathToGoal.push_back(currState);
        currState = currState->pred;
    }
    ret_pathToGoal.push_back(currState);
    traj_idx = 0;
}

std::priority_queue<std::shared_ptr<State>, std::vector<std::shared_ptr<State> >, CompareHValues> dijk_openList;
std::shared_ptr<State> roboStartState;

/**
 *
 * @param map The actual cost map that is used to estimate where obstacles are. This goes from [0, 0]
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
    if (!doneInit) {
        State::initStaticVars(map, collision_thresh, x_size, y_size, target_steps);

        roboStartState = std::make_shared<State>(robotposeX, robotposeY, 0);
        roboStartState->heuristic = 0;
        roboStartState->pred = nullptr;
        roboStartState->isInGoalTraj = true;
        roboStartState->g_stateCost = INT16_MAX;
        roboStartState->g_actionCost = INT16_MAX;
        goalStates.emplace(roboStartState);
#if DEBUG
#endif

        for (unsigned int goalT = 0; goalT < target_steps; ++goalT) {
            int goalX = target_traj[goalT];
            int goalY = target_traj[target_steps + goalT];
            std::shared_ptr<State> goalState = std::make_shared<State>(goalX, goalY, goalT);
            goalState->pred = nullptr;
            goalState->g_stateCost = 0;
            goalState->g_actionCost = 0;
            goalState->isInGoalTraj = false;
            goalState->heuristic = State::manhattan(roboStartState, goalState);
#if DEBUG
            printf("x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s \n",
                   goalState->x, goalState->y, goalState->t, goalState->g_stateCost, goalState->heuristic,
                   goalState->isInGoalTraj ? "true" : "false");
#endif
            allStates.emplace(goalState);
            openList.emplace(goalState);
        }
        doneInit = true;
    }

    int stateNum = 0;
    if (pathToGoal.empty()) {
        while (!openList.empty()) {

            // Get the element witht he smallest F = g+h value
            const std::shared_ptr<State> currState = openList.top();
            openList.pop();
            stateNum++;

            //Check to ensure no stale states (states that have already been processed and CLOSED at ta more optimal f value are expanded again)
            auto iter_cl = closedList.find(currState);
            if (iter_cl != closedList.end()) {
                // something == this, i.e. state exists in closed
                #if DEBUG
                printf("Skipping (%d, %d, %d) because explored before\n", (*iter_cl)->x, (*iter_cl)->y, (*iter_cl)->t);
                #endif
                continue;
            }

            //If the currently expanded state is the goalState,
            if (currState->isInGoalTraj || goalStates.contains(currState) || *currState == State::getImagGoal()) {
                printf("Starting backtracking after exploring %d states --------------- \n", stateNum);
                start_backtracking(currState, pathToGoal);
                std::reverse(pathToGoal.begin(), pathToGoal.end()); // uncomment for backwards A*
                std::shared_ptr<State> ret_state = pathToGoal.back();
                action_ptr[0] = ret_state->x;
                action_ptr[1] = ret_state->y;
                pathToGoal.pop_back();
                return;
            }

            #if DEBUG
            printf(
                "x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
                currState->x, currState->y, currState->t, currState->g_stateCost, currState->heuristic,
                currState->isInGoalTraj ? "true" : "false", openList.size(), closedList.size());
            #endif

            //Get the 3D successors
            std::vector<std::shared_ptr<State> > neighbors = State::get3DSuccessors(allStates, currState, false);

            for (char idx = 0; idx < neighbors.size(); ++idx) {
                std::shared_ptr<State> nbr = neighbors[idx];
                if ((nbr->heuristic > target_steps)) {
                    // continue;
                }
                // nbr->heuristic = State::manhattan(nbr, finalGoalState);
                nbr->heuristic = State::manhattan(nbr, roboStartState);
                int new_nbrCost = currState->g_stateCost + State::getPassedCostMap(nbr->x, nbr->y);
                if (nbr->g_stateCost > new_nbrCost && !closedList.contains(neighbors[idx])) {
                    nbr->g_actionCost = currState->g_actionCost + 1;
                    nbr->g_stateCost = new_nbrCost;
                    nbr->pred = currState;

                    #if DEBUG
                    printf(" -- x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
                        nbr->x, nbr->y, nbr->t, nbr->g_stateCost, nbr->heuristic,
                        nbr->isInGoalTraj ? "true" : "false", openList.size(), closedList.size());
                    #endif

                    openList.emplace(neighbors[idx]);
                    while (allStates.contains(neighbors[idx])) {
                        allStates.erase(neighbors[idx]);
                    }
                    allStates.emplace(neighbors[idx]);
                }
                else {
                    // printf("ignored\n");
                }
            }
            closedList.emplace(currState);
        }
    } else {
        printf(" -- -- SUCCESS OUTPUTING TRAJ -- -- \n ");
        const std::shared_ptr<State> &ret_state = pathToGoal.back();
        pathToGoal.pop_back();

        action_ptr[0] = ret_state->x;
        action_ptr[1] = ret_state->y;
    }
}
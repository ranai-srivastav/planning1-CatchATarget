//
// Created by ranais on 9/16/25.
//

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
#include <iostream>

#include "../include/planner.h"
#include <math.h>
#include <memory>
#include <queue>

// how many actions it took to traverse the array
// init to MAX_INT initially
// std::shared_ptr<State> states[2000][2000] = {};

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
    // ret_pathToGoal.push_back(currState);
    traj_idx = 0;
}


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

        for (unsigned int goalT = 0; goalT < target_steps; ++goalT) {
            int goalX = target_traj[goalT];
            int goalY = target_traj[target_steps + goalT];
            std::shared_ptr<State> goalState = std::make_shared<State>(goalX, goalY, 0);
            goalState->isInGoalTraj = true;
            goalState->heuristic = 0;
            printf("x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s \n",
                goalState->x, goalState->y, goalState->t, goalState->g_stateCost, goalState->heuristic, goalState->isInGoalTraj ? "true" : "false");
            goalStates.emplace(goalState);
        }
        printf("__________ DONE ADDING GOALS ___________ \n");

        finalGoalState = std::make_shared<State>(target_traj[target_steps], target_traj[2 * target_steps - 1], 0);
        finalGoalState->isInGoalTraj = true;
        finalGoalState->heuristic = 0;

        // printf("x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s \n", finalGoalState->x, finalGoalState->y, finalGoalState->t, finalGoalState->g_stateCost, finalGoalState->heuristic, finalGoalState->isInGoalTraj ? "true" : "false");

        // std::shared_ptr<State> test_goal_state = std::make_shared<State>(finalGoalState->x, finalGoalState->y, 0);
        // test_goal_state->isInGoalTraj = true;
        // test_goal_state->g_stateCost = 250;
        // test_goal_state->heuristic = 250;
        // openList.push(test_goal_state);
        // allStates.emplace(test_goal_state);

        auto roboStartState = std::make_shared<State>(robotposeX, robotposeY, 0);
        roboStartState->g_actionCost = 0;
        roboStartState->g_stateCost = 0;
        roboStartState->heuristic = State::manhattan(roboStartState, finalGoalState);
        roboStartState->pred = nullptr;

        allStates.emplace(roboStartState);
        openList.emplace(roboStartState);

        doneInit = true;
    }

    if (pathToGoal.empty()) {
        while (!openList.empty()) {
            const std::shared_ptr<State> currState = openList.top();
            auto iter = closedList.find(currState);
            if (iter != closedList.end()) { // something == this state exists in closed
                printf("Skipping (%d, %d) because explrored before\n", (*iter)->x, (*iter)->y);
                openList.pop();
                continue;
            }

            if (currState->isInGoalTraj || goalStates.contains(currState)) {
                start_backtracking(currState, pathToGoal);
                std::shared_ptr<State> ret_state = pathToGoal.back();
                action_ptr[0] = ret_state->x;
                action_ptr[1] = ret_state->y;
                pathToGoal.pop_back();
                return;
            }
            printf("x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
                currState->x, currState->y, currState->t, currState->g_stateCost, currState->heuristic, currState->isInGoalTraj ? "true" : "false", openList.size(), closedList.size());
            std::vector<std::shared_ptr<State>> neighbors = State::get2DSuccessors(allStates, currState);

            if (std::abs(currState->x - finalGoalState->x) < 1 && std::abs(currState->y - finalGoalState->y) < 1) {
                printf("!!!!!EXPANDING GOAL!!!!!!!!!");
            }

            for (char idx = 0; idx < neighbors.size(); ++idx) {
                std::shared_ptr<State> nbr = neighbors[idx];
                allStates.emplace(nbr);
                nbr->heuristic = State::manhattan(nbr, finalGoalState);
                int new_nbrCost = currState->g_stateCost + State::getPassedCostMap(nbr->x, nbr->y);
                if (nbr->g_stateCost > new_nbrCost) {
                    nbr->g_actionCost = currState->g_actionCost + 1;
                    nbr->g_stateCost = new_nbrCost;
                    nbr->pred = currState;
                    if (!closedList.contains(neighbors[idx])) {
                        openList.emplace(neighbors[idx]);
                    } else {
                        // printf("Not adding one\n");
                    }
                } else {
                    // printf("Not adding two\n");
                }

                if (neighbors.at(idx) == finalGoalState) {
                    std::cout << "You have added a goal state into the OPEN list" << std::endl;
                }
            }
            openList.pop();
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

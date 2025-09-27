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
    // ret_pathToGoal.push_back(currState);
    traj_idx = 0;
}

std::priority_queue<std::shared_ptr<State>, std::vector<std::shared_ptr<State> >, CompareHValues> dijk_openList;

void calcBackDijkHeuristic() {
    int hVals[State::getYSize()][State::getXSize()];
    int stateCount = 0;

    for (std::shared_ptr<State> s: goalStates) {
        s->g_actionCost = 0;
        s->g_stateCost = 0;
        s->heuristic = 0;
        dijk_openList.push(s);
        djik_allStates.emplace(s);
        hVals[s->y][s->x] = 0;
    }

    while (!dijk_openList.empty()) {
        std::shared_ptr<State> currState = dijk_openList.top();
        dijk_openList.pop();

        //Check to ensure no stale states (states that have already been processed and CLOSED at ta more optimal f value are expanded again)
        auto iter_cl = closedList.find(currState);
        if (iter_cl != closedList.end()) {
            // something == this state exists in closed
            printf("Skipping (%d, %d) because explored before\n", (*iter_cl)->x, (*iter_cl)->y);
            continue;
        }

        stateCount++;
        // printf(" lowest h val ----- \n");
        // printf("x=%d,\t y=%d,\t t=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
        //        currState->x, currState->y, currState->t, currState->heuristic,
        //        currState->isInGoalTraj ? "true" : "false", dijk_openList.size(), closedList.size());

        std::vector<std::shared_ptr<State> > neighbors = State::get2DSuccessors(djik_allStates, currState, false);

        for (std::shared_ptr<State> &nbr: neighbors) {
            int altCost = currState->heuristic + State::getPassedCostMap(nbr->x, nbr->y);
            // int altCost = currState->heuristic + 1;
            if (!closedList.contains(nbr) && (nbr->heuristic > altCost)) {
                nbr->heuristic = altCost;
                hVals[nbr->y][nbr->x] = altCost;
                // printf(" -- x=%d,\t y=%d,\t t=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
                //    nbr->x, nbr->y, nbr->t, nbr->heuristic, nbr->isInGoalTraj ? "true" : "false", dijk_openList.size(), closedList.size());
                dijk_openList.push(nbr);
                if (djik_allStates.contains(nbr)) {
                    djik_allStates.erase(nbr);
                }
                djik_allStates.emplace(nbr);
            }
        }
        closedList.emplace(currState);
    }

    printf("Total %d states expanded in Djikstras's, should have been %d \n",
           stateCount, State::getXSize() * State::getYSize());
    closedList.clear();
    // allStates.clear();

    std::ofstream hFile("output.txt");
    for (int y=0; y<State::getYSize(); y++) {
        for (int x=0; x<State::getXSize(); x++) {
            hFile << hVals[y][x] << "\t";
        }
        hFile << "\n";
    }
    hFile.close();
}

double getHeuristic(std::shared_ptr<State> currState) {
    // IN the most inefficient way possible, getting the 2D reverse dijkstras values
    std::shared_ptr<State> temp_state = std::make_shared<State>(currState->x, currState->y, 0);
    auto iter_as = djik_allStates.find(temp_state);
    if (iter_as != djik_allStates.end()) {
        // printf("h(x) is %f", (*iter_as)->heuristic);
        return (*iter_as)->heuristic;
    }
    printf("IMPOSSIBLE for (%d, %d, %d)", currState->x, currState->y, 0);
    throw std::invalid_argument("WTF");
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
        allStates.clear();
        goalStates.clear();

        // for (unsigned int goalT = 0; goalT < target_steps; ++goalT) {
        //     int goalX = target_traj[goalT];
        //     int goalY = target_traj[target_steps + goalT];
        //     std::shared_ptr<State> goalState = std::make_shared<State>(goalX, goalY, 0);
        //     goalState->isInGoalTraj = true;
        //     goalState->heuristic = 0;
        //     goalStates.emplace(goalState);
        //     allStates.emplace(goalState);
        // }

        // calcBackDijkHeuristic();

        printf("__________ ADDING THESE GOALS ___________ \n");
        for (unsigned int goalT = 0; goalT < target_steps; ++goalT) {
            int goalX = target_traj[goalT];
            int goalY = target_traj[target_steps + goalT];
            std::shared_ptr<State> goalState = std::make_shared<State>(goalX, goalY, goalT);
            goalState->isInGoalTraj = true;
            goalState->heuristic = 0;
            // printf("x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s \n",
            //        goalState->x, goalState->y, goalState->t, goalState->g_stateCost, goalState->heuristic,
            //        goalState->isInGoalTraj ? "true" : "false");
            // goalStates.emplace(goalState);
            allStates.emplace(goalState);
            openList.emplace(goalState);
        }
        printf("__________ DONE ADDING GOALS ___________ \n");
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
        roboStartState->heuristic = getHeuristic(roboStartState);
        roboStartState->pred = nullptr;

        // if (!allStates.empty()) {
        //     printf("AllStates not cleared. \n");
        //     throw std::invalid_argument("AllStates not cleared");
        // }
        if (!closedList.empty()) {
            printf("ClosedList not cleared. \n");
            throw std::invalid_argument("ClosedList not cleared");
        }
        if (!openList.empty()) {
            printf("OpenList not cleared. \n");
            throw std::invalid_argument("OpenList not cleared");
        }

        allStates.emplace(roboStartState);
        openList.emplace(roboStartState);

        doneInit = true;
    }
    int stateNum = 0;
    if (pathToGoal.empty()) {
        while (!openList.empty()) {
            // Get the element witht he smallest F = g+h value
            const std::shared_ptr<State> currState = openList.top();
            openList.pop();
            stateNum++;
            // printf("%d", stateNum);

            //Check to ensure no stale states (states that have already been processed and CLOSED at ta more optimal f value are expanded again)
            auto iter_cl = closedList.find(currState);
            if (iter_cl != closedList.end()) {
                // something == this state exists in closed
                printf("Skipping (%d, %d) because explored before\n", (*iter_cl)->x, (*iter_cl)->y);
                continue;
            }

            currState->heuristic = getHeuristic(currState);

            //If the currently expanded state is the goalState,
            if (currState->isInGoalTraj || goalStates.contains(currState) || *currState == State::getImagGoal()) {
                start_backtracking(currState, pathToGoal);
                std::shared_ptr<State> ret_state = pathToGoal.back();
                action_ptr[0] = ret_state->x;
                action_ptr[1] = ret_state->y;
                pathToGoal.pop_back();
                return;
            }

            printf(
                "x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
                currState->x, currState->y, currState->t, currState->g_stateCost, currState->heuristic,
                currState->isInGoalTraj ? "true" : "false", openList.size(), closedList.size());

            //Get the 3D successors
            std::vector<std::shared_ptr<State> > neighbors = State::get3DSuccessors(allStates, currState, true);

            for (char idx = 0; idx < neighbors.size(); ++idx) {
                std::shared_ptr<State> nbr = neighbors[idx];

                // nbr->heuristic = State::manhattan(nbr, finalGoalState);
                nbr->heuristic = getHeuristic(nbr);
                int new_nbrCost = currState->g_stateCost + State::getPassedCostMap(nbr->x, nbr->y);
                if (nbr->g_stateCost > new_nbrCost && !closedList.contains(neighbors[idx])) {
                    nbr->g_actionCost = currState->g_actionCost + 1;
                    nbr->g_stateCost = new_nbrCost;
                    nbr->pred = currState;

                    printf(" -- x=%d,\t y=%d,\t t=%d,\t g=%d,\t h=%f,\t isInGoalTraj=%s,\t len(openList)=%lu, len(closedList)=%lu\n",
                        nbr->x, nbr->y, nbr->t, nbr->g_stateCost, nbr->heuristic,
                        nbr->isInGoalTraj ? "true" : "false", openList.size(), closedList.size());

                    openList.emplace(neighbors[idx]);
                    if (allStates.contains(neighbors[idx])) {
                        allStates.erase(neighbors[idx]);
                    }
                    allStates.emplace(neighbors[idx]);
                }
            }
            closedList.emplace(currState);

            // if (currState->t > target_steps) {
            //     throw std::invalid_argument("RAN OUT OF TIME");
            // }
        }
    } else {
        printf(" -- -- SUCCESS OUTPUTING TRAJ -- -- \n ");
        const std::shared_ptr<State> &ret_state = pathToGoal.back();
        pathToGoal.pop_back();

        action_ptr[0] = ret_state->x;
        action_ptr[1] = ret_state->y;
    }
}

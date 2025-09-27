#include "../include/State.h"

#include <iostream>
#include <stdexcept>
#include <stdexcept>


State State::imagGoal;

/**
 *
 * @param inX
 * @param inY
 * @param inT
 * @param inCost
 */

State::State(int inX, int inY, int inT) : x(inX), y(inY), t(inT) {
    // if (t == 0) {
    //     printf("t == 0\n");
    // }
    if (passedCostMap == nullptr || xSize < 1 || ySize < 1) {
        std::cout << "passedCostMap = " << passedCostMap << "\t x=" << x << "\t y=" << y << "\t t=" << t << "\t" <<
                std::endl;
        throw std::invalid_argument("Static vars have probably not been initialized");
    }

    g_actionCost = INT16_MAX;
    g_stateCost = INT16_MAX;

    if (x < 1 || y < 1 || x > xSize || y > ySize) {
        printf("State::Constructor(params) have values less than 1 for either x=%d or y=%d\n", x, y);
        throw std::invalid_argument("");
    }

    if (passedCostMap[GETMAPINDEX(inX, inY, xSize, ySize)] > obsThresh) {
        isObs = true;
    } else {
        isObs = false;
    }

    if (passedCostMap[GETMAPINDEX(inX, inY, xSize, ySize)] < 0) {
        throw std::invalid_argument("State::Constructor(params) >> The map has negative values. WTF?!");
    }
}

double State::getFValue() const{
    return 2*this->heuristic + this->g_stateCost;
}


std::vector<std::shared_ptr<State> >
State::get2DSuccessors(const SharedPtr_State_UnorderedSet &openedStates, const std::shared_ptr<State> &currState,
                       bool checkGoal) {
    std::vector<std::shared_ptr<State> > retNeighbors;

    // unsigned char neighbor_idx = 0;
    for (char idx = 0; idx < 9; idx++) {
        int nbrX = currState->x + dX[idx];
        int nbrY = currState->y + dY[idx];

        //TODO if currState is already a goal, then do not add imagGoal. Will cause cycle?
        if ((nbrX > 0 && nbrX <= xSize) && (nbrY > 0 && nbrY <= ySize) &&
            (getPassedCostMap(nbrX, nbrY) < obsThresh)) {
            std::shared_ptr<State> new_nbr = std::make_shared<State>(nbrX, nbrY, 0);
            auto iter = openedStates.find(new_nbr);
            if (iter != openedStates.end()) {
                // using existing pointer
                std::shared_ptr<State> existing = *iter;
                retNeighbors.push_back(*iter);
            } else {
                // start == end so nothing exists in here
                retNeighbors.push_back(new_nbr);
            }
        }
    }

    if (checkGoal && currState->isInGoalTraj) {
        retNeighbors.push_back(std::make_shared<State>(State::imagGoal));
    }

    return retNeighbors;
}

std::vector<std::shared_ptr<State> > State::get3DSuccessors(const SharedPtr_State_UnorderedSet &openedStates,
                                                            const std::shared_ptr<State> &currState, bool checkGoal) {
    std::vector<std::shared_ptr<State> > retNeighbors;

    // unsigned char neighbor_idx = 0;
    for (char idx = 0; idx < 9; idx++) {
        int nbrX = currState->x + dX[idx];
        int nbrY = currState->y + dY[idx];

        //TODO if currState is already a goal, then do not add imagGoal. Will cause cycle?
        if ((nbrX > 0 && nbrX <= xSize) && (nbrY > 0 && nbrY <= ySize) &&
            (getPassedCostMap(nbrX, nbrY) < obsThresh)) {
            std::shared_ptr<State> new_nbr = std::make_shared<State>(nbrX, nbrY, currState->t + 1);
            auto iter = openedStates.find(new_nbr);
            if (iter != openedStates.end()) {
                // using existing pointer
                std::shared_ptr<State> existing = *iter;
                retNeighbors.push_back(*iter);
            } else {
                // start == end so nothing exists in here
                retNeighbors.push_back(new_nbr);
            }
        }
    }

    if (checkGoal && currState->isInGoalTraj) {
        retNeighbors.push_back(std::make_shared<State>(State::imagGoal));
    }

    return retNeighbors;
}

// void State::get3DSuccessors(std::shared_ptr<State> globalMap[2000][2000],
//                             const std::shared_ptr<State> &currState,
//                             std::array<std::shared_ptr<State>, 10> &retNeighbors) {
//     unsigned char neighbor_idx = 0;
//
//     for (char idx = 0; idx < 8; idx++) {
//         int nbrX = currState->x + dX[idx];
//         int nbrY = currState->y + dY[idx];
//
//         if ((nbrX >= 0 && nbrX < xSize) &&
//             (nbrY >= 0 && nbrY < ySize) &&
//             currState->t + 1 < trajLength) {
//             std::shared_ptr<State> newNeighbor = globalMap[nbrY][nbrX]; //TODO Look above for correct implementation
//             newNeighbor->t = currState->t + 1;
//             retNeighbors[neighbor_idx++] = newNeighbor;
//         }
//     }
// }

bool State::operator==(const State &other) const {
    // this const here says this function cannot change any values, only read them
    // std::printf("FYI Time is currently not being compared for equality between states.");
    return this->x == other.x && this->y == other.y && this->t == other.t;
}

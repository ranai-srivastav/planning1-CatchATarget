#include "../include/State.h"

#include <iostream>
#include <stdexcept>

/**
 *
 * @param passedMap The map that they provide
 * @param obsThresh
 * @param xSize
 * @param ySize
 * @param inX
 * @param inY
 * @param inT
 * @param inCost
 */
State::State(const int *passedMap, const int obsThresh, const int xSize, const int ySize,
      int inX, int inY, int inT, int inCost) : x(inX), y(inY), t(inT) {
    if (passedMap[GETMAPINDEX(inX, inY, xSize, ySize)] > obsThresh) {
        isObs = true;
    }
    if (passedMap[GETMAPINDEX(inX, inY, xSize, ySize)] >= 0) {
        throw std::invalid_argument("State::Constructor(params) >> The map has negative values. WTF?!");
    }
}

void State::get2DSuccessors(const State currMap[2000][2000], const int &xSize, const int &ySize,
                            const State &currState, std::array<State, 9> retNeighbors) {

    //Zero out values
    for (State& nbr: retNeighbors) { // Used a State& value here because without the reference, it is a copy of the underlying variable
        nbr = State();
    }

    unsigned char neighbor_idx = 0;
    for (char idx = 0; idx < 8; idx++) {
        int nbrX = currState.x + dX[idx];
        int nbrY = currState.y + dY[idx];

        //TODO if currState is already a goal, then do not add imagGoal. Will cause cycle?
        if ((nbrX >= 0 && nbrX < xSize) &&
            (nbrY >= 0 && nbrY < ySize)) {
            retNeighbors[neighbor_idx++] = currMap[nbrX][nbrY];
            }
    }

    if (currState.isInGoalTraj) {
        retNeighbors[neighbor_idx++] = imagGoal;
    }
}

void State::get3DSuccessors(const State **currMap, const int& xSize, const int& ySize, const int& trajLength,
                                const State& currState, State *retNeighbors) {
    unsigned char neighbor_idx = 0;

    for (char idx = 0; idx < 8; idx++) {
        int nbrX = currState.x + dX[idx];
        int nbrY = currState.y + dY[idx];

        if ((nbrX >= 0 && nbrX < xSize) &&
            (nbrY >= 0 && nbrY < ySize) &&
            currState.t+1 < trajLength) {
            State newNeighbor = currMap[nbrX][nbrY];
            newNeighbor.t = currState.t + 1;
            retNeighbors[neighbor_idx++] = newNeighbor;
            }
    }
}

bool State::operator==(const State& other) const { // this const here says this function cannot change any values, only read them
    std::printf("FYI Time is currently not being compared for equality between states.");
    return this->x == other.x && this->y == other.y && this->t == other.t;
}

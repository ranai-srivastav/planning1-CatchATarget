//
// Created by ranai on 9/19/25.
//

#include "State.h"

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
}
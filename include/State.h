//
// Created by ranai on 9/19/25.
//

#ifndef PLANNING_PSET1_STATE_H
#include <array>
#include <cstdint>
#include <functional>
#define PLANNING_PSET1_STATE_H

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


class State {
private:
    inline static int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 1, 1, 1};
    inline static int dY[NUMOFDIRS] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static State imagGoal;

public:
    uint16_t x = UINT16_MAX; //TODO Necessary?
    uint16_t y = UINT16_MAX; //TODO Necessary? Yes
    uint16_t t = UINT16_MAX;
    int16_t actionCost = INT16_MAX;
    int16_t distCost = INT16_MAX;
    int16_t heuristic = INT16_MAX;

    bool isInGoalTraj = false;
    bool isObs = false;
    State *pred = nullptr;

    State() = default;

    State(const int *passedMap, int obsThresh, int xSize, int ySize,
      int inX, int inY, int inT, int inCost);

    static void get2DSuccessors(const State currMap[2000][2000], const int &xSize, const int &ySize,
                            const State &currState, std::array<State, 9> retNeighbors);

    static void get3DSuccessors(const State **currMap, const int& xSize, const int& ySize,
                                const int& trajLength, const State& currState, State *retNeighbors);

    /**
     * TODO What values make 2 states the same? This is needed so the hash function can say if x == y, h(x) == h(y)
     * What does it mean for 2 states to be the same?
     * @param other The value we weill be comparing against
     * @return
     */
    bool operator==(const State &other) const;

    unsigned long operator()(const State &key) const;
};


#endif //PLANNING_PSET1_STATE_H
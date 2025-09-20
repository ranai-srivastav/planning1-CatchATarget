//
// Created by ranai on 9/19/25.
//

#ifndef PLANNING_PSET1_STATE_H
#include <array>
#include <cstdint>
#include <cstdio>
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

    // The least number of actions required to reach this state
    int16_t actionCost = INT16_MAX;

    //The least costly path of coming to this state
    int16_t stateCost = INT16_MAX;

    //An estimate of what this it costs to go from this state to the goal
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
};

namespace std {
    template<>
    struct hash<State> {
        size_t operator()(const State& key) const noexcept {
            size_t x_hash = hash<int>()(key.x);
            size_t y_hash = hash<int>()(key.y);
            size_t t_hash = hash<int>()(key.t);
            printf("FYI Time is currently not being hashed");

            return x_hash ^ (y_hash << 1) ^ (t_hash << 4);
        }
    };
}

template<auto MemberPtr>
struct StateComparator {
    /** Returns true if s2 gets popped first and then s1. `comparator(s1, s2)`
     * Returns True if s1.value > s2.value and s1 gets placed first, giving us descending order
     * The top of the heap is the "last element" or in this case, the smallest number which at the bottom.
     * @param s1 First State object to be compared
     * @param s2 Second State object to be compared
     * @return boolean if s1 is before s2
     */
    bool operator()(const State& s1, const State& s2) const noexcept {
        return s1.*MemberPtr > s2.*MemberPtr;
    }
};

using CompareGActionValues = StateComparator<&State::actionCost>;
using CompareGCostValues = StateComparator<&State::stateCost>;
using CompareHValues = StateComparator<&State::heuristic>;

struct CompareFValues {
    bool operator()(const State& s1, const State& s2) const noexcept {
        return (s1.actionCost + s1.heuristic) > (s2.actionCost + s2.heuristic);  // Min-heap behavior
    }
};


#endif //PLANNING_PSET1_STATE_H
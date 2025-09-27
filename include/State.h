//
// Created by ranai on 9/19/25.
//

#ifndef PLANNING_PSET1_STATE_H
#define PLANNING_PSET1_STATE_H

#include <array>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <cmath>
#include <unordered_set>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 9


// forward declare
class State;

// --- declare functor types (only declarations) -------------------------
struct SharedPtr_State_Hash {
    size_t operator()(const std::shared_ptr<State>& p) const noexcept; // declared here, defined later
};

struct SharedPtr_State_Eq {
    bool operator()(const std::shared_ptr<State>& a, const std::shared_ptr<State> &b) const noexcept;

    // declared here, defined later
};

// alias that depends on the functor *types* (they are declared above)
using SharedPtr_State_UnorderedSet = std::unordered_set<std::shared_ptr<State>, SharedPtr_State_Hash,
    SharedPtr_State_Eq>;


class State {
private:
    static State imagGoal;
    inline static int xSize = -1;
    inline static int ySize = -1;
    inline static int obsThresh = -1;
    inline static int trajLength = -1;
    inline static int *passedCostMap = nullptr;
    inline static int dX[NUMOFDIRS] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
    inline static int dY[NUMOFDIRS] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};

public:
    uint16_t x = UINT16_MAX;
    uint16_t y = UINT16_MAX;
    uint16_t t = UINT16_MAX;

    // The least number of actions required to reach this state
    int16_t g_actionCost = INT16_MAX;

    //The least costly path of coming to this state
    int16_t g_stateCost = INT16_MAX;

    //An estimate of what this it costs to go from this state to the goal
    double heuristic = INT16_MAX;

    bool isInGoalTraj = false;
    bool isObs = false;
    std::shared_ptr<State> pred = nullptr;

    State() = default;

    static State &getImagGoal() { return imagGoal; }
    static int getXSize() { return xSize; }
    static int getYSize() { return ySize; }
    static int getObsThresh() { return obsThresh; }
    static int getTrajLength() { return trajLength; }
    double getFValue(int epsilon) const;


    static int getPassedCostMap(const int x, const int y) { //TODO check
        // Add bounds checking:
        int index = GETMAPINDEX(x, y, xSize, ySize);
        int maxIndex = xSize * ySize - 1;
        if (index < 0 || index > maxIndex) {
            printf("ERROR: Index %d out of bounds [0, %d] for coords (%d, %d)\n", index, maxIndex, x, y);
            printf("Map dimensions: %dx%d\n", xSize, ySize);
            throw std::out_of_range("Map index out of bounds");
        }
        return passedCostMap[GETMAPINDEX(x, y, xSize, ySize)];
    }

    static void initStaticVars(int *passedMap, const int &obstacleThreshold, const int &x_map_size,
                               const int &y_map_size, const int &trajectoryLength) {
        imagGoal = State();
        imagGoal.x = -1;
        imagGoal.y = -1;
        imagGoal.t = -1;
        imagGoal.g_actionCost = -1;
        imagGoal.g_stateCost = 0;
        imagGoal.heuristic = 0;

        passedCostMap = passedMap;
        obsThresh = obstacleThreshold;
        xSize = x_map_size;
        ySize = y_map_size;
        trajLength = trajectoryLength;

    }

    static float euclidean(const std::shared_ptr<State> &s1, const std::shared_ptr<State> &s2) {
        return static_cast<float>(std::sqrt(std::pow((s1->x - s2->x), 2) + std::pow((s1->y - s2->y), 2)));
    }

    static float manhattan(const std::shared_ptr<State> &s1, const std::shared_ptr<State> &s2) {
        return (std::abs((s1->x - s2->x)) + std::abs((s1->y - s2->y)));
    }

    State(int inX, int inY, int inT);

    static std::vector<std::shared_ptr<State> > get2DSuccessors(
        const SharedPtr_State_UnorderedSet& exploredStates,
        const std::shared_ptr<State>& currState,
        bool checkGoal);

    // std::array<std::shared_ptr<State>, 10>& retNeighbors);

    static std::vector<std::shared_ptr<State>> get3DSuccessors(const SharedPtr_State_UnorderedSet& exploredStates,
                                const std::shared_ptr<State>& currState,
                                bool checkGoal);

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
        size_t operator()(const State &key) const noexcept {
            size_t x_hash = hash<int>()(key.x);
            size_t y_hash = hash<int>()(key.y);
            size_t t_hash = hash<int>()(key.t);
            // printf("FYI Time is currently not being hashed \n");

            return x_hash ^ (y_hash << 1) ^ (t_hash << 4);
        }
    };
}

// --- Define the functor implementations (need State complete) -----------
inline size_t SharedPtr_State_Hash::operator()(const std::shared_ptr<State> &p) const noexcept {
    if (!p) return 0;
    return std::hash<State>()(*p); // uses std::hash<State> specialization above
}

inline bool SharedPtr_State_Eq::operator()(const std::shared_ptr<State> &a,
                                           const std::shared_ptr<State> &b) const noexcept {
    if (a == b) return true;
    if (!a || !b) return false;
    return *a == *b; // uses State::operator==
}

template<auto MemberPtr>
struct StateComparator {
    /** Returns true if s2 gets popped first and then s1. `comparator(s1, s2)`
     * Returns True if s1.value > s2.value and s1 gets placed first, giving us descending order where lowerd idxs have larger values.
     * The top of the heap is the "last element" in the underlying data structure or in this case, the smallest number. Creates Ascending order.
     * @param s1 First State object to be compared
     * @param s2 Second State object to be compared
     * @return boolean if s1 is before s2
     */
    bool operator()(const std::shared_ptr<State> &s1, const std::shared_ptr<State> &s2) const noexcept {
        return *s1.*MemberPtr > *s2.*MemberPtr;
    }
};

using CompareGActionValues = StateComparator<&State::g_actionCost>;
using CompareGCostValues = StateComparator<&State::g_stateCost>;
using CompareHValues = StateComparator<&State::heuristic>;

struct CompareFValues {
    bool operator()(const std::shared_ptr<State> &s1,
                    const std::shared_ptr<State> &s2) const noexcept
    {
        double f1 = s1->getFValue(2);
        double f2 = s2->getFValue(2);

        constexpr double eps = 1e-9;
        if (std::fabs(f1 - f2) > eps) {
            // min-heap behavior: element with smaller f has higher priority,
            return f1 > f2;
        }

        // tie-break: when f1 == f2, element with LARGER timestamp should be popped first
        // => treat larger timestamp as higher priority. So s1 is lower priority when
        // its timestamp is smaller than s2's.
        return s1->heuristic < s2->heuristic;
    }
};

#endif //PLANNING_PSET1_STATE_H

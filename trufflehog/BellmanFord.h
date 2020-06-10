/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#ifndef BCP_MAPF_BELLMANFORD_H
#define BCP_MAPF_BELLMANFORD_H

#include "AbstractPathfinder.h"

namespace TruffleHog {
    class BellmanFord : public AbstractPathfinder {

    // Instance
    const Map& map_;

    public:
        BellmanFord() = delete;
        BellmanFord(const Map& map);
        BellmanFord(const BellmanFord&) = delete;
        BellmanFord(BellmanFord&&) = delete;
        BellmanFord& operator=(const BellmanFord&) = delete;
        BellmanFord& operator=(BellmanFord&&) = delete;
        ~BellmanFord() = default;

        // Getters
        // TODO(@Isha) implement
        Time max_path_length() override { return 3; }
        ReservationTable& reservation_table() override {  };
        EdgePenalties& edge_penalties() override { }
        Vector<Cost>& time_finish_penalties() override { }
#ifdef USE_GOAL_CONFLICTS

        Vector<GoalCrossing>& goal_crossings() override {};
#endif
//         Solve
        void compute_h(const Node goal) override { }

        Pair<Vector<NodeTime>, Cost> solve(NodeTime start,
                                           Node goal,
                                           Time goal_earliest = 0,
                                           Time goal_latest = std::numeric_limits<Time>::max(),
                                           Cost max_cost = std::numeric_limits<Cost>::infinity()) override {

        return Pair<Vector<NodeTime>, Cost>({NodeTime(1, 2), NodeTime(3, 2)}, 3);
        }

    };

}

#endif //BCP_MAPF_BELLMANFORD_H

/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#ifndef BCP_MAPF_BELLMANFORD_H
#define BCP_MAPF_BELLMANFORD_H

#include "AbstractPathfinder.h"
#include "Instance.h"
#include <iostream>
#include <list>
#include <limits>

namespace TruffleHog {
    class BellmanFord : public AbstractPathfinder {

        // Instance
        const Instance &instance_;


    public:
        BellmanFord() = delete;

        BellmanFord(const Instance &instance);

        BellmanFord(const BellmanFord &) = delete;

        BellmanFord(BellmanFord &&) = delete;

        BellmanFord &operator=(const BellmanFord &) = delete;

        BellmanFord &operator=(BellmanFord &&) = delete;

        ~BellmanFord() = default;

        // Getters
        // TODO(@Isha) implement
        Time max_path_length() override { return 3; }

        bool hasReservationTable() override { return false; };

        ReservationTable &reservation_table() override {};

        // Solve
        void compute_h(const Node goal) override {}

        Pair<Vector<NodeTime>, Cost> solve(NodeTime start,
                                           Node goal,
                                           Time goal_earliest = 0,
                                           Time goal_latest = std::numeric_limits<Time>::max(),
                                           Cost max_cost = std::numeric_limits<Cost>::infinity());


    private:
        int mapSize;
        std::list<Node> vertexCover;
        Vector<bool> uncovered;
        Vector<Int> degree;


        //void updateDistances(int d, int MAX_VALUE, Vector<Int> *distances1, Node w, std::list<Node> *queue_new);

        void reset();


        void
        updateDistances(int d, int MAX_VALUE, Vector <TruffleHog::Cost> *distances1,
                        Vector <TruffleHog::NodeTime> *parents1,
                        NodeTime w, NodeTime v, std::list<NodeTime> *queue_new, Cost cost);
    };

}

#endif //BCP_MAPF_BELLMANFORD_H

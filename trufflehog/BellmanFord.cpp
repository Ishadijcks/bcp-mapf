/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include "BellmanFord.h"

namespace TruffleHog {
    BellmanFord::BellmanFord(const Map &map)
            : map_(map) {
    }

    Pair<Vector<NodeTime>, Cost>
    BellmanFord::solve(NodeTime start, Node goal, Time goal_earliest, Time goal_latest, Cost max_cost) {
        return Pair<Vector<NodeTime>, Cost>({NodeTime(1, 2), NodeTime(3, 2)}, 3);
    }


    void
    BellmanFord::updateDistances(int d, int MAX_VALUE, Vector<Int> *distances1, Node w, std::list<Node> *queue_new) {
        bool changed = false;
        if (d < MAX_VALUE && (*distances1)[w] > d + 1) {
            (*distances1)[w] = d + 1;
            changed = true;
        }

        if (changed) {
            bool found = (std::find(queue_new->begin(), queue_new->end(), w) != queue_new->end());
            if (!found) {
                queue_new->push_front(w);
            }
        }

    }

}
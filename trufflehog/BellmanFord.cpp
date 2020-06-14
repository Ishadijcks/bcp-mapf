/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include "BellmanFord.h"

namespace TruffleHog {
    BellmanFord::BellmanFord(const Map& map)
            : map_(map){
    }

    Pair<Vector<NodeTime>, Cost>
    BellmanFord::solve(NodeTime start, Node goal, Time goal_earliest, Time goal_latest, Cost max_cost)  {
        return Pair<Vector<NodeTime>, Cost>({NodeTime(1, 2), NodeTime(3, 2)}, 3);
    }


}
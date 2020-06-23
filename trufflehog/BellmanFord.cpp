/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include "BellmanFord.h"
#include "Instance.h"

namespace TruffleHog {


    BellmanFord::BellmanFord(const Map& map) :
    map_(map),
    heuristic_(map, label_pool_)
    {

        this->reset();
    }

    Pair<Vector<NodeTime>, Cost>
    BellmanFord::solve(NodeTime start, Node goal, Time goal_earliest, Time goal_latest, Cost max_cost) {

        auto map = map_;
        int mapSize = map.size();

        std::list<NodeTime> vertexCover = {};

        Vector<bool> uncovered (mapSize);
        Vector<Int> degree (mapSize);

        Cost segment_cost = std::numeric_limits<int>::max();

        Node source = start.n;
        Vector<Cost> distances1 (mapSize * goal_latest, std::numeric_limits<int>::max());
        Vector<NodeTime> parents1 (mapSize * goal_latest);

        int MAX_VALUE = max_cost;

        distances1[source] = start.t;

        std::list<NodeTime> queue_cur;
        std::list<NodeTime> queue_new;
        std::list<NodeTime>* temp;

        queue_new.push_front(source);

        for ( int i = 0; i < mapSize * goal_latest; ++i ) {

            if (queue_new.empty()) break;

            temp = &queue_cur;
            queue_cur = queue_new;
            queue_new = *temp;
            while (!queue_new.empty())
                queue_new.pop_back();

            while (!queue_cur.empty()) {
                NodeTime v = queue_cur.back();

                queue_cur.pop_back();

                //TODO check
                const auto edge_costs = edge_penalties_.get_edge_costs<1>(v);

                Cost d = distances1[v.t * mapSize + v.n];


                if (const auto next_n = map.get_north(v.n);
                        map[next_n] && !std::isnan(edge_costs.north)) {
                    NodeTime w = NodeTime{next_n, v.t + 1};

                    updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.north, &segment_cost, goal);
                }

                //south
                if (const auto next_n = map.get_south(v.n);
                        map[next_n] && !std::isnan(edge_costs.south))  {
                    NodeTime w = NodeTime{next_n, v.t + 1};

                    updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.south, &segment_cost, goal);
                }

                //east
                if (const auto next_n = map.get_east(v.n);
                        map[next_n] && !std::isnan(edge_costs.east)) {
                    NodeTime w = NodeTime{next_n, v.t + 1};


                    updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.east, &segment_cost, goal);
                }

                //west
                if (const auto next_n = map.get_west(v.n);
                        map[next_n] && !std::isnan(edge_costs.west)) {
                    NodeTime w = NodeTime{next_n, v.t + 1};

                    updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.west, &segment_cost, goal);
                }

                //wait
                if (const auto next_n = map.get_wait(v.n);
                        map[next_n] && !std::isnan(edge_costs.wait)) {
                    NodeTime w = NodeTime{next_n, v.t + 1};

                    updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.wait, &segment_cost, goal);
                }

            }

        }


        std::list<NodeTime> queue;
        Vector<NodeTime> segment (0);

        //Find path
        Node n = goal;
        queue.emplace_back(NodeTime{goal, static_cast<Int>(segment_cost)});
        NodeTime node = parents1[segment_cost * mapSize + n];
        queue.emplace_back(node);
        n = node.n;
        while (parents1[node.t * mapSize + n] != NULL){
            node = parents1[node.t * mapSize + n];
            queue.emplace_back(node);
            n = node.n;
        }

        while (!queue.empty()) {
            segment.push_back(queue.back());
            queue.pop_back();
        }

        return Pair<Vector<NodeTime>, Cost>(segment, segment_cost);
    }

    void
    BellmanFord::updateDistances(int d, int MAX_VALUE, Vector<Cost> *distances1, Vector<NodeTime> *parents1,
            NodeTime w, NodeTime v, std::list<NodeTime> *queue_new, Cost cost, Cost* segment_cost, Node goal) {
        bool changed = false;
        if (d < MAX_VALUE &&  ((*distances1)[w.t * mapSize + w.n] > d + cost)) {
            (*distances1)[w.t * mapSize + w.n] = d + cost;
            if (w.n == goal && d + cost < (*segment_cost)) {
                (*segment_cost) = d + cost;
            }
            (*parents1)[w.t * mapSize + w.n] = v;
            changed = true;
        }

        if (changed) {
            bool found = (std::find(queue_new->begin(), queue_new->end(), w) != queue_new->end());
            if (!found) {
                queue_new->push_front(w);
            }
        }

    }


    void BellmanFord::reset() {
        mapSize = map_.size();
        vertexCover = {};
        uncovered = Vector<bool>(mapSize);
        degree = Vector<Int>(mapSize);
    }


}
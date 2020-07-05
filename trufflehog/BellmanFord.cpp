/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include "BellmanFord.h"
#include "Instance.h"

namespace TruffleHog {



    BellmanFord::BellmanFord(const Instance &instance) :
    instance_(instance),
    heuristic_(instance.map, label_pool_)
    {
        //instance.map.printPassable();
        //instance.map.printNodes();

        this->reset();
    }

    Pair<Vector<NodeTime>, Cost>
    BellmanFord::solve(NodeTime start, Node goal, Time goal_earliest, Time goal_latest, Cost max_cost) {

        auto map = instance_.map;
        int mapSize = map.size();

        std::list<NodeTime> vertexCover = {};

        Vector<bool> uncovered (mapSize);
        Vector<Int> degree (mapSize);


        for (int i=0; i<instance_.agents.size(); i++){

            auto agent = instance_.agents[i];

            uncovered[agent.start] = true;
            uncovered[agent.goal] = true;

            degree[agent.start] = degree[agent.start] + 1;
            degree[agent.goal] = degree[agent.goal] + 1;
        }
        for (int i=0; i<instance_.agents.size(); i++){
            auto agent = instance_.agents[i];
            if ( uncovered[agent.start] && uncovered[agent.goal] ) {
                if ( degree[agent.start] == 1 ) {

                    vertexCover.push_front(NodeTime{start.n, start.t});
                    uncovered[agent.start] = false;

                } else if ( degree[agent.goal] == 1 ) {

                    //never visited
                    vertexCover.push_front(NodeTime{goal, 0});
                    uncovered[agent.goal] = false;

                }
            }
        }
        for (int i=0; i<instance_.agents.size(); i++){
            auto agent = instance_.agents[i];
            if ( uncovered[agent.start] && uncovered[agent.goal] ) {

                //never used
                vertexCover.push_front(NodeTime{start.n, start.t});
                vertexCover.push_front(NodeTime{goal, 0});
                uncovered[agent.start] = false;
                uncovered[agent.goal] = false;
            }
        }

        std::cout << "Vertex Cover: ";
        for (NodeTime item : vertexCover) {
            fmt::print("{} ", item.n);
        }
        std::cout << '\n';

        Vector<Vector<Cost>*> outLengths (mapSize);
        Vector<Vector<NodeTime>*> outParents (mapSize);
        Vector<Cost> AllSegmentCosts (mapSize);


        //per source-node
        for (NodeTime sourceTuple : vertexCover) {

            Cost segment_cost = std::numeric_limits<int>::max();
            int time = 0;

            Node source = sourceTuple.n;
            Vector<Cost> distances1 (mapSize * goal_latest, std::numeric_limits<int>::max());
            Vector<NodeTime> parents1 (mapSize * goal_latest);

            int MAX_VALUE = max_cost;

            distances1[source] = sourceTuple.t;

            std::list<NodeTime> queue_cur;
            std::list<NodeTime> queue_new;
            std::list<NodeTime>* temp;

            queue_new.push_front(source);

            for ( int i = 0; i < mapSize; ++i ) {

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

                    //fmt::print("{} {} {} {}\n", edge_costs.north, edge_costs.east, edge_costs.south, edge_costs.west);

                    if (const auto next_n = map.get_north(v.n);
                            map[next_n] && !std::isnan(edge_costs.north)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.north, &segment_cost, goal, &time);
                    }

                    //south
                    if (const auto next_n = map.get_south(v.n);
                            map[next_n] && !std::isnan(edge_costs.south))  {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.south, &segment_cost, goal, &time);
                    }

                    //east
                    if (const auto next_n = map.get_east(v.n);
                            map[next_n] && !std::isnan(edge_costs.east)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};


                        updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.east, &segment_cost, goal, &time);
                    }

                    //west
                    if (const auto next_n = map.get_west(v.n);
                            map[next_n] && !std::isnan(edge_costs.west)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.west, &segment_cost, goal, &time);
                    }

                    //wait
                    if (const auto next_n = map.get_wait(v.n);
                            map[next_n] && !std::isnan(edge_costs.wait)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.wait, &segment_cost, goal, &time);
                    }

                }

            }

            outLengths[source] = new Vector<Cost>;
            outParents[source] = new Vector<NodeTime>;
            for (int i=0; i<distances1.size(); i++) {
                outLengths[source]->push_back(distances1[i]);
                outParents[source]->push_back(parents1[i]);
            }
            AllSegmentCosts[source] = time;

        }


        for (int i=0; i<instance_.agents.size(); i++){

            std::list<NodeTime> queue;
            Vector<NodeTime> segment (0);

            fmt::print("\nShortest distance from:\n");
            auto agent = instance_.agents[i];

            fmt::print("{}({},{}) : ", agent.start, agent.start_y, agent.start_x);
            fmt::print("to {}({},{}): ", agent.goal, agent.goal_y, agent.goal_x);

            if (outLengths[agent.goal] != NULL) {
                //not used
                fmt::print("{}\n", (*outLengths[agent.goal])[agent.start]);

            } else {
                fmt::print("{}\n", (*outLengths[agent.start])[AllSegmentCosts[agent.start] * mapSize + agent.goal]);

                //Find path
                Node n = agent.goal;
                queue.emplace_back(NodeTime{agent.goal, static_cast<Int>(AllSegmentCosts[agent.start])});
                NodeTime node = (*outParents[agent.start])[AllSegmentCosts[agent.start] * mapSize + n];
                queue.emplace_back(node);
                n = node.n;
                while ((*outParents[agent.start])[node.t * mapSize + n] != NULL){
                    node = (*outParents[agent.start])[node.t * mapSize + n];
                    queue.emplace_back(node);
                    n = node.n;
                }

                while (!queue.empty()) {
                    segment.push_back(queue.back());
                    queue.pop_back();
                }
            }


        }

        //return Pair<Vector<NodeTime>, Cost>(segment, segment_cost);
        //find way to return all Paths and the cost for all those paths
        return Pair<Vector<NodeTime>, Cost>(NULL, 0);
    }

    void
    BellmanFord::updateDistances(int d, int MAX_VALUE, Vector<Cost> *distances1, Vector<NodeTime> *parents1,
            NodeTime w, NodeTime v, std::list<NodeTime> *queue_new, Cost cost, Cost* segment_cost, Node goal, int* time) {
        bool changed = false;
        if (d < MAX_VALUE &&  ((*distances1)[w.t * mapSize + w.n] > d + cost)) {
            (*distances1)[w.t * mapSize + w.n] = d + cost;
            if (w.n == goal && d + cost < (*segment_cost)) {
                (*segment_cost) = d + cost;
                (*time) = w.t;
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
        mapSize = instance_.map.size();
        vertexCover = {};
        uncovered = Vector<bool>(mapSize);
        degree = Vector<Int>(mapSize);
    }


}
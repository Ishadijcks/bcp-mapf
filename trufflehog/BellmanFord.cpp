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

        //select signle agent (extra step for SSSP Bellman-Ford)
        int c;
        auto agents = instance_.agents;
        for (c=0; c<agents.size(); c++){
            if (agents[c].start == start.n && agents[c].goal == goal){
                break;
            }
        }

        //for (int i=0; i<instance_.agents.size(); i++){

            auto agent = instance_.agents[c];

            uncovered[agent.start] = true;
            uncovered[agent.goal] = true;

            degree[agent.start] = degree[agent.start] + 1;
            degree[agent.goal] = degree[agent.goal] + 1;
        //}
        //for (int i=0; i<instance_.agents.size(); i++){
            agent = instance_.agents[c];
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
        //}
        //for (int i=0; i<instance_.agents.size(); i++){
            agent = instance_.agents[c];
            if ( uncovered[agent.start] && uncovered[agent.goal] ) {

                //never used
                vertexCover.push_front(NodeTime{start.n, start.t});
                vertexCover.push_front(NodeTime{goal, 0});
                uncovered[agent.start] = false;
                uncovered[agent.goal] = false;
            }
        //}

        std::cout << "Vertex Cover: ";
        for (NodeTime item : vertexCover) {
            fmt::print("{} ", item.n);
        }
        std::cout << '\n';

        Vector<Vector<Cost>*> outLengths (mapSize);
        Vector<Vector<NodeTime>*> outParents (mapSize);

        //per source-node
        for (NodeTime sourceTuple : vertexCover) {

            Node source = sourceTuple.n;
            Vector<Cost> distances1 (mapSize, goal_latest);
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

                    Cost d = distances1[v.n];

                    fmt::print("{} {} {} {}\n", edge_costs.north, edge_costs.east, edge_costs.south, edge_costs.west);
                    bool edge_block = false;
                    if (edge_costs.north > 1 || edge_costs.east > 1 || edge_costs.south > 1 || edge_costs.west > 1){
                        edge_block = true;
                    }

                    bool move_stored = false;

                    if (const auto next_n = map.get_north(v.n);
                            map[next_n] && !std::isnan(edge_costs.north)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        move_stored = updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.north, move_stored, edge_block);
                    }

                    //south
                    if (const auto next_n = map.get_south(v.n);
                            map[next_n] && !std::isnan(edge_costs.south))  {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        move_stored = updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.south, move_stored, edge_block);
                    }

                    //east
                    if (const auto next_n = map.get_east(v.n);
                            map[next_n] && !std::isnan(edge_costs.east)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};


                        move_stored = updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.east, move_stored, edge_block);
                    }

                    //west
                    if (const auto next_n = map.get_west(v.n);
                            map[next_n] && !std::isnan(edge_costs.west)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        move_stored = updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.west, move_stored, edge_block);
                    }

                    //wait
                    if (const auto next_n = map.get_wait(v.n);
                            map[next_n] && !std::isnan(edge_costs.wait)) {
                        NodeTime w = NodeTime{next_n, v.t + 1};

                        updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.wait, move_stored, edge_block);
                    }

                }

            }

            outLengths[source] = new Vector<Cost>;
            outParents[source] = new Vector<NodeTime>;
            for (int i=0; i<distances1.size(); i++) {
                outLengths[source]->push_back(distances1[i]);
            }
            for (int i=0; i<parents1.size(); i++) {
                outParents[source]->push_back(parents1[i]);
            }

        }

        std::list<NodeTime> queue;
        Vector<NodeTime> segment (0);
        Cost segment_cost = 0;


        fmt::print("\nShortest distance from:\n");
            agent = instance_.agents[c];

            fmt::print("{}({},{}) : ", agent.start, agent.start_y, agent.start_x);
            fmt::print("to {}({},{}): ", agent.goal, agent.goal_y, agent.goal_x);

            if (outLengths[agent.goal] != NULL) {
                //not used
                fmt::print("{}\n", (*outLengths[agent.goal])[agent.start]);

            } else {
                fmt::print("{}\n", (*outLengths[agent.start])[agent.goal]);
                segment_cost = (*outLengths[agent.start])[agent.goal];

                //Find path
                Node n = agent.goal;
                queue.emplace_back(agent.goal, segment_cost);
                NodeTime node = (*outParents[agent.start])[segment_cost * mapSize + n];
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


        //}

        return Pair<Vector<NodeTime>, Cost>(segment, segment_cost);
    }

    bool
    BellmanFord::updateDistances(int d, int MAX_VALUE, Vector<Cost> *distances1, Vector<NodeTime> *parents1,
            NodeTime w, NodeTime v, std::list<NodeTime> *queue_new, Cost cost, bool move_stored, bool edge_block) {
        bool changed = false;
        if (d < MAX_VALUE &&  ((*distances1)[w.n] > d + cost /* the rest are edge-cost exceptions*/ || (w.n == v.n && edge_block /* && !move_stored */ ))) {
            (*distances1)[w.n] = d + cost;
            (*parents1)[w.t * mapSize + w.n] = v;
            changed = true;
        }

        if (changed) {
            bool found = (std::find(queue_new->begin(), queue_new->end(), w) != queue_new->end());
            if (!found) {
                queue_new->push_front(w);
            }
            return true;
        }
        return false;

    }


    void BellmanFord::reset() {
        mapSize = instance_.map.size();
        vertexCover = {};
        uncovered = Vector<bool>(mapSize);
        degree = Vector<Int>(mapSize);
    }


}
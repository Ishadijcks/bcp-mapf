/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include "BellmanFord.h"
#include "Instance.h"

namespace TruffleHog {



    BellmanFord::BellmanFord(const Instance &instance) : instance_(instance) {
        instance.map.printPassable();
        instance.map.printNodes();

        this->reset();
    }

    Pair<Vector<NodeTime>, Cost>
    BellmanFord::solve(NodeTime start, Node ggoal, Time goal_earliest, Time goal_latest, Cost max_cost) {

        auto map = instance_.map;
        int mapSize = map.size();

        //extra
        int extMapSize = mapSize * goal_latest;
        NodeTime goal;

        std::list<std::tuple<NodeTime, bool>> vertexCover = {};
        //std::list<Node> vertexCover = { };

        Vector<bool> uncovered (extMapSize);
        Vector<Int> degree (extMapSize);
        //Vector<bool> uncovered (mapSize);
        //Vector<Int> degree (mapSize);

        for (int i=0; i<instance_.agents.size(); i++){
            auto agent = instance_.agents[i];

            start = NodeTime{agent.start, 0};
            //TODO check this one again
            goal = NodeTime{agent.goal, goal_latest};
            uncovered[agent.start] = true;
            uncovered[agent.goal] = true;
            //uncovered[agent.start] = true;
            //uncovered[agent.goal] = true;


            degree[agent.start] = degree[agent.start] + 1;
            degree[agent.goal] = degree[agent.goal] + 1;
        }
        for (int i=0; i<instance_.agents.size(); i++){
            auto agent = instance_.agents[i];
            if ( uncovered[agent.start] && uncovered[agent.goal] ) {
                if ( degree[agent.start] == 1 ) {

                    vertexCover.push_front(std::tuple<NodeTime, bool>{goal, true});
                    //vertexCover.push_front(agent.goal);

                    uncovered[agent.goal] = false;
                } else if ( degree[agent.goal] == 1 ) {

                    vertexCover.push_front(std::tuple<NodeTime, bool>{start, false});
                    //vertexCover.push_front(agent.start);

                    uncovered[agent.start] = false;
                }
            }
        }
        for (int i=0; i<instance_.agents.size(); i++){
            auto agent = instance_.agents[i];
            if ( uncovered[agent.start] && uncovered[agent.goal] ) {

                vertexCover.push_front(std::tuple<NodeTime, bool>{goal, true});
                vertexCover.push_front(std::tuple<NodeTime, bool>{start, false});
                //vertexCover.push_front(agent.goal);
                //vertexCover.push_front(agent.start);

                uncovered[agent.start] = false;
                uncovered[agent.goal] = false;
            }
        }

        std::cout << "Vertex Cover: ";
        for (std::tuple<NodeTime, bool> item : vertexCover) {
        //for (Node n : vertexCover) {
            fmt::print("{} ", std::get<0>(item).n);
        }
        std::cout << '\n';

        Vector<Vector<Cost>*> outLengths (extMapSize);
        Vector<Vector<NodeTime>*> outParents (extMapSize);
        //Vector<Vector<Int>*> outLengths (mapSize);

        //per source-node

        for (std::tuple<NodeTime, bool> sourceTuple : vertexCover){
        //for (Node source : vertexCover) {

            Node source = std::get<0>(sourceTuple).n;
            bool isGoal = std::get<1>(sourceTuple);
            Vector<Cost> distances1 (extMapSize, std::numeric_limits<Int>::max());
            Vector<NodeTime> parents1 (extMapSize);
            //Vector<Int> distances1 (mapSize, std::numeric_limits<Int>::max());

            //TODO check again
            int MAX_VALUE = max_cost;

            distances1[source] = 0;
            //distances1[source] = 0;

            std::list<NodeTime> queue_cur;
            std::list<NodeTime> queue_new;
            std::list<NodeTime>* temp;
            double default_cost = 1;
            //std::list<Node> queue_cur;
            //std::list<Node> queue_new;
            //std::list<Node>* temp;

            queue_new.push_front(source);

            for ( int i = 0; i < extMapSize; ++i ) {
            //for ( int i = 0; i < mapSize; ++i ) {
                if (queue_new.empty()) break;

                temp = &queue_cur;
                queue_cur = queue_new;
                queue_new = *temp;
                while (!queue_new.empty())
                    queue_new.pop_back();

                while (!queue_cur.empty()) {
                    NodeTime v = queue_cur.back();
                    //Node v = queue_cur.back();

                    queue_cur.pop_back();

                    //TODO check
                    const auto edge_costs = edge_penalties_.get_edge_costs<1>(v);

                    Cost d = distances1[v.n];
                    //int d  = distances1[v];

                    if((v.t + 1 <= goal_latest && !isGoal) || (v.t - 1 >= 0 && isGoal)) {
                        //north
                        if (const auto next_n = map.get_north(v.n);
                                map[next_n] && !std::isnan(edge_costs.north))
                        {
                            NodeTime w;
                            if(isGoal) {
                                w = NodeTime{next_n, v.t - 1};
                            } else {
                                w = NodeTime{next_n, v.t + 1};
                            }

                            updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.north);
                        }

                        //south
                        if (const auto next_n = map.get_south(v.n);
                                map[next_n] && !std::isnan(edge_costs.south))
                        {
                            NodeTime w;
                            if(isGoal) {
                                w = NodeTime{next_n, v.t - 1};
                            } else {
                                w = NodeTime{next_n, v.t + 1};
                            }
                            updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.south);
                        }

                        //east
                        if (const auto next_n = map.get_east(v.n);
                                map[next_n] && !std::isnan(edge_costs.east))
                        {
                            NodeTime w;
                            if(isGoal) {
                                w = NodeTime{next_n, v.t - 1};
                            } else {
                                w = NodeTime{next_n, v.t + 1};
                            }
                            updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.east);
                        }

                        //west
                        if (const auto next_n = map.get_west(v.n);
                                map[next_n] && !std::isnan(edge_costs.west))
                        {
                            NodeTime w;
                            if(isGoal) {
                                w = NodeTime{next_n, v.t - 1};
                            } else {
                                w = NodeTime{next_n, v.t + 1};
                            }
                            updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.west);
                        }

                        //wait
                        if (const auto next_n = map.get_wait(v.n);
                                map[next_n] && !std::isnan(edge_costs.wait))
                        {
                            NodeTime w;
                            if(isGoal) {
                                w = NodeTime{next_n, v.t - 1};
                            } else {
                                w = NodeTime{next_n, v.t + 1};
                            }
                            updateDistances(d, MAX_VALUE, &distances1, &parents1, w, v, &queue_new, edge_costs.wait);
                        }
                    }





                    /*
                    //south
                    Node w = map.get_south(v);
                    if (map.is_passable(w))
                        updateDistances(d, MAX_VALUE, &distances1, w, &queue_new);
                    //north
                    w = map.get_north(v);
                    if (map.is_passable(w))
                        updateDistances(d, MAX_VALUE, &distances1, w, &queue_new);
                    //east
                    w = map.get_east(v);
                    if (map.is_passable(w))
                        updateDistances(d, MAX_VALUE, &distances1, w, &queue_new);
                    //west
                    w = map.get_west(v);
                    if (map.is_passable(w))
                        updateDistances(d, MAX_VALUE, &distances1, w, &queue_new);
                    */

                }

            }

            outLengths[source] = new Vector<Cost>;
            outParents[source] = new Vector<NodeTime>;
            //outLengths[source] = new Vector<Int>;
            for (int i=0; i<distances1.size(); i++) {
                outLengths[source]->push_back(distances1[i]);

                //TODO check
                outParents[source]->push_back(parents1[i]);
            }

        }

        fmt::print("\nShortest distance from:\n");
        for (int i=0; i<instance_.agents.size(); i++) {
            auto agent = instance_.agents[i];

            fmt::print("{}({},{}) : ", agent.start, agent.start_y, agent.start_x);
            fmt::print("to {}({},{}): ", agent.goal, agent.goal_y, agent.goal_x);

            if (outLengths[agent.goal] != NULL) {
                fmt::print("{}\n", (*outLengths[agent.goal])[agent.start]);

            } else {
                fmt::print("{}\n", (*outLengths[agent.start])[agent.goal]);
            }
        }

        return Pair<Vector<NodeTime>, Cost>({NodeTime(1, 2), NodeTime(3, 2)}, 3);
    }

    void
    BellmanFord::updateDistances(int d, int MAX_VALUE, Vector<Cost> *distances1, Vector<NodeTime> *parents1,
            NodeTime w, NodeTime v, std::list<NodeTime> *queue_new, Cost cost) {
        bool changed = false;
        if (d < MAX_VALUE && (*distances1)[w.n] > d + cost) {
            (*distances1)[w.n] = d + cost;
            (*parents1)[w.n] = v;
            changed = true;
        }

        if (changed) {
            bool found = (std::find(queue_new->begin(), queue_new->end(), w) != queue_new->end());
            if (!found) {
                queue_new->push_front(w);
            }
        }

    }

    /*
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
     */

    void BellmanFord::reset() {
        mapSize = instance_.map.size();
        vertexCover = {};
        uncovered = Vector<bool>(mapSize);
        degree = Vector<Int>(mapSize);
    }


}
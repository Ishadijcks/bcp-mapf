/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include "BellmanFord.h"
#include "Instance.h"
#include "bcp/Includes.h"
#include "AStar.h"
#include <regex>

int main(int argc, char **argv) {
    // Get instance name.
    const char* scenario_path = "../instances/custom/small-corridor-5x5-2-agents.scen";    // File path to scenario
    const Agent nb_agents = 2;

    String instance_name("mapf");
    {
        const String str(scenario_path);
        std::smatch m;
        if (std::regex_match(str, m, std::regex(".*\\/(.+)\\.map.scen")))
        {
            instance_name = m.str(1);
        }
        else if (std::regex_match(str, m, std::regex(".*\\/(.+)\\.scen")))
        {
            instance_name = m.str(1);
        }
    }
    if (nb_agents < std::numeric_limits<Agent>::max())
    {
        instance_name += fmt::format("-{}agents", nb_agents);
    }

    // Load instance.
    auto instance = std::make_shared<Instance>(scenario_path, nb_agents);

    // Create pricing solver.
    SharedPtr<AbstractPathfinder> bellmanFord = std::make_shared<BellmanFord>(instance->map);


    bellmanFord->solve(1, 2);
    return 0;
}
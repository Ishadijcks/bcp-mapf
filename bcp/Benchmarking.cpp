/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include <utility>

#include "Solve.h"

std::string &replace(std::string &s, const std::string &from, const std::string &to) {
    if (!from.empty())
        for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += to.size())
            s.replace(pos, from.size(), to);
    return s;
}

std::string outputPath = "build-release/outputs/";
std::string instancePath = "../instances/";

struct Benchmark {
    Benchmark(std::string folder, std::string name) : folder(std::move(folder)), name(std::move(name)), maxAgents(-1),
                                                      timeLimit(-1) {}

    Benchmark(std::string folder, std::string name, int timeLimit) : folder(std::move(folder)), name(std::move(name)),
                                                                     maxAgents(-1), timeLimit(timeLimit) {}

    Benchmark(std::string folder, std::string name, int timeLimit, int maxAgents) : folder(std::move(folder)),
                                                                                    name(std::move(name)),
                                                                                    maxAgents(maxAgents),
                                                                                    timeLimit(timeLimit) {}

    std::string folder;
    std::string name;
    int maxAgents;
    int timeLimit;

    std::vector<char*> argv;
    std::vector<std::string> arguments;


    std::string getInstancePath() {
        return instancePath + folder + "/" + name;
    }

    std::string getOutputPath() {
        return outputPath + replace(name, ".scen", ".sol");
    }

    std::vector<char *> getArgs(char* workingDirectory) {

        if(argv.size() != 0) {
            return argv;
        }

        if (maxAgents > -1) {
            arguments.push_back("--agents-limit=" + std::to_string(maxAgents));
        }
        if (timeLimit > -1) {
            arguments.push_back("--time-limit=" + std::to_string(timeLimit));
        }
        arguments.push_back(getInstancePath());

        argv.push_back(workingDirectory);

        for (const auto& arg : arguments)
            argv.push_back((char*)arg.data());
        argv.push_back(nullptr);

        return argv;
    }
};

int main(int argc, char **argv) {
    Benchmark benchmarks[] = {
            Benchmark("custom", "small-corridor-5x5-2-agents.scen")
    };
    auto args = benchmarks[0].getArgs(argv[0]);

    const SCIP_RETCODE retcode1 = start_solver(args.size() - 1, args.data());
    if (retcode1 != SCIP_OKAY)
    {
        SCIPprintError(retcode1);
        return -1;
    }
    return 0;
}

int getLastResult(Benchmark benchmark) {
    return 0;
}
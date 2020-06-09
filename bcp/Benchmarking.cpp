/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include <utility>
#include <fstream>

#include "Solve.h"

std::string &replace(std::string &s, const std::string &from, const std::string &to) {
    if (!from.empty())
        for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += to.size())
            s.replace(pos, from.size(), to);
    return s;
}

std::string outputPath = "outputs/";
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

    std::vector<char *> getArgs() {
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

        for (const auto& arg : arguments)
            argv.push_back((char*)arg.data());
        argv.push_back(nullptr);

        return argv;
    }
};

int getLastResult(Benchmark benchmark) {
    const std::string &path = benchmark.getOutputPath();
    std::ifstream infile(path);

    int output = -1;
    if (infile.good())
    {
        std::string sLine;
        getline(infile, sLine);
        output = std::stoi(sLine);
        std::cout << output << std::endl;
    } else {
        throw std::runtime_error("Could not retrieve results for benchmark " + path);
    }

    infile.close();
    return output;
}

bool runBenchMark(char **argv, Benchmark benchmark) {
    auto args = benchmark.getArgs();
    args.insert(args.begin(), argv[0]);
    const SCIP_RETCODE retcode1 = start_solver(args.size() - 1, args.data());
    if (retcode1 != SCIP_OKAY)
    {
        SCIPprintError(retcode1);
        return false;
    }

    getLastResult(benchmark);
    return true;
}

int main(int argc, char **argv) {
    Benchmark benchmarks[] = {
            Benchmark("custom", "small-corridor-5x5-2-agents.scen")
    };

    runBenchMark(argv, benchmarks[0]);
    return 0;
}


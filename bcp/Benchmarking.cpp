/*
Author: Isha Dijcks <i.e.dijcks@student.tudelft.nl>
*/

#include <utility>
#include <fstream>
#include <chrono>
#include <ctime>

#include "Solve.h"

// Helper method to replace substrings within strings
std::string &replace(std::string &s, const std::string &from, const std::string &to) {
    if (!from.empty())
        for (size_t pos = 0; (pos = s.find(from, pos)) != std::string::npos; pos += to.size())
            s.replace(pos, from.size(), to);
    return s;
}

const std::string outputPath = "outputs/";
const std::string instancePath = "../instances/";

// Data class to easily add new benchmarks
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

    std::vector<char *> argv;
    std::vector<std::string> arguments;

    std::string getDescription() {
        std::string res;
        res += name;
        if (timeLimit > -1) {
            res += " (" + std::to_string(timeLimit) + "s)";
        }
        if (maxAgents > -1) {
            res += " (" + std::to_string(maxAgents) + "a)";
        }
        return res;
    }

    std::string getInstancePath() {
        return instancePath + folder + "/" + name;
    }

    std::string getOutputPath() {
        std::string agentPart = maxAgents > -1 ? "-" + std::to_string(maxAgents) + "agents" : "";
        return outputPath + replace(name, ".scen", "") + agentPart + ".sol";
    }

    std::vector<char *> getArgs() {
        if (argv.size() != 0) {
            return argv;
        }

        arguments.push_back(getInstancePath());

        if (maxAgents > -1) {
            arguments.push_back("--agents-limit=" + std::to_string(maxAgents));
        }
        if (timeLimit > -1) {
            arguments.push_back("--time-limit=" + std::to_string(timeLimit));
        }

        for (const auto &arg : arguments)
            argv.push_back((char *) arg.data());
        argv.push_back(nullptr);

        return argv;
    }
};

// Read the written output file to retrieve the found optimum
int getLastResult(Benchmark benchmark) {
    const std::string &path = benchmark.getOutputPath();
    std::ifstream infile(path);

    int output = -1;
    if (infile.good()) {
        std::string sLine;
        getline(infile, sLine);
        std::cout << sLine << std::endl;

        try {
            output = std::stoi(sLine);
        } catch (const std::invalid_argument &e) {
            std::cout << e.what() << std::endl;
            output = -1;
        }
        std::cout << output << std::endl;
    } else {
        throw std::runtime_error("Could not retrieve results for benchmark " + path);
    }

    infile.close();
    return output;
}

// Runs a single benchmark and returns the optimal solution value
int runBenchMark(char **argv, Benchmark benchmark) {
    auto args = benchmark.getArgs();
    args.insert(args.begin(), argv[0]);
    const SCIP_RETCODE retcode1 = start_solver(args.size() - 1, args.data());
    if (retcode1 != SCIP_OKAY) {
        SCIPprintError(retcode1);
        return -1;
    }

    int solution = getLastResult(benchmark);

    return solution;
}

// Run the provided benchmarks and report the results
int main(int argc, char **argv) {
    Benchmark benchmarks[] = {
            Benchmark("custom", "small-corridor-5x5-2-agents.scen"),
            Benchmark("custom", "small-corridor-5x7-3-agents.scen"),
            //Benchmark("movingai_2019", "random-32-32-10-even-1.scen", 60),
            //Benchmark("movingai_2018/20x20map", "20x20map-60agents-49.scen", 5),
            //Benchmark("movingai_2018/20x20map", "20x20map-60agents-49.scen", 5, 2),
            //Benchmark("movingai_2018/20x20map", "20x20map-60agents-49.scen", 10),
            //Benchmark("movingai_2018/20x20map", "20x20map-60agents-49.scen", 20),
    };



    auto start = std::chrono::system_clock::now();
    // Some computation here

    int benchmarkCount = (sizeof(benchmarks) / sizeof(*benchmarks));
    int results[benchmarkCount];

    for (int i = 0; i < benchmarkCount; ++i) {
        results[i] = runBenchMark(argv, benchmarks[i]);
    }

    std::cout << "------BENCHMARK RESULTS------" << std::endl;
    for (int i = 0; i < benchmarkCount; ++i) {
        std::cout << benchmarks[i].getDescription() << ": " << results[i] << std::endl;
    }

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "finished computation at " << std::ctime(&end_time)
              << "elapsed time: " << elapsed_seconds.count() << "s\n";


    return 0;
}


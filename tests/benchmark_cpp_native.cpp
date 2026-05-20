#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "mppi/MPPIController.hpp"
#include "mppi/PendulumDynamics.hpp"
#include "mppi/PendulumCost.hpp"

using namespace mppi;

struct ResultRow {
    int episode_steps;
    double total_command_time;
    double average_command_time;
    double final_theta;
    double final_theta_dot;
    double final_cost;
};

std::filesystem::path benchmarkOutputPath() {
    const std::filesystem::path fromRepoRoot =
        "mppi_comparison/cpu_based/data/cpp_native_cpu_sweep.csv";
    if (std::filesystem::exists(fromRepoRoot.parent_path())) {
        return fromRepoRoot;
    }

    return "../mppi_comparison/cpu_based/data/cpp_native_cpu_sweep.csv";
}

int main() {
    std::vector<int> episode_lengths = {40, 80, 120, 160, 200};
    std::vector<ResultRow> results;
    results.reserve(episode_lengths.size());

    for (int episode_steps : episode_lengths) {
        Config cfg;
        cfg.nx = 2;
        cfg.nu = 1;
        cfg.K = 200;
        cfg.T = 15;
        cfg.lambda = 1.0;
        cfg.uScale = 1.0;
        cfg.uPerCommand = 1;

        cfg.uMin = Vector::Constant(1, -2.0);
        cfg.uMax = Vector::Constant(1, 2.0);
        cfg.uInit = Vector::Zero(1);

        cfg.noiseMu = Vector::Zero(1);
        cfg.noiseSigmaDiag = Vector::Constant(1, 10.0);

        PendulumDynamics dynamics;
        PendulumCost cost;
        MPPIController controller(cfg, dynamics, cost);

        Vector state(2);
        state << M_PI, 1.0;

        double total_command_time = 0.0;
        double final_cost = 0.0;

        for (int step = 0; step < episode_steps; ++step) {
            auto start = std::chrono::high_resolution_clock::now();

            Vector action = controller.command(state);

            state = dynamics.propagate(state, action, step);
            final_cost = cost.evaluate(state, action, step);

            auto end = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> elapsed = end - start;
            total_command_time += elapsed.count();
        }

        double average_command_time = total_command_time / episode_steps;

        results.push_back({
            episode_steps,
            total_command_time,
            average_command_time,
            state(0),
            state(1),
            final_cost
        });

        std::cout << "Steps: " << episode_steps
                  << " | total: " << total_command_time
                  << " | avg: " << average_command_time
                  << " | final cost: " << final_cost
                  << "\n";
    }

    const auto outputPath = benchmarkOutputPath();
    std::ofstream file(outputPath);

    if (!file.is_open()) {
        std::cerr << "Could not open output CSV file.\n";
        return 1;
    }

    file << "episode_steps,total_command_time,average_command_time,"
         << "final_theta,final_theta_dot,final_cost\n";

    for (const auto& row : results) {
        file << row.episode_steps << ","
             << row.total_command_time << ","
             << row.average_command_time << ","
             << row.final_theta << ","
             << row.final_theta_dot << ","
             << row.final_cost << "\n";
    }

    file.close();

    std::cout << "Saved " << outputPath << "\n";

    return 0;
}

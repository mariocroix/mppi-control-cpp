#include "mppi/cuda/CUDAPendulumMPPI.hpp"

#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <thrust/device_ptr.h>
#include <thrust/extrema.h>
#include <thrust/reduce.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace mppi {

static void checkCuda(cudaError_t err, const char* msg) {
    if (err != cudaSuccess) {
        throw std::runtime_error(
            std::string(msg) + ": " + cudaGetErrorString(err)
        );
    }
}

__device__ double clampDevice(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

__device__ double angleNormalizeDevice(double x) {
    const double pi = 3.14159265358979323846;
    while (x > pi) x -= 2.0 * pi;
    while (x < -pi) x += 2.0 * pi;
    return x;
}

__global__ void setupRngKernel(curandState* states, int K, unsigned long seed) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k < K) {
        curand_init(seed, k, 0, &states[k]);
    }
}

__global__ void shiftKernel(double* U, int T) {
    for (int t = 0; t < T - 1; ++t) {
        U[t] = U[t + 1];
    }
    U[T - 1] = 0.0;
}

__global__ void rolloutKernel(
    double* U,
    double* noise,
    double* costs,
    curandState* rngStates,
    int K,
    int T,
    double lambda,
    double noiseStd,
    double theta0,
    double thetaDot0
) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k >= K) return;

    curandState localState = rngStates[k];

    double theta = theta0;
    double thetaDot = thetaDot0;
    double totalCost = 0.0;

    const double g = 10.0;
    const double m = 1.0;
    const double l = 1.0;
    const double dt = 0.05;

    for (int t = 0; t < T; ++t) {
        double eps = noiseStd * curand_normal_double(&localState);
        double u = clampDevice(U[t] + eps, -2.0, 2.0);

        noise[k * T + t] = eps;

        double newThetaDot =
            thetaDot +
            (
                3.0 * g / (2.0 * l) * sin(theta)
                + 3.0 / (m * l * l) * u
            ) * dt;

        newThetaDot = clampDevice(newThetaDot, -8.0, 8.0);

        double newTheta = theta + newThetaDot * dt;

        theta = newTheta;
        thetaDot = newThetaDot;

        double angle = angleNormalizeDevice(theta);

        // Match original Python cost:
        // angle^2 + 0.1 * theta_dot^2
        totalCost += angle * angle + 0.1 * thetaDot * thetaDot;
    }

    costs[k] = totalCost;
    rngStates[k] = localState;
}

__global__ void computeWeightsKernel(
    const double* costs,
    double* weights,
    int K,
    double beta,
    double lambda
) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k < K) {
        weights[k] = exp(-(costs[k] - beta) / lambda);
    }
}

__global__ void normalizeWeightsKernel(double* weights, int K, double eta) {
    int k = blockIdx.x * blockDim.x + threadIdx.x;
    if (k < K) {
        weights[k] /= eta;
    }
}

__global__ void updateUKernel(
    double* U,
    const double* noise,
    const double* weights,
    int K,
    int T
) {
    int t = blockIdx.x * blockDim.x + threadIdx.x;
    if (t >= T) return;

    double delta = 0.0;

    for (int k = 0; k < K; ++k) {
        delta += weights[k] * noise[k * T + t];
    }

    U[t] = clampDevice(U[t] + delta, -2.0, 2.0);
}

CUDAPendulumMPPI::CUDAPendulumMPPI(const Config& config)
    : cfg_(config),
      UHost_(Matrix::Zero(config.T, config.nu)) {
    allocateDevice();
    copyUToDevice();

    int block = 256;
    int grid = (cfg_.K + block - 1) / block;

    setupRngKernel<<<grid, block>>>(
        static_cast<curandState*>(d_rngStates_),
        cfg_.K,
        40
    );

    checkCuda(cudaGetLastError(), "setup rng launch");
    checkCuda(cudaDeviceSynchronize(), "setup rng sync");
}

CUDAPendulumMPPI::~CUDAPendulumMPPI() {
    freeDevice();
}

void CUDAPendulumMPPI::allocateDevice() {
    checkCuda(cudaMalloc(&d_U_, sizeof(double) * cfg_.T), "cudaMalloc U");
    checkCuda(cudaMalloc(&d_noise_, sizeof(double) * cfg_.K * cfg_.T), "cudaMalloc noise");
    checkCuda(cudaMalloc(&d_costs_, sizeof(double) * cfg_.K), "cudaMalloc costs");
    checkCuda(cudaMalloc(&d_weights_, sizeof(double) * cfg_.K), "cudaMalloc weights");
    checkCuda(cudaMalloc(&d_rngStates_, sizeof(curandState) * cfg_.K), "cudaMalloc rng states");
}

void CUDAPendulumMPPI::freeDevice() {
    if (d_U_) cudaFree(d_U_);
    if (d_noise_) cudaFree(d_noise_);
    if (d_costs_) cudaFree(d_costs_);
    if (d_weights_) cudaFree(d_weights_);
    if (d_rngStates_) cudaFree(d_rngStates_);

    d_U_ = nullptr;
    d_noise_ = nullptr;
    d_costs_ = nullptr;
    d_weights_ = nullptr;
    d_rngStates_ = nullptr;
}

void CUDAPendulumMPPI::copyUToDevice() {
    checkCuda(
        cudaMemcpy(
            d_U_,
            UHost_.data(),
            sizeof(double) * cfg_.T,
            cudaMemcpyHostToDevice
        ),
        "copy U to device"
    );
}

void CUDAPendulumMPPI::copyUToHost() {
    checkCuda(
        cudaMemcpy(
            UHost_.data(),
            d_U_,
            sizeof(double) * cfg_.T,
            cudaMemcpyDeviceToHost
        ),
        "copy U to host"
    );
}

Vector CUDAPendulumMPPI::command(const Vector& state) {
    double theta0 = state(0);
    double thetaDot0 = state(1);

    int block = 256;
    int gridK = (cfg_.K + block - 1) / block;
    int gridT = (cfg_.T + block - 1) / block;

    shiftKernel<<<1, 1>>>(d_U_, cfg_.T);
    checkCuda(cudaGetLastError(), "shift kernel");

    double noiseStd = std::sqrt(cfg_.noiseSigmaDiag(0));

    rolloutKernel<<<gridK, block>>>(
        d_U_,
        d_noise_,
        d_costs_,
        static_cast<curandState*>(d_rngStates_),
        cfg_.K,
        cfg_.T,
        cfg_.lambda,
        noiseStd,
        theta0,
        thetaDot0
    );

    checkCuda(cudaGetLastError(), "rollout kernel");
    checkCuda(cudaDeviceSynchronize(), "rollout sync");

    thrust::device_ptr<double> costsPtr(d_costs_);
    thrust::device_ptr<double> weightsPtr(d_weights_);

    double beta = *thrust::min_element(costsPtr, costsPtr + cfg_.K);

    computeWeightsKernel<<<gridK, block>>>(
        d_costs_,
        d_weights_,
        cfg_.K,
        beta,
        cfg_.lambda
    );

    checkCuda(cudaGetLastError(), "compute weights kernel");
    checkCuda(cudaDeviceSynchronize(), "compute weights sync");

    double eta = thrust::reduce(weightsPtr, weightsPtr + cfg_.K, 0.0, thrust::plus<double>());

    normalizeWeightsKernel<<<gridK, block>>>(
        d_weights_,
        cfg_.K,
        eta
    );

    checkCuda(cudaGetLastError(), "normalize weights kernel");
    checkCuda(cudaDeviceSynchronize(), "normalize weights sync");

    updateUKernel<<<gridT, block>>>(
        d_U_,
        d_noise_,
        d_weights_,
        cfg_.K,
        cfg_.T
    );

    checkCuda(cudaGetLastError(), "update U kernel");
    checkCuda(cudaDeviceSynchronize(), "update U sync");

    copyUToHost();

    Vector action(1);
    action(0) = UHost_(0, 0);

    return action;
}

const Matrix& CUDAPendulumMPPI::getActionSequence() const {
    return UHost_;
}

void CUDAPendulumMPPI::reset() {
    UHost_ = Matrix::Zero(cfg_.T, cfg_.nu);
    copyUToDevice();
}

}  // namespace mppi
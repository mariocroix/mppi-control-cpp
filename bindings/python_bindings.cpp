#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "mppi/Config.hpp"
#include "mppi/MPPIController.hpp"
#include "mppi/SMPPIController.hpp"

#include "mppi/LinearDynamics.hpp"
#include "mppi/PendulumDynamics.hpp"

#include "mppi/QuadraticCost.hpp"
#include "mppi/PendulumCost.hpp"
#include "mppi/PendulumTerminalCost.hpp"

namespace py = pybind11;
using namespace mppi;

PYBIND11_MODULE(mppi_cpp, m) {
    m.doc() = "C++ MPPI and SMPPI Python bindings";

    // =========================================================
    // Config
    // =========================================================

    py::class_<Config>(m, "Config")
        .def(py::init<>())

        .def_readwrite("nx", &Config::nx)
        .def_readwrite("nu", &Config::nu)

        .def_readwrite("K", &Config::K)
        .def_readwrite("T", &Config::T)

        .def_readwrite("lambda_", &Config::lambda)
        .def_readwrite("uScale", &Config::uScale)
        .def_readwrite("uPerCommand", &Config::uPerCommand)

        .def_readwrite("uMin", &Config::uMin)
        .def_readwrite("uMax", &Config::uMax)
        .def_readwrite("uInit", &Config::uInit)

        .def_readwrite("noiseMu", &Config::noiseMu)
        .def_readwrite("noiseSigmaDiag", &Config::noiseSigmaDiag);

    // =========================================================
    // Base interfaces
    // =========================================================

    py::class_<Dynamics>(m, "Dynamics");

    py::class_<RunningCost>(m, "RunningCost");

    py::class_<TerminalCost>(m, "TerminalCost");

    // =========================================================
    // Dynamics
    // =========================================================

    py::class_<LinearDynamics, Dynamics>(m, "LinearDynamics")
        .def(py::init<const Matrix&>())

        .def(
            "propagate",
            &LinearDynamics::propagate,
            py::arg("state"),
            py::arg("action"),
            py::arg("t")
        );

    py::class_<PendulumDynamics, Dynamics>(m, "PendulumDynamics")
        .def(py::init<>())

        .def(
            "propagate",
            &PendulumDynamics::propagate,
            py::arg("state"),
            py::arg("action"),
            py::arg("t")
        );

    // =========================================================
    // Costs
    // =========================================================

    py::class_<QuadraticCost, RunningCost>(m, "QuadraticCost")
        .def(py::init<const Vector&>())

        .def(
            "evaluate",
            &QuadraticCost::evaluate,
            py::arg("state"),
            py::arg("action"),
            py::arg("t")
        );

    py::class_<PendulumCost, RunningCost>(m, "PendulumCost")
        .def(py::init<>())

        .def(
            "evaluate",
            &PendulumCost::evaluate,
            py::arg("state"),
            py::arg("action"),
            py::arg("t")
        );

    py::class_<PendulumTerminalCost, TerminalCost>(
        m,
        "PendulumTerminalCost"
    )
        .def(py::init<>())

        .def(
            "evaluate",
            &PendulumTerminalCost::evaluate,
            py::arg("state")
        );

    // =========================================================
    // MPPI Controller
    // =========================================================

    py::class_<MPPIController>(m, "MPPIController")
        .def(
            py::init<
                const Config&,
                const Dynamics&,
                const RunningCost&,
                const TerminalCost*
            >(),

            py::arg("config"),
            py::arg("dynamics"),
            py::arg("running_cost"),
            py::arg("terminal_cost") = nullptr
        )

        .def(
            "command",
            &MPPIController::command,
            py::arg("state")
        )

        .def("reset", &MPPIController::reset)

        .def(
            "get_action_sequence",
            &MPPIController::getActionSequence
        )

        .def(
            "shift_nominal_trajectory",
            &MPPIController::shiftNominalTrajectory
        )

        .def(
            "change_horizon",
            &MPPIController::changeHorizon,
            py::arg("new_horizon")
        )

        .def(
            "get_params",
            &MPPIController::getParams
        );

    // =========================================================
    // SMPPI Controller
    // =========================================================

    py::class_<SMPPIController, MPPIController>(
        m,
        "SMPPIController"
    )
        .def(
            py::init<
                const Config&,
                const Dynamics&,
                const RunningCost&,
                const TerminalCost*,
                double,
                double
            >(),

            py::arg("config"),
            py::arg("dynamics"),
            py::arg("running_cost"),
            py::arg("terminal_cost") = nullptr,
            py::arg("wActionSeqCost") = 1.0,
            py::arg("deltaT") = 1.0
        )

        .def(
            "command",
            &SMPPIController::command,
            py::arg("state")
        )

        .def("reset", &SMPPIController::reset)

        .def(
            "get_action_sequence",
            &SMPPIController::getActionSequence
        );
}
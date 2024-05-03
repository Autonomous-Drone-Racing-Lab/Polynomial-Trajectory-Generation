#include <pybind11/pybind11.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <pybind11/numpy.h>
#include <Eigen/Core>

namespace py = pybind11;

py::array_t<double> generateTrajectory(py::array_t<double> waypoints, double v_max, double a_max, double sampling_intervall){
    // config
    int dimension = 3;
    int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    const int N = 10; // polynomial order





    py::buffer_info buf = waypoints.request();
    double *point_ptr = (double *) buf.ptr;
    int n = buf.shape[0];
    int m = buf.shape[1];
    // assert that the second axis is of size 3
    if (m != 3){
        throw std::runtime_error("Input should have shape (n, 3)");
    }

    // assert that the number of waypoints is at least 2
    if (n < 2){
        throw std::runtime_error("At least two waypoints are required");
    }

    std::vector<Eigen::Vector3d> points;
    for (int i = 0; i < n; i++){
        points.emplace_back(point_ptr[i*m], point_ptr[i*m + 1], point_ptr[i*m + 2]);
    }

    // Create a vector of vertices
    mav_trajectory_generation::Vertex::Vector vertices;

    // Handle the first vertex
    mav_trajectory_generation::Vertex start(dimension);
    start.makeStartOrEnd(points.front(), derivative_to_optimize);
    vertices.push_back(start);

    // Handle the middle vertices
    for (size_t i = 1; i < points.size() - 1; ++i) {
        mav_trajectory_generation::Vertex vertex(dimension);
        vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, points[i]);
        vertices.push_back(vertex);
    }

    // Handle the last vertex
    mav_trajectory_generation::Vertex end(dimension);
    end.makeStartOrEnd(points.back(), derivative_to_optimize);
    vertices.push_back(end);


    // Create an optimizer object and solve
    std::vector<double> segment_times = estimateSegmentTimes(vertices, v_max, a_max);
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.solveLinear();

    // Convert to trajectory
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    // Sample the trajectory
    double min_time = trajectory.getMinTime();
    double max_time = trajectory.getMaxTime();
    
    // sample position
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    std::vector<Eigen::VectorXd> position_samples;
    std::vector<double> sampling_times;
    trajectory.evaluateRange(min_time, max_time, sampling_intervall, derivative_order, &position_samples, &sampling_times);

    // sample velocity
    derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
    std::vector<Eigen::VectorXd> velocity_samples;
    trajectory.evaluateRange(min_time, max_time, sampling_intervall, derivative_order, &velocity_samples);


    constexpr size_t elsize = sizeof(double);
    size_t shape[2] = {position_samples.size(), 7};
    size_t strides[2] = {elsize * 7, elsize};
    auto a = py::array_t<double>(shape, strides);
    auto view = a.mutable_unchecked<2>();

    for (size_t i = 0; i < position_samples.size(); i++){
        view(i, 0) = position_samples[i][0];
        view(i, 1) = velocity_samples[i][0];
        view(i, 2) = position_samples[i][1];
        view(i, 3) = velocity_samples[i][1];
        view(i, 4) = position_samples[i][2];
        view(i, 5) = velocity_samples[i][2];
        view(i, 6) = sampling_times[i];
    }

    return a;
}

PYBIND11_MODULE(polynomial_trajectory, m) {
    m.doc() = "Binding to code for generating polynomial trajectories according to paper Polynpmial trajectory planning for aggressive indoor quadrotor flight"; // optional module docstring
    m.def("generate_trajectory", &generateTrajectory, "A function to generate a trajectory from waypoints.");
}
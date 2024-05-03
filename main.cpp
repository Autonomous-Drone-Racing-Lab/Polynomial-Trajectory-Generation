#define CLOG_MAIN
#include <glog/logging.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <Eigen/Core>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

void parseVerticesFromFile(const std::string& file_path, mav_trajectory_generation::Vertex::Vector& vertices, int dimension, int derivative_to_optimize) {
    // Assert vertices is empty
    if (!vertices.empty()) {
        LOG(ERROR) << "Vertices vector is not empty.";
        return;
    }
    
    std::ifstream file(file_path);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << file_path;
        return;
    }

    std::vector<Eigen::Vector3d> points;
    std::string line;
    while (getline(file, line)) {
        std::istringstream iss(line);
        double x, y, z;
        if (!(iss >> x >> y >> z)) {
            LOG(ERROR) << "Error parsing line: " << line;
            continue;
        }

        points.emplace_back(x, y, z);
    }
    file.close();

    if (points.empty()) {
        LOG(ERROR) << "No points read from file.";
        return;
    }

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
    if (points.size() > 1) {
        mav_trajectory_generation::Vertex end(dimension);
        end.makeStartOrEnd(points.back(), derivative_to_optimize);
        vertices.push_back(end);
    }
}

void writeSamplingPointsToFile(const std::string& file_path, const std::vector<Eigen::VectorXd>& points, const std::vector<double>& times) {
    std::ofstream file(file_path);
    if (!file.is_open()) {
        LOG(ERROR) << "Failed to open file: " << file_path;
        return;
    }

    for (size_t i = 0; i < points.size(); ++i) {
        // return comma separated values
        file << times[i] << ", ";
        for (int j = 0; j < points[i].size() - 1; ++j) {
            file << points[i][j] << ", ";
        }
        file << points[i][points[i].size() - 1];
        file << std::endl;
    }
}


int main(){
    mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  std::string input_path = "/home/tim/code/lsy_drone_racing/path.txt";
  std::string output_path = "/home/tim/code/lsy_drone_racing/traj.txt";

  parseVerticesFromFile(input_path, vertices, dimension, derivative_to_optimize);


std::vector<double> segment_times;
const double v_max = 3.0;
const double a_max = 5.0;
segment_times = estimateSegmentTimes(vertices, v_max, a_max);

const int N = 10;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();

// Convert to trajectory
mav_trajectory_generation::Trajectory trajectory;
opt.getTrajectory(&trajectory);

// Sample from trajectory
double min_time = trajectory.getMinTime();
double max_time = trajectory.getMaxTime();
double sampling_intervall = 0.1;
int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
std::vector<Eigen::VectorXd> result;
std::vector<double> sampling_times;
trajectory.evaluateRange(min_time, max_time, sampling_intervall, derivative_order, &result, &sampling_times);

std::cout << "Tejectory generated that passed track in " << max_time << " seconds." << std::endl;

writeSamplingPointsToFile(output_path, result, sampling_times);
}

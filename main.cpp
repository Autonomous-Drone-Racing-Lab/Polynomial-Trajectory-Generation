#define CLOG_MAIN
#include <glog/logging.h>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
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

        std::cout << "Adding point (" << x << ", " << y << ", " << z << ") to path." << std::endl;

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


int main(){
    mav_trajectory_generation::Vertex::Vector vertices;
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
  std::string file_path = "/home/tim/code/lsy_drone_racing/path.txt";

  parseVerticesFromFile(file_path, vertices, dimension, derivative_to_optimize);


std::vector<double> segment_times;
const double v_max = 2.0;
const double a_max = 2.0;
segment_times = estimateSegmentTimes(vertices, v_max, a_max);

const int N = 10;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();

// trajectory time is the sum of all segment times
opt.getSegmentTimes(&segment_times);

// print segment times
for (size_t i = 0; i < segment_times.size(); ++i) {
  LOG(INFO) << "Segment " << i << " time: " << segment_times[i];
}



mav_trajectory_generation::Segment::Vector segments;
opt.getSegments(&segments);

// print segments
for (size_t i = 0; i < segments.size(); ++i) {
 // LOG(INFO) << "Segment " << i << ": " << segments[i];
}

}

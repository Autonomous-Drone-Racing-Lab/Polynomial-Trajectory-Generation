#include "poly_traj/trajectory_generator.h"
#include "Eigen/Dense"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "vector"
#include <iostream>

namespace poly_traj
{   



    bool generateTrajectory(const std::vector<Eigen::Vector3d> &waypoints, double v_max, double a_max, double sampling_intervall, const double startTimeOffset, const Eigen::Vector3d& initialVel, const Eigen::Vector3d& initialAcc, Eigen::MatrixXd &result)
    {

        
        // config
        int dimension = 3;
        int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
        const int N = 10; // polynomial order


        // assert that the number of waypoints is at least 2
        if (waypoints.size() < 2)
        {
           
            throw std::invalid_argument("At least two waypoints are required");
        }

        // Create a vector of vertices
        mav_trajectory_generation::Vertex::Vector vertices;

        // Handle the first vertex
        mav_trajectory_generation::Vertex start(dimension);
        // stack start position, velocity and acceleration
        Eigen::VectorXd startConstraint(3*dimension);
        startConstraint << waypoints[0], initialVel, initialAcc; 
        start.makeStartOrEnd(startConstraint, derivative_to_optimize);
        vertices.push_back(start);

        // Handle the middle vertices
        for (size_t i = 1; i < waypoints.size() - 1; ++i)
        {
            mav_trajectory_generation::Vertex vertex(dimension);
            vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);
            vertices.push_back(vertex);
        }

        // Handle the last vertex
        mav_trajectory_generation::Vertex end(dimension);
        const int last_index = waypoints.size() - 1;
        end.makeStartOrEnd(waypoints[last_index], derivative_to_optimize);
        vertices.push_back(end);

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

        // sample acceleration
        derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
        std::vector<Eigen::VectorXd> acceleration_samples;
        trajectory.evaluateRange(min_time, max_time, sampling_intervall, derivative_order, &acceleration_samples);

        // write to result
        const int no_samples = position_samples.size();
        result.resize(no_samples, 10);
        for (int i = 0; i < no_samples; i++)
        {
            result(i, 0) = position_samples[i](0);
            result(i, 1) = velocity_samples[i](0);
            result(i, 2) = acceleration_samples[i](0);
            result(i, 3) = position_samples[i](1);
            result(i, 4) = velocity_samples[i](1);
            result(i, 5) = acceleration_samples[i](1);
            result(i, 6) = position_samples[i](2);
            result(i, 7) = velocity_samples[i](2);
            result(i, 8) = acceleration_samples[i](2);
            result(i, 9) = sampling_times[i] + startTimeOffset;
        }


        return true;
    }

    // /**
    //  * @brief Generate a trajectory from a list of waypoints. To make it smooth last waypoint of each segment is merged with first of the next by linear interpolation
    // */
    // bool generateTrajectory(const std::vector<Eigen::MatrixXd> &waypoints, double v_max, double a_max, double sampling_intervall, const double startTimeOffset, const Eigen::Vector3d& initialVel, const Eigen::Vector3d& initialAcc, Eigen::MatrixXd &result)
    // {   
    //     int no_waypoints = 0;
    //     for (const auto &waypoint : waypoints)
    //     {
    //         no_waypoints += waypoint.rows();
    //     }

    //     int no_consecutive_segments = waypoints.size() - 1;
    //     no_waypoints -= no_consecutive_segments;

    //     // add all waypoints to a single matrix
    //     // however for each consecutive segment merge the last point of the previous
    //     // and the first point of the next, by linear interpolation
    //     Eigen::MatrixXd mergedPoints(no_consecutive_segments, 3);
    //     for(int i = 0; i < waypoints.size() - 1; i++){
    //         const Eigen::VectorXd& lastPoint = waypoints[i].row(waypoints[i].rows() - 1);
    //         const Eigen::VectorXd& firstPoint = waypoints[i + 1].row(0);

    //         Eigen::VectorXd mergedPoint = (lastPoint + firstPoint) / 2;
    //         mergedPoints.row(i) = mergedPoint;
    //     }
        
    //     Eigen::MatrixXd waypoints_matrix(no_waypoints, 3);
        
        
    //     int row = 1;
    //     // add first row, as it is not merged
    //     waypoints_matrix.row(0) = waypoints[0].row(0);
    //     int merged_point_idx = 0;
    //     for(const auto& innerWaypoints : waypoints){
    //        // add second until second to last element
    //        int noWaypoints = innerWaypoints.rows() - 2;
    //        waypoints_matrix.block(row, 0, noWaypoints, 3) = innerWaypoints.block(1, 0, noWaypoints, 3);
    //         row += noWaypoints;
    //        // add merged point
    //        if(merged_point_idx < mergedPoints.rows()){
    //            waypoints_matrix.row(row) = mergedPoints.row(merged_point_idx);
    //            row++;
    //            merged_point_idx++;
    //        }
    //     }
    //     // add last row, as it is not merged
    //     const auto& lastWaypoints = waypoints[waypoints.size() - 1];
    //     waypoints_matrix.row(row) = lastWaypoints.row(lastWaypoints.rows() - 1);
        

    //     return generateTrajectory(waypoints_matrix, v_max, a_max, sampling_intervall, startTimeOffset, initialVel, initialAcc, result);
    // }
} // namespace poly_traj
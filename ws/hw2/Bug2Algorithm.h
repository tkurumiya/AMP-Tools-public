#pragma once
#include "AMPCore.h"
#include "hw/HW2.h"
#include "CollisionChecker.h"
#include <Eigen/Core>
#include <utility>
#include <vector>

class Bug2Algorithm : public amp::BugAlgorithm {
public:
    // I tuned these parameters to account for numerical precision and to prevent overshooting of paths
    double step_size  = 0.01;  // finer step size means better precision
    double tol_goal   = 1e-6;   // tolerance of how close to goal
    double mline_tol  = 1e-2;   // tolerance of how close to mline

    amp::Path2D plan(const amp::Problem2D& problem) override;
    Bug2Algorithm();

private:
    // Add any member variables here...
    // Helpers
    Eigen::Vector2d turn_right(const Eigen::Vector2d& v);
    Eigen::Vector2d turn_left (const Eigen::Vector2d& v);
    // Instantiate collision checker
    CollisionChecker checker;

    // returns perpendicular distance to m-linm and projection parameter 
    std::pair<double,double> distance_and_param_m_line(const Eigen::Vector2d& q_start,
                                                       const Eigen::Vector2d& q_goal,
                                                       Eigen::Vector2d& cur_pos);

    // move on m-line and output q_hit and u_hit when obstacle encountered
    bool move_on_mline(const amp::Problem2D& prob,
                       amp::Path2D& path,
                       Eigen::Vector2d& q_hit,
                       double& u_hit,
                       const Eigen::Vector2d& m_line_vec);

    // return boolean result of <reached goal, ended_without_leave>
    std::pair<bool,bool> follow_boundary(const amp::Problem2D& prob,
                                         amp::Path2D& path,
                                         const Eigen::Vector2d& q_hit,
                                         double u_hit,
                                         const Eigen::Vector2d& m_line_vec);

};

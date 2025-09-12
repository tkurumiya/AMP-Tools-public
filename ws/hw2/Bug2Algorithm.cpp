#include "Bug2Algorithm.h"
#include <algorithm>
#include <cmath>
#include <iostream>


// ----- Helpers ----- //
Eigen::Vector2d Bug2Algorithm::turn_right(const Eigen::Vector2d& v){ 
    return {v.y(),-v.x()}; 
}
Eigen::Vector2d Bug2Algorithm::turn_left (const Eigen::Vector2d& v){ 
    return {-v.y(),v.x()};}

Bug2Algorithm::Bug2Algorithm() {
    checker.collision_tol = 1e-20; // Make stricter tolerance for workspace 2 case where m-line and intermedaite edges are collinear
}

// ----- Algorithm functions ----- // 
// Compute distance to m-line segment
std::pair<double,double> Bug2Algorithm::distance_and_param_m_line(const Eigen::Vector2d& q_start, const Eigen::Vector2d& q_goal,Eigen::Vector2d& cur_pos)
{
    Eigen::Vector2d mline_vec = q_goal - q_start;
    // Get projection paramter onto m_line to check if later than last hit point   
    double u_param = (cur_pos - q_start).dot(mline_vec) / mline_vec.squaredNorm(); 
    // Projection point 
    Eigen::Vector2d proj = q_start + u_param*mline_vec; 
    // Compute perpendicular distance 
    double dist_to_m_line = (cur_pos-proj).norm(); 
    return{dist_to_m_line, u_param};
}

// ----- Move on m-line toward goal until hit or goal ----- //
bool Bug2Algorithm::move_on_mline(const amp::Problem2D& prob,
                                  amp::Path2D& path,
                                  Eigen::Vector2d& q_hit,
                                  double& u_hit,
                                  const Eigen::Vector2d& m_line_vec)
{
    Eigen::Vector2d m_line_dir = m_line_vec.normalized();

    while (true) {
        Eigen::Vector2d cur_pos = path.waypoints.back();

        // if very close to goal move to it to avoid errors due to numerical precision
        if ((prob.q_goal - cur_pos).norm() <= std::max(step_size, tol_goal)) {
            path.waypoints.push_back(prob.q_goal);
            return true;
        }

        Eigen::Vector2d next_pos = cur_pos + step_size * m_line_dir;

        if (checker.collides(next_pos, prob.obstacles)) {
            q_hit = cur_pos;  // last free config
            auto [_, u_param] = distance_and_param_m_line(prob.q_init, prob.q_goal, q_hit);
            u_hit = u_param;
            return false;
        }

        path.waypoints.push_back(next_pos);
    }
}
std::pair<bool,bool> Bug2Algorithm::follow_boundary(
    const amp::Problem2D& prob, 
    amp::Path2D& path, 
    const Eigen::Vector2d& q_hit,
    double u_hit,
    const Eigen::Vector2d& m_line_vec
){
    // left turning robot
    Eigen::Vector2d dir_to_goal = turn_left((prob.q_goal - path.waypoints.back()).normalized());

    // loop-closure due to reaching qhit
    double loop_tol = 1e-3;
    int steps_since_hit = 0;

    while (true) {
        Eigen::Vector2d cur_pos = path.waypoints.back();

        // if boundary heading is almost parallel to the m-line, use m-line step
        Eigen::Vector2d m_dir  = m_line_vec.normalized();
        double col_parallel = std::abs((prob.q_goal-path.waypoints.back()).normalized().dot(m_dir));
        if (col_parallel > 0.99) {
            Eigen::Vector2d test_mline = cur_pos + step_size * m_dir;

            if (!checker.collides(test_mline, prob.obstacles)) {
                auto [dist_to_mline, u_next] = distance_and_param_m_line(prob.q_init, prob.q_goal, test_mline);
                if (dist_to_mline <= mline_tol && (u_next > u_hit) && (u_next <= 1.0)) {
                    path.waypoints.push_back(test_mline);
                    return {false, false}; // leave boundary; resume m-line mode
                }
            }
        }
        // ----------------------------------------------------------------

        Eigen::Vector2d tryL = (turn_right(dir_to_goal)).normalized();
        Eigen::Vector2d tryF = (dir_to_goal).normalized();
        Eigen::Vector2d tryR = (turn_left(dir_to_goal)).normalized();
        Eigen::Vector2d tryB = (-dir_to_goal).normalized();

        Eigen::Vector2d chosen   = tryL;
        Eigen::Vector2d next_pos = cur_pos + step_size * tryL;

        if (checker.collides(next_pos, prob.obstacles)) {
            next_pos = cur_pos + step_size * tryF; chosen = tryF;
            if (checker.collides(next_pos, prob.obstacles)) {
                next_pos = cur_pos + step_size * tryR; chosen = tryR;
                if (checker.collides(next_pos, prob.obstacles)) {
                    next_pos = cur_pos + step_size * tryB; chosen = tryB;
                }
            }
        }

        dir_to_goal = chosen;
        path.waypoints.push_back(next_pos);
        ++steps_since_hit;

        // goal reached during following
        if ((prob.q_goal - next_pos).norm() <= std::max(step_size, tol_goal)) {
            path.waypoints.push_back(prob.q_goal);
            return {true, false};
        }

        // standard Bug-2 leave condition (on m-line and strictly beyond the hit)
        auto [dist_to_mline, u_now] = distance_and_param_m_line(prob.q_init, prob.q_goal, next_pos);
        if (dist_to_mline <= mline_tol && (u_now > u_hit) && (u_now <= 1.0)) {
            return {false, false};
        }

        // loop closure: back near the hit neighborhood
        if (steps_since_hit > 3 && (next_pos - q_hit).norm() <= loop_tol) {
            return {false, true};
        }
    }
}


// ----- Main motion planning algorithm ----- //
amp::Path2D Bug2Algorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    // m-line vector from init to goal
    Eigen::Vector2d m_line_vec = problem.q_goal - problem.q_init;

    while (true) {
        // march along m-line
        Eigen::Vector2d q_hit;
        double u_hit = 0.0;

        if (move_on_mline(problem, path, q_hit, u_hit, m_line_vec)) {
            return path; // reached goal
        }

        // follow boundary until next m-line crossing or loop
        auto [reached_goal, looped] = follow_boundary(problem, path, q_hit, u_hit, m_line_vec);
        if (reached_goal) return path;
        if (looped)       return path;

    }
}

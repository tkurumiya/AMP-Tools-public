#include "Bug1Algorithm.h"
#include <cmath>
#include <cstddef>
#include <algorithm>

// ----- Helpers ----- //
// Helper function for 90 degree turns left or right 
Eigen::Vector2d Bug1Algorithm::turn_right(const Eigen::Vector2d &v){ 
    return {v.y(),-v.x()}; 
}
Eigen::Vector2d Bug1Algorithm::turn_left(const Eigen::Vector2d &v){ 
    return {-v.y(),v.x()}; 
}

// ----- Algorithm functions ----- //
//  Move-to-goal part of algorithm
bool Bug1Algorithm::move_toward_goal_until_hit(const amp::Problem2D& prob, amp::Path2D& path, Eigen::Vector2d& q_hit){
    while (true){
        Eigen::Vector2d cur_pos = path.waypoints.back(); // Get last path waypoint 
        if ((prob.q_goal-cur_pos).norm() <= std::max(step_size,tol_goal)) { // If very close to goal, move to goal to avoid problems with overshoot
            path.waypoints.push_back(prob.q_goal);
            return true; // reached goal
        }
        // If goal is not reached get direction towards goal
        Eigen::Vector2d dir_to_goal = (prob.q_goal-cur_pos).normalized();
        Eigen::Vector2d next_pos = cur_pos+step_size*dir_to_goal; // Increment next desired position based on unit vec towards goal

        // Check if the desired position collides with any obstacles in workspace 
        if (checker.collides(next_pos, prob.obstacles)) {
            q_hit = cur_pos; // record position at hit
            return false; 
        }
        path.waypoints.push_back(next_pos); 
    }
}

// Boundary-following part of algorithm
std::pair<bool,bool> Bug1Algorithm::follow_boundary_record_leave(const amp::Problem2D& prob, 
    amp::Path2D& path, 
    const Eigen::Vector2d& q_hit,
    size_t& q_leave_idx)
{
    size_t q_hit_idx = path.waypoints.size()-1;
    q_leave_idx = q_hit_idx;

    // Initiate best distance to goal along boundary
    double best_dist = (prob.q_goal - path.waypoints.back()).norm();

    // Initiate wall-follow direction
    Eigen::Vector2d dir_goal = (prob.q_goal - path.waypoints.back()).normalized();
    Eigen::Vector2d dir_to_goal = turn_left(dir_goal);

    while (true){
        Eigen::Vector2d cur_pos = path.waypoints.back();

        Eigen::Vector2d tryR = (turn_right(dir_to_goal)).normalized();
        Eigen::Vector2d tryF = (dir_to_goal).normalized();
        Eigen::Vector2d tryL = (turn_left(dir_to_goal)).normalized();
        Eigen::Vector2d tryB = (-dir_to_goal).normalized();

        Eigen::Vector2d chosen = tryR;
        Eigen::Vector2d next_pos = cur_pos + step_size * tryR;

        if (checker.collides(next_pos, prob.obstacles)) {
            next_pos = cur_pos + step_size * tryF; chosen = tryF;
            if (checker.collides(next_pos, prob.obstacles)) {
                next_pos = cur_pos + step_size * tryL; chosen = tryL;
                if (checker.collides(next_pos, prob.obstacles)) {
                    next_pos = cur_pos + step_size * tryB; chosen = tryB;
                }
            }
        }

        dir_to_goal = chosen;
        path.waypoints.push_back(next_pos);

        // If reached goal during boundary following, exit 
        if ((prob.q_goal - next_pos).norm() <= std::max(step_size, tol_goal)) {
            path.waypoints.push_back(prob.q_goal);
            return {true, false};
        }

        // Record closest point to goal
        double dist_to_goal = (prob.q_goal - next_pos).norm();
        if (dist_to_goal <best_dist) { best_dist = dist_to_goal; q_leave_idx = path.waypoints.size() - 1; }

        // Case where hit point is reencountered 
        if ((path.waypoints.size()-q_hit_idx) > 3 &&
            (path.waypoints[q_hit_idx] - next_pos).norm() <= step_size) {
            return {false, true}; // didn’t reach goal; loop closed at hit
        }

    }
}

// go to leave point along traced perimeter
void Bug1Algorithm::go_to_leave_point(amp::Path2D& path, size_t q_hit_idx, size_t q_leave_idx) {
    size_t end_idx = path.waypoints.size()-1;
    // The faster path is the min of the number of discrete steps from qhit to qleave or from current step to qleave
    bool forward_shorter = (q_leave_idx-q_hit_idx)<(end_idx-q_leave_idx);

    // Retrace streps in correct direction
    if (forward_shorter) {
        for (size_t i =q_hit_idx; i<= q_leave_idx; i++)
            path.waypoints.push_back(path.waypoints[i]);
    } 
    else {
        for (size_t i = end_idx + 1; i-- > q_leave_idx; )
            path.waypoints.push_back(path.waypoints[i]);
    }
}

// ----- Main motion planning algorithm -----
amp::Path2D Bug1Algorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init); // Initialize first leave point to q start 

    while (true) {
        // From last leave point move in straight line toward q_goal
        Eigen::Vector2d q_hit; // Declare q_hit to record where robot hits obstacle
        bool reached_goal = move_toward_goal_until_hit(problem, path, q_hit);
        if (reached_goal) return path;
        // Follow boundary and record point closest to goal or if hit point if reencountered
        size_t q_leave_idx = path.waypoints.size()-1; 
        auto [goal_on_boundary, reencounter_hit]=follow_boundary_record_leave(problem, path, q_hit, q_leave_idx);
        if (goal_on_boundary) return path;

        // Go to leave point along the perimeter 
        size_t q_hit_idx = std::distance(path.waypoints.begin(),
                                               std::find(path.waypoints.begin(), path.waypoints.end(), q_hit));
        go_to_leave_point(path, q_hit_idx, q_leave_idx);
        // Loop back to move to goal prt 

    }
}

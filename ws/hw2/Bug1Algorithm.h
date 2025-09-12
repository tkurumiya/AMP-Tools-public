#pragma once
#include "AMPCore.h"
#include "CollisionChecker.h"
#include "hw/HW2.h"
#include <Eigen/Core>
#include <vector>

// Bug 1 motion planner 
class Bug1Algorithm : public amp::BugAlgorithm {
public:

    // Define step size and tolerances to goal
    double step_size = 0.01;    
    double tol_goal  = 0;     

    // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
    virtual amp::Path2D plan(const amp::Problem2D& problem) override;

private:
    // Add any member variables here...
    // Helpers
    Eigen::Vector2d turn_right(const Eigen::Vector2d& v);
    Eigen::Vector2d turn_left(const Eigen::Vector2d& v);

    // Instantiate collision checker
    CollisionChecker checker;

    // Methods for path-planning 
    /* From the current waypoint, continue toward goal until goal is reached or obstacle is hit.
    q_hit is set to the last waypoint before collision. Returns true if goal reached */
    bool move_toward_goal_until_hit(const amp::Problem2D& prob,
                                    amp::Path2D& path,
                                    Eigen::Vector2d& q_hit);

    /* Follow boundary recording the closest-to-goal index q_leave_idx among perimeter points.
    Stops when goal or hit-point is reached. Returns boolean pair respectively to reached goal, encountered hit
     */
    std::pair<bool,bool> follow_boundary_record_leave(const amp::Problem2D& prob,
                                                      amp::Path2D& path,
                                                      const Eigen::Vector2d& q_hit,
                                                      size_t& q_leave_idx);

    // Move from q_hit to q_leave via on shorter path
    void go_to_leave_point(amp::Path2D& path, size_t q_hit_idx, size_t q_leave_idx);

};

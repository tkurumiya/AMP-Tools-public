#pragma once
#include "AMPCore.h"
#include <Eigen/Core>
#include <vector>

// Header file for collision checker class 
class CollisionChecker {
    public:
        double collision_tol = 1e-9; // Use tolerance to account for floating point

        // Returns true if point is inside workspace obstacle
        bool point_in_poly(const Eigen::Vector2d& p, const amp::Obstacle2D& poly) const;

        // Returns true if point lies inside any union of obstacles 
        bool collides(const Eigen::Vector2d& p, const std::vector<amp::Obstacle2D>& obstacles) const;
};

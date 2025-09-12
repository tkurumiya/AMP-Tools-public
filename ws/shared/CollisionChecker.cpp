#include "CollisionChecker.h"
#include <cmath>


// Function checks if robot is inside a workspace obstacle
bool CollisionChecker::point_in_poly(const Eigen::Vector2d& point, const amp::Obstacle2D& poly) const {
    const auto& vertices = poly.verticesCCW();       // Get CCW vertices of polygon

    // Iterate for each vertex until each side is accounted for
    for (int i = 0; i < vertices.size(); i++){
        const Eigen::Vector2d& vi = vertices[i];
        const Eigen::Vector2d& vj = vertices[(i+1) % vertices.size()]; // Wrap around to first index if i is last index

        // Compute inward normal vector
        const Eigen::Vector2d edge_vec = vj - vi;
        Eigen::Vector2d normal_vec(-edge_vec.y(), edge_vec.x()); normal_vec = normal_vec / normal_vec.norm();
        normal_vec = normal_vec.normalized(); 

        // Check if point inside polygon
        if ((point - vi).dot(normal_vec) < -collision_tol) return false; // If negative dot product between point and edge, then outside polygon. No collision
    }
    return true; // If all dot products are positive, then inside polygon.
}

bool CollisionChecker::collides(const Eigen::Vector2d& point, const std::vector<amp::Obstacle2D>& obstacles) const {
    for (const amp::Polygon obstacle : obstacles)
        if (point_in_poly(point, obstacle)) return true;
    return false;
}

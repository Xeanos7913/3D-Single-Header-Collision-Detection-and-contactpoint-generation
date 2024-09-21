#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtx/string_cast.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

struct OBB {
    glm::vec3 center;      // Center of the OBB
    glm::vec3 halfExtents; // Half-sizes along each axis
    glm::vec3 axes[3];     // Local x, y, z axes (unit vectors)

    // Transforms OBB using a model matrix
    void transform(const glm::mat4& model) {
        center = glm::vec3(model * glm::vec4(center, 1.0f));
        axes[0] = glm::normalize(glm::vec3(model * glm::vec4(axes[0], 0.0f)));
        axes[1] = glm::normalize(glm::vec3(model * glm::vec4(axes[1], 0.0f)));
        axes[2] = glm::normalize(glm::vec3(model * glm::vec4(axes[2], 0.0f)));
    }

    // Get all 8 vertices of the OBB
    std::vector<glm::vec3> getVertices() const {
        std::vector<glm::vec3> vertices(8);
        glm::vec3 hX = axes[0] * halfExtents.x;
        glm::vec3 hY = axes[1] * halfExtents.y;
        glm::vec3 hZ = axes[2] * halfExtents.z;

        vertices[0] = center + hX + hY + hZ;
        vertices[1] = center + hX + hY - hZ;
        vertices[2] = center + hX - hY + hZ;
        vertices[3] = center + hX - hY - hZ;
        vertices[4] = center - hX + hY + hZ;
        vertices[5] = center - hX + hY - hZ;
        vertices[6] = center - hX - hY + hZ;
        vertices[7] = center - hX - hY - hZ;

        return vertices;
    }

    // Get edges of the OBB (each edge is a pair of vertices)
    std::vector<std::pair<glm::vec3, glm::vec3>> getEdges() const {
        std::vector<glm::vec3> vertices = getVertices();
        std::vector<std::pair<glm::vec3, glm::vec3>> edges;

        // Edges along x-axis
        edges.emplace_back(vertices[0], vertices[1]);
        edges.emplace_back(vertices[2], vertices[3]);
        edges.emplace_back(vertices[4], vertices[5]);
        edges.emplace_back(vertices[6], vertices[7]);

        // Edges along y-axis
        edges.emplace_back(vertices[0], vertices[2]);
        edges.emplace_back(vertices[1], vertices[3]);
        edges.emplace_back(vertices[4], vertices[6]);
        edges.emplace_back(vertices[5], vertices[7]);

        // Edges along z-axis
        edges.emplace_back(vertices[0], vertices[4]);
        edges.emplace_back(vertices[1], vertices[5]);
        edges.emplace_back(vertices[2], vertices[6]);
        edges.emplace_back(vertices[3], vertices[7]);

        return edges;
    }
};

// Define a struct to store the collision result
struct CollisionResult {
    bool collisionDetected = false;
    glm::vec3 vertexFaceContactPoint = glm::vec3(0.0f);
    glm::vec3 edgeEdgeContactPoint = glm::vec3(0.0f);
};

// Helper function to project a point onto a plane
glm::vec3 projectPointOntoPlane(const glm::vec3& point, const glm::vec3& planeOrigin, const glm::vec3& planeNormal) {
    float dist = glm::dot(point - planeOrigin, planeNormal);
    return point - dist * planeNormal;
}

// Compute the signed distance from a point to a plane
float signedDistanceToPlane(const glm::vec3& point, const glm::vec3& planeOrigin, const glm::vec3& planeNormal) {
    return glm::dot(point - planeOrigin, planeNormal);
}

// Checks if a point is inside the bounds of a face
bool isPointInFaceBounds(const glm::vec3& point, const glm::vec3& faceCenter, const glm::vec3& u, const glm::vec3& v, float uHalf, float vHalf) {
    glm::vec3 rel = point - faceCenter;
    float uCoord = glm::dot(rel, u);
    float vCoord = glm::dot(rel, v);
    return std::abs(uCoord) <= uHalf && std::abs(vCoord) <= vHalf;
}

// Vertex-face collision detection
glm::vec3 vertexFaceCollision(const OBB& obb1, const OBB& obb2) {
    std::vector<glm::vec3> vertices1 = obb1.getVertices();
    std::vector<glm::vec3> vertices2 = obb2.getVertices();

    glm::vec3 closestPoint = glm::vec3(0.0f);
    float closestDistance = std::numeric_limits<float>::max();

    // Test OBB1 vertices against OBB2 faces
    for (const auto& vertex : vertices1) {
        for (int i = 0; i < 3; ++i) {
            glm::vec3 faceNormal = obb2.axes[i];
            glm::vec3 faceCenter = obb2.center;
            glm::vec3 u = obb2.axes[(i + 1) % 3];
            glm::vec3 v = obb2.axes[(i + 2) % 3];

            // Compute the signed distance from the vertex to the face plane
            float distance = signedDistanceToPlane(vertex, faceCenter, faceNormal);

            // If the vertex is behind the face (penetrating the OBB)
            if (distance < 0.0f) {
                // Check if the vertex is inside the bounds of the face
                if (isPointInFaceBounds(vertex, faceCenter, u, v, obb2.halfExtents[(i + 1) % 3], obb2.halfExtents[(i + 2) % 3])) {
                    // If this vertex is closer than the previously recorded one, update
                    if (std::abs(distance) < closestDistance) {
                        closestDistance = std::abs(distance);
                        closestPoint = vertex;
                    }
                }
            }
        }
    }

    // Test OBB2 vertices against OBB1 faces
    for (const auto& vertex : vertices2) {
        for (int i = 0; i < 3; ++i) {
            glm::vec3 faceNormal = obb1.axes[i];
            glm::vec3 faceCenter = obb1.center;
            glm::vec3 u = obb1.axes[(i + 1) % 3];
            glm::vec3 v = obb1.axes[(i + 2) % 3];

            // Compute the signed distance from the vertex to the face plane
            float distance = signedDistanceToPlane(vertex, faceCenter, faceNormal);

            // If the vertex is behind the face (penetrating the OBB)
            if (distance < 0.0f) {
                // Check if the vertex is inside the bounds of the face
                if (isPointInFaceBounds(vertex, faceCenter, u, v, obb1.halfExtents[(i + 1) % 3], obb1.halfExtents[(i + 2) % 3])) {
                    // If this vertex is closer than the previously recorded one, update
                    if (std::abs(distance) < closestDistance) {
                        closestDistance = std::abs(distance);
                        closestPoint = vertex;
                    }
                }
            }
        }
    }

    return closestPoint; // Return the exact vertex-face contact point
}

// Compute closest points between two line segments
glm::vec3 closestPointBetweenLines(const glm::vec3& p1, const glm::vec3& q1, const glm::vec3& p2, const glm::vec3& q2) {
    glm::vec3 d1 = q1 - p1;  // Direction vector of segment S1
    glm::vec3 d2 = q2 - p2;  // Direction vector of segment S2
    glm::vec3 r = p1 - p2;
    float a = glm::dot(d1, d1);  // Squared length of segment S1
    float e = glm::dot(d2, d2);  // Squared length of segment S2
    float f = glm::dot(d2, r);

    float s, t;
    if (a <= 1e-6f && e <= 1e-6f) {
        // Both segments degenerate into points
        return p1;
    }
    if (a <= 1e-6f) {
        // First segment degenerate into a point
        s = 0.0f;
        t = f / e; // Use the closest point on segment S2 to point p1
    }
    else {
        float c = glm::dot(d1, r);
        if (e <= 1e-6f) {
            // Second segment degenerate into a point
            t = 0.0f;
            s = -c / a; // Use the closest point on segment S1 to point p2
        }
        else {
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b;

            if (denom != 0.0f) {
                s = (b * f - c * e) / denom;
            }
            else {
                s = 0.0f;
            }

            t = (b * s + f) / e;
        }
    }

    // Clamp s and t to [0, 1] and compute the closest points
    s = glm::clamp(s, 0.0f, 1.0f);
    t = glm::clamp(t, 0.0f, 1.0f);
    glm::vec3 c1 = p1 + s * d1;
    glm::vec3 c2 = p2 + t * d2;
    return (c1 + c2) / 2.0f; // Return midpoint as contact point
}

// Edge-edge collision detection
glm::vec3 edgeEdgeCollision(const OBB& obb1, const OBB& obb2) {
    std::vector<std::pair<glm::vec3, glm::vec3>> edges1 = obb1.getEdges();
    std::vector<std::pair<glm::vec3, glm::vec3>> edges2 = obb2.getEdges();

    float minDistance = std::numeric_limits<float>::max();
    glm::vec3 contactPoint;

    // Test all edge pairs
    for (const auto& edge1 : edges1) {
        for (const auto& edge2 : edges2) {
            glm::vec3 closestPoint = closestPointBetweenLines(edge1.first, edge1.second, edge2.first, edge2.second);
            float distance = glm::length(closestPoint - obb1.center);

            if (distance < minDistance) {
                minDistance = distance;
                contactPoint = closestPoint;
            }
        }
    }

    return contactPoint;
}

// Helper function to project an OBB onto an axis and compute min/max projections
void projectOBB(const OBB& obb, const glm::vec3& axis, float& min, float& max) {
    float centerProjection = glm::dot(obb.center, axis);
    float extent = obb.halfExtents.x * std::abs(glm::dot(obb.axes[0], axis)) +
        obb.halfExtents.y * std::abs(glm::dot(obb.axes[1], axis)) +
        obb.halfExtents.z * std::abs(glm::dot(obb.axes[2], axis));

    min = centerProjection - extent;
    max = centerProjection + extent;
}

// Checks for overlap along a single axis
bool overlapOnAxis(const OBB& obb1, const OBB& obb2, const glm::vec3& axis) {
    float min1, max1, min2, max2;
    projectOBB(obb1, axis, min1, max1);
    projectOBB(obb2, axis, min2, max2);
    return !(max1 < min2 || max2 < min1);
}

// Perform SAT collision test between two OBBs and return contact points in CollisionResult
bool SATCollision(const OBB& obb1, const OBB& obb2, CollisionResult& result) {
    glm::vec3 testAxes[15];

    // 3 axes from OBB1
    testAxes[0] = obb1.axes[0];
    testAxes[1] = obb1.axes[1];
    testAxes[2] = obb1.axes[2];

    // 3 axes from OBB2
    testAxes[3] = obb2.axes[0];
    testAxes[4] = obb2.axes[1];
    testAxes[5] = obb2.axes[2];

    // 9 cross product axes
    int idx = 6;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            testAxes[idx++] = glm::cross(obb1.axes[i], obb2.axes[j]);
        }
    }

    // Test for overlap on all 15 axes
    for (int i = 0; i < 15; ++i) {
        if (glm::length(testAxes[i]) < 1e-6f) continue; // Skip degenerate axes
        if (!overlapOnAxis(obb1, obb2, glm::normalize(testAxes[i]))) {
            return false; // Separating axis found, no collision
        }
    }

    // If no separating axis is found, there is a collision
    result.collisionDetected = true;

    // Call the vertex-face collision detection and store the result
    result.vertexFaceContactPoint = vertexFaceCollision(obb1, obb2);

    // Call the edge-edge collision detection and store the result
    result.edgeEdgeContactPoint = edgeEdgeCollision(obb1, obb2);

    return true; // Collision detected
}
# Collision-Detection-and-contactpoint-generation

A simple C++ header-only single file that uses the Separating Axis Theorem (SAT) to detect collisions between two Oriented Bounding Boxes (OBB) and generate contact points for vertex-face collisions and edge-edge collisions.

## Features
- **3D**: Handles 3D collision detection.
- **Single header**: Easily expandable; just include the header.
- **Uses GLM**: Utilizes the GLM library for mathematics.

## Usage
1. Include the header in your project.
2. Create your OBBs (oriented bounding boxes).
3. Use the provided functions to detect collisions and generate contact points.

## Future plans
1. Actually implement the edge-edge collision.
2. Expand the algorithm to handle general convex hulls (point clouds of vertices).
3. Implement Contact Manifold generation.

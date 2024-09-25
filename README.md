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
1. Expand the algorithm to handle general convex hulls (point clouds of vertices).
2. Implement Contact Manifold generation.

## Demo videos
- **Contact Point Generation:** [https://youtu.be/EaaDjnE3eZ4](https://youtu.be/6cStV7SiHKU)
- **Physics solver:** https://youtu.be/lEXJKrqvyLk

## Some Statements
- For rendering, I used my own game engine 'Calcium Engine', and it is not on github yet.
- Still working on perfecting collision detection for my Calcium Engine.
- In the near future, I might post Calcium Engine's source code on github, but it's very messy primarily because I'm not that good with C++.
- I reccommend using some existing rendering engine for visualization.

## References

- **SAT Algorithm** -> https://dyn4j.org/2010/01/sat/
- **Contact Point generation for edge-edge collisions** -> https://www.geeksforgeeks.org/shortest-distance-between-two-lines-in-3d-space-class-12-maths/
- **Contact Manifold generation** -> https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics5collisionmanifolds/2017%20Tutorial%205%20-%20Collision%20Manifolds.pdf

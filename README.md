# Autonomous Cascade PID Flight Controller

A dual layer cascade PID control architecture implemented in Python for Quadcopter drones.

## Architecture Overview
The controller decouples trajectory tracking into two nested control loops:

1. **Outer Loop (Position & Yaw):**
   - Computes desired body-frame velocities and yaw rates based on global 3D position error.
   - Applies 2D planar rotation matrices for real-time global-to-body frame coordinate transformations.
   - Enforces angle normalization to ensure shortest-path angular error calculation ($-\pi$ to $+\pi$).
   - Implements bounded acceleration smoothing to eliminate aggressive attitude tilting.

2. **Inner Loop (Velocity & Disturbance Rejection):**
   - High-frequency tracking of body-frame velocity setpoints.
   - Utilizes backward-difference numerical velocity estimation from position state streams.
   - Integrates velocity feedforward compensation for rapid setpoint convergence.
   - Employs integral anti-windup clamping to prevent actuator saturation during persistent wind disturbances.
  
## Core Files
- `controller.py`: Primary cascade PID implementation containing `OuterLoopController` and `InnerLoopController` classes.
- Telemetry & performance validation data available in repository.

# Technical explanation and interview preparation

## What did I build?

I implemented a Python ROS 2 mission controller intended to operate a PX4 X500 quadrotor in Gazebo. It computes a Figure-8 search path, subscribes to vehicle position and status, and monitors geofence limits. The PX4 autopilot remains responsible for lower-level stabilization. Flight and failsafe validation must be demonstrated before claiming mission success.

## Why ROS 2?

It provides typed publish/subscribe messaging, clear separation between mission logic and autopilot, and integration with robotics tools. It also introduces DDS QoS compatibility requirements.

## Why PX4?

It provides a mature autopilot and offboard interfaces, allowing me to focus on mission planning and safety logic rather than implementing attitude stabilization.

## Why Gazebo?

It allows repeatable, low-risk simulation tests of trajectories and faults before attempting physical flight.

## What is offboard mode?

PX4 receives external control setpoints from a companion program. A continuing offboard-control heartbeat is required; losing it can cause PX4 to exit offboard mode according to its configuration. Merely publishing a target once is not enough.

## What is NED?

A local frame with North (+X), East (+Y), and Down (+Z). To rise 5 m from a home Z coordinate, the target is home Z minus 5 m.

## How does the Figure-8 work?

`x = x_home + A*sin(2*pi*t/T)` and `y = y_home + B*sin(4*pi*t/T)`. The Y component oscillates twice per X cycle, producing a Figure-8-like path. `z = z_home - altitude`.

## What does the geofence do?

It calculates horizontal distance from home as `sqrt((x-x0)^2 + (y-y0)^2)` and height as `z0-z`. It reports a breach if distance or height exceeds configured limits. A real safety case must also address stale telemetry, missing command acknowledgments, PX4 failsafe states, and vehicle behavior after a landing request.

## What are the main limitations?

This is a software-in-the-loop simulation. It does not establish real-world airworthiness. The existing controller requires end-to-end flight validation and stronger handling of lost telemetry and rejected commands.

## How would I improve it?

Add command acknowledgments, telemetry age checks, state timeouts, parameterized mission configuration, recorded trajectory tracking error, and fault-injection tests. Later, add perception-driven search or obstacle avoidance as a separate milestone.

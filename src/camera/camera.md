# RealSense Camera Node Requirements

## Goal
Define the logic and configuration flow that this folder will own for launching the RealSense ROS 2 node with a globally applied configuration profile.

## Scope
- Manage launch-time parameters for the RealSense depth camera ROS 2 node.
- Ensure a single source of truth for camera settings that every consumer (local nodes or remote clients) can rely on.
- Provide hooks for future extensions such as multi-camera support or dynamic reconfiguration.

## Requirements Draft
1. **Configuration Source**
   - Central YAML (or TOML) file describing sensors, resolutions, frame rates, frame IDs, TF tree roots, and topics.
   - Support environment-variable overrides for deployment-specific tweaks without editing the base file.
2. **Launch Interface**
   - CLI entry point (likely a ROS 2 launch file) that consumes the configuration and spins up the `realsense2_camera` node.
   - Validation step that fails fast if the config is missing mandatory keys.
3. **Global Application**
   - Once loaded, the configuration should be made available to other packages (e.g., via parameters service, topic, or shared library) so every node shares identical camera assumptions.
4. **Monitoring & Health**
   - Basic health topic or diagnostic updater to report camera status, firmware, temperature, and frame drops.
5. **Extensibility Hooks**
   - Placeholder for runtime dynamic parameter updates.
   - Optional record/playback integration (rosbag2) toggled via config flags.

## Open Questions
- Do we plan to run multiple RealSense devices concurrently?
- Should calibration files live alongside this package or be fetched from a central store?
- What is the target ROS 2 distribution and minimum librealsense version?
- How should failures (USB disconnect, bad calibration) be surfaced to the broader system?

## Next Steps
1. Confirm answers to the open questions above.
2. Lock down the configuration file format and schema.
3. Create the launch/utility code that consumes the schema.
4. Add tests or simulation flows to validate the startup logic.

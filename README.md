# Conveyor Belt Synchronization Simulation

A MATLAB simulation environment for evaluating 1D closed-loop conveyor belt synchronization control.

## Overview
This project simulates the synchronization of a conveyor belt (with a parcel) to match a dynamic target moving at a constant mainline velocity. It utilizes a feedforward smoothstep (cubic Hermite) motion planner combined with a proportional feedback control loop ($K_p = 8$) to eliminate tracking errors caused by noise, slip, and numerical discretization.

## File Structure
- [main.m](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/main.m): Main simulation execution script. Runs the real-time simulation loop and plots results.
- [ConveyorSyncController.m](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/ConveyorSyncController.m): State machine and closed-loop control system.
- [evalSmoothstepProfile.m](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/evalSmoothstepProfile.m): Generates the analytical $C^1$-smooth motion profile.
- [requiredGap.m](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/requiredGap.m): Computes the safety distance gap needed for collision-free synchronization.
- [interpolateCrossing.m](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/interpolateCrossing.m): Performs linear interpolation for precise trigger timing.
- [REPORT.md](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/REPORT.md): Comprehensive evaluation report containing mathematical derivations.
- [AGENTS.md](file:///c:/Users/axdev/Documents/MATLAB/Examples/R2026a/belt/AGENTS.md): Multi-Agent specification for the synchronization system.

## Quick Start
Open MATLAB, navigate to this directory, and execute:
```matlab
main
```
This will run the simulation and display real-time position/velocity tracking, acceleration profiles, control errors, and loop iteration timing.

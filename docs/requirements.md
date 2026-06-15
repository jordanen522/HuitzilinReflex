# HuitzilinReflex — Requirements

## Functional Requirements

| ID | Requirement |
|----|-------------|
| REQ-01 | The drone shall autonomously patrol a defined area without human input |
| REQ-02 | The drone shall detect an incoming projectile using the OAK-D Lite stereo camera |
| REQ-03 | The drone shall predict the projectile's intercept trajectory |
| REQ-04 | The drone shall execute an evasive maneuver within 200ms of threat detection |
| REQ-05 | The drone shall activate the alarm (buzzer + LED strobe) upon threat detection |
| REQ-06 | The drone shall return to patrol after a successful evasion |

## Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| REQ-07 | End-to-end latency (detect → evade command) shall be ≤ 200ms |
| REQ-08 | Evasion success rate shall be ≥ 80% in controlled testing |
| REQ-09 | Payload weight overhead shall not exceed 200g |

## Constraints

| ID | Constraint |
|----|------------|
| REQ-10 | Airframe: 3.5" ducted BNF quadrotor (GEPRC CineLog35 V2) |
| REQ-11 | Flight controller: H7-based, ArduPilot-compatible (MicoAir H743 V2) |
| REQ-12 | Compute: Raspberry Pi 5 4GB |
| REQ-13 | Perception: OAK-D Lite fixed-focus, stereo depth computed on-chip |

## Non-Goals

- No event camera
- No multi-layer LiDAR
- No custom depth math (use on-chip DepthAI pipeline)
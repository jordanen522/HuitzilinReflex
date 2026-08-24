# HuitzilinReflex — Requirements

Evidence points at where a requirement is exercised or measured. Everything measured
so far is in **simulation**; nothing here has been shown on hardware. A number quoted
from the oracle lane describes the tracker, trigger and airframe given a sensor of that
reach — not the real detector. `docs/RESULTS.md` §10 has the scoring rules.

## Functional Requirements

| ID | Requirement | Evidence |
|----|-------------|----------|
| REQ-01 | The drone shall autonomously patrol a defined area without human input | `patrol_node.py`; bring-up in `CLAUDE.md` |
| REQ-02 | The drone shall detect an incoming projectile using the OAK-D Lite stereo camera | `RESULTS.md` §8 (real detector, ~3.4 m reach). See the REQ-13 note below |
| REQ-03 | The drone shall predict the projectile's intercept trajectory | `kalman.py`; `RESULTS.md` §2 |
| REQ-04 | The drone shall execute an evasive maneuver within 150 ms of threat detection | `RESULTS.md` §3, §6 |
| REQ-05 | The drone shall activate the alarm (buzzer + LED strobe) upon threat detection | `payload_node.py`; `test_payload.py` |
| REQ-06 | The drone shall return to patrol after a successful evasion | `docs/state_machine.md` (EVADE → PATROL) |

## Non-Functional Requirements

| ID | Requirement | Evidence |
|----|-------------|----------|
| REQ-07 | End-to-end latency (detect → evade command) shall be ≤ 150 ms | `RESULTS.md` §3, §6 |
| REQ-08 | Evasion shall succeed on every hit-intent throw inside the reaction envelope. Success is reported split by ball speed, never as a single blended rate across speeds | `RESULTS.md` §8 |
| REQ-09 | Payload weight overhead shall not exceed 200 g | `HuitzilinReflex_v2.md` Appendix A. **Not verifiable in simulation** |

## Constraints

| ID | Constraint | Evidence |
|----|------------|----------|
| REQ-10 | Airframe: 3.5" ducted BNF quadrotor (GEPRC CineLog35 V2) | `HuitzilinReflex_v2.md` Appendix A |
| REQ-11 | Flight controller: H7-based, ArduPilot-compatible (MicoAir H743 V2) | `HuitzilinReflex_v2.md` Appendix A |
| REQ-12 | Compute: Raspberry Pi 5 4GB | `HuitzilinReflex_v2.md` Appendix A |
| REQ-13 | Perception: OAK-D Lite fixed-focus, stereo depth computed on-chip | `RESULTS.md` §4.1 — see note |

**Note on REQ-02 / REQ-13.** The OAK-D Lite's measured reach bounds the achievable ball
speed. `RESULTS.md` §4 gives the reach a faster dodge would require and §4.1 names the
sensor that would supply it. Treat the OAK-D Lite as the constraint the current envelope
was measured under, not as a constraint on every future envelope.

## Safety Criteria

| ID | Requirement | Evidence |
|----|------------|----------|
| REQ-14 | System responds to link loss safely | `SAFETY_CASE.md` §1, §5; `test_supervisor.py` |
| REQ-15 | Kill-switch cuts motors immediately | `SAFETY_CASE.md` §3, §5 |
| REQ-16 | All faults land in a safe state | `SAFETY_CASE.md` §1, §5; `docs/state_machine.md`. Asserted over all states × faults × `armed` in `test_supervisor.py` |


## Non-Goals

- No event camera
- No multi-layer LiDAR
- No custom depth math (use on-chip DepthAI pipeline)

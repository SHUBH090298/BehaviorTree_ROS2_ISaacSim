# BT Orchestration for Physical AI Agents — Week 1 Log & Demo Runbook

**Scope of this milestone:** a simulated orchestration layer that coordinates independent agents over ROS 2 and degrades safely when they fail. Behaviour trees and Groot2 are tooling; the contribution under test is the orchestration and fallback structure.

**Environment (pinned):** Ubuntu 22.04 · RTX 5000 Ada (16 GB) · driver 580.173.02 · ROS 2 Humble · BehaviorTree.CPP 4.x (`ros-humble-behaviortree-cpp`) · Groot2 v1.9.0 · Isaac Sim 5.1.0 + local asset pack 5.1.0 · `IsaacSim-ros_workspaces` @ tag `IsaacSim-5.1.0`

---

## 1. What was built, day by day

### Day 1 — System preparation and ROS 2 Humble
Installed ROS 2 Humble and base toolchain. Two configuration decisions carry forward:

- **Forced X11 instead of Wayland.** Isaac Sim's renderer fails or shows a black window under Wayland.
- **ROS is *not* auto-sourced in `~/.bashrc`;** a `sros` alias sources it on demand. Isaac Sim decides which ROS libraries to load by inspecting `ROS_DISTRO`, so the terminal that launches it must have a deliberately chosen environment rather than an inherited one. `ROS_DOMAIN_ID=17` is exported everywhere to isolate the system from other machines on the network.

### Day 2 — BehaviorTree.CPP 4.x, Groot2, workspace skeleton
Installed BT.CPP 4.x and Groot2, and created the `bt_thesis` package (executor, node library, tree XMLs, palette generator). Ran a first tree with the Groot2 monitor attached to confirm the toolchain end to end before the simulator was involved.

Notes: `ros-humble-behaviortree-cpp-v3` conflicts with v4 and must be removed first; the Groot2 AppImage requires `libfuse2`; the Groot2 publisher binds **two** consecutive ports (1667 *and* 1668). Groot2's free tier caps live monitoring at 20 nodes, which is a real design constraint on tree size — the final tree was kept to 15.

### Day 3 — Isaac Sim 5.1 and local assets
Installed Isaac Sim 5.1 and the Complete asset pack locally, then pointed `persistent.isaac.asset_root.default` at it in `isaacsim.exp.base.kit`. Local assets were worth the ~100 GB: scene loads drop from minutes to seconds, the demo has no network dependency, and asset versions stay fixed for the life of the thesis.

### Day 4 — ROS 2 bridge and the two launch modes
Established the simulator↔ROS boundary and verified it in both directions using a Franka Emika Panda stage (`~/thesis_ws/scenes/ros_test.usd`) with Clock and JointState OmniGraphs — `/joint_states` out, `/joint_command` in.

The central finding is the **two-mode distinction**, which shapes the architecture:

| | Launch | Isaac Sim can serialize |
|---|---|---|
| **Mode A** | clean terminal (`isaac`) | standard messages only |
| **Mode B** | Build B sourced (`isaac_custom`) | custom messages, in-process `rclpy` |

Mode B requires a ROS 2 built against Isaac Sim's embedded interpreter (Python 3.11 for 5.1), produced in Docker via `build_ros.sh`. Build B was completed and verified, then **deliberately set aside** — see §3.

**Design rule adopted:** keep the simulator boundary on standard messages (`JointState`, `Clock`, TF). This keeps development in Mode A, where a `.msg` change costs a `colcon build` rather than a 30–60 minute Docker rebuild, and means the simulator can later be swapped for hardware without touching message definitions.

### Day 5 — Closing the loop, then the orchestration layer
**(a) BT ↔ simulator.** Replaced the initial mobile-base nodes with manipulator equivalents. `MoveToJoints` is a `StatefulActionNode`: it publishes a joint command, stays RUNNING until every named joint is within tolerance, and — importantly — freezes the arm at its current position in `onHalted()` so a preempted motion cannot drift.

**(b) Orchestration.** Added two Python stub agents and the nodes to coordinate them:

| Agent | Stands in for | Interface |
|---|---|---|
| `perception_agent` | detector (VLM / pose estimator) | `/part_detected`, heartbeat @ 2 Hz |
| `skill_agent` | manipulation skill executor | `/skill_request` → `/skill_status`, heartbeat @ 2 Hz |

New BT nodes: `AgentHealthy` (heartbeat liveness), `WaitForPerception`, `RequestSkill`.

Three design decisions worth defending in the write-up:

1. **Liveness is separate from task status.** An agent that is *alive but failing* is a different fault class from one that is *gone*. The first is retryable; the second is not.
2. **`AgentHealthy` fails when an agent has never been seen,** not only when it goes stale — default-deny, so "not yet started" and "died" route identically.
3. **`RequestSkill::onHalted()` explicitly cancels the agent.** An orchestrator that abandons a request without cancelling leaves the agent doing work nobody is waiting for.

The tree (`trees/orchestration.xml`) puts all fault handling at priority 1 under a `ReactiveFallback`; the nominal task branch contains **no** error handling and no knowledge that the e-stop or the agents exist. Adding a fourth agent to the safety envelope costs one `Inverter`+`AgentHealthy` pair and no C++.

---

## 2. Demo runbook

### Pre-flight (30 min before)
Close all terminals; `pkill -f isaac-sim.sh`; `ros2 daemon stop`; `rm -rf /dev/shm/fastrtps_* /tmp/fastrtps*`. Terminal fonts ≥ 16 pt, notifications and screen lock off. Verify the backup recording opens.

### Launch order

| # | Terminal | Command | Expect |
|---|---|---|---|
| 1 | clean | `isaac` → open `~/thesis_ws/scenes/ros_test.usd` → **Play** | arm holds pose |
| 2 | `sros` | `ros2 run bt_thesis_agents perception_agent` | `perception_agent up` |
| 3 | `sros` | `ros2 run bt_thesis_agents skill_agent` | `skill_agent up` |
| 4 | `sros` | `ros2 run bt_thesis bt_executor --ros-args -p use_sim_time:=true` | `Groot2 monitor on ports 1667/1668` |
| 5 | `sros` | `ros2 topic echo /skill_status` | `idle`→`running`→`success` |
| 6 | `sros` | command terminal for the aliases | — |
| — | — | `groot2` → open `trees/orchestration.xml` → **Monitor** → `localhost:1667` | nodes animate |

Terminals 2 and 3 must be **visible and clearly labelled** — killing a process the audience can see is the strongest moment of the demo. Terminal 4 must be sourced *after* the last `colcon build`.

Aliases: `estop_on` / `estop_off` · `fault_on` / `fault_off` · `part_on` / `part_off`.

### Demo sequence (~6 min)

1. **Architecture (45 s).** Four independent processes sharing nothing but ROS topics. The orchestrator has no access to what is behind either agent.
2. **One clean cycle (45 s).** Follow it in Groot2: `await_detection` → `approach` → `run_inspection` → `return_home`.
3. **Fault class 1 — task failure (60 s).** `fault_on`. The skill agent logs `skill FAILED`, `run_inspection` goes red, the sequence aborts and retries. `fault_off` to recover. *The orchestrator never needed to know why it failed.*
4. **Fault class 2 — agent death (90 s, the centrepiece).** `Ctrl-C` the `skill_agent` terminal. Within ~2 s `skill_alive?` flips red, `degraded_mode` preempts the running motion, the arm moves to the safe pose and **holds**. Let it hold for a few seconds. Restart the agent — the system resumes with no intervention. *Nothing polled for this: liveness is a first-class condition, and recovery is the absence of a fault rather than a recovery routine.*
5. **Fault class 3 — operator stop (30 s).** `estop_on` mid-cycle, `estop_off` to resume. All three fault classes route through the same branch.
6. **Extensibility, demonstrated (60 s).** In the Groot2 editor add one `Inverter`+`AgentHealthy` pair for an agent that does not exist, save, restart the executor: the arm goes straight to safe hold, because that agent has never sent a heartbeat.
7. **Close (30 s).** Stub agents today; the interface is the contribution. Replacing either with a real model changes nothing above the topic boundary.

**If something breaks live:** *"That's a fault the orchestrator has no branch for — which is exactly the classification problem this thesis is about."* Then scroll back through the `bt_executor` log and walk them through the transitions.

### Known fragilities
Groot2 needs both ports 1667 **and** 1668 free. The tree must stay under 20 nodes for live monitoring. After any `colcon build`, every sourced terminal needs `source install/setup.bash` or it silently runs the old binary. `AgentHealthy`'s 2 s timeout is roughly 4× the heartbeat period — tightening it risks false positives from scheduling jitter.

---

## 3. Deferred work and integration plan

| Skipped | Why | How it comes back |
|---|---|---|
| **PlotJuggler traces** | second visualisation tool to configure under time pressure; Groot2 plus terminals carry the story | For the write-up, not the demo. `RosStatusLogger` (written, optional) publishes each node's status as `/bt/status/<node>`; subscribing to those plus `/joint_states` gives a time-aligned figure showing condition→halt→retreat on one axis. |
| **`ros2 bag` + SQL analysis** | adds nothing an audience sees | The first quantitative result. `bt_executor` already writes `/tmp/bt_trace.db3` on every run. Query the interval between the health condition reporting FAILURE and `safe_hold` entering RUNNING, over ~10 runs, and report mean and spread. This converts "the fallback works" into a measured latency. |
| **Mode B / custom messages** | every `.msg` edit costs a 30–60 min Docker rebuild | Built and verified, then shelved. Only needed if Isaac Sim's *own process* must serialize a custom type. Custom messages between my own nodes need only the normal build. Re-enter Mode B only if an OmniGraph node or in-simulator `rclpy` script requires it. |
| **Real agents (Telekinesis)** | orchestration layer had to be validated first | Next milestone. It is Python and manipulation-oriented (UR10e, 6D pose, motion planning) with its own Isaac Sim extension. Plan: run each skill as an ordinary Python 3.10 node *outside* Isaac Sim and let DDS carry the traffic — a process boundary rather than a shared interpreter. Substitution should require no change to the tree. |
| **Real hardware** | out of scope for this milestone | The interface is `/joint_command` + `/joint_states`. If the real arm speaks the same topics, this is a remap, not a rewrite. |

### Why stubs first (defensible, not an apology)
Stub agents make failure modes *triggerable on demand* that a real model produces only rarely and unpredictably. Building the fault taxonomy against stubs is what makes it testable; validating the orchestration layer against a real model first would conflate orchestration bugs with model behaviour.

### Open questions to carry forward
- **Fault granularity.** Every fault currently halts everything. Real systems need partial degradation — a perception failure might permit a reduced-capability mode rather than a full stop. Likely means grouping agents into subtrees by criticality.
- **Scaling.** `AgentHealthy` as one condition per agent stays readable to roughly a dozen agents. Beyond that the safety branch needs structure.
- **Orchestrator failure.** Currently out of scope; the drive controller holding its last commanded position is the only backstop. Orchestrator redundancy is a deployment-layer concern and a natural later chapter.
- **Timing semantics.** The tree runs on sim time; the Python agents run on wall clock. Fine at present because heartbeat cadence need not track sim time, but this must be revisited before any timing claim is made.

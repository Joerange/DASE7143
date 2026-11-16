# Runtime contract v1: online SLAM and explicit runtime authority

Application messages use `std_msgs/String` with JSON `schema=1`. `stamp` is ROS seconds; observations retain their acquisition timestamp. Freshness checks combine that timestamp with local monotonic receipt time. Geometry uses meters, radians, `map` and `base_link`. All processes must use consistent ROS time and the same ROS domain. The installed console service defaults to domain 180 and `ROS_LOCALHOST_ONLY=1`; static integration uses a separate domain 179.

Mission/perception envelopes carry `task_generation` (mission UUID) and `geometry_epoch` (integer). Mission owns both; VLM output cannot assign either or replace request/track identities. Console lease envelopes instead use backend `session_id`. No message or service acceptance alone proves physical standstill.

## Ownership and startup

- `task.launch.py enable_navigation:=true` starts one asynchronous SLAM Toolbox plus Nav2. No `map` argument or saved map is required; the task has no fixed-map branch and starts neither AMCL nor map_server.
- SLAM alone owns `/map` and dynamic `map→odom`, subscribes `/scan`, and remaps its real matched pose output to `/slam_toolbox/pose`. The chassis owns real `/odom` and dynamic `odom→base_link`. Static sensor extrinsics remain separate.
- Nav2's global static layer consumes the live map and subscribes to updates. Unknown space stays prohibited (`allow_unknown=false`), realtime obstacle layers and the existing footprint remain enabled. Mission independently checks the complete footprint and remaining path against current known-free cells. Nav recovery may wait/replan; no blind spin or reverse.
- `allow_runtime_control=false` is the default and sole capability gate: guard publishes zero on `/cmd_vel_guard_preview`, creates no `/cmd_vel` publisher, and rejects enable requests.
- `allow_runtime_control=true` creates the final `/cmd_vel` publisher but starts disarmed, continuously outputting zero at 50 Hz. It is a capability, not authorization. Legacy `motion_enabled:=true` and `start_chassis:=true` task-launch shortcuts are rejected. Task launch never configures CAN or starts a chassis. `autostart` cannot grant motion or start a task.
- Global authority always starts false after guard/backend restart and cannot be restored from parameters or files. Explicit authority and an additional manual task start are required.

## Task state and observations

`/mission/control_state` publishes compact control state at 20 Hz; `/mission/state` publishes full display geometry at up to 5 Hz:

- `state`: `INIT`, `SEARCH_CONTINUOUS`, `ACQUIRE`, `CONFIRM_PENDING`, `WAIT_PATH`, `READY_TO_NAV`, `NAVIGATE`, `APPROACH`, `REOBSERVE_RECOVERY`, `VERIFY_STOP`, `DONE`, `HOLD`.
- `active` bool, `reason`, `source` (`none`, `search`, `nav`), `control_epoch` integer, `task_generation`, `geometry_epoch`, `motion_enabled` bool derived from fresh independent guard authorization.
- `target_track_id` (string or empty), `target_label`, `coverage_radians`, `reconfirm_required`, `map_revision`, `localization_mode='slam'`.
- `nav_ready` bool from fresh active states of planner/controller/behavior/BT/smoother lifecycle nodes; `navigation_idle` true only with no pending goal, active goal or pending cancellation.
- Read-only geometry: `pose`, `goal`, `path`, `target`, `planning_tracks`, `validated_paths`; `waiting_for` is `depth`, `known_free_path`, or empty. `request_id` and `observation_id` are the actual pending batch IDs, or null when none; IDs are not fabricated for timeline display.

`/mission/control_state` excludes path/target/pose/goal arrays and includes `decision_stamp`. Its transport stamp may advance independently, but the guard ages the actual decision stamp (1.5 s) and receipt separately, so a frozen mission cannot be renewed by heartbeat replay. The independent final guard subscribes only to this compact control channel.

`source=search` only for `SEARCH_CONTINUOUS`/`ACQUIRE`; ACQUIRE commands zero while confirmation/planning continue. `source=nav` covers NAVIGATE and APPROACH with the same control epoch. Other states use none. `control_epoch` changes whenever source changes and clears cached velocity commands. INIT is inactive before manual start; after start it is active but source none while new-generation readiness arrives. HOLD and DONE are inactive.

`/perception/candidates`: `observation_id`, `task_generation`, `geometry_epoch`, `candidates[{id:int, track_id:string, bbox:[x1,y1,x2,y2], score, label, stable:bool}]`, `ready`, `processing_hz`. Continuous, including empty frames. Track IDs are not reused within a generation.

`/perception/candidate_event`: the same context plus unique `request_id`, at most six candidates, annotated `rgb_jpeg` and `crops[{id,jpeg}]`. One immutable observed batch. No candidate means no cloud request. VLM consumes only this event topic.

`/perception/confirmation`: matching observation/request/generation/geometry context, `candidates[{id,track_id,status}]` with status `match`, `no_match`, or `uncertain`, `valid`, `error`, `elapsed`, and optional retry metadata. One worker and an eight-second request deadline. Local validation binds IDs to the submitted batch; model text never supplies trusted envelope IDs. Old, duplicate, mismatched or expired answers cannot authorize a target.

`/target/estimates`: observation context plus `targets[{track_id,valid,reason,map_point:[x,y,z],base_point:[x,y,z],gap,bearing,quality,spread,stamp}]`. `gap=base_point.x-0.345` is physical front-to-visible-surface distance, distinct from optical-axis depth Z. Publishes current estimates for all visible tracks and explicit invalid estimates. No reuse of stale depth as a new observation.

Search may end as soon as identity, fresh RGB-D geometry, actual stop and a known-free path are accepted. A full rotation and a complete saved map are not prerequisites. A confirmed target without a reliable path stays in WAIT_PATH with source none. There is no Option B exploration.

## Online-map changes and recovery

Before Nav2 dispatch, valid map revisions recheck cached docking poses and paths in world coordinates. Checks use the oriented 0.690 x 0.570 m rectangular body plus 0.05 m padding and intersecting cells, including segment and turn sampling. The old circumscribed-disk requirement has been removed from mission path acceptance. The current pose connects to the closest path segment, not a previous waypoint.

At the operator's request (confirmed clear corridor), an accepted Nav2 action owns live replanning: ordinary occupancy/origin/size updates no longer cancel NAVIGATE/APPROACH through a duplicate mission-level whole-route check. The mission tracks /plan updates with matching frame, goal endpoint and fresh stamp. Nav2 continues to use the live map, allow_unknown=false and obstacle costmaps; the final velocity guard remains active. A pending, not-yet-accepted docking path can still be invalidated and reobserved. Invalid maps, map resets and localization jumps still revoke the mission motion source and cancel navigation before reobservation. This operator revision supersedes the prior rule that every blocked cached active route automatically entered REOBSERVE_RECOVERY.

Resolution/origin-orientation changes, map load-epoch changes, major loss of known world-space support or map publisher replacement are reset evidence. Significant map→odom correction also invalidates old coordinates. These rotate `task_generation`, increment `geometry_epoch`, clear old observations/plans and require fresh candidate confirmation. In locked-map navigation, subsequent detector geometry conflicts do not change the frozen target or trigger recovery. Callback results with obsolete context cannot restore navigation. Recovery is bounded; timeout leads to HOLD. Multiple map publishers are rejected.

Runtime sensor/SLAM health loss revokes the global authority latch as well as stopping task motion. Once that latch is false, healthy inputs alone cannot resume navigation: the operator must explicitly authorize and start again.

## Independent final velocity guard

`/cmd_vel_search` is mission `geometry_msgs/Twist`, 20 Hz only while source search. Nav2 raw output is `/cmd_vel_nav`; smoother output is `/cmd_vel_nav_smoothed`. The guard is the sole final `/cmd_vel` publisher in runtime-capable mode. A smoother cannot refresh the original raw-command receipt. Raw command deadline is 0.20 s. Source/epoch changes clear all caches and require a subsequent fresh command. UI/backend/helper do not publish velocity.

`/approach/speed_limit`: `stamp`, `task_generation`, `control_epoch`, `max_linear`, `valid`. Mission publishes at 20 Hz during nav states, plus `nav2_msgs/SpeedLimit` on `/speed_limit` with `percentage=false`. Navigation uses distance to the currently accepted map target and current odometry, independently recomputed by the guard. The upstream planner envelope is at least 0.075 m/s so a linear stop does not prevent alignment-turn selection; the guard enforces the exact lower linear cap, including zero. Invalid limits use a positive epsilon upstream (zero means no limit), and the guard outputs zero. Cruise limit is 0.25 m/s, navigation angular limit 0.30 rad/s, near-zone cap 0.10 m/s and terminal cap 0.10 m/s within 0.45 m, before braking-distance reduction; search remains at most 0.15 rad/s. The approach cap is raised to the terminal cap when needed to prevent acceleration at the terminal-zone boundary.

The guard runs an independent steady-clock 50 Hz timer. It checks current mission identity/source, command age, LiDAR sector coverage, actual velocity, locked target context/current pose and minimum gap. Its additional LiDAR/depth braking sweep is configurable and disabled for search/navigation in the current operator profile. require_depth=false means a bad/missing camera frame cannot revoke global permission; depth remains available for display and initial target locking still requires genuine RGB-D geometry. Front/half-width are 0.345/0.285 m with 0.080 m clearance when the optional sweep is enabled; reverse is disabled. Invalid required inputs clear commands and output zero.

SLAM-mode health requires:

- Valid nonempty `/map` in map frame with known free cells; default acquisition/receipt deadline 5 s.
- Fresh valid `/scan` (task profile: 2.5 s), real `/odom` and odometry pose (0.6 s). Depth is optional in the current task profile; its measured freshness remains reported separately.
- Actual dynamic `/tf` receipts for map→odom and odom→base_link, valid transforms, a connected map→base_link lookup, and current transform header stamps. Task dynamic receipt/odom TF deadline is 0.6 s; SLAM map TF header age may be up to 2.8 s, with bounded 0.35 s future tolerance for Toolbox's transform offset. A successful old-buffer lookup or static transform cannot satisfy this requirement.
- A real accepted `/slam_toolbox/pose`, associated with its acquisition-time odometry. If that pose is older than 3.5 s, current fresh odometry must remain within 0.08 m and 0.10 rad of the accepted anchor; otherwise `slam_pose_not_progressing` blocks motion. Stationary pose persistence does not waive fresh map/scan/odom/TF requirements.

No AMCL pose or covariance is fabricated. The guard's standalone legacy `localization_mode=amcl` health branch still checks real `/amcl_pose` age and finite nonnegative covariance against limits. This compatibility branch is not a task-launch fixed-map mode.

`/safety/status` is published periodically (about 20 Hz) and on reason changes:

- `schema`, `stamp`, `task_generation`, `geometry_epoch`, `ready`, `stopped`, `reason`, `output_topic`, `localization_mode`.
- `authorized`, `authorization_reason`, `runtime_control_capable`, `authority_session`, `control_session`, `output_zero`.
- `sensor_health`: scan/depth/odom, plus map/tf/slam_pose_seen in SLAM mode. `slam_pose_seen` alone is not localization readiness; use fresh `ready` and the reason.

`ready` describes sensor/localization health, not runtime permission or complete Nav2 readiness. `authorized` is the independent global permission latch. `output_zero` reports the most recently published final command, not physical speed. `stopped` also depends on healthy measurements and measured |v|<0.015 m/s, |w|<0.025 rad/s; one status sample is not the 0.7 s stop proof. Clients must reject stale status, not retain an old authorized display as evidence.

## Runtime authorization services and console lease

`/safety/set_motion_enabled` (`std_srvs/SetBool`):

- `data=false` always accepts disarming, even in preview mode or with camera/map/network failure. It immediately revokes the latch, clears commands, outputs zero on the configured output and asynchronously requests `/mission/stop`, retrying without blocking the guard timer.
- `data=true` requires runtime capability, fresh matching backend heartbeat and chassis health, current mission state with `nav_ready=true`, healthy sensors/SLAM and an actually stopped robot with task source none. Otherwise it denies with a reason. An already-authorized valid same-session request is idempotent and does not start a task.

`/console/control_heartbeat`: `{schema:1, stamp, session_id}`. `/chassis/health`: `{schema:1, stamp, session_id, communication_ok:bool, driver_running:bool, can_interface:string, can_up:bool}` plus read-only feedback/velocity diagnostics. The independent ordinary-user control backend publishes these at 20 Hz; read-only monitor mode does not. Task-launch deadlines are 1.0 s, using acquisition and receipt times. Duplicate/older stamps cannot renew freshness. Backend session changes retire the old session and revoke authority.

`communication_ok` requires interface UP, running driver, live passive CAN feedback and fresh `/bunker_status` and odometry. Interface state alone is insufficient. `authority_session` is `guard_boot_uuid:revision:backend_session_id`; guard restart changes the boot ID. Mission rejects retired guard boots, regressed revisions, stale/replayed stamps and invalid bools.

Control heartbeat/chassis/mission/Nav2/SLAM health failure revokes authority, forces zero and requests task stop. Target geometry and overspeed faults also revoke it. Ordinary obstacle-in-sweep detections immediately output zero but retain the operator permission; the mission remains parked until a clear sweep, actual standstill and a fresh command allow progression. Reconnection never silently rearms. Final-guard process death itself must be covered by the chassis hardware communication watchdog; software cannot publish after process death.

`/mission/start` (`std_srvs/Trigger`) requires fresh guard authorization, current-generation/geometry guard readiness, Nav2 ready and no active task/pending navigation cancellation. It generates a new task UUID, clears prior readiness and waits for new-context inputs in INIT. A global guard lease is independent of mission UUID; per-task safety readiness remains strictly bound to UUID/geometry.

`/mission/stop` (`std_srvs/Trigger`) holds the task, revokes its source, publishes inactive state and requests navigation cancellation. It does not by itself revoke the global authority latch. The console stop/disarm flow requests both services, then confirms resulting state. Service acknowledgment is not stop confirmation.

## Docking and stop evidence

The current profile targets a 0.10 m front-to-visible-surface gap. DONE requires fresh localization within 0.02 m and 0.10 rad of the accepted docking pose, estimated map gap 0.08–0.12 m, no pending goal update, navigation terminal/cancellation complete, and measured odometry standstill continuously for at least 0.7 s. When a current, untruncated, associated RGB-D observation is available during arrival, it must also pass the gap/bearing band, 0.03 m lateral-centering tolerance and fitted visible-face normal alignment when available. completion_evidence=locked_map_only explicitly means physical front gap was not remeasured; rgbd_and_map requires the fresh matching observation. Arrival freezes further corrections; a failed pose/standstill interval cannot restart motion. The independent minimum gap is 0.06 m, the braking envelope reaches zero at 0.08 m, and there is no automatic reversing.

Console stop success independently requires fresh disarmed guard status with output_zero, fresh inactive mission source none with navigation_idle, and fresh measured odometry below the same velocity limits for 0.7 s. Controlled driver shutdown/CAN shutdown additionally require live passive CAN standstill evidence and driver ownership. Timeout is unknown/failed, never “stopped”.

## Native console and privilege boundary

`./scripts/run_console.sh monitor --fullscreen` opens the native Qt window and connects to an existing backend or creates a monitor-only backend. Default endpoint is `/run/user/<uid>/robot-console.sock`, same-UID Unix clients only. Rendering runs separately from ROS supervision. Starting the window never starts sensors, CAN, chassis, authority or mission. `--demo --fullscreen` is explicitly SIMULATED and does not connect to ROS/control/CAN.

The four views show time-matched RGB detections, optical depth distinct from front gap, base-frame LiDAR and the current map origin/resolution with robot/target/docking/path overlays. Freshness states are waiting/normal/stale/disconnected. Missing real speed is unknown, not zero. Disconnect disables unsafe actions; reconnect reads actual backend state. Window close and Esc are display operations, not stop actions.

Backend actions are `can_enable`, `can_disable`, `driver_start`, `driver_stop`, `arm`, `disarm`, `mission_start`, `mission_stop`, `export`. Requests return operation IDs; result is pending/success/failed/unknown with phase/message. Snapshot controls describe available actions, not proof that a prior click succeeded. Stop requests preempt other operations and do not wait for rendering/cloud calls.

Only the separately reviewed privileged helper can configure fixed can0/500000. It authenticates the configured ordinary-user control service's actual systemd PID; the UI cannot submit arbitrary commands or trusted stop flags. `prepare_down`/single-use `commit_down` require the trusted backend's stop-and-driver-exit sequence. Backend/helper/window restart does not automatically cut CAN. See `console_service/HELPER_PROTOCOL.txt` for the exact protocol and installation boundary. Installation/startup is a separate explicitly authorized operation.

Console journal/export whitelists task/context/operation metadata and excludes credentials, images, API request bodies and prompts. Existing perception keyframe files are managed independently. Detailed usage and operator wording are in [CONSOLE.txt](CONSOLE.txt).

## Validation boundary

Five packages are built together: robot_perception, robot_mission, robot_safety, robot_bringup and robot_console. Core software tests can run without ROS using their source paths; ROS adapter and Qt tests require their actual dependencies. Do not report skipped tests as passed.

`scripts/verify_online_slam.py` uses isolated domain 179, real live sensor inputs and actual Toolbox outputs. Its explicitly TEST-ONLY zero-odometry/dynamic-odom-TF fixture proves integration only. Controllers/behaviors/smoother remain inactive; only ComputePathToPose is requested. It does not start a chassis, grant authority, submit a navigation motion goal, or configure CAN. An already-UP interface requires the explicit `--allow-preexisting-can-up` option and no driver, preserves configuration and does not repeat hardware baseline tests. `verify_nav_config.py` aliases this online check.

Software, static ROS integration and real vehicle navigation are separate acceptance layers. Current evidence and remaining vehicle checks belong in [VALIDATION.txt](../robot_console_20261109/VALIDATION.txt). No synthetic map, odometry, console demo or passing planner result establishes actual localization accuracy, braking performance, watchdog behavior or measured docking distance.

## Managed desktop session (2026-11-16 operator revision)

Running scripts/start_robot.sh explicitly requests automatic CAN and supervised driver preparation and one motion grant after continuous readiness. It never calls /mission/start; the operator starts the task in the window. A subsequent fault remains latched, without automatic regrant or mission restart. Standalone monitoring/demo retains its read-only behavior.

The foreground launcher owns the desktop lifecycle. Managed-window close, Ctrl+C and terminal HUP stop the mission, disarm, shut down the owned driver, then the task process group and both fixed system services. scripts/stop_robot.sh --all exposes the same sequence over SSH. No arbitrary processes or services are stopped. CAN configuration remains intact; unresolved stop evidence leaves protection running and reports failure. A root-owned sudoers entry permits only the exact start/stop/restart command lines for robot-console-can.service and robot-console.service, never arbitrary systemctl arguments or shell commands.

The backend uses a separate callback group for control/odometry and serializes immutable map snapshots outside the shared control lock. Decision heartbeats use a separate mission executor group. Depth processing keeps only the latest frame on a worker and commits its original stamp/receipt atomically. This revision was deployed without regression or vehicle tests at the operator's request.

`/mission/safety_stop` (Trigger) is the independent guard's cancellation endpoint. It records a safety-origin HOLD and resolves the reason from guard authorization status; operator tools retain `/mission/stop`. Guard revocations are persisted as MOTION_REVOKED lines in the task log, including source and freshness ages. A normal control-backend shutdown automatically archives its redacted event journal to console_exports/events_session_*.json. These recording changes do not remove motion interlocks or establish vehicle acceptance.

## Search collision handling update (2026-11-16)

The recorded search interruption was guard:obstacle_in_braking_sweep, with fresh mission/TF and complete scan sectors. Where sweep checking applies, the guard collision-checks the acceleration-limited command it will actually issue together with measured velocity, retaining the full footprint and clearance. Depth removes only the known 0.690 x 0.570 x 0.335 m chassis volume, matching LiDAR; other returns remain in obstacle inputs. Ordinary collision blocking retains authorization and emits zero, with motion_blocked=true and collision={sensor,point,time_ahead,velocity,footprint}; it does not bypass the standstill/rearm barrier. MOTION_BLOCKED details are saved to the task log at most every two seconds. Health, command, identity, localization and minimum-gap checks remain active.

At the operator's request, the task profile sets search_sweep_enabled=false. Only source=search with a zero translation command (within 1e-6 m/s) and measured |linear velocity|<0.015 m/s skips the LiDAR/depth braking sweep, including its starting-footprint collision test. The operator checks the surrounding rotation space before starting. Navigation/approach, including pure navigation turns, and measured translation retain full collision checking. Scan coverage, sensor/TF freshness, speed/acceleration limits, lease and manual start remain unchanged. The standalone Config default remains true; /safety/status.search_sweep_enabled reports the loaded value. This is a launch-time setting and requires restarting the task stack. No tests were run for this operator-requested revision.

lidar_filter_node is the single /scan publisher. It uses a depth-one best-effort raw-cloud subscription and directly projects the transformed, height/body-filtered newest cloud. It retains acquisition stamps, range limits and unknown Inf rays; /perception/lidar_filtered is published on demand for viewers. No separate pointcloud_to_laserscan process is launched by sensors.launch.py. This avoids the old depth-five backlog, hidden 0.4s upstream expiry and a second cloud serialization/projection hop. No regression or vehicle tests were run at the operator's request.

## Live navigation and LiDAR scheduling revision (2026-11-16)

The task profile now permits scan acquisition/receipt age up to 2.5 s, raw-cloud processing age up to 1.5 s, map-to-odom header age up to 2.8 s, and SLAM pose age up to 3.5 s. Mission map-to-base and Nav2 controller/costmap transform tolerances are 2.8 s; dynamic TF receipts and odom-to-base freshness remain at 0.6 s. Nav2 scan observation intervals match 2.5 s. Mission plan validity is 10 s and /plan telemetry age limit is 3 s. Expired/disconnected sources still stop the guard; neither old frames nor TF headers are restamped to simulate freshness. The braking prediction includes actual scan age plus command delay when it exceeds the configured reaction time. Console LiDAR freshness follows the live guard's reported limit.

Raw pointcloud callbacks only replace a single pending frame. A worker performs capture-time TF/body filtering and projection; elementwise coordinate transforms avoid launching a BLAS pool for Nx3 multiplication. /perception/lidar_health reports receive/publish/drop/TF counters and acquisition/receipt ages, also exposed in the console. LIDAR_PIPELINE log records distinguish missing raw input from delayed projection. Original stamps, unknown rays and body geometry are retained. No tests or vehicle commands were run for this revision; deployment compilation is not navigation acceptance.

## Fixed target and speed revision (2026-11-16)

The operator requests driving to the first confirmed, depth-localized and planned docking point even if the bin disappears from the image. mission.lock_target_on_navigation=true and guard.locked_target_navigation=true select this mode. /mission/control_state carries navigation_mode=locked_map and locked_target={track_id,map_point,goal,observation_stamp}; the original observation timestamp is retained. The guard freezes this anchor per task/geometry/track context and rejects changes within the same context. A new detector track, position drift, near-zone visual loss or semantic-record age no longer cancels the accepted action. The task deadline and actual Nav2 failures still apply.

Both mission and guard compose the last valid map→odom alignment with fresh odom→base_link for current map pose, preserving the real odometry acquisition stamp and dynamic-TF freshness checks. locked_geometry telemetry is explicitly a map estimate, separate from /target/estimates and measured RGB-D depth. The guard independently derives its range limit from the frozen anchor. No old image or inferred map range is republished as new camera evidence. Global permission is not revoked by depth_invalid/depth_stale in this profile; chassis/control-session loss, required LiDAR/localization failure and explicit stop/disarm still stop motion. Automatic regrant is not introduced.

Nav2 controller, smoother, mission cap and final guard all use 0.25 m/s maximum linear velocity and 0.30 rad/s maximum navigation angular velocity. Near-zone cap is 0.075 m/s with the original braking envelope; acceleration and braking parameters remain unchanged. This revision is deployed without running tests or starting a mission, as requested.

The operator confirms the transit corridor has no obstacles and requests no additional avoidance layers. The task profile therefore sets navigation_sweep_enabled=false: during source=nav, the final guard omits its duplicate LiDAR/depth trajectory-sweep test. Nav2 retains its existing obstacle layers, footprint checks and local planner. The guard continues enforcing fresh commands, runtime authorization, LiDAR/TF/odometry health, velocity limits and locked-target range limits. /safety/status reports the actual search/navigation sweep settings. This supersedes the earlier task-profile statement that all navigation retained the extra sweep; standalone Config defaults still enable it. The target coordinates remain frozen while live map/robot localization continues to update.

Once a target is locked, a map-frame reset or significant localization jump stops the run with HOLD and retains the original anchor for display. It does not automatically reconfirm and replace the goal; a new target lock requires a new manual task start. The earlier automatic reobservation recovery still applies before target lock and in legacy visual mode.

## 10 cm docking with visual refinement (latest operator revision)

This revision supersedes the frozen-coordinate and 0.55–0.65 m profile above. lock_target_on_navigation retains the confirmed object identity and last accepted geometry, while visual_target_refinement=true selects navigation_mode=adaptive_map. YOLO detections are tied to real RGB-D observations. A stable same-frame, unique spatial match can associate a replacement tracker ID to the same confirmed bin; ambiguous association, invalid depth, clipped boxes and position outliers leave the previous goal intact.

Two distinct, consistent observations are required. A 0.35 filter factor, at most 0.08 m target displacement and 0.10 rad yaw change per proposal, a small deadband and a 1 s update interval bound corrections. The visible depth cluster supplies an optional vertical-face normal from a covariance fit; no unseen bin orientation is invented. Software depth admission is reduced to 0.15 m, but missing hardware depth remains invalid. The revised docking pose is the observed surface center minus the 0.345 m front offset and 0.10 m desired gap along its approach orientation. Its footprint must remain in known free map cells.

Updates use NavigateToPose preemption with the same default behavior tree, as supported by [Humble's navigator](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp). Mission commits the new anchor only after action acceptance. Results belonging to preempted goals cannot abort the new goal, and stop/disarm during a pending update cancels the newly accepted handle. Goal requests that exceed their response deadline stop with an explicit reason. No control epoch or global motion regrant is needed for a normal target correction.

locked_target now includes target_revision. The guard accepts strictly newer observations/revisions only in adaptive_map mode with allow_target_refinement=true and bounded anchor/goal/yaw changes; equal revisions cannot mutate coordinates. Its own current-pose range check and fresh selected-target minimum-gap check enforce 0.06 m without revoking permission for ordinary proximity stopping. Nav2 goal tolerances are 0.015 m/0.08 rad with a non-latching XY check. UI and redacted exports include target_revision, observed_target_track_id and refinement_status; camera measurements and map estimates remain separate. This revision is built/deployed without tests or vehicle operation at the operator's request; actual 10 cm accuracy and front-face alignment remain vehicle acceptance items.

## Sparse LiDAR noise and pre-navigation diagnostics

The 2026-11-16 run confirmed both candidates as match, but repeatedly alternated missing_map_tf and valid target geometry; it never accepted a navigation goal. Nav2 additionally reported stale /scan observation buffers while the guard still received fresh scans. This is not evidence of a physical obstacle in the transit corridor.

Before producing /scan and /perception/lidar_filtered, reject isolated XY returns with fewer than three points in the surrounding 3x3 neighborhood of 5 cm voxels, unless a return is also supported by the preceding cloud in odometry-fixed XY coordinates within 0.35 s. The repeat check uses capture-time odometry and does not retain old points as new observations. Clusters are admitted immediately; sparse repeated returns are retained. No filtered/no-return ray is relabeled as free space. Existing map cells and the confirmed bin itself are never blindly cleared.

SLAM and the guard continue consuming /scan. Both Nav2 obstacle layers now consume the same filtered measured PointCloud2 directly, with explicit per-source height bounds, marking and raytracing. Humble's [obstacle layer](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp) adds a 50 ms future-transform gate to LaserScan subscriptions; its PointCloud2 path does not add that gate or re-project a synthetic scan. Acquisition timestamps, TF checks, existing footprint/padding, unknown-space blocking and speed settings are retained. This input change addresses a potential contributor to the observed buffer stalls; live effectiveness remains untested.

The perception worker retries the original image-time map transform for at most 150 ms within the existing 0.5 s frame deadline; no ROS callback waits, no newer transform is substituted, and missing transforms remain invalid. Mission planning_status now identifies missing geometry, rejected docking cells (including cell coordinates/occupancy), pending/rejected/timed-out planner requests and invalid paths. Console status and exported events carry those actual reasons. No tests or hardware operation were run for this revision.

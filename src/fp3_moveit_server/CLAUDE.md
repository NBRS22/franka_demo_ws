# fp3_moveit_server

Sole owner of the FP3 arm's `move_group` bringup. Every other package in this
workspace controls the arm exclusively through this package's public action
(`mtc_pick`) -- nothing else is allowed to hold a `MoveGroupInterface` or an
MTC `Task`, and nothing else launches `move_group`.

## Référence supprimée : `motion_server_node` (MoveGroupInterface simple)

`motion_server_node` a été retiré du package (fichier `src/motion_server_node.cpp`
supprimé, entrée retirée du router et du launch). Il exposait une action
`move_to_pose` (`MoveToPose.action`) qui déplaçait le bras vers une pose cible
unique via `MoveGroupInterface` — l'approche la plus directe pour un mouvement
point-à-point sans séquençage MTC.

### Pattern MoveGroupInterface (pour référence future)

Si tu veux réécrire un node de mouvement simple basé sur `MoveGroupInterface`
plutôt que MTC, voici les points non-évidents appris en test réel :

**Récupération du robot_description depuis move_group**
```cpp
// Ne pas reprocesser le xacro localement — récupérer depuis /move_group
// pour être sûr d'utiliser exactement le même modèle.
auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "move_group");
param_client->wait_for_service(std::chrono::seconds(10));
auto results = param_client->get_parameters(
    {"robot_description", "robot_description_semantic"}).get();
for (const auto & param : results)
    node->declare_parameter<std::string>(param.get_name(), param.as_string());
// Ensuite seulement : construire MoveGroupInterface
auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, "fp3_arm");
```

**Gate /joint_states avant d'exposer l'action server**
```cpp
// MoveGroupInterface::computeCartesianPath() / plan() appellent
// current_state_monitor qui doit avoir reçu au moins un /joint_states.
// Sans cette gate, un client rapide obtient un IK contre un état vide
// ("Found empty JointState message") et échoue silencieusement.
std::atomic<bool> received{false};
auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [&received](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->name.empty()) received = true;
    });
// spin dans un thread séparé + attendre received pendant max 15s
// SEULEMENT ENSUITE : construire l'action server
```

**Vérification IK/collision avant le mouvement (precheck sans motion)**
```cpp
auto ik_req = std::make_shared<GetPositionIK::Request>();
ik_req->ik_request.group_name     = "fp3_arm";
ik_req->ik_request.pose_stamped   = target_pose;
ik_req->ik_request.avoid_collisions = true;
ik_req->ik_request.timeout        = rclcpp::Duration::from_seconds(1.0);
auto ik_resp = ik_client_->async_send_request(ik_req).get();
if (ik_resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    // Pose inatteignable ou en collision — rejeter avant de bouger
}
```

**Séquence plan/execute**
```cpp
move_group->setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
if (ok) move_group->execute(plan);
// ATTENTION : execute() bloque indéfiniment si le controller ne répond pas.
// Pas de timeout natif — à envelopper dans un thread avec deadline si besoin.
```

**Vitesse limitée par défaut**
```cpp
// Toujours setter explicitement avant le premier plan() —
// le défaut MoveIt est 100% de la vitesse max.
move_group->setMaxVelocityScalingFactor(0.1);
move_group->setMaxAccelerationScalingFactor(0.1);
```

**`MoveGroupInterface` nécessite que le node tourne déjà (executor actif)**
```cpp
// Créer un executor + thread AVANT de construire MoveGroupInterface :
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread spin_thread([&executor]() { executor.spin(); });
// Ensuite seulement :
auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node, "fp3_arm");
```

## Development history (what was built, in order)

1. **Package created from scratch**, replacing the old standalone
   `fp3_motion_server` package (deleted). That package launched its own
   `move_group`; once a second, MTC-based package was going to need
   `move_group` too, two independent instances able to command the same arm
   became a real hazard -- see "Why this package exists" below.
2. **`franka_demo_interfaces` recreated** (it existed before an earlier
   workspace cleanup deleted it): `MoveToPose.action` (ported as-is, plus a
   new `feedback: string status` field it didn't have before) and a new
   `PickObject.action` (goal: grasp candidates + object id/dimensions;
   result: success/message/`used_pose_index`; feedback: `current_stage`).
3. **`scene_setup_node`, `motion_server_node`, `command_router_node` built**
   first (no MTC dependency): table collision object applied once at
   startup, single-pose motion primitive ported from the old
   `fp3_motion_server_node.cpp`, and the router that makes both look like a
   single arm-control endpoint to clients (busy-flag arbitration -- see "Why
   a router" below). `bringup.launch.py` assembled as an adapted copy of
   `franka_fp3_moveit_config/launch/moveit.launch.py` (see "Why `move_group`
   is a copy" below). All three verified with a real end-to-end
   `move_to_pose` run in fake hardware before moving on.
4. **`moveit_task_constructor_core`/`capabilities`/`msgs` installed** via
   `apt` (`ros-jazzy-moveit-task-constructor-*`) -- confirmed absent first
   (`ros2 pkg list`), confirmed available for this exact distro before
   installing.
5. **SRDF facts verified by reading `franka_description`'s actual xacro
   files**, not assumed: arm group `fp3_arm`, hand group `fp3_hand`, end
   effector name `fp3_hand`, TCP/IK link `fp3_hand_tcp`, hand touch links
   `fp3_hand`/`fp3_leftfinger`/`fp3_rightfinger`. Also confirmed live once
   `move_group` was running (`Joint weights for group 'fp3_arm'` in its
   log).
6. **`pick_place_node` built** against the real installed MTC headers
   (`/opt/ros/jazzy/include/moveit/task_constructor/...`), not from memory --
   see "pick_place_node internals" below for the two-phase design (MTC for
   approach, plain action call for the real grasp, MTC again for
   attach/retreat) and why a single MTC Task with a custom gripper stage
   would have been unsafe.
7. **Three real bugs found and fixed by actually running it**, not just
   compiling -- full details in "pick_place_node internals": a
   `PipelinePlanner` parameter-namespace mismatch (crashed on first goal), a
   missing `ComputeIK` property wiring (crashed on first goal), and a
   nonzero-terminal-velocity trajectory rejected by `fp3_arm_controller`
   (silent planning success, execution failure -- fixed via a controller
   parameter override file in this package, franka_ros2_ws untouched).
8. **Gripper phase made testable in fake hardware** via a `simulate_gripper`
   parameter (wired to `use_fake_hardware`), since `franka_gripper_node`
   (the real action server) isn't even launched in fake mode. Confirmed with
   a full live run: all six `pick_object` feedback stages, `SUCCEEDED`,
   correct `used_pose_index`.
9. **Two client demo packages built** (`fp3_apriltag_demo`,
   `fp3_apriltag_mtc_demo`, both outside this package): AprilTag detection ->
   `solvePnP` (corners + `camera_info` + tag size) -> TF transform into
   `fp3_link0` -> a goal on this package's public actions. The first uses
   `move_to_pose`; the second uses `pick_object`. Both surfaced real bugs in
   *this* package (below), found only by actually running them against a
   live camera/tag setup, not by re-reading the code.
10. **Startup-race class of bugs found and fixed**, all via live testing: (a)
    `motion_server_node`/`pick_place_node`'s action servers were
    discoverable the instant the node started, before `move_group`'s
    `current_state_monitor` had received a single `/joint_states` message --
    fixed with an explicit wait-for-valid-`/joint_states` gate before
    constructing the server in both files' `main()`. (b)
    `command_router_node`'s *public* action servers had the identical
    problem one level up: they became discoverable before the *internal*
    `motion_server_node`/`pick_place_node` backends existed, so a fast
    client's `wait_for_server()` on the public action could return true and
    then get its goal flatly rejected. Fixed by gating `command_router_node`'s
    own server construction on both internal action clients being
    `wait_for_action_server()`-ready first.
11. **Table resized to 4m x 4m at `z=0`, centered on the base** (user
    request) -- immediately exposed that `fp3_link0` (the robot's own
    mounting point) now touches the table, which MoveIt correctly flags as a
    collision since nothing tells it that contact is expected. Fixed in
    `scene_setup_node` by adding an `AllowedCollisionMatrix` entry for
    `table` x `table.allowed_touch_links` (default: just the base link).
12. **That ACM fix broke self-collision checking entirely** on the first
    attempt: sending `moveit_msgs::msg::PlanningScene.allowed_collision_matrix`
    with only the 2 new entries doesn't *merge* into the existing matrix --
    it silently *replaces* the whole thing (unlike `world.collision_objects`,
    which really is a proper diff via each object's own ADD/REMOVE
    `operation`). Every SRDF-derived `disable_collisions` pair (hand vs
    fingers, adjacent arm links, etc.) got wiped, so MTC started reporting
    "`fp3_hand` colliding with `fp3_rightfinger`" -- a pair the SRDF
    explicitly disables -- regardless of target pose. Fixed by fetching the
    *current* ACM via `/get_planning_scene` first and extending it (new
    name + new row/column) instead of constructing one from scratch.
13. **`pick_object` extended from pick-only to pick-and-place**, entirely
    inside `pick_place_node` (no new action, no new package): opens the
    gripper before approaching, and after the retreat, plans/executes to a
    fixed `place.pose_xyz`, detaches the object, and opens the gripper again
    to release -- all gripper motion via the real `franka_gripper` actions
    (`Grasp` to close, `Move` to open), never through MoveIt. See
    "pick_place_node internals" below for why gripper actions live in this
    node's own code and not inside an MTC stage.
14. **Grasp verified against the expected object width, not just the
    driver's success flag.** `franka_gripper`'s own `Grasp` action already
    does an epsilon-window check internally, but `closeGripper()` now also
    captures the action's live feedback (`current_width`), and confirms it
    lands within `[grasp.width - tolerance, grasp.width + tolerance]`.
    Catches a `success=true` result that isn't actually holding the
    expected object.
15. **Speed capped at 10% by default everywhere this package plans a
    motion** (`velocity_scaling_factor`/`acceleration_scaling_factor`,
    default `0.1`): `MoveGroupInterface` in `motion_server_node`, and both
    the `PipelinePlanner` and `CartesianPath` solvers in `pick_place_node`.
    A real-hardware test earlier in this session ran at full (unset, 100%)
    speed -- this closes that gap. Override via parameters if a specific
    goal actually needs more.
16. **Width check split into its own step, run strictly after the grasp
    action completes.** Originally the comparison lived inside
    `closeGripper()` itself; refactored into `closeGripper()` (just the
    physical action, returns driver success + final width) and a separate
    `verifyGrasp(width)`, called from `execute()` only after
    `closeGripper()` has returned -- with its own `checking` feedback
    stage between `grasping` and `attaching`, so a client can tell "the
    Grasp action itself failed" (`grasping`) apart from "it reported
    success but the width doesn't match" (`checking`).
17. **That width check was reusing the wrong tolerance and couldn't
    actually catch a miss.** `verifyGrasp()` originally compared against
    `max(grasp.epsilon_inner, grasp.epsilon_outer)` -- but those control
    the *driver's own* force-detection acceptance window, which is
    deliberately wide for compliant objects. Observed live: `grasp.width:
    0.06` with `grasp.epsilon_inner: 0.06` means franka_gripper's own
    window already reaches down to a fully-closed 0m, so a grasp that
    closed on empty air (`final_width=0.0003`) was reported `success=true`
    by the driver itself, and the (reused-tolerance) check rubber-stamped
    it too -- `pick_object` returned `SUCCEEDED` for a pick that never
    grasped anything. Fixed with a new, independent
    `grasp.width_check_tolerance` parameter (default `0.01` m), used only
    by `verifyGrasp()`.

## Why this package exists

Earlier iterations had `fp3_motion_server` launch its own `move_group`, and a
planned MTC-based pick&place package would have launched a second one. Two
`move_group` processes able to command the same physical/simulated arm is a
real hazard (both can send trajectories to the same `ros2_control`
controller). This package collapses that into a single bringup, and grows by
adding action servers, not by adding launch files.

## Node graph

```
        clients (pick_task_node, ...)
                        |
                        v
            command_router_node
              (public: mtc_pick)
                        |
                        v
          /internal/pick_place/mtc_pick
                        |
                        v
               pick_place_node
          (MTC Tasks approach/lift +
           franka_gripper Grasp/Move
           pour chaque action gripper)
                   /         \
                  v           v
            /move_group    franka_gripper_node
         (instance unique,  (open/close via les
          lancée par ce      vraies actions Grasp/Move
          package)           — jamais via MoveIt)
```

`scene_setup_node` tourne une seule fois au démarrage, avant qu'aucun goal
ne puisse arriver, et ajoute la table + le mur à la planning scene via
`/apply_planning_scene`. Il ne fait pas partie du chemin de commande.

## Public action (ce que les clients appellent)

Une seule action sur `command_router_node`, définie dans `franka_demo_interfaces` :

- **`mtc_pick`** (`MtcPick`) : pick complet à partir d'une liste ordonnée de
  poses de grasp (issues de GraspGen via `pick_task_node`).
  Feedback : `filtering` -> `opening` -> `approaching` -> `grasping` ->
  `attaching` -> `lifting`.
  Essaie chaque candidat survivant dans l'ordre via un `MTC Task` dédié
  (`CurrentState` -> `Connect` -> `MoveRelative` approach -> `FixedCartesianPoses`
  \+ `ComputeIK` -> `ModifyPlanningScene`), s'arrête au premier qui planifie,
  ferme le gripper (`franka_gripper/Grasp`), attache l'objet à la planning
  scene, puis lève (`MoveRelative` +Z monde). Renvoie `used_pose_index`.

Clients never call `pick_place_node` directly — it is reachable only under
`/internal/pick_place/mtc_pick`, wired in `bringup.launch.py` via
`remappings`, not enforced by ROS itself.

## Why a router even with a single action

`command_router_node` holds the `busy_` flag and the startup gate (waits for
`pick_place_node` before exposing the public server). Keeping the router
means clients never depend on `pick_place_node`'s name or namespace — adding
a future action type only requires adding it to the router, not changing any
client code.

## Why `move_group` is a copy of `franka_fp3_moveit_config/launch/moveit.launch.py`, not an include

`bringup.launch.py` reconstructs the same `move_group` `Node()` action
in-line instead of `IncludeLaunchDescription`-ing the upstream file. Reason:
MTC's `pick_place_node` will need `move_group` to load the
`move_group/ExecuteTaskSolutionCapability` plugin (so `Task::execute()` has
an `/execute_task_solution` action server to talk to), and the upstream
launch file's `move_group` `Node()` has no argument to inject that. All the
yaml/xacro *content* still comes from `franka_fp3_moveit_config` /
`franka_bringup` / `franka_description` (read via `load_yaml`/xacro `Command`
substitution, same as upstream) -- nothing is copied into this package's
`config/`, so there is no risk of the two configs drifting apart. Only the
Python assembling the `Node()` calls is duplicated.
`franka_ros2_ws` is treated as a read-only external dependency and is never
edited.

## pick_place_node internals worth knowing before touching it

- **Multi-phase execution, not one MTC Task.** MTC stages `compute()` during
  `task.plan()`, *before* `task.execute()`. A custom Stage that called
  `franka_gripper/Grasp` or `Move` directly from `compute()` would
  physically move the gripper while MTC is still exploring/pruning candidate
  branches, not when the chosen solution is actually executed -- a real
  safety bug, not just a style issue. So every gripper action (open before
  approach, close to grasp, open to release) is a **plain `rclcpp_action`
  call** in `pick_place_node`'s own code, run *between* separate MTC Tasks
  (approach, place, retreat), never a stage inside any of them.
- **Startup-race gate in `main()`.** The action server (created inside
  `PickPlaceServer`'s constructor) is not constructed until this node has
  confirmed a valid `/joint_states` message exists -- otherwise MTC's
  `CurrentState` stage can run against `move_group`'s still-empty internal
  robot state. Same pattern, same fix, as `motion_server_node.cpp`. A fast
  client that beats this window sees a *rejected* goal from
  `command_router_node` (which has its own, higher-level version of the
  same gate -- see "Why a router" below), not a silent bad plan.
- **`AllowedCollisionMatrix` diffs must be extended, never replaced.**
  `scene_setup_node` fetches the current ACM via `/get_planning_scene`
  before adding `table`'s allowed-touch-link entries, rather than sending a
  small ACM built from scratch. `moveit_msgs::msg::PlanningScene`'s
  `allowed_collision_matrix` field is NOT merged incrementally by
  `/apply_planning_scene` the way `world.collision_objects` is (each
  `CollisionObject` carries its own ADD/REMOVE `operation`, but
  `AllowedCollisionMatrix` has no such per-entry semantics) -- sending a
  partial matrix silently discards every SRDF-derived `disable_collisions`
  pair. Hit live: `fp3_hand` vs `fp3_rightfinger` (an SRDF-disabled
  "Adjacent" pair) started failing IK checks, unrelated to target pose,
  right after the first (naive) version of this fix shipped.
- **One MTC `Task` per candidate, tried in order**, not a single
  `FixedCartesianPoses` loaded with all candidates. This was a deliberate
  simplification: it makes `used_pose_index` trivial (it's just the loop
  index) instead of needing to reverse-engineer which candidate MTC's
  internal search picked.
- **`PipelinePlanner(node, "move_group")`, not `"move_group", "ompl")`.**
  The pipeline-name argument is a *parameter namespace*, not a pipeline
  label -- it must match whatever namespace the OMPL config dict was loaded
  under for this node. `ompl_planning_pipeline_config` (reused as-is from
  `franka_fp3_moveit_config/launch/moveit.launch.py`) is nested under a
  `'move_group'` key, so that's the namespace to pass, even though the
  actual plugin loaded is `ompl_interface/OMPLPlanner`. Passing `"ompl"`
  compiles fine and fails at the first `task.plan()` call with "Planning
  plugin name is empty or not defined in namespace 'ompl'".
- **`ComputeIK` needs an explicit `configureInitFrom(Stage::INTERFACE,
  {"target_pose"})`.** Without it, MTC throws `Property 'target_pose':
  undefined` the first time `task.plan()` actually runs (not at
  construction) -- `FixedCartesianPoses` does set the property on the
  `InterfaceState` it spawns, but `ComputeIK` doesn't read it from there
  unless told to.
- **`fp3_arm_controller` needs `allow_nonzero_velocity_at_trajectory_end:
  true`.** `CartesianPath`-generated segments (used for the approach/retreat
  `MoveRelative` stages) can leave a tiny nonzero terminal velocity (observed
  ~0.0019 rad/s on one joint) even after explicitly setting
  `TimeOptimalTrajectoryGeneration` as the solver's time parameterization --
  that fix alone did *not* resolve it (same residual, bit-for-bit, before
  and after). `joint_trajectory_controller` rejects such trajectories
  outright by default. Fixed via `config/controller_overrides.yaml`, loaded
  as an extra parameter *file* on `ros2_control_node` in `bringup.launch.py`
  -- has to be a real yaml file, not an inline Python parameter dict:
  controller_manager loads per-controller parameters through its own
  yaml-file mechanism, which silently ignores plain launch parameter dicts
  passed the normal way. No `franka_ros2_ws` file was touched.

## Known limitations / not yet verified live

- **Real hardware pick-and-place confirmed working end-to-end** (previously
  only fake hardware had been exercised): full live run with
  `use_fake_hardware:=false` completed `opening` -> `filtering` -> `planning`
  -> `approaching` -> `grasping` -> `attaching` -> `retreating` -> `placing`
  -> `detaching` -> `releasing`, real gripper open/close via the real
  `franka_gripper` actions, `SUCCEEDED`. `simulate_gripper` (wired to
  `use_fake_hardware`) still exists for fake-hardware testing, since
  `franka_gripper_node` isn't launched there at all in that mode.
- **`place.pose_xyz` is a placeholder**, same convention/caveat as
  `fp3_apriltag_demo`'s old `ready_pose_xyz`: no guarantee it's
  reachable/collision-free in any given scene. Adjust per deployment.
- **Grasp candidate frame assumption.** `filterCandidates()` assumes
  `grasp_candidates` are already expressed in the planning frame
  (`fp3_link0`). Both client demo packages now do this transform themselves
  (camera frame -> `fp3_link0` via TF) before calling `pick_object`, so this
  is handled correctly for AprilTag-based candidates; still an open question
  for any future GraspGen integration, which may emit a different frame.
- **Approach-axis convention assumption.** The geometric prefilter and the
  approach motion both assume a candidate pose's local **+Z axis** is the
  gripper's approach/insertion direction (pointing away from the object,
  toward the gripper). Both client demo packages currently work around this
  by forcing a fixed straight-down orientation on one of their two
  candidates (real tag orientations were observed live to often be
  kinematically unreachable) -- verify against GraspGen's actual output
  convention before relying on the raw candidate orientation for a new
  source.
- `motion_server_node.cpp`'s `execute()` step has no timeout on
  `MoveGroupInterface::execute()` (documented in the source) -- a wedged
  controller blocks that goal indefinitely.
- `pick_place_node`'s cancel handling is best-effort: it's checked between
  phases, but an in-flight MTC `plan()`/`execute()` call or an in-progress
  gripper action is not interrupted -- there's no interrupt hook exposed for
  either, and stopping a grasp mid-close is its own hazard.
- **Retry-on-failure is a client responsibility, not this package's.**
  `pick_object` reports failure once and stops; it does not retry
  internally (it has no camera/tag access to get a fresh candidate anyway).
  `fp3_apriltag_mtc_demo` re-scans and retries client-side, but only when
  the failure happened at the `grasping` or `checking` feedback stage (the
  `Grasp` action itself failed, or it reported success but the achieved
  width didn't match the expected object) -- earlier-stage failures
  (filtering/IK) mean the candidate pose itself was bad, so a rescan just
  repeats it; later-stage failures (after a successful grasp) mean the
  object may still be held, so retrying would mean re-approaching with
  something already in the gripper.

## Running it

```
ros2 launch fp3_moveit_server bringup.launch.py use_fake_hardware:=true
```

Then, from any client package:

```
ros2 action send_goal /move_to_pose franka_demo_interfaces/action/MoveToPose "..."
ros2 action send_goal /pick_object franka_demo_interfaces/action/PickObject "..."
```

Or use one of the two AprilTag client packages, which each bring this
package up themselves and don't need a manually-constructed goal:

```
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py use_fake_hardware:=true
ros2 launch fp3_apriltag_mtc_demo apriltag_pick_once.launch.py use_fake_hardware:=true
```

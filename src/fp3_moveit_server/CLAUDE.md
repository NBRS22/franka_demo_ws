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
  `attaching` -> `lifting` -> `detaching`.
  Essaie chaque candidat survivant dans l'ordre via un `MTC Task` dédié
  (`CurrentState` -> `Connect` -> `MoveRelative` approach -> `FixedCartesianPoses`
  \+ `ComputeIK` -> `ModifyPlanningScene`), s'arrête au premier qui planifie,
  ferme le gripper (`franka_gripper/Grasp`), attache l'objet à la planning
  scene, lève (`MoveRelative` +Z monde), puis détache l'objet de la planning
  scene (`detachObject`, miroir d'`attachObject` — `AttachedCollisionObject`
  + `CollisionObject` en `REMOVE` dans le même diff, pour qu'il disparaisse
  au lieu de réapparaître flottant dans le monde à la dernière pose attachée,
  comportement par défaut de MoveIt sur un simple détachement). Best-effort :
  un échec du détachement ne fait pas échouer le pick (juste un `WARN`), mais
  laisse un objet attaché fantôme qui peut fausser les collisions du prochain
  pick. Renvoie `used_pose_index`.
  Depuis l'ajout de `prioritizeTopDown` (cf. ci-dessous), "dans l'ordre" ne
  veut plus dire "dans l'ordre reçu de GraspGen" mais "par tilt croissant,
  la pose la plus verticale d'abord" — l'ordre brut envoyé par
  `pick_task_node` n'est plus celui réellement essayé.

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
- **`prioritizeTopDown` reorders candidates before the try-loop, without
  dropping any.** Added to force the straightest (most top-down) plannable
  grasp to win, instead of leaving MTC to try candidates in whatever order
  GraspGen happened to return them (which has no notion of verticality --
  cf. root CLAUDE.md, diffusion sampler has no gravity/up conditioning).
  After `filterPoses` (which can be bypassed entirely via
  `filter.enabled=false`, cf. above -- the two are independent steps),
  every surviving candidate is sorted by `approachTiltDeg(pose)` ascending
  (`std::stable_sort`) -- most vertical first, most lateral last. MTC's
  try-loop is unchanged (first that plans wins), so a lateral grasp is only
  ever used as a last resort, once every straighter candidate has failed
  IK/collision.
  **Historique** : la première version (session précédente) faisait un tri
  en deux passes -- un groupe "top-down" (`tilt <= filter.top_down_priority_tilt_deg`,
  30° par défaut) trié par score GraspGen, puis le reste, aussi trié par
  score -- nécessitant de faire transiter `scores` jusqu'à `pick_place_node`
  via un nouveau champ `float64[] scores` sur `MtcPick.action` (toujours
  présent, `goal->scores`, désormais utilisé uniquement par
  `publishFilteredGraspMarkers` pour colorer le meilleur score en vert, plus
  par `prioritizeTopDown`). Remplacé par un tri direct par tilt sur retour
  utilisateur ("commencer avec les poses avec l'angle le plus droit") --
  le score GraspGen n'entre plus du tout dans l'ordre d'essai, seul le tilt
  compte désormais. `filter.top_down_priority_tilt_deg` reste utilisé, mais
  uniquement pour le compte informatif loggé (`Tilt priority: N candidate(s)
  ... M of them <= 30.0deg`), plus comme seuil de regroupement.
  **Non testé en conditions réelles** -- à confirmer au prochain lancement
  que le premier candidat essayé est bien celui au tilt le plus faible du
  lot, et que le fallback vers un grasp plus latéral fonctionne toujours si
  aucun candidat proche de la verticale ne plan.
- **`/pick/executed_grasp_pose` (`geometry_msgs/PoseStamped`) published right
  before the actual grasp motion executes**, not after. Hooked into
  `mtc_tasks.cpp::planAndExecuteApproach` -- the only function that plans
  *and* executes the grasp itself (`task.plan(1)` then, if it succeeded,
  `task.execute(...)`) -- via a new optional `pose_pub` parameter, published
  in the gap between those two calls. `pick_place_node`'s per-candidate
  try-loop only reaches this point for the candidate that actually planned
  successfully (candidates that fail `task.plan()` never reach the publish
  call), so this topic reflects the grasp that is genuinely about to be
  executed, not every candidate merely attempted. Publisher created once in
  `MtcPickServer`'s constructor (`executed_grasp_pose_pub_`), not per-call.

  **`/pick/executed_grasp_marker` (`visualization_msgs/Marker`, single
  ARROW) ajouté juste après** : `/pick/executed_grasp_pose` reste une pose
  brute (pour un consommateur programmatique) -- si affichée dans RViz via
  un display `Pose` en mode `Arrow`, elle montre l'orientation **brute** de
  la pose (axe de fermeture des doigts, +X local), pas la direction
  d'approche, exactement le même piège que celui déjà corrigé sur
  `/pick/grasp_markers` (cf. `graspgen_bridge/CLAUDE.md`). Plutôt que de
  déformer la pose brute publiée sur `executed_grasp_pose` (elle doit rester
  correcte pour un usage programmatique), un second topic dédié à
  l'affichage porte le même traitement que
  `visualize_grasps_node._approach_to_arrow_orientation`/
  `_approach_axis_world` : pointe de flèche exactement sur le point de
  grasp, orientation remappée pour que l'axe que RViz dessine (+X local du
  marker) corresponde à l'axe d'approche réel (+Z local de la pose du
  grasp). Réimplémenté en C++ dans `mtc_tasks.cpp` (fonctions statiques
  `approachToArrowOrientation`/`approachAxisWorld`/`approachArrowMarker`,
  namespace anonyme) plutôt que partagé avec le Python -- formules
  identiques, transcription vérifiée constante par constante contre la
  version Python déjà validée numériquement (2000+ rotations aléatoires,
  cf. `graspgen_bridge/CLAUDE.md`). Couleur ambre/or (`r=1.0,g=0.85,b=0.0`)
  pour se distinguer visuellement du vert/cyan de `/pick/grasp_markers`.
  `visualization_msgs` ajouté comme dépendance explicite de ce package
  (`package.xml`+`CMakeLists.txt`, cible `mtc_tasks` et `pick_place_node`)
  -- absent avant, ne compilait que parce que tiré transitivement par les
  headers MoveIt/MTC, même type de lacune que le `cv_bridge` manquant
  corrigé historiquement dans `sam3_bridge` (cf. CLAUDE.md racine,
  Historique).

  **Non testé en conditions réelles** -- à confirmer au prochain lancement
  qu'un message apparaît sur les deux topics juste avant que le bras ne
  bouge, et que la flèche de `/pick/executed_grasp_marker` a bien sa pointe
  sur le point de grasp avec l'orientation d'approche correcte dans RViz.

  **`/pick/filtered_grasp_markers` (`visualization_msgs/MarkerArray`) ajouté
  juste après** : montre tous les candidats qui ont survécu au filtre
  géométrique (`filterPoses` -- hauteur table / portée / tilt), publié juste
  après ce filtre dans `execute()` (avant que `prioritizeTopDown` ne les
  réordonne -- même ensemble dans les deux cas, seul l'ordre change).
  Rien publié si le filtre est vide (le goal abort de toute façon dans ce
  cas) ou si `filter.enabled=false` bypass entièrement le filtre (alors
  "survécu au filtre" == tous les candidats reçus, publiés tels quels).
  Même convention visuelle que `graspgen_bridge/visualize_grasps_node.py` :
  une flèche `ARROW` par candidat (pointe sur le point de grasp, orientation
  = direction d'approche réelle, via les mêmes fonctions que
  `/pick/executed_grasp_marker`), le meilleur score **parmi ce sous-
  ensemble filtré** en vert opaque, les autres en cyan semi-transparent.
  Les trois fonctions de remap (`approachToArrowOrientation`,
  `approachAxisWorld`, `approachArrowMarker`) ont été sorties du namespace
  anonyme de `mtc_tasks.cpp` vers `mtc_tasks.hpp`/`.cpp` (fonctions non-
  anonymes, `approachArrowMarker` généralisée avec des paramètres
  `ns`/`id`/couleur, défauts = l'usage amber `executed_grasp` d'origine)
  pour être réutilisables ici sans dupliquer le code. **Non testé en
  conditions réelles** -- à confirmer au prochain lancement.
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
18. **Bug corrigé, trouvé en réel : le lift échouait systématiquement juste
    après un grasp réussi**, message `"Lift failed (object remains
    attached)"` (`Found a contact between 'table' ... and 'picked_object'`,
    `move_group` annule toute l'exécution en vol dès le tout premier
    waypoint du lift, avant même que le bras ait bougé — l'objet repose
    encore sur la table au moment de l'attache, exactement là où il vient
    d'être saisi).

    **Premier correctif tenté (session précédente), confirmé sans effet en
    re-testant en réel** : ajouter `"table"` à `attached.touch_links` dans
    `attachObject()`, en supposant (par analogie avec l'usage de
    `table.allowed_touch_links` dans `scene_setup_node`) que `touch_links`
    alimente l'`AllowedCollisionMatrix`. **Faux** — vérifié directement
    contre le source MoveIt (`moveit_core/planning_scene/src/planning_scene.cpp`,
    `processAttachedCollisionObjectMsg`) : `touch_links` est transmis tel
    quel à `RobotState::attachBody()` et ne sert *que* pour l'auto-collision
    du corps attaché contre des **liens du robot** — il ne touche jamais
    `acm_`. Ajouter `"table"` (un `CollisionObject` du monde, pas un lien)
    à cette liste ne fait donc strictement rien ; le bug persistait
    identique au prochain test réel.

    **Vrai correctif** : même technique que `scene_setup_node` pour
    `table`/`wall` (point 11) — récupérer l'ACM courante via
    `/get_planning_scene` (nouveau client `get_scene_client_`), l'étendre
    avec une entrée `(picked_object, table)` autorisée (nouvelle méthode
    `extendAcm`, algorithme identique à celui de `scene_setup_node`,
    dupliqué ici plutôt que partagé entre les deux nodes/exécutables), puis
    l'envoyer dans le même diff que l'attache via
    `diff.allowed_collision_matrix`. Comme pour le point 12 : ne jamais
    construire une ACM depuis zéro, toujours étendre celle en vigueur
    (`PlanningScene.allowed_collision_matrix` envoyée en diff *remplace*
    la matrice entière plutôt que de la fusionner). `attached.touch_links
    = hand_touch_links` reste inchangé et nécessaire (ce mécanisme-là
    fonctionne correctement pour l'auto-collision pince/objet, seul le
    cas "objet vs objet du monde" avait besoin de l'ACM explicite).
    **Non re-testé en conditions réelles après ce second fix** -- à
    confirmer au prochain lancement qu'un pick complet (grasp + lift)
    réussit jusqu'au bout sans que `move_group` n'annule l'exécution.
19. **Bug corrigé, trouvé par l'utilisateur en réel : `move_group` voyait
    toujours la pince fermée pour le check de collision**, peu importe son
    état physique réel -- causant des faux positifs de collision contre
    `table` (approche/plan refusés alors que la vraie pince, ouverte,
    n'aurait pas touché la table) et probablement une partie des vrais
    `cartesian_reflex` observés en conditions réelles (le check plan-time
    d'un côté disait "collision" à tort, tandis qu'à l'exécution le modèle
    fermé pouvait aussi manquer de vraies collisions dans d'autres
    configurations -- les deux symptômes viennent de la même cause : l'état
    des joints de la pince utilisé par MoveIt ne reflète jamais l'état réel).

    Cause : `bringup.launch.py` démarre un `joint_state_publisher`
    (`source_list: ['franka/joint_states', 'fp3_gripper/joint_states']`) qui
    fusionne les joint states du bras et de la pince en un seul topic
    `/joint_states`, celui que `move_group`/`current_state_monitor` écoute
    réellement. Mais `fp3_gripper/joint_states` n'existe pas -- le vrai nom
    du topic publié par `franka_gripper_node` (driver réel *et* le
    stand-in fake-hardware `fake_gripper_state_publisher.py`, tous deux dans
    `franka_ros2_ws/src/franka_gripper/`) est `franka_gripper/joint_states`
    (topic relatif `~/joint_states` du node nommé `franka_gripper`, sans
    namespace). `joint_state_publisher` ne recevait donc jamais aucun
    message pour les joints des doigts, et retombait sur son propre défaut
    interne (`(min+max)/2` si l'intervalle ne contient pas 0, sinon `0` --
    pour `finger_joint1`, `[0.0, 0.04]`, ça retombe sur `0.0` = fermé) publié
    indéfiniment sur `/joint_states`.

    Fixé en corrigeant le nom du topic dans `source_list`
    (`'franka_gripper/joint_states'`). Une ligne. **Non re-testé en
    conditions réelles** -- à confirmer au prochain lancement que l'état
    des doigts dans RViz/la planning scene suit bien l'ouverture/fermeture
    réelle de la pince, et que les faux positifs de collision contre la
    table pendant l'approche (pince ouverte) disparaissent.

## Table height calibrée par contact réel (`scene.yaml`)

`table.position.z` valait `-0.45` (avec `table.dimensions.z = 0.90`, donc surface
déclarée à `Z = -0.45 + 0.90/2 = 0.0` dans `fp3_link0`) -- jamais mesuré, une
estimation. Symptôme observé : le bras touche la vraie table pendant un pick
via la pipeline complète (SAM3/GraspGen), alors qu'un plan MoveIt manuel
(pose choisie à la main dans RViz) ne la touche jamais -- pourtant MTC valide
son plan comme sans collision dans les deux cas. Ça n'a rien à voir avec un
drift des joints (`robot_mode` vérifié `IDLE`, aucune erreur) ni avec la
calibration eye-on-base caméra->robot (celle-ci décale la pose *cible*, pas
la table elle-même) : MoveIt évite correctement *sa* table interne, mais si
cette table interne est mal placée par rapport à la vraie, un plan "propre"
du point de vue de MoveIt peut quand même toucher la vraie table.

**Mesure faite** : bras en gravity compensation (`franka_bringup
example.launch.py ... gravity_compensation_example_controller`), amené à la
main en contact réel avec la table, puis `ros2 run tf2_ros tf2_echo
fp3_link0 fp3_hand_tcp` au moment du contact :
```
Translation: [-0.154, 0.627, 0.034]
```
`Z = 0.034` au contact réel vs `Z = 0.0` déclaré -- écart brut de **3.4cm**.
Nuance découverte en creusant : `fp3_hand_tcp` n'est pas exactement au bout
du doigt -- géométrie de collision du "rubber tip" (`franka_hand.xacro`)
vérifiée à la main : origine du doigt à 58.4mm de `hand` + centre du pad à
45.25mm dans le repère du doigt + demi-hauteur du pad 9.25mm = **112.9mm**
de `hand`, contre **103.4mm** pour `hand_tcp` -- écart de ~9.5mm, le pad
touche donc ~1cm plus loin que ce que lit `hand_tcp`. Avec l'orientation
quasi verticale du contact (roll≈178.5°, pitch≈-3.1°), la vraie hauteur de
table serait plutôt `≈0.034 - 0.0095 ≈ 0.0245`. Cela dit, l'hypothèse de
départ du projet (table au même niveau que `fp3_link0`, `Z=0`) contredit à
la fois la mesure de contact et un "jeu" table réelle/virtuelle déjà
rapporté avant ce chantier -- première décision de l'utilisateur, tranchant
entre ces valeurs : `table.position.z = -0.43` (surface déclarée à
`Z = 0.02`).

**La table n'est en fait pas plane par rapport à `fp3_link0`** -- 3 points de
contact réel mesurés en gravity compensation :

| Point | x | y | z |
|---|---|---|---|
| 1 | 0.137 | 0.769 | 0.041 |
| 2 | 0.161 | 0.495 | 0.032 |
| 3 | -0.062 | 0.597 | 0.036 |

Plan ajusté sur ces 3 points : `z ≈ -0.00306·x + 0.0326·y + 0.0164` -- pente
quasi entièrement en Y (~3.3%, ~1.9°), quasi nulle en X. Sur toute la
largeur Y déclarée de la table (1.40m), ça représente jusqu'à ±2.3cm de
variation de hauteur réelle entre les deux bords -- une seule valeur
`table.position.z` ne peut donc pas être exacte partout (`scene_setup_node`
ne modélise la table qu'à plat, pas de paramètre d'orientation). Fix propre
identifié mais pas implémenté : ajouter `table.orientation` (quaternion,
dérivé de la normale du plan mesuré) et l'appliquer à la primitive `BOX`
dans `scene_setup_node.cpp`.

**Décision finale de l'utilisateur (solution rapide, pas la table inclinée)** :
uniformiser autour du point le plus haut mesuré (0.041), donc le sens sûr --
table virtuelle jamais plus basse que la réalité -- plutôt que modéliser
l'inclinaison. `table.position.z = -0.40` (surface déclarée à
`Z = -0.40 + 0.90/2 = 0.05`).

**Non re-testé en conditions réelles après ce correctif.**

## Known limitations / not yet verified live

- **Real hardware pick-and-place confirmed working end-to-end** (previously
  only fake hardware had been exercised): full live run with
  `use_fake_hardware:=false` completed the sequence, real gripper open/close
  via the real `franka_gripper` actions, `SUCCEEDED`. `simulate_gripper`
  (wired to `use_fake_hardware`) still exists for fake-hardware testing,
  since `franka_gripper_node` isn't launched there at all in that mode.
  **The exact step sequence quoted here (`retreating -> placing -> detaching
  -> releasing`) is stale** — it described an earlier, richer pipeline with
  a real place step that no longer exists in the current code. The current
  `execute()` stops at `lifting -> detaching` (cf. "Public action" above):
  the object is detached from the planning scene right after the lift
  succeeds, nothing is placed/released anywhere yet (no place target, no
  gripper re-open at the end) — that's still roadmap, not implemented.
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

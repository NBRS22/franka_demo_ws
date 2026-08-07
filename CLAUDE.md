# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Vue d'ensemble

Pipeline pick-and-place robotique pour un Franka FR3, orchestré en ROS2 (Jazzy), avec la perception (segmentation, génération de grasp) externalisée dans des serveurs Python séparés (SAM3, GraspGen) accédés via ZMQ+msgpack, et une commande déclenchée par Gemini ER (VLM tournant sur une machine séparée).

Le workspace n'est **pas** un dépôt git (`git init` pas encore fait dans `~/franka_demo_ws`).

## Commandes courantes

### Build

```bash
cd ~/franka_demo_ws
colcon build                                   # tout le workspace
colcon build --packages-select flow_manager    # un seul package
source install/setup.bash
```

`franka_demo_interfaces` est `ament_cmake` (génère les `.srv`) ; tous les autres packages sont `ament_python`.

### Lancer le pipeline complet

```bash
ros2 launch flow_manager flow_manager.launch.py
```

Ce launcher démarre : Realsense D455 (`align_depth.enable:=true`), `camera_bridge`, `camera_buffer_node`, `task_validator_node`, `pointcloud_node`, `grasp_selector_node`, `flow_manager_node`, `sam3_bridge`, `graspgen_bridge`, `command_bridge`. Les serveurs SAM3, GraspGen et Gemini ER (env conda séparés) doivent être lancés **avant**, hors ROS2.

### Lancer un node individuellement (debug)

```bash
ros2 run flow_manager flow_manager_node
ros2 run flow_manager camera_buffer_node
ros2 run sam3_bridge sam3_bridge_node
ros2 run graspgen_bridge graspgen_bridge_node
ros2 run gemini_er_bridge command_bridge_node
ros2 run gemini_er_bridge camera_bridge_node
```

### Serveurs externes (hors colcon, envs conda séparés)

```bash
conda activate SAM3        # serveur SAM3, port 5557, CUDA requis
conda activate GraspGen    # serveur GraspGen, port 5556, CUDA requis
conda activate ER          # Gemini ER / gemini_er_simulator.py, sans sourcer ROS2
```
Ne jamais mélanger ces envs conda avec le Python système utilisé par les nodes ROS2.

### Tests

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
Les tests présents sont uniquement les templates `ament_copyright` / `ament_flake8` / `ament_pep257` — pas de tests fonctionnels sur la logique métier pour l'instant (cf. dette technique plus bas).

### Introspection utile

```bash
ros2 service list
ros2 service call /execute_task franka_demo_interfaces/srv/ExecuteTask \
  "{task_type: 'pick', object_label: 'red mug', point_x: 640, point_y: 360}"
ros2 topic echo /best_grasp_pose
ros2 topic echo /grasp_poses
```

## Architecture

### Flux d'un pick (déclenché par un clic dans `gemini_er_simulator.py`)

```
Realsense D455 → camera_bridge → ZMQ PUB 5555 → Gemini ER
Gemini ER → ZMQ REQ 5558 → command_bridge → service /execute_task
flow_manager (handle_pick, séquentiel et bloquant) :
  1. /validate_task     → task_validator_node
  2. /get_frames        → camera_buffer_node (dernier RGB+depth+camera_info bufferisés)
  3. /segment_object     → sam3_bridge → ZMQ REQ 5557 → serveur SAM3
  4. /fuse_mask_depth    → pointcloud_node (déprojection masque+depth → PointCloud2 via intrinsèques K)
  5. /generate_grasp_pose → graspgen_bridge → ZMQ REQ 5556 → serveur GraspGen
  6. /select_best_grasp  → grasp_selector_node → publie /grasp_poses + /best_grasp_pose (RViz)
  7. MoveIt2 → TODO, pas encore branché
```

Le pipeline `flow_manager_node._handle_pick` est entièrement synchrone (`_call_service` fait `client.call_async` + `spin_until_future_complete` avec timeout par étape). Chaque étape échoue proprement avec `success=False` + message si le service précédent échoue ou timeout.

### Ports ZMQ

| Service | Port | Pattern |
|---|---|---|
| camera_bridge → Gemini ER | 5555 | PUB/SUB |
| GraspGen server | 5556 | REQ/REP |
| SAM3 server | 5557 | REQ/REP |
| command_bridge | 5558 | REP (Gemini ER en REQ) |

`camera_bridge_node` et `command_bridge_node` sont packagés ensemble sous `gemini_er_bridge` (même package Python, `share/gemini_er_bridge`) mais restent deux nodes/exécutables ROS2 distincts, chacun avec son propre `zmq.Context`/socket — pas de serveur ZMQ partagé, donc pas de couplage de blocage entre eux (cf. dette nested spinning ci-dessous, qui ne concerne que `command_bridge_node`).

### Interfaces ROS2 (`franka_demo_interfaces/srv/`)

`ExecuteTask` (command_bridge/flow_manager→flow_manager, et flow_manager→task_validator), `GetFrames` (→camera_buffer), `SegmentObject` (→sam3_bridge), `FuseMaskDepth` (→pointcloud_node), `GenerateGraspPose` (→graspgen_bridge), `SelectBestGrasp` (→grasp_selector).

### Sérialisation ZMQ — piège msgpack / msgpack_numpy

- `sam3_bridge_node.py` : `msgpack.unpackb(raw, raw=False)` → clés `str` (`result["status"]`, `result["mask"]`...). Pas de tableaux numpy dans ce channel (masque transmis en bytes bruts + `mask_shape`).
- `graspgen_bridge_node.py` : `msgpack.unpackb(raw)` → clés `str` aussi (`result["grasps"]`, `result.get("status")`...) depuis un fix appliqué après un vrai `KeyError: b'grasps'` en test réel. **Ancienne note obsolète, gardée en historique** : ce fichier utilisait avant `result[b"grasps"]` (clés bytes), sous l'hypothèse que `msgpack_numpy` (`m.patch()`, utilisé ici pour sérialiser les `ndarray`) avait besoin de `raw=True`/clés bytes pour son hook de décodage. **Vérifié faux avec la version installée sur ce système** (`msgpack==1.2.1`, où `raw=False` est déjà le défaut de `unpackb()` — testé en direct : `msgpack_numpy` reconstruit correctement les `ndarray` même avec des clés `str`). Si `msgpack` est un jour rétrogradé vers une version pré-1.0 (`raw=True` par défaut), cette hypothèse redeviendrait vraie et il faudrait revenir aux clés bytes — vérifier `msgpack.unpackb(msgpack.packb({'a': 1}))` pour trancher.
- Ne pas "harmoniser" ces deux fichiers sans vérifier le comportement réel de la version de `msgpack` installée (cf. point précédent).
- Après un timeout sur un socket ZMQ REQ, il faut fermer et recréer le socket (pattern déjà appliqué dans `_check_sam3_server`/`_check_graspgen_server` au démarrage, **pas** appliqué aux appels normaux — cf. dette technique).
- `CONFLATE` incompatible avec `send_multipart` → utiliser `SNDHWM`/`RCVHWM=1`.
- `setsockopt(zmq.SUBSCRIBE, b"rgb")` avec des `bytes`, pas une `str`.
- `connect("tcp://127.0.0.1:5555")`, pas `localhost`.

### Convention de coordonnées

- Pixels bruts dans la résolution native caméra (1280×720 par défaut D455) — `point_x`/`point_y` transitent tels quels de Gemini ER jusqu'à SAM3, aucune remise à l'échelle nulle part dans le pipeline.
- GraspGen retourne les poses dans `camera_color_optical_frame`.
- MoveIt2 attend des poses dans `fr3_link0` → transform TF2 nécessaire après calibration main-œil (non fait, cf. roadmap).

### Environnements

| Env | Usage | Notes |
|---|---|---|
| Python système Ubuntu | Nodes ROS2 | Ne pas mélanger avec conda |
| conda `ER` | Gemini ER + simulateur | Sans sourcer ROS2, opencv-python==4.9.0.80, numpy==1.26.4 |
| conda `GraspGen` | Serveur GraspGen | CUDA requis |
| conda `SAM3` | Serveur SAM3 | CUDA requis |

### Note environnement VS Code

`.vscode/settings.json` référence à la fois `franka_pick_interfaces` et `franka_demo_interfaces` dans les chemins d'intellisense — probablement un reliquat d'un renommage de package. Le package réel est `franka_demo_interfaces`. Sans impact sur le build, mais source de confusion si des artefacts `install/franka_pick_interfaces` traînent.

## Roadmap (non fait)

### 1. Hand-eye calibration (eye-on-base)
AprilTag `tag36h11` ID 0 fixé sous la bride FR3, `apriltag_ros` + `easy_handeye2`, calibration `eye_on_hand:=false` avec `robot_base_frame:=fr3_link0`, `robot_effector_frame:=fr3_hand_tcp`, `tracking_base_frame:=camera_color_optical_frame`. 15+ échantillons avec poses variées. Générer un `static_transform_publisher` `fr3_link0`→`camera_color_optical_frame` à ajouter dans `flow_manager.launch.py`.

### 2. Intégration MoveIt2 dans flow_manager
- Tester `test_moveit.py` avec `franka_fr3_moveit_config` en fake hardware.
- Ajouter `_transform_to_robot_frame()` (TF2) et `_execute_moveit()` (`moveit_py`) dans `flow_manager_node`.
- Dépendances `tf2_ros`/`tf2_geometry_msgs` dans `flow_manager/package.xml`.
- Launcher MoveIt2 dans `flow_manager.launch.py`.
- Ouverture/fermeture gripper via actions `franka_gripper`.

### 3. Tests pipeline complet
Test bout-en-bout avec `gemini_er_simulator.py`, validation visuelle RViz, puis test sur robot réel après calibration.

### 4. Objets de collision dans la planning scene (sol, murs)
Actuellement aucun obstacle statique déclaré : `move_group`/OMPL ne connaît que le robot lui-même, rien n'empêche de planifier une trajectoire qui traverserait le sol ou un mur. À ajouter via `PlanningSceneInterface::applyCollisionObject()` (formes primitives `shape_msgs/SolidPrimitive`, ex. boîte plate pour le sol en z=0, boîte fine verticale pour un mur), une fois au démarrage — pas besoin de le repasser à chaque `MoveToPose`, la scène persiste tant que `move_group` tourne. Concerne aussi bien `fp3_motion_server` que `panda_motion_server`. À ne pas confondre avec le warning `"planning volume was not specified"` déjà présent dans les logs, qui concerne les `workspace_parameters` (région d'échantillonnage OMPL), pas les objets de collision.

### 5. Vraie API Gemini Robotics-ER : conversion de coordonnées à ajouter
`gemini_er_simulator.py` (dans `~/Documents/FP3/GeminiRoboticsER/`) envoie aujourd'hui des pixels bruts (clic OpenCV direct, cf. "Convention de coordonnées" plus haut) — cohérent de bout en bout avec `task_validator`/`sam3_bridge`/serveur SAM3, vérifié dans le code (SAM3 attend des pixels et normalise lui-même en interne via `point_x / W`, `point_y / H`). `main.py` ne fait encore qu'appeler ce simulateur, aucun appel réel à l'API Gemini pour l'instant.

Le vrai modèle Gemini Robotics-ER (une fois branché à la place du simulateur) retourne ses coordonnées de pointing normalisées sur une échelle **0-1000**, pas en pixels bruts. Il faudra donc ajouter une conversion (`x_pixel = x_gemini / 1000 * largeur_image`, idem en y) quelque part avant que le point atteigne `task_validator`/`sam3_bridge` — rien dans le pipeline actuel ne le fait, à ajouter à ce moment-là (pas urgent tant que c'est le simulateur qui pilote).

## Dette technique identifiée — à traiter

### Critique
- **ZMQ REQ sans timeout hors health-check** : `sam3_bridge._call_sam3()` et `graspgen_bridge._call_graspgen()` font un `recv()` bloquant sans `RCVTIMEO`. Si le serveur SAM3/GraspGen crash ou freeze pendant un pick, le bridge reste bloqué indéfiniment (lockstep REQ/REP), et comme les nodes sont single-thread, le bridge devient injoignable pour toute requête future. Le pattern retry/recreate-socket documenté n'est appliqué qu'au démarrage.
- **`task_type: "stop"` ne peut rien interrompre** : `flow_manager_node.handle_execute_task` répond juste `"stopped"` sans agir. Comme `command_bridge` utilise un socket REP à alternance stricte et que `_handle_command` bloque tout le node pendant un pick, un "stop" ne peut même pas être reçu avant la fin du pick en cours. Problème de sécurité une fois MoveIt2 branché (bras en mouvement).
- **Nested spinning** (`spin_until_future_complete` dans `flow_manager_node._call_service`, boucle `spin_once` dans `command_bridge_node._handle_command`) : anti-pattern ROS2 (risque de réentrance/deadlock). Passer à un `MultiThreadedExecutor` + callbacks async permettrait aussi de fixer le point "stop" ci-dessus.

### Incohérence d'API
- **`GenerateGraspPose.srv` : `max_grasps` et `gripper_type` ignorés.** `flow_manager_node` envoie `grasp_req.max_grasps = 10` et `gripper_type = "franka_panda"`, mais `graspgen_bridge_node.handle_generate_grasp_pose` ne lit jamais ces champs — il utilise ses propres paramètres de lancement (`num_grasps`, `topk_num_grasps`). À corriger (lire la requête) ou à retirer du `.srv`.

### Important
- **`task_validator_node`** : borne seulement `point_x`/`point_y` par le bas (`< 0`), pas de vérification contre la résolution native (1280×720) → un point hors image passe silencieusement jusqu'à SAM3.
- **`camera_buffer_node`** : pas de vérification de fraîcheur ni de synchronisation temporelle entre `last_rgb`/`last_depth`/`last_camera_info`. Si la caméra se déconnecte, le pipeline continue avec des frames obsolètes sans erreur.
- **Duplication du pattern health-check ZMQ** : `sam3_bridge` (retry loop 10 tentatives) et `graspgen_bridge` (une tentative, simple warning) divergent pour la même logique. Factoriser dans une classe utilitaire partagée réduirait le risque d'incohérence (cf. piège msgpack ci-dessus, spécifique à chaque bridge).

### Nice-to-have
- `graspgen_bridge_node._rotation_matrix_to_quaternion` : ~25 lignes de conversion matrice→quaternion écrites à la main ; `scipy.spatial.transform.Rotation` ferait la même chose avec moins de risque de bug de signe/branche.
- Aucun test unitaire sur la logique métier pure (validation dans `task_validator`, déprojection dans `pointcloud_node`, conversion quaternion dans `graspgen_bridge`) — testable sans ROS2, à ajouter avant de complexifier avec MoveIt2.
- `camera_bridge` bind sur `tcp://*:5555` (toutes interfaces), aucune authentification sur les sockets ZMQ — acceptable si LAN fermé de labo, à garder en tête si le setup évolue.

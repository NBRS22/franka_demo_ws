# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Vue d'ensemble

Pipeline pick-and-place robotique pour un Franka FR3, orchestré en ROS2 (Jazzy), avec la perception (segmentation, génération de grasp) externalisée dans des serveurs Python séparés (SAM3, GraspGen) accédés via ZMQ+msgpack, et une commande déclenchée par Gemini ER (VLM tournant sur une machine séparée).

**Le pipeline s'arrête volontairement à la génération/visualisation des grasp poses pour le moment** — l'exécution du mouvement (MoveIt2) a été entièrement retirée du repo (`motion_node`, `scene_publisher_node`, `ExecuteGrasp.srv` supprimés), cf. "Architecture" et roadmap #2. Ce n'est pas juste débranché : il faudra la réécrire pour la rebrancher.

Packages du workspace :

| Package | Type | Rôle |
|---|---|---|
| `franka_demo_interfaces` | `ament_cmake` | Définitions des `.srv` |
| `franka_demo_bringup` | `ament_python` (launch only) | Launch file racine : RealSense + inclusion du launch de `robot_task_manager` |
| `robot_task_manager` | `ament_python` | Orchestration du pick (`pick_task_node`), buffer caméra, pointcloud — son launch file (`launch/robot_task_manager.launch.py`) démarre aussi les bridges (`gemini_er_bridge`, `sam3_bridge`, `graspgen_bridge`) |
| `gemini_er_bridge` | `ament_python` | Pont ZMQ vers Gemini ER (`camera_bridge_node`, `command_bridge_node`) |
| `sam3_bridge` | `ament_python` | Pont ZMQ vers le serveur SAM3 (`sam3_bridge_node`) + visualisation RViz de la segmentation (`visualize_segmentation_node`, déclenché par `sam3_bridge_node` dès le masque reçu) — a son propre launch file (`launch/sam3_bridge.launch.py`) pour lancer ses 2 nodes isolément |
| `graspgen_bridge` | `ament_python` | Pont ZMQ vers le serveur GraspGen (`graspgen_bridge_node`) + visualisation RViz des grasps (`visualize_grasps_node`, déclenché par `graspgen_bridge_node` dès les grasps reçus) — a son propre launch file (`launch/graspgen_bridge.launch.py`) pour lancer ses 2 nodes isolément |

## Commandes courantes

### Build

```bash
cd ~/franka_demo_ws
colcon build                                        # tout le workspace
colcon build --packages-select robot_task_manager    # un seul package
source install/setup.bash
```

`franka_demo_interfaces` est `ament_cmake` (génère les `.srv`) ; tous les autres packages sont `ament_python`.

### Lancer le pipeline complet

```bash
ros2 launch franka_demo_bringup franka_demo.launch.py
```

`franka_demo_bringup/launch/franka_demo.launch.py` est le launch file racine. Il ne cible plus qu'un robot réel — **la branche Isaac Sim (`panda_motion_server`) et l'argument `use_sim` ont été retirés**, il n'y a plus qu'un seul chemin de lancement. Il démarre maintenant lui-même les serveurs externes SAM3 et GraspGen (via `conda run`, cf. "Serveurs externes" ci-dessous), attend qu'ils répondent healthy sur leur port ZMQ, puis seulement démarre RealSense D455 (`align_depth.enable:=true`, résolution color/depth 1280x720x30 — résolution native max, cf. "Convention de coordonnées" ; `pointcloud.enable` volontairement **non** activé — bug connu de `realsense-ros`, cf. dette technique "Important") et **inclut** (`IncludeLaunchDescription`) le launch file de `robot_task_manager`, qui lui démarre tous les nodes ROS2 du pipeline (bridges compris) :

```
franka_demo_bringup/launch/franka_demo.launch.py
  ├─ ExecuteProcess(sam3_server)       — conda run -n SAM3, ~/Documents/FP3/SAM3
  ├─ ExecuteProcess(graspgen_server)   — conda run -n GraspGen, ~/Documents/FP3/GraspGen
  ├─ ExecuteProcess(wait_for_sam3)     — poll ZMQ health jusqu'à 'status': 'ok' (timeout 180s)
  │     └─ succès → ExecuteProcess(wait_for_graspgen) — même poll, port GraspGen
  │           └─ succès → démarre RealSense + robot_task_manager (ci-dessous)
  │                       échec/timeout → Shutdown() de tout le launch
  ├─ RealSense D455 (rs_launch.py de realsense2_camera)
  └─ IncludeLaunchDescription(robot_task_manager/launch/robot_task_manager.launch.py)
        ├─ Node(gemini_er_bridge, camera_bridge_node)
        ├─ Node(gemini_er_bridge, command_bridge_node)
        ├─ IncludeLaunchDescription(sam3_bridge/launch/sam3_bridge.launch.py)
        │     ├─ Node(sam3_bridge, sam3_bridge_node)
        │     └─ Node(sam3_bridge, visualize_segmentation_node)
        ├─ IncludeLaunchDescription(graspgen_bridge/launch/graspgen_bridge.launch.py)
        │     ├─ Node(graspgen_bridge, graspgen_bridge_node)
        │     └─ Node(graspgen_bridge, visualize_grasps_node)
        ├─ camera_buffer_node
        ├─ create_pointcloud_node
        └─ pick_task_node
```

Si `sam3_server` ou `graspgen_server` meurt à n'importe quel moment (pas seulement au démarrage — un crash pendant un pick en cours aussi), un `OnProcessExit` dédié à chacun émet un événement `Shutdown()` qui arrête tout l'arbre de launch (RealSense, bridges, `robot_task_manager` compris). Le check de santé initial (`wait_for_sam3`/`wait_for_graspgen`) est un *gate* séquentiel : les deux process serveurs démarrent en parallèle, mais RealSense/`robot_task_manager` n'est lancé qu'une fois les deux health-checks passés (la vérification est séquentielle, pas le boot des serveurs eux-mêmes, donc le temps d'attente total ≈ max des deux temps de chargement, pas leur somme).

`robot_task_manager.launch.py` démarre donc aussi des executables d'autres packages (`gemini_er_bridge`, `sam3_bridge`, `graspgen_bridge`) — d'où les `exec_depend` correspondants ajoutés à `robot_task_manager/package.xml`. Ce découpage permet de faire `ros2 launch robot_task_manager robot_task_manager.launch.py` isolément pour lancer tout le pipeline ROS2 sans RealSense (mais alors sans le démarrage/health-check automatique de SAM3/GraspGen — à lancer manuellement dans ce cas, cf. section suivante) — cf. section suivante.

Les deux nodes de `sam3_bridge` (`sam3_bridge_node` + `visualize_segmentation_node`) et les deux nodes de `graspgen_bridge` (`graspgen_bridge_node` + `visualize_grasps_node`) ne sont plus listés directement en `Node()` dans `robot_task_manager.launch.py` : celui-ci **inclut** (`IncludeLaunchDescription`) le launch file propre à chacun de ces deux packages, qui démarre ses deux nodes. Seul `gemini_er_bridge` reste listé directement en `Node()` (pas de launch file dédié pour l'instant).

Gemini ER (`gemini_er_simulator.py`, env conda `ER`) reste lancé **à part, manuellement** — c'est un script de test interactif (clic pour simuler le VLM), pas un serveur à health-checker automatiquement. Seuls SAM3 et GraspGen sont désormais auto-lancés par `franka_demo.launch.py`.

### Lancer un node individuellement (debug)

```bash
ros2 launch robot_task_manager robot_task_manager.launch.py   # tout le pipeline ROS2 (bridges + robot_task_manager), sans RealSense
ros2 launch sam3_bridge sam3_bridge.launch.py                 # juste les 2 nodes sam3_bridge (sam3_bridge_node + visualize_segmentation_node)
ros2 launch graspgen_bridge graspgen_bridge.launch.py         # juste les 2 nodes graspgen_bridge (graspgen_bridge_node + visualize_grasps_node)

ros2 run robot_task_manager pick_task_node
ros2 run robot_task_manager camera_buffer_node
ros2 run robot_task_manager create_pointcloud_node
ros2 run sam3_bridge sam3_bridge_node
ros2 run sam3_bridge visualize_segmentation_node
ros2 run graspgen_bridge graspgen_bridge_node
ros2 run graspgen_bridge visualize_grasps_node
ros2 run gemini_er_bridge command_bridge_node
ros2 run gemini_er_bridge camera_bridge_node
```

### Serveurs externes (hors colcon, envs conda séparés)

**SAM3 et GraspGen sont désormais démarrés automatiquement par `ros2 launch franka_demo_bringup franka_demo.launch.py`** (cf. section précédente) — plus besoin de les lancer à la main dans le flow normal. Le launch file les lance via `conda run -n <env> --no-capture-output` (pas de `conda activate` interactif), avec le répertoire de travail et le nom d'env codés en dur dans `franka_demo_bringup/launch/franka_demo.launch.py` (`SAM3_DIR`, `SAM3_CONDA_ENV`, `GRASPGEN_DIR`, `GRASPGEN_CONDA_ENV` — chemins spécifiques à ce poste de dev, `/home/ngr/Documents/FP3/{SAM3,GraspGen}`, à adapter si le workspace est cloné ailleurs).

Lancement manuel (debug isolé, ou pour `ros2 launch robot_task_manager robot_task_manager.launch.py` qui ne les démarre pas lui-même) :

```bash
conda activate SAM3        # serveur SAM3, python -m sam3_server, port 5557, CUDA requis
conda activate GraspGen    # serveur GraspGen, python client-server/graspgen_server.py, port 5558, CUDA requis
conda activate ER          # Gemini ER / gemini_er_simulator.py, sans sourcer ROS2 — reste manuel, jamais auto-lancé
```
Ne jamais mélanger ces envs conda avec le Python système utilisé par les nodes ROS2.

`franka_demo_bringup/scripts/wait_for_zmq_health.py` (nouveau, installé via `data_files` dans `setup.py` — pas un entry_point/node ROS2, un simple script appelé en `ExecuteProcess(cmd=['python3', <chemin installé>, ...])`) poll `{'action': 'health'}` sur le port ZMQ du serveur jusqu'à recevoir `{'status': 'ok'}` (retry toutes les 2s, `--timeout` 180s par défaut) — réutilise le même protocole que `_check_sam3_server`/`_check_graspgen_server` dans les bridges. Retourne 0 si healthy avant le timeout, 1 sinon (déclenche alors un `Shutdown()` du launch, cf. section précédente).

**Non vérifié en conditions réelles** (session de dev sans lancement effectif des vrais serveurs SAM3/GraspGen ni du hardware) : le comportement de `conda run` sous un `ExecuteProcess`/`Shutdown()` de `launch_ros` lors d'un arrêt (Ctrl+C ou crash déclenchant le `Shutdown()`) — `conda run` a une réputation connue de mal propager certains signaux à son sous-processus, ce qui pourrait laisser un process Python (SAM3/GraspGen) orphelin plutôt que proprement tué. À confirmer dès que testable (`ros2 launch ...` puis Ctrl+C, vérifier `ps aux | grep sam3_server` après coup).

### Tests

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
Les tests présents sont uniquement les templates `ament_copyright` / `ament_flake8` / `ament_pep257` — pas de tests fonctionnels sur la logique métier pour l'instant (cf. dette technique plus bas).

### Introspection utile

```bash
ros2 service list
ros2 service call /execute_pick_task franka_demo_interfaces/srv/ExecutePickTask \
  "{object_label: 'red mug', point_x: 640, point_y: 360}"
ros2 topic echo /pick/grasp_markers
ros2 topic echo /pick/segmentation_visualization
ros2 topic echo /pick/pointcloud
```

## Architecture

### Flux d'un pick (déclenché par un clic dans `gemini_er_simulator.py`)

```
Realsense D455 → camera_bridge → ZMQ PUB 5555 → Gemini ER
Gemini ER → ZMQ REQ 5556 → command_bridge → service /execute_pick_task
pick_task_node (handle_pick_task, séquentiel, callbacks async via ReentrantCallbackGroup) :
  1. /get_frames              → camera_buffer_node (dernier RGB+depth+camera_info+cloud bufferisés —
                                  ⚠️ plus aucune vérification de synchronisation, cf. dette technique)
  2. /segment_object           → sam3_bridge → ZMQ REQ 5557 → serveur SAM3
                                  (sam3_bridge_node appelle lui-même /visualize_segmentation dès le masque
                                  reçu, en fire-and-forget — pas d'appel depuis pick_task_node, cf. note ci-dessous)
  3. /create_pointcloud        → create_pointcloud_node (déprojette l'objet depuis aligned_depth_to_color + camera_info, masqué par SAM3)
  4. /generate_grasp_pose      → graspgen_bridge → ZMQ REQ 5558 → serveur GraspGen
                                  (graspgen_bridge_node appelle lui-même /visualize_grasps dès les grasps
                                  reçus, en fire-and-forget — même pattern que sam3_bridge_node, cf. note ci-dessous)
  -- fin du flow actuel --
```

`create_pointcloud_node` (renommé depuis `filter_pointcloud_node`, cf. Historique) **déprojette manuellement** — pas de dépendance au nuage natif RealSense (`/camera/camera/depth/color/points`). Raison : avec `align_depth.enable:=true` + `pointcloud.enable:=true` combinés, ce topic a un **bug non résolu côté `realsense-ros`** — le nuage généré est calculé indépendamment de l'alignement depth→couleur et se retrouve décalé spatialement (quelques cm, en translation) par rapport à la vraie position des objets ; confirmé en test réel sur ce setup (objet segmenté correctement, mais nuage décalé d'un bloc par rapport au nuage natif affiché dans RViz) et documenté upstream, sans fix officiel : [issue #2595](https://github.com/IntelRealSense/realsense-ros/issues/2595), [issue #3050](https://github.com/realsenseai/realsense-ros/issues/3050) (`pointcloud.enable` n'est donc plus activé du tout au launch — cf. `franka_demo_bringup`).

À la place, `create_pointcloud_node` prend `mask` (SAM3) + `rgb` + `depth` (`aligned_depth_to_color`, garanti pixel-aligné sur la couleur par construction — fonctionnalité indépendante et non affectée par le bug ci-dessus) + `camera_info` (intrinsèques `K` : `fx=k[0]`, `cx=k[2]`, `fy=k[4]`, `cy=k[5]`) en requête (`CreatePointcloud.srv`, renommé depuis `FilterPointcloud.srv`), et déprojette lui-même en pinhole standard (`X=(u-cx)*Z/fx`, `Y=(v-cy)*Z/fy`, `Z=depth_mm*0.001`) — vectorisé numpy, pas de boucle par pixel. Élimine au passage toute la classe de bugs de parsing `PointCloud2` rencontrée précédemment (reshape inversé, padding `row_step` non lu par `sensor_msgs_py` — cf. Historique) puisqu'aucun nuage externe n'est plus parsé : la géométrie est calculée directement depuis l'image depth. `camera_buffer_node` n'a donc plus de subscription au nuage natif (`_last_cloud` retiré), et `GetFrames.srv` n'a plus de champ `cloud`.

Les deux nuages publiés (`/pick/pointcloud` l'objet, et `/pick/scene_pointcloud` la scène) sont **colorés** — champs `x`/`y`/`z`/`rgb`, `rgb` étant un `float32` dont les bits représentent en réalité un entier 24 bits packé (`(r<<16)|(g<<8)|b`, reinterprété — pas casté — en `float32`) : convention PCL/RViz standard pour les nuages colorés (`PointField.RGB8` color transformer de RViz), la même que celle qu'utilise le nuage natif RealSense. Couleur échantillonnée depuis `rgb` (BGR8 via `cv_bridge`) au même pixel que la profondeur déprojetée, pour l'objet comme pour la scène. `graspgen_bridge_node._pointcloud2_to_numpy` n'est pas affecté par ce champ `rgb` en plus (présent sur les deux nuages envoyés à GraspGen) : il sélectionne explicitement `field_names=('x','y','z')`, ignore le reste.

`create_pointcloud_node` calcule aussi le **complément** du masque — `xyz[mask_raw == 0]`, c-à-d. la scène **sans** l'objet ciblé (table, sol, autres objets) — filtré des `NaN` et sous-échantillonné à 20 000 points max (`_MAX_SCENE_POINTS`), publié sur `/pick/scene_pointcloud` et renvoyé dans `CreatePointcloud.srv` (`response.scene_cloud`). `pick_task_node` le transmet à `/generate_grasp_pose` (`GenerateGraspPose.srv`, champ requête `scene_cloud`), et `graspgen_bridge_node` le forwarde au serveur GraspGen (`scene_point_cloud` dans la requête ZMQ, uniquement si non-vide — opt-in, aucun changement de comportement si absent) pour que celui-ci **filtre les grasps qui entreraient en collision avec la scène** (table, sol...) — cf. section "Filtrage de collision GraspGen" plus bas.

Bufferiser rgb/depth/camera_info dans `camera_buffer_node` (plutôt que de les récupérer séparément) corrige un problème de cohérence temporelle : masque et depth utilisés pour la déprojection viennent du même appel `/get_frames`, au tout début du flow — donc du même instant caméra, avant même le round-trip SAM3 (potentiellement 1s+).

`visualize_grasps_node` (déplacé du package `robot_task_manager` vers `graspgen_bridge`, service `visualize_grasps`, topic `/pick/grasp_markers`) est un node séparé, inchangé dans son fonctionnement interne (markers RViz, flèches, meilleur grasp en vert opaque, les autres en cyan semi-transparent). Exactement comme pour `sam3_bridge`/`visualize_segmentation_node` : ce n'est plus `pick_task_node` qui appelle `/visualize_grasps`, c'est `graspgen_bridge_node` lui-même — `handle_generate_grasp_pose` appelle `_trigger_visualization(response.grasps, response.scores)` juste après avoir construit sa réponse, en fire-and-forget (`service_is_ready()` non-bloquant + `call_async`/`add_done_callback`, sans jamais bloquer `handle_generate_grasp_pose`). `pick_task_node` n'a donc plus de client ni d'appel vers `visualize_grasps` (retiré : `viz_grasps_client`, méthode `_visualize_grasps`).

`visualize_segmentation_node` (package `sam3_bridge`, service `visualize_segmentation`, topic `/pick/segmentation_visualization`) reste un node séparé, inchangé dans son fonctionnement interne (overlay = fond désaturé/assombri hors masque + croix rouge au point cliqué). Ce qui a changé, c'est **qui l'appelle** : ce n'est plus `pick_task_node` mais `sam3_bridge_node` lui-même — `handle_segment_object` appelle `/visualize_segmentation` (`_trigger_visualization`) juste après avoir construit `response.mask`, en *fire-and-forget* (`call_async` + `add_done_callback`, sans attendre/bloquer sur le résultat côté `handle_segment_object`, qui répond à `pick_task_node` sans attendre la fin de la visualisation ; un log `info`/`warn` selon le résultat arrive plus tard via `_on_visualization_done`). `pick_task_node` n'a donc plus de client ni d'appel vers ce service ; il ne connaît même plus son existence.

Le choix fire-and-forget (plutôt qu'un appel bloquant comme `pick_task_node._call_service`) est délibéré : `sam3_bridge_node` tourne avec un exécuteur mono-thread par défaut (`rclpy.spin(node)`, pas de `MultiThreadedExecutor`) — un appel bloquant depuis l'intérieur de `handle_segment_object` recréerait le même risque de deadlock par nested-spin que celui déjà corrigé dans `pick_task_node` (cf. Historique), puisque le callback de complétion du futur ne pourrait pas être traité tant que le thread unique reste bloqué à attendre ce même futur. En ne bloquant jamais, ce problème ne se pose pas.

`pick_task_node.handle_pick_task` s'arrête après l'étape 5 (visualisation des grasps), volontairement, pour l'instant. **L'exécution du mouvement (`motion_node`, `scene_publisher_node`, `ExecuteGrasp.srv`) a été supprimée du repo**, pas juste débranchée — à réécrire entièrement pour rebrancher cette étape (cf. roadmap #2).

`pick_task_node._call_service` utilise `call_async` + `threading.Event` (pas de `spin_until_future_complete`), le node tourne sous `MultiThreadedExecutor` — l'ancien anti-pattern de nested spinning est corrigé (cf. dette technique, section historique). Chaque étape échoue proprement avec `success=False` + message si le service précédent échoue ou timeout.

**`pick_task_node` ne valide plus `point_x`/`point_y` nulle part** (l'ancien `task_validator_node` a été supprimé, aucune étape équivalente ne l'a remplacé) — cf. dette technique.

### Ports ZMQ

| Service | Port | Pattern | Host (côté ROS2) |
|---|---|---|---|
| camera_bridge → Gemini ER | 5555 | PUB/SUB | bind `0.0.0.0` |
| command_bridge ← Gemini ER | 5556 | REP (Gemini ER en REQ) | bind `0.0.0.0` |
| GraspGen server | 5558 | REQ/REP | connect `127.0.0.1` par défaut (param `graspgen_bridge_host`/`graspgen_bridge_port`, à repasser à `172.22.62.94`/`5556` pour un vrai déploiement LAN) |
| SAM3 server | 5557 | REQ/REP | connect `127.0.0.1` par défaut (param `sam3_bridge_host`, à repasser à `172.22.62.94` pour un vrai déploiement LAN) |

`command_bridge` et le serveur GraspGen partageaient le port **5556** dans la config LAN d'origine — ce n'était pas un conflit car ce sont deux machines différentes (`command_bridge` bind localement, `graspgen_bridge` se connecte à `172.22.62.94`), mais source de confusion à la lecture des logs/configs. Depuis, `graspgen_bridge_port` a été changé en `5558` pendant le développement local (coïncidence avec l'ancien port de `command_bridge`, qui était lui-même `5558` dans une version antérieure du pipeline avant de passer à `5556`) — donc les deux ports ne se chevauchent plus par défaut aujourd'hui, mais à revérifier lors du retour à la config LAN réelle (`172.22.62.94`/`5556`).

`camera_bridge_node` et `command_bridge_node` sont packagés ensemble sous `gemini_er_bridge` (même package Python, `share/gemini_er_bridge`) mais restent deux nodes/exécutables ROS2 distincts, chacun avec son propre `zmq.Context`/socket — pas de serveur ZMQ partagé, donc pas de couplage de blocage entre eux (cf. dette "stop" ci-dessous, qui ne concerne que `command_bridge_node`).

### Interfaces ROS2 (`franka_demo_interfaces/srv/`)

| Service | Client → Serveur | Rôle |
|---|---|---|
| `ExecutePickTask` | `command_bridge` → `pick_task_node` | point d'entrée du pipeline |
| `GetFrames` | `pick_task_node` → `camera_buffer_node` | dernier RGB+depth+camera_info+cloud |
| `SegmentObject` | `pick_task_node` → `sam3_bridge` | masque de segmentation |
| `VisualizeSegmentation` | `sam3_bridge_node` → `visualize_segmentation_node` (les deux dans `sam3_bridge`) | overlay RViz, déclenché en interne (fire-and-forget) — `pick_task_node` n'y participe plus |
| `CreatePointcloud` | `pick_task_node` → `create_pointcloud_node` | déprojette l'objet depuis depth+K, renvoie aussi le nuage "scène sans objet" (`scene_cloud`) |
| `GenerateGraspPose` | `pick_task_node` → `graspgen_bridge` | génération de grasps, avec filtrage de collision optionnel via `scene_cloud` |
| `VisualizeGrasps` | `graspgen_bridge_node` → `visualize_grasps_node` (les deux dans `graspgen_bridge`) | markers RViz, déclenché en interne (fire-and-forget) — `pick_task_node` n'y participe plus |

### Filtrage de collision GraspGen (nuage de scène)

Le serveur GraspGen ne connaît par défaut que l'objet ciblé (`object_cloud`) — sans contexte sur la table/le sol, il peut générer des grasps dont la pince traverserait la table. GraspGen (repo externe, `~/Documents/FP3/GraspGen`) expose en interne un filtre de collision basé sur la géométrie réelle de la pince (`filter_colliding_grasps_fast`, utilisé jusque-là uniquement par les scripts de démo `scripts/demo_scene_pc.py --filter_collisions`) — mais ne l'exposait **pas** via son serveur ZMQ. Ce chantier a étendu le protocole ZMQ (côté GraspGen, `grasp_gen/serving/zmq_server.py`/`zmq_client.py`, cf. `client-server/README.md` mis à jour dans ce repo externe) et branché le pipeline ROS2 correspondant :

1. `create_pointcloud_node` calcule `xyz[mask_raw == 0]` (le complément exact du masque objet — mêmes tests unitaires confirmant zéro chevauchement et une somme égale au nuage total), le filtre des `NaN`, le sous-échantillonne à 20 000 points max, le publie sur `/pick/scene_pointcloud` et le renvoie dans `CreatePointcloud.srv` (`response.scene_cloud`).
2. `pick_task_node` transmet ce nuage à `/generate_grasp_pose` (`GenerateGraspPose.srv`, requête `scene_cloud`).
3. `graspgen_bridge_node` le convertit en numpy et l'inclut dans la requête ZMQ sous `scene_point_cloud` — **uniquement s'il contient des points** (`request.scene_cloud.data` non-vide) ; sinon le champ est omis et le serveur se comporte exactement comme avant (opt-in, rétro-compatible). Deux nouveaux paramètres ROS2 déclarés : `collision_threshold` (défaut `0.01` m — abaissé depuis `0.02` m, cf. Historique) et `max_scene_points` (défaut `8192`). `graspgen_bridge_node` envoie toujours sa valeur explicitement, le défaut serveur GraspGen (resté `0.02`) n'est donc jamais utilisé par ce pipeline.
4. Côté serveur GraspGen : la géométrie de collision de la pince (`get_gripper_info(gripper_name).collision_mesh`, connue via le `gripper_config` déjà chargé au démarrage — `graspgen_franka_panda.yml` dans ce pipeline) est pré-échantillonnée **une fois** à l'initialisation (2000 points de surface), pour ne rien resample par requête. Si `scene_point_cloud` est présent dans la requête `infer`, le serveur sous-échantillonne à `max_scene_points`, appelle `filter_colliding_grasps_fast(scene_pc=..., grasp_poses=grasps_np, collision_threshold=..., gripper_surface_points=...)` sur les grasps déjà générés, et ne renvoie que les grasps collision-free. La réponse gagne alors `num_grasps_before_collision_filter` et `timing.collision_filter_ms` (absents si aucun filtrage n'a eu lieu) — `graspgen_bridge_node` les logue s'ils sont présents.

**Vérifié de bout en bout avec le vrai serveur GraspGen (GPU RTX 3090 de ce poste)**, nuage objet + nuage scène synthétiques (cube posé sur un plan représentant une table) : sans `scene_point_cloud` → 60 grasps, comportement inchangé (pas de clé collision dans la réponse) ; avec → 60 grasps générés puis 24 collision-free après filtrage, `num_grasps_before_collision_filter`/`timing.collision_filter_ms` bien présents. La logique de split objet/scène dans `create_pointcloud_node` a aussi été vérifiée séparément (déprojection synthétique, masque partiel → object_points + scene_points == nuage total, zéro chevauchement en xy).

**Non testé en conditions réelles** (RealSense + objet + table physiques) — le comportement à vérifier au prochain lancement : que le `scene_cloud` réel (background RealSense, potentiellement bruité) filtre bien les grasps qui traverseraient la vraie table sans sur-filtrer des grasps valides. En usage réel, le filtre rejetait initialement 100% des grasps (`0/N`) — corrigé en deux temps : dilatation du masque objet avant exclusion de la scène (cf. `robot_task_manager/CLAUDE.md`, cause principale identifiée) et `collision_threshold` abaissé `0.02`→`0.01`m (second levier, plus permissif). Aucun des deux correctifs n'a encore été re-testé sur le pipeline réel.

### Sérialisation ZMQ — piège msgpack / msgpack_numpy

- `sam3_bridge_node.py` : `msgpack.unpackb(raw, raw=False)` → clés `str` (`result["status"]`, `result["mask"]`...). Pas de tableaux numpy dans ce channel (masque transmis en bytes bruts + `mask_shape`).
- `graspgen_bridge_node.py` : `msgpack.unpackb(raw)` → clés `str` aussi (`result["grasps"]`, `result.get("status")`...) depuis un fix appliqué après un vrai `KeyError: b'grasps'` en test réel. **Ancienne note obsolète, gardée en historique** : ce fichier utilisait avant `result[b"grasps"]` (clés bytes), sous l'hypothèse que `msgpack_numpy` (`m.patch()`, utilisé ici pour sérialiser les `ndarray`) avait besoin de `raw=True`/clés bytes pour son hook de décodage. **Vérifié faux avec la version installée sur ce système** (`msgpack==1.2.1`, où `raw=False` est déjà le défaut de `unpackb()` — testé en direct : `msgpack_numpy` reconstruit correctement les `ndarray` même avec des clés `str`). Si `msgpack` est un jour rétrogradé vers une version pré-1.0 (`raw=True` par défaut), cette hypothèse redeviendrait vraie et il faudrait revenir aux clés bytes — vérifier `msgpack.unpackb(msgpack.packb({'a': 1}))` pour trancher.
- Ne pas "harmoniser" ces deux fichiers sans vérifier le comportement réel de la version de `msgpack` installée (cf. point précédent).
- Après un timeout sur un socket ZMQ REQ, il faut fermer et recréer le socket — pattern appliqué systématiquement dans `sam3_bridge` et `graspgen_bridge` (santé au démarrage + chaque appel normal, `_recreate_socket()` sur `zmq.Again`).
- `CONFLATE` incompatible avec `send_multipart` → utiliser `SNDHWM`/`RCVHWM=1`.
- `setsockopt(zmq.SUBSCRIBE, b"rgb")` avec des `bytes`, pas une `str`.
- `connect("tcp://127.0.0.1:5555")`, pas `localhost`.

### Convention de coordonnées

- Pixels bruts dans la résolution caméra configurée au launch (1280×720 — résolution native max du D455 commune aux flux color/depth, cf. `REALSENSE_COLOR_PROFILE`/`REALSENSE_DEPTH_PROFILE` dans `franka_demo_bringup/launch/franka_demo.launch.py` — abaissée temporairement à 848×480 puis 424×240 pendant le diagnostic de déconnexions USB de la caméra, cf. CLAUDE.md `franka_demo_bringup`/"Dépannage", puis remontée au max une fois la vraie cause identifiée : gestion d'énergie USB, pas un problème de débit/résolution ; color/depth profiles gardés identiques exprès pour que le depth aligné reste indexé pixel-à-pixel comme la couleur) — `point_x`/`point_y` transitent tels quels de Gemini ER jusqu'à SAM3, aucune remise à l'échelle nulle part dans le pipeline. **Rien dans le code ROS2 (`create_pointcloud_node`, `sam3_bridge_node`, ...) ne hardcode une résolution** : `create_pointcloud_node` lit `depth_raw.shape` du message reçu, `sam3_bridge_node` déduit la shape du masque de `mask_shape` renvoyé par SAM3 — la résolution est donc déjà dynamique de bout en bout, seul le launch RealSense avait besoin d'être touché pour la changer.
- GraspGen retourne les poses dans le frame du cloud envoyé (`request.object_cloud.header.frame_id`, hérité de `frame.depth.header.frame_id` — `camera_color_optical_frame` en pratique puisque le depth est aligné à la couleur).
- Il n'y a plus de code de transform caméra→robot dans le repo (l'ancien `motion_node` avait une matrice codée en dur, calibrée pour Isaac Sim — supprimée avec le node). À réintroduire via une vraie calibration main-œil (roadmap #1) quand l'exécution du mouvement sera réécrite.

### Environnements

| Env | Usage | Notes |
|---|---|---|
| Python système Ubuntu | Nodes ROS2 | Ne pas mélanger avec conda |
| conda `ER` | Gemini ER + simulateur | Sans sourcer ROS2, opencv-python==4.9.0.80, numpy==1.26.4 |
| conda `GraspGen` | Serveur GraspGen | CUDA requis |
| conda `SAM3` | Serveur SAM3 | CUDA requis |

## Roadmap (non fait)

### 1. Hand-eye calibration (eye-on-base) — toujours non fait
AprilTag `tag36h11` ID 0 fixé sous la bride FR3, `apriltag_ros` + `easy_handeye2`, calibration `eye_on_hand:=false` avec `robot_base_frame:=fr3_link0`, `robot_effector_frame:=fr3_hand_tcp`, `tracking_base_frame:=camera_color_optical_frame`. 15+ échantillons avec poses variées. Le résultat alimentera le transform caméra→robot dont aura besoin la future implémentation de l'exécution du mouvement (cf. #2) — idéalement via un `static_transform_publisher` TF2 plutôt qu'une constante Python codée en dur (l'ancienne approche, supprimée avec `motion_node`).

### 2. Exécution du mouvement (MoveIt2) — à réécrire depuis zéro
L'ancienne implémentation (`motion_node.py`, `scene_publisher_node.py`, `ExecuteGrasp.srv`) a été **entièrement supprimée du repo** : le pipeline s'arrête désormais à la génération/visualisation des grasp poses (cf. "Architecture"). Elle utilisait l'action `/move_action` (`moveit_msgs/MoveGroup`) + gripper via `/panda_hand_controller/gripper_cmd`, avec des noms de frame/groupe Panda (`panda_arm`, `panda_hand`, `panda_link0`) hérités d'une calibration Isaac Sim — pas la convention FR3 (`fr3_arm`, `fr3_hand`, `fr3_link0`...). En la réécrivant : utiliser les bons noms FR3 dès le départ, prévoir le lancement d'un `move_group` réel (rien ne le fournit dans `franka_demo_bringup`), et rebrancher `/execute_grasp` dans `pick_task_node.handle_pick_task` une fois prête.

### 3. Tests pipeline complet
Test bout-en-bout avec `gemini_er_simulator.py`, validation visuelle RViz jusqu'à la génération de grasp (l'exécution du mouvement n'existe plus, cf. #2), puis test sur robot réel après calibration et réécriture de l'exécution.

### 4. Objets de collision dans la planning scene — à refaire
L'ancien `scene_publisher_node` (plan de sol statique) a été supprimé avec le reste du code MoveIt2 (cf. #2). À réintroduire en même temps que l'exécution du mouvement : sol + murs/obstacles latéraux via `PlanningSceneInterface`/`shape_msgs/SolidPrimitive`. À ne pas confondre avec le warning `"planning volume was not specified"` dans les logs, qui concerne les `workspace_parameters` (région d'échantillonnage OMPL), pas les objets de collision.

### 5. Vraie API Gemini Robotics-ER : conversion de coordonnées à ajouter
`gemini_er_simulator.py` (dans `~/Documents/FP3/GeminiRoboticsER/`) envoie aujourd'hui des pixels bruts (clic OpenCV direct, cf. "Convention de coordonnées" plus haut) — cohérent de bout en bout avec `sam3_bridge`/serveur SAM3, vérifié dans le code (SAM3 attend des pixels et normalise lui-même en interne via `point_x / W`, `point_y / H`). `main.py` ne fait encore qu'appeler ce simulateur, aucun appel réel à l'API Gemini pour l'instant.

Le vrai modèle Gemini Robotics-ER (une fois branché à la place du simulateur) retourne ses coordonnées de pointing normalisées sur une échelle **0-1000**, pas en pixels bruts. Il faudra donc ajouter une conversion (`x_pixel = x_gemini / 1000 * largeur_image`, idem en y) quelque part avant que le point atteigne `sam3_bridge` — rien dans le pipeline actuel ne le fait, à ajouter à ce moment-là (pas urgent tant que c'est le simulateur qui pilote).

## Dette technique identifiée — à traiter

### Critique
- **Pas d'exécution du mouvement** : cf. roadmap #2, à réécrire entièrement (attention au mismatch de noms Panda vs FR3 dans l'ancienne implémentation supprimée). Pas un blocage pour l'usage courant du pipeline (qui s'arrête à la génération de grasp), mais à traiter avant d'aller plus loin.
- **`task_type: "stop"` ne peut rien interrompre** : `command_bridge` utilise un socket REP à alternance stricte et `_handle_pick_command` bloque le thread ZMQ dédié pendant tout un pick → un "stop" ne peut pas être reçu avant la fin du pick en cours. Toujours vrai malgré le passage à `MultiThreadedExecutor`/callbacks async côté `pick_task_node` (ce refactor a supprimé le risque de deadlock ROS2, mais pas la contrainte d'alternance stricte du socket ZMQ REP). Problème de sécurité tant que le bras peut être en mouvement pendant l'attente.
- **`pick_task_node` ne valide plus `point_x`/`point_y`** : l'ancien `task_validator_node` (qui au moins bornait `< 0`) a été retiré du pipeline sans équivalent de remplacement — un point hors image ou aberrant passe désormais silencieusement jusqu'à SAM3.
- **⚠️ Régression : `camera_buffer_node` n'a plus aucune vérification de synchronisation RGB/depth.** Le check qui rejetait `handle_get_frames` si `rgb`/`depth` étaient désynchronisés de plus de 100ms (`_SYNC_TOLERANCE_S`, comparaison de `header.stamp`) a disparu du code à un moment de ce chantier, sans suppression volontaire ni changement d'architecture qui l'expliquerait (contrairement aux autres suppressions de ce repo, toutes documentées et délibérées). `handle_get_frames` renvoie désormais `success=True` dès que les 4 buffers (`rgb`/`depth`/`camera_info`/`cloud`) sont non-`None`, sans jamais comparer leurs timestamps entre eux — un `depth` arbitrairement plus vieux que le `rgb` (caméra qui rame, republish partiel) passerait silencieusement jusqu'à SAM3 et au filtrage du nuage. À restaurer si ce n'est pas un choix délibéré.

### Important
- ~~`filter_pointcloud_node` suppose que le nuage natif RealSense est organisé/aligné sur la couleur...~~ **Devenu sans objet** — cette chaîne de trois bugs successifs (nuage non organisé sans `pointcloud.ordered_pc`, reshape `(width,height)` inversé dans `sensor_msgs_py.read_points()`, padding `row_step` jamais lu par la même fonction) courait tous après le même symptôme : faire coller un nuage tiers (`/camera/camera/depth/color/points`) au masque SAM3. Ce chantier de debug a fini par révéler la vraie cause de fond : `align_depth.enable:=true` + `pointcloud.enable:=true` combinés est un **bug non résolu de `realsense-ros` lui-même** (le nuage généré est calculé indépendamment de l'alignement depth→couleur, décalage spatial de quelques cm) — [issue #2595](https://github.com/IntelRealSense/realsense-ros/issues/2595), [issue #3050](https://github.com/realsenseai/realsense-ros/issues/3050), sans fix officiel. Les trois correctifs successifs corrigeaient chacun un vrai bug de parsing, mais ne pouvaient de toute façon jamais résoudre le décalage de fond puisqu'il vient du driver, pas du parsing. Solution finale : abandon complet du nuage natif RealSense au profit d'une déprojection manuelle depuis `aligned_depth_to_color` (garanti pixel-aligné sur la couleur par construction, fonctionnalité indépendante non affectée par ce bug) — `create_pointcloud_node`, cf. "Architecture" et Historique. Élimine toute la classe de bugs `PointCloud2`/`row_step`/reshape rencontrée en chemin, puisqu'aucun nuage externe n'est plus parsé du tout.

### Nice-to-have
- `camera_bridge` bind sur `tcp://*:5555` (toutes interfaces), aucune authentification sur les sockets ZMQ — acceptable si LAN fermé de labo, à garder en tête si le setup évolue.
- **Nommage incohérent du champ d'erreur dans les `.srv`** : la plupart des services utilisent `string message` en réponse (`GetFrames`, `CreatePointcloud`, `VisualizeSegmentation`, `VisualizeGrasps`), mais `SegmentObject` et `GenerateGraspPose` utilisent `string error_msg` à la place — même rôle, deux noms différents selon le fichier. Sans impact fonctionnel (chaque node/client utilise le bon nom du bon côté), mais source de confusion à la lecture/l'écriture d'un nouveau client. À harmoniser sur un seul nom si ces interfaces sont retouchées.
- `franka_demo_interfaces/package.xml` : maintainer encore au placeholder `you@example.com`/`you` (jamais renseigné, contrairement aux autres packages). `version` à `0.1.0` alors que tous les autres packages sont restés à `0.0.0` — incohérence mineure de méta-données.
- `graspgen_bridge/package.xml`+`setup.py` : `description` encore à `TODO: Package description`, `maintainer`/`maintainer_email` encore aux placeholders `ngr`/`ngr@todo.todo` — seul package du repo où ni la description ni le mainteneur n'ont été renseignés (les autres ont au moins une vraie description, et `nour.el.bachari@accenture.com` comme mainteneur).
- `license` à `TODO: License declaration` dans 4 des 6 packages (`franka_demo_bringup`, `gemini_er_bridge`, `graspgen_bridge`, `sam3_bridge`) — seuls `franka_demo_interfaces` et `robot_task_manager` ont `Apache-2.0`. Pas un vrai choix de licence délibéré, juste ce qui existait déjà avant ce chantier (cf. discussion précédente sur pourquoi `franka_demo_bringup` a hérité d'`Apache-2.0` au départ, depuis repassé à `TODO` par une édition externe).

## Améliorations nodes — à faire plus tard

### `command_bridge_node` (`gemini_er_bridge/command_bridge_node.py`)
- **`task_type` absent vs non supporté** : si le champ `task_type` manque complètement, `command.get('task_type')` retourne `None` et le log dit "Unsupported task_type : 'None'" — trompeur. Distinguer les deux cas explicitement.
- **Pas de log de connexion client** : ZMQ REP ne notifie pas les connexions/déconnexions — impossible de savoir si Gemini ER est connecté ou non sans recevoir un message.

### `camera_bridge_node` (`gemini_er_bridge/camera_bridge_node.py`)
- **QoS** : subscriber en `RELIABLE` depth=10. Testé et fonctionnel avec la vraie RealSense D455 et Isaac Sim — ne pas changer sans raison.
- **Validation `jpeg_quality`** : si la valeur passée est hors 0–100, OpenCV peut planter silencieusement. Ajouter un clamp ou une vérification au démarrage.

## Historique (dette résolue depuis la dernière révision de ce document)

- Migration `flow_manager` → `robot_task_manager` (renommage de node également : `flow_manager_node` → `pick_task_node`).
- `grasp_selector_node`/`task_validator_node` supprimés, remplacés par `visualize_grasps_node`/`visualize_segmentation_node` (la logique de validation n'a pas été portée — cf. dette technique ci-dessus).
- MoveIt2 implémenté (`motion_node.py`) puis retiré à nouveau (cf. entrées plus bas) — plus un TODO "jamais fait", mais un chantier à reprendre.
- Nested spinning (`spin_until_future_complete`) éliminé dans `pick_task_node` : tourne sous `MultiThreadedExecutor` + `ReentrantCallbackGroup`, `_call_service` utilise `call_async` + `threading.Event`.
- `command_bridge_node` : `wait_for_service` bloquant remplacé par `service_is_ready()` non-bloquant ; flag `_shutdown` ajouté pour un arrêt propre du thread ZMQ.
- `camera_buffer_node` : vérification de synchronisation temporelle RGB/depth ajoutée (mais staleness globale toujours pas couverte, cf. dette technique).
- `GenerateGraspPose.srv` : champs `max_grasps`/`gripper_type` retirés (l'incohérence "champs ignorés côté serveur" n'existe plus).
- Launch file déplacé de `robot_task_manager/launch/` vers le nouveau package `franka_demo_bringup/launch/franka_demo.launch.py`.
- Branche Isaac Sim (`panda_motion_server`) et argument `use_sim` retirés du launch, RealSense n'est plus derrière un `UnlessCondition`.
- `motion_node.py`, `scene_publisher_node.py` et `ExecuteGrasp.srv` **supprimés du repo** (pas juste débranchés) ; `pick_task_node.handle_pick_task` n'appelle plus `/execute_grasp` — le pipeline s'arrête désormais volontairement après la génération/visualisation des grasp poses (cf. "Architecture", roadmap #2).
- `robot_task_manager` a maintenant son propre launch file (`launch/robot_task_manager.launch.py`), qui démarre tous les nodes ROS2 du pipeline (bridges `gemini_er_bridge`/`sam3_bridge`/`graspgen_bridge` compris). `franka_demo_bringup/launch/franka_demo.launch.py` ne fait plus que démarrer RealSense et inclure ce launch file — les `Node()` des bridges qui y étaient directement listés ont été déplacés dans `robot_task_manager.launch.py`.
- `graspgen_bridge_node._rotation_matrix_to_quaternion` remplacé par `scipy.spatial.transform.Rotation.from_matrix(R).as_quat()`.
- `camera_bridge` : `SNDHWM=1` + watchdog de fraîcheur des frames (log `WARN` si aucune frame depuis >5s/10s).
- `.vscode/settings.json` (reliquat `franka_pick_interfaces`) n'existe plus dans le repo.
- `franka_demo_bringup/package.xml` : `exec_depend` redondants sur `gemini_er_bridge`/`sam3_bridge`/`graspgen_bridge` retirés — son launch file ne les référence plus directement (démarrés transitivement via `robot_task_manager.launch.py`, qui a déjà ses propres `exec_depend` dessus).
- `visualize_segmentation_node` déplacé du package `robot_task_manager` vers `sam3_bridge` (reste un node/service ROS2 séparé, `VisualizeSegmentation.srv` conservé) — mais son appelant change : ce n'est plus `pick_task_node` qui appelle `/visualize_segmentation`, c'est `sam3_bridge_node` lui-même (`_trigger_visualization`, fire-and-forget via `call_async`+`add_done_callback`, sans bloquer) juste après avoir reçu le masque de SAM3 dans `handle_segment_object`. `pick_task_node` n'a plus de client ni d'appel vers ce service (client + méthode `_visualize_segmentation` retirés). `robot_task_manager.launch.py` lançait alors les deux nodes de `sam3_bridge` (`sam3_bridge_node` + `visualize_segmentation_node`) directement en `Node()` (cf. entrée plus bas pour le passage ultérieur à `IncludeLaunchDescription` du launch file propre à `sam3_bridge`). `cv_bridge` ajouté aux dépendances de `sam3_bridge` — au passage, ce n'était pas uniquement pour `visualize_segmentation_node` : `sam3_bridge_node.py` l'utilisait déjà (`CvBridge()` pour encoder/décoder les masques) sans jamais le déclarer dans `package.xml`, un manque préexistant corrigé du même coup. (Une piste intermédiaire de fusion complète dans `sam3_bridge_node`, sans node séparé, a été explorée puis abandonnée au profit de ce découpage à deux nodes.)
- `camera_buffer_node` bufferise maintenant aussi le nuage natif RealSense (`/camera/camera/depth/color/points`) et le renvoie dans `GetFrames.srv` (champ `cloud`) — capturé au même instant que rgb/depth/camera_info, au lieu d'être récupéré séparément par `filter_pointcloud_node` après le round-trip SAM3 (correction d'un désalignement temporel masque/nuage, cf. "Architecture"). `FilterPointcloud.srv` prend désormais `cloud` en requête ; `filter_pointcloud_node` n'a plus de subscription ROS2 propre, il ne fait plus que filtrer le nuage reçu.
- `sam3_bridge` a maintenant son propre launch file (`launch/sam3_bridge.launch.py`, démarre `sam3_bridge_node` + `visualize_segmentation_node`), utilisable isolément via `ros2 launch sam3_bridge sam3_bridge.launch.py`. `robot_task_manager.launch.py` a été corrigé pour l'**inclure** (`IncludeLaunchDescription`) au lieu de lister ces deux `Node()` directement — même pattern imbriqué que `franka_demo_bringup` → `robot_task_manager`. `gemini_er_bridge` reste listé directement (pas de launch file dédié pour l'instant), `graspgen_bridge` a reçu le même traitement juste après (cf. entrée suivante).
- `filter_pointcloud_node` ne déprojette plus manuellement depth+intrinsèques `K` : la RealSense génère elle-même le nuage 3D (`pointcloud.enable:=true` ajouté au launch, en plus de `align_depth.enable:=true`), `filter_pointcloud_node` s'y abonne et filtre par le masque SAM3 via indexation directe du nuage organisé. `FilterPointcloud.srv` simplifié en conséquence (`depth`/`camera_info` retirés de la requête, ne reste que `mask`).
- `visualize_grasps_node` déplacé du package `robot_task_manager` vers `graspgen_bridge`, exactement selon le même pattern que `visualize_segmentation_node`/`sam3_bridge` : node/service ROS2 séparé conservé (`VisualizeGrasps.srv` inchangé), mais appelé par `graspgen_bridge_node` lui-même (`_trigger_visualization`, fire-and-forget via `service_is_ready()` + `call_async`/`add_done_callback`) juste après avoir construit sa réponse dans `handle_generate_grasp_pose`, plutôt que par `pick_task_node` (client + méthode `_visualize_grasps` retirés de `pick_task_node`). `visualization_msgs` déplacé de `robot_task_manager/package.xml` vers `graspgen_bridge/package.xml` (plus aucun autre usage dans `robot_task_manager` après le déplacement). `graspgen_bridge` a aussi reçu son propre launch file (`launch/graspgen_bridge.launch.py`, démarre `graspgen_bridge_node` + `visualize_grasps_node`, utilisable isolément via `ros2 launch graspgen_bridge graspgen_bridge.launch.py`), et `robot_task_manager.launch.py` a été corrigé pour l'inclure via `IncludeLaunchDescription` au lieu de lister ces deux `Node()` directement.
- `sam3_bridge_node` a maintenant un health-check au démarrage (`_check_sam3_server`, non-bloquant, même pattern que `_check_graspgen_server` dans `graspgen_bridge` : envoie `{'action': 'health'}`, attend jusqu'à 3s, `warn` + `_recreate_socket()` si `zmq.Again`) — l'incohérence entre les deux bridges notée en dette technique est résolue. **Non vérifié contre le vrai protocole du serveur SAM3** (code externe, hors de ce repo) : si le serveur SAM3 ne comprend pas `{'action': 'health'}`, le check le détectera quand même (`warn` sur statut inattendu ou timeout) sans rien casser, mais à confirmer/adapter dès qu'un test en conditions réelles est possible.
- `_trigger_visualization` (dans `sam3_bridge_node` **et** `graspgen_bridge_node`) utilisait `wait_for_service(timeout_sec=5.0)` (bloquant, jusqu'à 5s) pour vérifier la disponibilité du service de visualisation avant l'appel fire-and-forget — régression introduite après coup dans les deux fichiers, qui allait à l'encontre de l'objectif du design fire-and-forget (ne jamais ajouter de latence sur le chemin critique du pick). Remplacé dans les deux fichiers par `service_is_ready()` (non-bloquant, retourne juste `True`/`False` sans attendre), log inchangé (`error` "Service ... not available").
- `graspgen_bridge_host`/`graspgen_bridge_port` par défaut changés `172.22.62.94`/`5556` → `127.0.0.1`/`5558` pendant le développement local (même pattern que `sam3_bridge_host`) — CLAUDE.md racine mis à jour (table des ports ZMQ, flux du pick), le narratif "port 5556 partagé avec `command_bridge`" ne concerne donc plus la config par défaut actuelle. `m.patch()` (activation `msgpack_numpy`) déplacé du niveau module vers `GraspGenBridgeNode.__init__`.
- `CreatePointcloud.srv` renommé en `FilterPointcloud.srv` (service `create_pointcloud` → `filter_pointcloud`) : le node ne crée plus de nuage, il filtre celui reçu en requête (déjà vrai depuis le passage au nuage natif RealSense, cf. entrée plus haut — le nom ne reflétait plus le comportement réel). Renommage propagé partout : `.srv`, `CMakeLists.txt`, import/service côté `filter_pointcloud_node.handle_filter_pointcloud`, client/méthode côté `pick_task_node._filter_pointcloud`. Un rebuild complet de `franka_demo_interfaces` (suppression de `build/`/`install/`/`log/` du package) a été nécessaire pour éliminer les artefacts générés de l'ancien nom, qu'un `colcon build` incrémental ne nettoie pas tout seul.
- Le node `pointcloud_node` lui-même renommé en `filter_pointcloud_node` (fichier `pointcloud_node.py` → `filter_pointcloud_node.py`, classe `PointcloudNode` → `FilterPointcloudNode`, nom ROS2 du node, entry_point dans `setup.py`, `Node()` dans `robot_task_manager.launch.py`) — même logique que le renommage du service, pour rester cohérent de bout en bout. Même mésaventure d'artefacts périmés que pour `franka_demo_interfaces` : `build/`/`install/`/`log/` de `robot_task_manager` nettoyés pour faire disparaître l'ancien exécutable `pointcloud_node` de `install/`.
- `franka_demo_bringup/launch/franka_demo.launch.py` lance désormais lui-même les serveurs SAM3 et GraspGen (`ExecuteProcess` + `conda run -n <env> --no-capture-output`, chemins/env codés en dur pour ce poste de dev), attend qu'ils répondent healthy via un nouveau script `franka_demo_bringup/scripts/wait_for_zmq_health.py` (poll `{'action': 'health'}`/`{'status': 'ok'}`, même protocole que les health-checks des bridges) avant de démarrer RealSense/`robot_task_manager`, et arrête tout le launch (`Shutdown()`) si l'un des deux serveurs meurt à n'importe quel moment. Gemini ER (`gemini_er_simulator.py`) reste manuel, hors scope de ce changement. Cf. section "Serveurs externes" pour le détail — **non testé contre les vrais serveurs SAM3/GraspGen** (seul le protocole health-check et le mécanisme de launch ont été vérifiés dans cette session, avec un serveur ZMQ factice).
- **Filtrage de collision GraspGen ajouté** (cf. section "Filtrage de collision GraspGen" plus haut) : `FilterPointcloud.srv` (depuis renommé `CreatePointcloud.srv`, cf. entrée suivante) et `GenerateGraspPose.srv` gagnent chacun un champ `scene_cloud` (nuage "scène sans objet", calculé comme complément exact du masque SAM3, publié aussi sur le nouveau topic `/pick/scene_pointcloud`). `graspgen_bridge_node` le forwarde en `scene_point_cloud` au serveur GraspGen si non-vide (opt-in), avec deux nouveaux paramètres ROS2 (`collision_threshold`, `max_scene_points`). Côté serveur GraspGen (repo externe `~/Documents/FP3/GraspGen`, pas dans ce workspace) : `grasp_gen/serving/zmq_server.py` étendu pour pré-échantillonner la géométrie de collision de la pince au démarrage et appeler `filter_colliding_grasps_fast` (déjà utilisé par `scripts/demo_scene_pc.py --filter_collisions`, jamais exposé en ZMQ avant) quand `scene_point_cloud` est fourni ; `zmq_client.py` et `client-server/README.md` mis à jour en cohérence. Au passage, `graspgen_bridge_node.handle_generate_grasp_pose` détecte maintenant explicitement `result['error']` (au lieu du message générique "unexpected server response keys" quand le serveur répond une erreur). Vérifié de bout en bout avec le vrai serveur GraspGen (GPU) sur un nuage synthétique — non testé avec un vrai nuage RealSense/une vraie table.
- **Retour à la déprojection manuelle, abandon complet du nuage natif RealSense.** Après le fix `scene_cloud` ci-dessus, retour utilisateur : le nuage de l'objet segmenté ne colle pas au nuage natif affiché dans RViz (décalage en translation, pas une symétrie). Investigation en trois temps : (1) hypothèse "problème de frame" — écartée dans un premier temps, header/`frame_id` transmis tels quels sans transform nulle part dans le pipeline ; (2) `ros2 topic echo --field header.frame_id` sur le vrai matériel a en fait confirmé l'hypothèse sous un autre angle : `/camera/camera/depth/color/points` a pour `frame_id` `camera_depth_optical_frame`, alors que `aligned_depth_to_color` et `color/image_raw` ont `camera_color_optical_frame` — le nuage natif n'est *pas* dans le repère couleur malgré `align_depth.enable:=true` ; (3) recherche du wrapper `realsense-ros` (headers C++ locaux `/opt/ros/jazzy/include/{pointcloud_filter,align_depth_filter}.h` montrant deux blocs de filtre indépendants, confirmé par recherche web) → bug amont connu et non résolu, [issue #2595](https://github.com/IntelRealSense/realsense-ros/issues/2595) et [issue #3050](https://github.com/realsenseai/realsense-ros/issues/3050) : `align_depth.enable` + `pointcloud.enable` combinés produisent un nuage dont la géométrie est calculée indépendamment de l'alignement depth→couleur, décalage spatial de quelques cm — exactement le symptôme rapporté. Un simple recalage TF n'aurait pas suffi (le nuage est organisé selon la grille du capteur *depth*, pas *couleur* — même une transform rigide correcte n'aurait pas changé *quels* points l'indexation par le masque sélectionne).

  Décision : `pointcloud.enable` retiré du launch (`franka_demo_bringup/launch/franka_demo.launch.py`) ; `filter_pointcloud_node` renommé en `create_pointcloud_node` (retour au nom d'avant son renommage en sens inverse, cf. entrées plus haut — cette fois le nom redevient exact) et réécrit pour déprojeter manuellement depuis `aligned_depth_to_color` (garanti pixel-aligné sur la couleur par construction, fonctionnalité indépendante du bug pointcloud) + `camera_info` (`K`), en pinhole standard vectorisé numpy — élimine du même coup toute la classe de bugs `PointCloud2`/`row_step`/reshape rencontrée juste avant (plus aucun nuage externe parsé). `FilterPointcloud.srv` renommé `CreatePointcloud.srv`, requête changée (`mask`+`depth`+`camera_info` au lieu de `mask`+`cloud`) ; `camera_buffer_node` n'a plus de subscription au nuage natif (`_last_cloud`/`_cloud_callback` retirés) ; `GetFrames.srv` n'a plus de champ `cloud`. `pick_task_node._filter_pointcloud`/`filter_pointcloud_client` renommés `_create_pointcloud`/`create_pointcloud_client`. Rebuild complet nécessaire (`franka_demo_interfaces` et `robot_task_manager` nettoyés de leurs `build/`/`install/`/`log/`, même mésaventure d'artefacts périmés que les renommages précédents). **Déprojection vérifiée par test synthétique** (point principal, mise à l'échelle linéaire, gestion `depth=0`→`NaN`) — **non re-testé sur le vrai D455** après ce correctif (à confirmer au prochain lancement réel que le décalage a disparu).

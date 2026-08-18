# CLAUDE.md — `graspgen_bridge`

Ce fichier documente spécifiquement le package `graspgen_bridge`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Pont ZMQ REQ/REP vers le serveur GraspGen (génération de grasps à partir d'un nuage de points), et visualisation RViz des grasps obtenus. Deux nodes :

| Node | Fichier | Service exposé | Rôle |
|---|---|---|---|
| `graspgen_bridge` | `graspgen_bridge_node.py` | `generate_grasp_pose` | convertit le cloud en `xyz` numpy, appelle GraspGen via ZMQ REQ, convertit les matrices renvoyées en `PoseArray`, déclenche la visualisation |
| `visualize_grasps_node` | `visualize_grasps_node.py` | `visualize_grasps` | construit les markers RViz (flèches, meilleur grasp en vert) et les publie sur `/pick/grasp_markers` |

Les deux nodes sont démarrés par `robot_task_manager/launch/robot_task_manager.launch.py` dans le pipeline complet (via `IncludeLaunchDescription`). Le package a aussi son propre launch file, `launch/graspgen_bridge.launch.py`, pour les lancer isolément (debug) :

```bash
ros2 launch graspgen_bridge graspgen_bridge.launch.py
```

## `graspgen_bridge_node`

- Socket ZMQ **REQ**, créé via `_make_socket()` (`connect`, pas `bind`), recréé via `_recreate_socket()` après un `zmq.Again` — même pattern que `sam3_bridge_node` (cf. CLAUDE.md racine, section "piège msgpack").
- `graspgen_bridge_host` par défaut : `127.0.0.1`, `graspgen_bridge_port` par défaut : `5558` (changés depuis `172.22.62.94`/`5556` pendant le développement local — à repasser aux valeurs LAN réelles pour un déploiement hors poste de dev ; CLAUDE.md racine mis à jour en conséquence, cf. notes de cohérence plus bas).
- `num_grasps` (200) et `topk_num_grasps` (10) : paramètres ROS2 déclarés (pas de constantes magiques), transmis tels quels au serveur GraspGen dans la requête `infer`.
- `collision_threshold` (0.01 m — abaissé depuis `0.02` m par défaut : le filtre rejetait la totalité des grasps en usage réel, cf. `robot_task_manager/CLAUDE.md` "Dilatation du masque..." pour la cause principale ; ce paramètre reste le second levier) et `max_scene_points` (8192) : paramètres ROS2 déclarés pour le filtrage de collision (cf. ci-dessous). `graspgen_bridge_node` envoie toujours sa propre valeur explicitement dans la requête ZMQ (`_call_graspgen`), donc le défaut du serveur GraspGen (`grasp_gen/serving/zmq_server.py`, resté à `0.02` — repo externe générique, pas de raison de le changer là-bas) n'est en pratique jamais utilisé par ce pipeline.
- `_check_graspgen_server` : health-check non-bloquant au démarrage (`{'action': 'health'}`, timeout 3s) — même pattern que `_check_sam3_server` dans `sam3_bridge`.
- `handle_generate_grasp_pose` : convertit le `PointCloud2` reçu en `xyz` numpy (`_pointcloud2_to_numpy`), appelle GraspGen (timeout 60s, l'inférence peut être lente sur un gros nuage), convertit les matrices 4×4 renvoyées en `PoseArray` (`_matrix_to_pose_array`, quaternion via `scipy.spatial.transform.Rotation`), puis appelle `_trigger_visualization(...)` — **sans attendre le résultat** (fire-and-forget, même justification que `sam3_bridge_node` : exécuteur mono-thread `rclpy.spin()`, un appel bloquant recréerait un risque de deadlock). Détecte aussi explicitement `result['error']` dans la réponse ZMQ (message serveur surfacé tel quel dans `response.message`, au lieu du générique "unexpected server response keys" d'avant).

### Planner GraspMoE (`planner: "graspmoe"`)

`GenerateGraspPose` transmet désormais un champ `planner` au serveur GraspGen dans chaque requête `infer` (paramètre ROS2 `planner`, défaut **`"graspmoe"`**). Contexte : le sampler diffusion pur (`"diffusion"`, comportement d'origine) n'a aucune notion de "haut"/gravité — il ne génère quasiment que des grasps latéraux pour un objet compact posé sur une table, et ces grasps latéraux ont peu de garde au-dessus de la table, donc échouent souvent le filtrage de collision (`scene_cloud` ci-dessous) sur un petit objet. `"graspmoe"` (côté serveur GraspGen, `grasp_gen/samplers/graspmoe.py`, jusque-là utilisé uniquement par les scripts de démo, jamais exposé en ZMQ avant ce chantier — extension faite dans `~/Documents/FP3/GraspGen`, repo externe) additionne aux grasps diffusion des candidats **déterministes** balayés sur l'OBB (oriented bounding box) de l'objet : un grasp top-down garanti au-dessus de l'objet + les 4 faces latérales (`moe_obb_density: "dense-topandside"`, le défaut), tous scorés par le même discriminateur. `_call_graspgen` n'ajoute les paramètres `moe_*` (`moe_num_yaws`, `moe_z_offsets_cm`, `moe_outlier_threshold`, `moe_outlier_k`, `moe_obb_mode`, `moe_skip_obb_rule`, `moe_obb_density`, `moe_obb_position_spacing_cm`, tous déclarés comme paramètres ROS2) que si `planner == "graspmoe"`. Repasser `planner` à `"diffusion"` restaure exactement le comportement d'avant ce chantier.

Si la réponse contient `branch_tags` (présent uniquement en mode `"graspmoe"` — liste `"diff"`/`"obb"` par grasp, déjà découpée côté serveur si le filtrage de collision a éliminé des candidats), `handle_generate_grasp_pose` logue le compte diffusion/OBB (et si la branche OBB a été sautée, `skipped_obb` — arrive quand chaque étendue de l'OBB dépasse la largeur de la pince, cf. `moe_skip_obb_rule: "auto"`) — purement informatif, n'affecte jamais `response.grasps`/`response.scores` (toujours la même union brute qu'avant, `GenerateGraspPose.srv` inchangé).

**Non testé en conditions réelles** (GPU + vrai nuage RealSense) : seule la logique de dispatch côté serveur (`_handle_infer`) a été vérifiée avec un modèle mocké — la géométrie OBB elle-même (`grasp_gen/samplers/graspmoe.py`, code déjà existant dans GraspGen, non modifié par ce chantier) et son comportement sur un petit objet réel restent à confirmer au prochain lancement.

### Filtrage de collision (`scene_cloud` → `scene_point_cloud`)

`request.scene_cloud` (nouveau champ de `GenerateGraspPose.srv`, cf. CLAUDE.md racine "Filtrage de collision GraspGen") est converti en numpy et inclus dans la requête ZMQ sous `scene_point_cloud` **uniquement si `request.scene_cloud.data` est non-vide** — sinon le champ est simplement omis, comportement strictement identique à avant (le serveur GraspGen ne filtre rien si `scene_point_cloud` est absent). `_call_graspgen(xyz, scene_xyz=None)` porte cette logique conditionnelle.

Si le serveur a appliqué le filtrage (réponse contenant `num_grasps_before_collision_filter`), `handle_generate_grasp_pose` logue le ratio collision-free et `timing.collision_filter_ms` — purement informatif, n'affecte jamais `response.success`.

**Vérifié de bout en bout avec le vrai serveur GraspGen** (nuage objet + nuage scène synthétiques, cf. CLAUDE.md racine) — le champ `scene_point_cloud` construit ici correspond exactement à ce que le serveur attend. **Non testé avec le vrai `scene_cloud` produit par `create_pointcloud_node` (déprojection depth+K, cf. `robot_task_manager/CLAUDE.md`) sur un vrai depth RealSense.**
- `_trigger_visualization` / `_on_visualization_done` : client ROS2 vers `visualize_grasps`, vérifie `service_is_ready()` (non-bloquant) avant l'appel, log `info` si succès, `warn`/`error` si échec — mais ne fait jamais échouer `/generate_grasp_pose` lui-même.
- `m.patch()` (activation de `msgpack_numpy`, nécessaire pour sérialiser/désérialiser les `ndarray` du nuage de points et des matrices de grasp via msgpack) est appelé dans `__init__`, pas au niveau module.

## `visualize_grasps_node`

- Publisher `/pick/grasp_markers` (`visualization_msgs/MarkerArray`).
- `handle_visualize` : efface les anciens markers (`Marker.DELETEALL`), crée une flèche (`Marker.ARROW`) par grasp reçu — le meilleur (score max) en vert opaque, les autres en cyan semi-transparent — puis publie le tout en un seul `MarkerArray`.
- Logique de rendu inchangée depuis sa création dans `robot_task_manager` (avant son déplacement vers ce package).

## Améliorations possibles

### `graspgen_bridge_node`
1. **Pas de validation que `len(grasps) == len(scores)`** avant de construire `response.grasps`/`response.scores` et de déclencher la visualisation — si le serveur GraspGen renvoyait un jour des listes de tailles différentes, `visualize_grasps_node` calculerait `best_idx` à partir de `scores` mais l'appliquerait à l'indexation de `grasps`, ce qui pourrait colorer en vert un grasp qui n'est pas réellement le meilleur (pas de crash, mais résultat silencieusement incohérent).
2. **`msgpack.unpackb(raw)` sans `raw=False` explicite** dans `_check_graspgen_server` et `_call_graspgen` (contrairement à `sam3_bridge_node`, qui passe `raw=False` explicitement partout) — repose sur le défaut de la version de `msgpack` installée. Comportement correct aujourd'hui (`msgpack==1.2.1`, `raw=False` par défaut — cf. CLAUDE.md racine, piège msgpack), mais implicite : si `msgpack` est un jour rétrogradé vers une version pré-1.0, ce fichier casserait silencieusement (clés bytes au lieu de str) sans qu'aucun changement de code ne l'indique. Ajouter `raw=False` explicitement rendrait l'intention robuste au changement de version.

### `visualize_grasps_node`
1. **Bug confirmé (pas juste théorique) : `grasps` non vide + `scores` vide → `IndexError` masqué.** `handle_visualize` ne rejette que le cas `not grasps` (ligne 28) ; si `request.grasps.poses` est non vide mais `request.scores` est vide, `best_idx = scores.index(max(scores)) if scores else 0` retombe sur `0`, puis la ligne de log finale `f"score={scores[best_idx]:.3f}"` fait `scores[0]` sur une liste vide → `IndexError: list index out of range`. L'exception est rattrapée par le `except Exception` englobant (`response.success=False`, log `error`), donc le node ne plante pas, mais le message d'erreur renvoyé au client ("list index out of range") ne dit rien de la vraie cause (mismatch grasps/scores) — confusant à débugger. Plus généralement, même sans liste vide, un simple mismatch de longueur (`len(grasps) != len(scores)`, tous deux non vides) ne provoque pas d'erreur mais peut marquer en vert une pose qui ne correspond pas réellement au meilleur score. Ajouter une validation explicite `len(grasps) == len(scores)` en tout début de `handle_visualize` réglerait les deux cas avec un message d'erreur clair.

## Notes de cohérence avec le CLAUDE.md racine

- `graspgen_bridge_host`/`graspgen_bridge_port` par défaut changés `172.22.62.94`/`5556` → `127.0.0.1`/`5558` — CLAUDE.md racine mis à jour en conséquence (le narratif "port 5556 partagé avec `command_bridge`" ne s'applique donc plus au port par défaut actuel, uniquement à la config de déploiement LAN d'origine), mais **à corriger avant un déploiement réel** (le serveur GraspGen tourne sur une machine séparée, pas en local).

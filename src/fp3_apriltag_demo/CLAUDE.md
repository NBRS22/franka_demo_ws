# CLAUDE.md — `fp3_apriltag_demo`

Ce fichier documente spécifiquement le package `fp3_apriltag_demo`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Vérification **matérielle réelle** de la calibration eye-on-base (`handeye_tf_publisher`) : détecte un AprilTag, calcule sa pose 3D par `solvePnP`, la transforme en `fp3_link0` via TF, puis envoie cette pose comme unique candidat à `mtc_pick` (`command_router_node`, cf. `fp3_moveit_server`). Si le pick réussit (approche + fermeture de la pince + lift), c'est une confirmation physique forte que la calibration est bonne — en position **et** en profondeur, contrairement à un simple contrôle visuel du nuage de points dans RViz (cf. session de calibration de ce workspace : un décalage de quelques cm peut passer inaperçu à l'œil).

Porté depuis un package du même nom dans un autre workspace (`~/Documents/FP3/franka_demo_ws/src/fp3_apriltag_demo`), adapté à l'architecture actuelle :

- **Le calcul de pose (solvePnP) et le transform TF sont repris tel quel** — logique indépendante de l'architecture du pipeline.
- **La partie "déplacement" a changé** : l'ancienne version appelait une action `move_to_pose` (`motion_server_node`), supprimée depuis ce projet. Ici, `mtc_pick` fait déjà tout en un seul appel (`opening -> approaching -> grasping -> attaching -> lifting -> detaching`, cf. `fp3_moveit_server/CLAUDE.md`) — donc `apriltag_move_once_node` n'a plus besoin de ses propres clients d'action pince ni d'une étape séparée de retour à une pose "ready" ; il détecte le tag, calcule la pose, envoie un seul goal `mtc_pick`, logue le résultat.
- **`handeye_tf_publisher` est inclus directement dans le launch file** (`launch/apriltag_move_once.launch.py`), contrairement à l'ancienne version qui le supposait déjà lancé ailleurs — cohérent avec le fait que ce package sert spécifiquement à vérifier *cette* calibration.

## Lancement — package auto-contenu

`apriltag_move_once.launch.py` démarre **tout lui-même** : `fp3_moveit_server/bringup.launch.py` (move_group, ros2_control, scene_setup_node, pick_place_node, command_router_node), `realsense2_camera` (`align_depth.enable:=true`, `initial_reset:=true`, profils color/depth 1280x720x30, via `franka_demo_bringup/scripts/launch_realsense_with_retry.sh` — même mécanisme de retry qu'`franka_demo.launch.py`, cf. `franka_demo_bringup/CLAUDE.md` "Dépannage — RealSense a besoin d'un reset à chaque lancement"), `handeye_tf_publisher/publish.launch.py` (la calibration eye-on-base vérifiée), `apriltag_ros`/`apriltag_node` (mêmes remappings/params-file que pendant la session de calibration), et `apriltag_move_once_node`. Une seule commande suffit — rien à lancer séparément (contrairement à l'ancienne version de ce package dans l'autre workspace, qui supposait caméra+calibration déjà démarrées ailleurs). `exec_depend` sur `franka_demo_bringup` ajouté pour cette raison (le script est localisé via `FindPackageShare('franka_demo_bringup')`).

## Lancer la vérification

```bash
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1 \
  calibration_name:=<nom_du_.calib> \
  target_tag_id:=0 tag_size:=0.04
```

FCI doit être actif sur Desk (`use_fake_hardware:=false`). `calibration_name` doit correspondre à un fichier existant dans `~/.ros2/easy_handeye2/calibrations/` (cf. `handeye_tf_publisher/README.md` pour la calibration elle-même).

## `force_gripper_down`

Comme dans l'ancienne version : par défaut (`true`), seule la **position** calculée du tag est gardée, l'orientation est remplacée par une orientation fixe "droit vers le bas" (quaternion `(1,0,0,0)`, rotation de 180° autour de X — mappe l'axe d'approche local +Z de `pick_place_node` vers -Z monde). L'orientation native du tag est souvent inatteignable en IK ; ce n'est de toute façon pas ce qu'on veut vérifier ici (seule la position/profondeur compte pour juger la calibration).

## Comportement de `mtc_pick` sur cette pose — ce que ça teste vraiment

`mtc_pick` fait un **vrai grasp + lift** de ce qui se trouve à la pose du tag (pas juste une approche). Choix délibéré (cf. discussion de conception) : un grasp réussi est une confirmation plus forte qu'un simple survol — la précision en profondeur (pas seulement en position XY) doit être bonne pour que la pince ferme correctement sur l'objet. Implication pratique : le tag doit être monté sur un objet réellement saisissable par la pince (comme le cube utilisé pendant la calibration elle-même, cf. `handeye_tf_publisher/README.md`), pas juste posé à plat sur la table — sinon le grasp échouera pour des raisons indépendantes de la calibration.

`object.dimensions`/`object.id` (utilisés par `attachObject()` dans `pick_place_node`) restent ceux configurés globalement dans `fp3_moveit_server/config/pick_place.yaml` — ce package ne les override pas. Pour un test représentatif, s'assurer qu'ils correspondent approximativement à l'objet portant le tag.

## Non testé en conditions réelles

Ce package vient d'être porté/adapté dans ce workspace — jamais exécuté ici. À valider au premier lancement réel : que `mtc_pick` accepte bien la pose transformée, et que le message de succès/échec (`CALIBRATION CHECK PASSED`/`FAILED` dans les logs) reflète bien la réalité physique observée.

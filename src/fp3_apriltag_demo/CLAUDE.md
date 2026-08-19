# CLAUDE.md — `fp3_apriltag_demo`

Ce fichier documente spécifiquement le package `fp3_apriltag_demo`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Vérification **matérielle réelle** de la calibration eye-on-base (`handeye_tf_publisher`) : détecte un AprilTag, calcule sa pose 3D par `solvePnP`, la transforme en `fp3_link0` via TF, puis envoie cette pose comme unique candidat à `mtc_pick` (`command_router_node`, cf. `fp3_moveit_server`). Si le pick réussit (approche + fermeture de la pince + lift), c'est une confirmation physique forte que la calibration est bonne — en position **et** en profondeur, contrairement à un simple contrôle visuel du nuage de points dans RViz (cf. session de calibration de ce workspace : un décalage de quelques cm peut passer inaperçu à l'œil).

Porté depuis un package du même nom dans un autre workspace (`~/Documents/FP3/franka_demo_ws/src/fp3_apriltag_demo`), adapté à l'architecture actuelle :

- **Le calcul de pose (solvePnP) et le transform TF sont repris tel quel** — logique indépendante de l'architecture du pipeline.
- **La partie "déplacement" a changé** : l'ancienne version appelait une action `move_to_pose` (`motion_server_node`), supprimée depuis ce projet. Ici, `mtc_pick` fait déjà tout en un seul appel (`opening -> approaching -> grasping -> attaching -> lifting -> detaching`, cf. `fp3_moveit_server/CLAUDE.md`) — donc `apriltag_move_once_node` n'a plus besoin de ses propres clients d'action pince ni d'une étape séparée de retour à une pose "ready" ; il détecte le tag, calcule la pose, envoie un seul goal `mtc_pick`, logue le résultat.
- **`handeye_tf_publisher` est inclus directement dans le launch file** (`launch/apriltag_move_once.launch.py`), contrairement à l'ancienne version qui le supposait déjà lancé ailleurs — cohérent avec le fait que ce package sert spécifiquement à vérifier *cette* calibration.

## Ce qui n'est pas lancé par ce package

`realsense2_camera` et `apriltag_ros` (le node de détection) ne sont **pas** inclus dans `apriltag_move_once.launch.py` — à lancer séparément (mêmes commandes que pendant la session de calibration, cf. `handeye_tf_publisher/README.md`) :

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/camera/camera/color/image_raw \
  -r camera_info:=/camera/camera/color/camera_info \
  --params-file ~/franka_demo_ws/src/handeye_tf_publisher/tags/36h11_0_0.04.yaml
```

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

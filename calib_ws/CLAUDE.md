# CLAUDE.md — `calib_ws`

Workspace ROS2 (Jazzy) dédié à la calibration eye-on-base `fp3_link0 → camera_link` (D455 fixe, AprilTag rigide sur `fp3_hand`) de bout en bout : session de calibration, publication du résultat, et vérification/diagnostic — indépendant de `franka_demo_ws` (le pipeline de pick complet), qui n'a **pas été touché** lors de la création de ce workspace.

## Packages

| Package | Rôle |
|---|---|
| `calib_bringup` | Launch racine — une commande lance tout (caméra, apriltag, easy_handeye2, bras réel, tour de poses) |
| `handeye_tf_publisher` | Publie `fp3_link0 → camera_link` depuis un `.calib` easy_handeye2, + outil `watch_calibration_convergence.py` |
| `calib_pose_tour` | Fait parcourir au bras 20 poses (proches de la caméra, orientations variées) pour la prise d'échantillons |
| `calib_rigidity_test` | Diagnostic : le tag/cube est-il rigide sur `fp3_hand` (pas de glissement) ? |
| `calib_intrinsics_test` | Diagnostic intrinsèques caméra — 2 méthodes : `intrinsics_test` (déplacement du bras connu par FK) et `grid_intrinsics_check` (statique, sans bras, feuille de 9 AprilTags à distances connues) |
| `fp3_apriltag_demo` | Vérification physique par grasp+lift réel sur la pose calculée du tag |
| `easy_handeye2` / `easy_handeye2_msgs` | Clone externe — solveur de calibration (Tsai/Park/Horaud/Andreff/Daniilidis) |
| `apriltag_ros` | Clone externe — détection AprilTag |

**Doublons volontaires avec `franka_demo_ws`** : `handeye_tf_publisher`, `fp3_apriltag_demo`, `easy_handeye2(_msgs)`, `apriltag_ros` existent aussi dans `franka_demo_ws/src/` — décision explicite de l'utilisateur ("ne touche pas à ce ws, juste créer un nouveau") plutôt que déplacer/partager. Les deux copies peuvent diverger avec le temps ; pas de mécanisme de synchronisation automatique.

## Dépendances externes — PAS dupliquées, sourcées à côté

`fp3_moveit_server` (move_group, `pick_place_node`, `command_router_node`) et `franka_demo_interfaces` (action `MtcPick`, utilisée par `fp3_apriltag_demo`) restent uniquement dans `franka_demo_ws`. `franka_bringup`/`franka_description` restent dans `franka_ros2_ws`. Aucune dépendance circulaire au build (colcon ne vérifie pas les `exec_depend` à la compilation, seulement au lancement) — juste une contrainte de sourcing :

```bash
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
source $FP3_ROOT/calib_ws/install/setup.bash
```

Cet ordre (les 3 premiers avant `calib_ws`) fait aussi que la copie de `handeye_tf_publisher`/`fp3_apriltag_demo` de `calib_ws` **prend le dessus** sur celle de `franka_demo_ws` (avertissement `colcon build` attendu à ce sujet, sans conséquence).

## Build

```bash
cd $FP3_ROOT/calib_ws
colcon build
```

## Lancer une session de calibration complète

```bash
ros2 launch calib_bringup calib_bringup.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1
```

Démarre : `fp3_moveit_server/bringup.launch.py` (bras réel), la caméra (`scripts/launch_realsense_with_retry.sh` — `initial_reset:=true` + relance auto si la course de ré-énumération USB fait planter la première tentative, cf. historique dans `franka_demo_ws/src/franka_demo_bringup/CLAUDE.md`), `apriltag_node`, `easy_handeye2 calibrate.launch.py` (interface `rqt` de prise d'échantillons), puis après un délai de 15s (le temps que le reste démarre), `calib_pose_tour` qui fait parcourir au bras 20 poses en pause de 4s chacune pour la prise d'échantillons manuelle.

**Dans `rqt`** : sélectionner l'algorithme **Park** (pas Tsai-Lenz, le défaut) dans le menu déroulant avant de prendre le premier échantillon. Sauvegarder les échantillons bruts (`save_samples`, pas juste `save_calibration`) si tu veux pouvoir en rajouter plus tard sans tout refaire :
```bash
ros2 service call /easy_handeye2/calibration/save_samples easy_handeye2_msgs/srv/SaveSamples "{}"
```

`run_pose_tour:=false` pour une session manuelle classique sans le tour automatique (ex: juste re-sauvegarder une calibration existante).

## Publier une calibration

```bash
ros2 launch handeye_tf_publisher publish.launch.py \
  calibration_name:=<nom_du_.calib>
```

## Diagnostics

**Statique, sans bras** (juste une feuille imprimée de 9 AprilTags 36h11 ids 0-8, 4cm chacun, grille 3x3, lus gauche→droite haut→bas — `id = row*3 + col` —, espacement centre-à-centre 6cm en X et Y) :
```bash
ros2 launch calib_intrinsics_test intrinsics_grid_check.launch.py
```
Lance sa propre caméra + `apriltag_node` (config dédiée `tags/36h11_grid_3x3_0.04.yaml`, tous les ids 0-8) — autonome, rien d'autre à lancer avant. Calcule les 36 distances par paire de tags (repère caméra, `solvePnP`, aucune calibration main-œil impliquée), compare à la distance attendue selon la position dans la grille, et **sépare le résultat par direction** (horizontal / vertical / diagonal) plutôt qu'une moyenne globale — un biais Fx≠Fy peut s'annuler dans une moyenne globale mais pas si on regarde chaque axe séparément. Rapporte aussi un facteur d'échelle global (régression measured/expected).

**Avec bras** (`calib_bringup` ou manuellement) :
```bash
ros2 run calib_rigidity_test rigidity_test      # le tag glisse-t-il dans la pince ?
ros2 run calib_intrinsics_test intrinsics_test  # déplacement connu du bras vs mesure caméra
```

Les trois comparent contre une vérité terrain indépendante (géométrie connue de la feuille, ou cinématique directe du bras) — **aucun ne dépend de la calibration `fp3_link0 → camera_link` en cours de diagnostic**, exprès, pour ne pas fausser le résultat en présupposant ce qu'on cherche à vérifier.

## Vérification physique finale

```bash
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1 \
  calibration_name:=<nom_du_.calib>
```

Grasp+lift réel sur la pose calculée du tag — `CALIBRATION CHECK PASSED`/`FAILED` dans les logs. Cf. `fp3_apriltag_demo/CLAUDE.md` pour le détail (`force_gripper_down`, ce que le test valide vraiment).

## Historique de diagnostic (ce qui a déjà été écarté)

Sur ce montage, dans l'ordre testé :
1. **Intrinsèques caméra** : écartées (`calib_intrinsics_test` — écart de 1.75% sur un déplacement de 120mm, très inférieur à ce qu'il faudrait pour expliquer un biais de pick de plusieurs cm)
2. **Rigidité du montage tag/cube** : écartée (`calib_rigidity_test` — dérive du tag ≈ dérive propre de la main après retour en espace articulaire à la config de départ, pas de résidu inexpliqué)
3. **Solveur AX=XB "à une inconnue"** : le montage eye-on-base actuel (tag sur `fp3_hand`, caméra fixe) résout déjà l'offset gripper→tag comme sa propre inconnue — vérifié par dérivation mathématique directe et par lecture du code source `easy_handeye2` (`handeye_sampler.py`, le "trick" d'inversion pour `eye_on_base` est correct, pas un bug)
4. **Couverture de poses insuffisante** : piste principale restante, non encore confirmée/infirmée — `calib_pose_tour` a été construit pour ça (poses proches de la caméra, dans la vraie zone de travail des picks), mais une session avec ce tour a encore montré une "Maximum divergence" élevée à la dernière vérification — cause encore incertaine.

## Non testé

Ce workspace vient d'être créé — `calib_bringup.launch.py` n'a jamais été lancé de bout en bout (seule sa construction a été vérifiée, `--show-args`). Les packages `calib_pose_tour`/`calib_rigidity_test`/`calib_intrinsics_test` sont des copies fonctionnellement identiques de scripts déjà validés en conditions réelles sur ce robot (dans `franka_demo_ws`/scratchpad), mais pas encore ré-exécutés depuis leur nouvel emplacement dans `calib_ws`.

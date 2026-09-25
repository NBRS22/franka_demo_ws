# CLAUDE.md — `calib_ws`

Workspace ROS2 (Jazzy) dédié à la calibration eye-on-base `fp3_link0 → camera_link` (D455 fixe, AprilTag rigide sur `fp3_hand`) de bout en bout : session de calibration, publication du résultat, et vérification/diagnostic — indépendant de `franka_demo_ws` (le pipeline de pick complet), qui n'a **pas été touché** lors de la création de ce workspace.

## Packages

| Package | Rôle |
|---|---|
| `calib_bringup` | Launch racine — une commande lance tout (caméra, apriltag, easy_handeye2, bras réel) |
| `handeye_tf_publisher` | Publie `fp3_link0 → camera_link` depuis un `.calib` easy_handeye2, + outil `watch_calibration_convergence.py` |
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

Démarre : `fp3_moveit_server/bringup.launch.py` (bras réel), la caméra (`scripts/launch_realsense_with_retry.sh` — `initial_reset:=true` + relance auto si la course de ré-énumération USB fait planter la première tentative, cf. historique dans `franka_demo_ws/src/franka_demo_bringup/CLAUDE.md`), `apriltag_node` et `easy_handeye2 calibrate.launch.py` (interface `rqt` de prise d'échantillons).

**Le bras ne bouge jamais tout seul pendant une calibration** (le tour de poses automatique `calib_pose_tour` a été supprimé : dangereux, deux incidents matériels — violation de limite articulaire et collision). On place le bras à la main dans chaque pose, on attend que `calib_sample_guard` affiche `OK`, puis on prend l'échantillon dans `rqt` (≥ 15 poses variées, cf. `handeye_tf_publisher/README.md`).

**Dans `rqt`** : sélectionner l'algorithme **Park** (pas Tsai-Lenz, le défaut) dans le menu déroulant avant de prendre le premier échantillon. Sauvegarder les échantillons bruts (`save_samples`, pas juste `save_calibration`) si tu veux pouvoir en rajouter plus tard sans tout refaire :
```bash
ros2 service call /easy_handeye2/calibration/save_samples easy_handeye2_msgs/srv/SaveSamples "{}"
```

## Publier une calibration

```bash
ros2 launch handeye_tf_publisher publish.launch.py \
  calibration_name:=<nom_du_.calib>
```

## Vérification physique finale

```bash
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1 \
  calibration_name:=<nom_du_.calib>
```

Grasp+lift réel sur la pose calculée du tag — `CALIBRATION CHECK PASSED`/`FAILED` dans les logs. Cf. `fp3_apriltag_demo/CLAUDE.md` pour le détail (`force_gripper_down`, ce que le test valide vraiment).

## Historique de diagnostic (ce qui a déjà été écarté)

Sur ce montage, dans l'ordre testé (les packages de diagnostic `calib_intrinsics_test` et `calib_rigidity_test` ont depuis été supprimés de ce workspace, seules leurs conclusions restent) :
1. **Intrinsèques caméra** : écartées (`calib_intrinsics_test` — écart de 1.75% sur un déplacement de 120mm, très inférieur à ce qu'il faudrait pour expliquer un biais de pick de plusieurs cm)
2. **Rigidité du montage tag/cube** : écartée (`calib_rigidity_test` — dérive du tag ≈ dérive propre de la main après retour en espace articulaire à la config de départ, pas de résidu inexpliqué)
3. **Solveur AX=XB "à une inconnue"** : le montage eye-on-base actuel (tag sur `fp3_hand`, caméra fixe) résout déjà l'offset gripper→tag comme sa propre inconnue — vérifié par dérivation mathématique directe et par lecture du code source `easy_handeye2` (`handeye_sampler.py`, le "trick" d'inversion pour `eye_on_base` est correct, pas un bug)
4. **Couverture de poses insuffisante** : piste principale restante, non encore confirmée/infirmée — à traiter en guidant le bras à la main (poses proches de la caméra, dans la vraie zone de travail des picks, orientations variées). L'ancien tour automatique `calib_pose_tour` a été supprimé pour raison de sécurité.

## Non testé

Ce workspace vient d'être créé — `calib_bringup.launch.py` n'a jamais été lancé de bout en bout (seule sa construction a été vérifiée, `--show-args`).

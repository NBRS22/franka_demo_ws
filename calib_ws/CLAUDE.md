# CLAUDE.md — `calib_ws`

Workspace ROS2 (Jazzy) dédié à la calibration eye-on-base `fp3_link0 → camera_link` (D455 fixe, AprilTag rigide sur `fp3_hand`) de bout en bout : session de calibration, publication du résultat, et vérification/diagnostic — indépendant de `franka_demo_ws` (le pipeline de pick complet), qui n'a **pas été touché** lors de la création de ce workspace.

## Packages

Documentation complète (rôle de chaque package, setup, dépendances externes, procédure) : **`README.md`** de ce dossier.

| Package | Rôle |
|---|---|
| `calib_bringup` | Launch racine (caméra, apriltag, easy_handeye2, sample_guard, pile MoveIt optionnelle `start_arm_stack`) + `evaluate_calibration.launch.py` |
| `calib_sample_guard` | Garde-fou live à la prise d'échantillon (erreur de reprojection, inclinaison du tag) |
| `handeye_tf_publisher` | Publie `fp3_link0 → camera_link` depuis un `.calib`, configs de tags, `watch_calibration_convergence.py` |
| `calib_eye_in_hand` | Calibration de la D405 montée au poignet (voir son `CLAUDE.md`) |
| `calib_bridge` | Dérive la calibration D455 depuis celle de la D405 (voir son `CLAUDE.md`) |
| `fp3_apriltag_demo` | Amène `fp3_hand_tcp` sur la pose du tag (pas de grasp) — le bras bouge seul (voir son `CLAUDE.md`) |
| `src/external/` (git-ignoré) | `apriltag_ros`, `easy_handeye2`, `easy_handeye2_msgs` — clonés par `scripts/install_dependencies.sh` aux versions de `calib.repos` |

**Doublon restant avec `franka_demo_ws`** : `handeye_tf_publisher` existe aussi là-bas dans une version légèrement différente (à fusionner).

## Dépendances

`fp3_moveit_server` + `franka_fp3_moveit_config` (pile MoveIt, optionnelle via `start_arm_stack`) viennent de `franka_demo_ws` ; `franka_bringup`/`franka_description` viennent de `franka_ros2_ws` (Franka, non versionné, cf. `../README.md`). Ordre de sourcing obligatoire :

```bash
source /opt/ros/jazzy/setup.bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
source $FP3_ROOT/calib_ws/install/setup.bash
```

Installation des dépendances externes : `scripts/install_dependencies.sh` (cf. `README.md`, section 2).

## Build

```bash
cd $FP3_ROOT/calib_ws
colcon build --symlink-install
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

Amène `fp3_hand_tcp` sur la pose calculée du tag (pince ouverte, aucun grasp) ; comparer ensuite avec `ros2 run tf2_ros tf2_echo tag36h11:0 fp3_hand_tcp` (translation ≈ 0). **Le bras bouge seul.** Cf. `fp3_apriltag_demo/CLAUDE.md` pour le détail.

## Historique de diagnostic (ce qui a déjà été écarté)

Sur ce montage, dans l'ordre testé (les packages de diagnostic `calib_intrinsics_test` et `calib_rigidity_test` ont depuis été supprimés de ce workspace, seules leurs conclusions restent) :
1. **Intrinsèques caméra** : écartées (`calib_intrinsics_test` — écart de 1.75% sur un déplacement de 120mm, très inférieur à ce qu'il faudrait pour expliquer un biais de pick de plusieurs cm)
2. **Rigidité du montage tag/cube** : écartée (`calib_rigidity_test` — dérive du tag ≈ dérive propre de la main après retour en espace articulaire à la config de départ, pas de résidu inexpliqué)
3. **Solveur AX=XB "à une inconnue"** : le montage eye-on-base actuel (tag sur `fp3_hand`, caméra fixe) résout déjà l'offset gripper→tag comme sa propre inconnue — vérifié par dérivation mathématique directe et par lecture du code source `easy_handeye2` (`handeye_sampler.py`, le "trick" d'inversion pour `eye_on_base` est correct, pas un bug)
4. **Couverture de poses insuffisante** : piste principale restante, non encore confirmée/infirmée — à traiter en guidant le bras à la main (poses proches de la caméra, dans la vraie zone de travail des picks, orientations variées). L'ancien tour automatique `calib_pose_tour` a été supprimé pour raison de sécurité.

## Non testé

Ce workspace vient d'être créé — `calib_bringup.launch.py` n'a jamais été lancé de bout en bout (seule sa construction a été vérifiée, `--show-args`).

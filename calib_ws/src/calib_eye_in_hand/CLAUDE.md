# CLAUDE.md — calib_eye_in_hand

## Pourquoi ce package

L'ancienne vérification de la calibration eye-on-base (D455, `fp3_apriltag_demo`) s'est révélée
**circulaire** : la cible commandée EST la pose du tag mesurée via la calibration testée elle-même
— donc "le TF colle" ne prouve que la précision d'exécution du bras, pas que la calibration reflète
la vraie position physique du tag. Constat utilisateur : "l'explication est simple la calibration
est mauvaise" (confirmé indépendamment par ~10 recalibrations sans amélioration).

Solution retenue : calibrer une **deuxième caméra**, montée au poignet (D405, eye-in-hand), de
façon totalement indépendante de la calibration D455 sous test. Si les deux caméras s'accordent sur
la pose d'un même tag fixe dans le monde, la calibration D455 est validée sans circularité.

## État actuel — étape 1 seulement (calibration D405 seule)

`launch/calibrate_eye_in_hand.launch.py` calibre le D405 en eye-in-hand contre un **tag fixe** posé
quelque part dans l'espace de travail — l'image miroir de la session D455 (là-bas le tag bouge avec
l'effecteur devant une caméra fixe ; ici la caméra bouge avec l'effecteur devant un tag fixe).

- **`robot_effector_frame` = `fp3_hand`** : la Franka Hand a été **remontée** (le bracket D405 vient
  en plus, pas à la place — la piste "démonter la Hand" évoquée plus tôt n'a finalement pas été
  retenue). `load_gripper:=true` par défaut dans ce launch file, comme le défaut de
  `fp3_moveit_server` lui-même. Ce choix de frame aligne aussi les deux calibrations (D455 et D405)
  sur la même frame effecteur de référence (`fp3_hand`, déjà utilisée par la session D455 via
  `calib_bringup.launch.py`'s `robot_effector_frame`).
- **Même tag physique que la session D455** (36h11 id 0) — délibéré, pas un oubli : l'étape 2
  (cross-check, pas encore construite) aura besoin d'un seul tag de référence commun aux deux caméras.
- **Caméra non-namespacée** (`/camera/camera/...`, comme la session D455) : cette étape calibre le
  D405 seul, D455 débranché/non lancé — pas de collision de topics à gérer ici. Le namespacing
  (`camera_name:=`/`camera_namespace:=`) n'est nécessaire qu'à l'étape 2, quand les deux caméras
  tournent simultanément.
- **Pas de `calib_pose_tour` équivalent pour cette géométrie** : ce node-là déplace l'**effecteur**
  pour qu'il fasse face à une **caméra fixe** — le problème inverse de celui-ci (ici la caméra est
  sur l'effecteur, le tag est fixe). Prise d'échantillons **manuelle** pour l'instant : jog du bras
  (Desk, ou MoveIt Servo/rqt motion planning via le bringup `fp3_moveit_server` déjà inclus) en
  gardant le tag dans le champ du D405, même règles d'échantillonnage que la session D455 (README
  `handeye_tf_publisher` : ≥3 axes de rotation non-parallèles, éviter les tilts >60°, éviter les
  vues quasi de face).
- Profil D455 (1280x720) volontairement pas repris : D405 est courte portée
  (~7-50cm), profil réduit à 848x480, **depth désactivé** (`enable_depth:=false`) — seuls
  `color`/`camera_info` sont utilisés par `apriltag_node`.

## Pas encore fait (étape 2 — cross-check)

- `cross_check_node` (package non encore écrit) : les deux caméras + les deux `apriltag_node`
  tournant **simultanément** → collision de frame TF si les deux publient `tag36h11:0` par défaut.
  Plan retenu : ce node s'abonnera directement aux `/detections` bruts de chaque caméra
  (`apriltag_msgs/AprilTagDetectionArray`) + `camera_info`, fera lui-même le solvePnP/composition TF
  par caméra (réutiliser `estimate_tag_pose_camera_frame`/`_rotation_to_quat` de
  `calib_axis_test/axis_test_node.py`) — jamais dépendre du TF broadcast natif d'`apriltag_node` pour
  la comparaison finale (celui-ci reste utile, seul, pendant la calibration eye-in-hand elle-même).
- Il faudra alors namespacer le D405 (`camera_name:=d405 camera_namespace:=d405`, `serial_no` requis
  pour chaque appareil) puisque les deux caméras seront branchées en même temps.
- `easy_handeye2`'s `handeye_publisher` (déjà lu, réutilisable tel quel pour publier le TF statique
  `fp3_link8 -> camera_color_optical_frame` côté D405, cf. son cas `eye_in_hand` : `orig =
  robot_effector_frame`) + le `handeye_tf_publisher` custom existant côté D455 (cas eye-on-base,
  compose via `camera_link`) — les deux tournent en parallèle au moment du cross-check.

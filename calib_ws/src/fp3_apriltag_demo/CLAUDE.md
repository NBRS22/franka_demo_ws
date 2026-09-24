# CLAUDE.md — `fp3_apriltag_demo`

Ce fichier documente spécifiquement le package `fp3_apriltag_demo` dans `calib_ws`. Pour la vue d'ensemble du workspace, voir le `CLAUDE.md` à sa racine.

## Rôle du package

Vérification **matérielle réelle** de la calibration eye-on-base (`handeye_tf_publisher`) : détecte un AprilTag, calcule sa pose 3D par `solvePnP`, la transforme en `fp3_link0` via TF, puis déplace **directement** `fp3_hand_tcp` à cette pose (position, et orientation si `force_gripper_down:=false`) — **pince ouverte, aucun grasp, aucun lift**. Une fois arrivé, on compare `fp3_hand_tcp` au frame TF du tag lui-même (`tag36h11:<id>`, publié en continu par `apriltag_node` tant que le tag reste visible) — dans RViz ou avec `tf2_echo`.

## Pourquoi pas de grasp (différence avec la copie de `franka_demo_ws`)

Ce package existe aussi dans `franka_demo_ws/src/fp3_apriltag_demo/` (copie volontairement dupliquée, cf. `CLAUDE.md` racine de `calib_ws`) — cette version-là fait un vrai grasp+lift via `mtc_pick`. **La copie `calib_ws` a été simplifiée pour ne plus jamais grasper** : un grasp mélange deux questions indépendantes — "la calibration place-t-elle la cible au bon endroit" et "le grasp mécanique a-t-il réussi" (peut échouer pour plein de raisons sans rapport avec la calibration : le cube glisse, mauvaise largeur de prise, etc.). S'arrêter pince ouverte permet une comparaison directe et **continue** (le tag reste détecté en direct tant qu'il est visible, donc la comparaison reste valide bien après la fin du mouvement — pas juste un instantané) :

```bash
ros2 run tf2_ros tf2_echo tag36h11:<id> fp3_hand_tcp
```

Translation qui tend vers `[0,0,0]` une fois le bras arrivé et immobile = calibration correcte à cet endroit précis. Pas besoin d'interpréter un succès/échec de grasp.

`apriltag_move_once_node` n'a donc plus de client d'action `mtc_pick` ni de dépendance sur `franka_demo_interfaces` — juste un `ActionClient` direct sur `/move_action` (même pattern que `calib_axis_test`/`calib_click_test`) plus un `ActionClient` sur `/franka_gripper/move` pour ouvrir explicitement la pince avant de bouger (garantit qu'elle reste ouverte, peu importe son état précédent).

## Lancement — package auto-contenu

`apriltag_move_once.launch.py` démarre **tout lui-même** : `fp3_moveit_server/bringup.launch.py` (move_group, ros2_control, scene_setup_node — `pick_place_node`/`command_router_node` sont démarrés aussi mais ne sont plus utilisés par ce node), la caméra (`align_depth.enable:=true`, `initial_reset:=true`, profils 1280x720x30, via `scripts/launch_realsense_with_retry.sh` — copie locale, pas de dépendance croisée vers `franka_demo_ws`), `handeye_tf_publisher/publish.launch.py` (la calibration vérifiée), `apriltag_ros`/`apriltag_node`, et `apriltag_move_once_node`. Une seule commande suffit.

## Lancer la vérification

```bash
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1 \
  calibration_name:=<nom_du_.calib> \
  target_tag_id:=0 tag_size:=0.04
```

### `camera:=d455` (défaut) vs `camera:=d405`

Le node (`apriltag_move_once_node`) est agnostique de la caméra — topics `/detections`/
`/camera/camera/color/camera_info` paramétrables, transform final via TF générique
(`tf_buffer.transform(pose_camera, 'fp3_link0', ...)`). Ce qui différait selon la caméra a été
extrait dans le launch file, sélectionné par `camera:=`:

- **`d455`** (eye-on-base, `handeye_tf_publisher`) : D455 en 1280x720, `align_depth`+`pointcloud`
  activés (visuel RViz seulement), publisher = `handeye_tf_publisher/publish.launch.py` (compose
  via `camera_link`).
- **`d405`** (eye-in-hand, `calib_eye_in_hand`) : D405 en 848x480, pas de depth (inutile pour
  `apriltag_node`), publisher = `easy_handeye2/publish.launch.py` (un seul TF statique
  `fp3_hand -> camera_color_optical_frame`, pas de composition nécessaire pour ce type de
  calibration — cf. `calib_eye_in_hand/CLAUDE.md`). `calibration_name` doit alors être le nom
  sauvegardé par la session `calib_eye_in_hand` (ex. `fp3_hand_d405_camera_color_optical_frame_001`).

```bash
ros2 launch fp3_apriltag_demo apriltag_move_once.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1 \
  camera:=d405 calibration_name:=fp3_hand_d405_camera_color_optical_frame_001 \
  target_tag_id:=0 tag_size:=0.04
```

Même caveat de circularité que pour `d455` (cf. section suivante) : le fait que `fp3_hand_tcp`
arrive pile sur `tag36h11:0` ne prouve que la cohérence execution/calibration D405 ensemble, pas
que la calibration D405 reflète la vraie position physique du tag — pour ça, comparer plutôt
l'estimation D405 à celle du D455 sur le même tag (étape 2 du plan, pas encore construite,
cf. `calib_eye_in_hand/CLAUDE.md`). Utile ici surtout pour détecter une erreur grossière (frame
inversée, signe, mauvais solveur) avant d'aller plus loin.

FCI doit être actif sur Desk. `calibration_name` doit correspondre à un fichier existant dans `~/.ros2/easy_handeye2/calibrations/`.

Une fois le log `CALIBRATION CHECK: arrived` affiché : compare `fp3_hand_tcp` et `tag36h11:<id>` dans RViz (display TF), ou :
```bash
ros2 run tf2_ros tf2_echo tag36h11:<id> fp3_hand_tcp
```

## `force_gripper_down`

**Défaut changé à `false`** (c'était `true` dans l'ancienne version grasp-based) : sans grasp, il n'y a plus de raison de jeter l'orientation native du tag — la comparer directement à celle de `fp3_hand_tcp` est justement l'intérêt de ce test (détecte un biais de rotation dans la calibration, complémentaire du test chiffré `calib_axis_test`). Mettre à `true` si l'orientation native du tag s'avère cinématiquement inatteignable (la comparaison de position seule reste valide dans les deux cas).

## Non testé en conditions réelles

Cette version simplifiée (sans grasp) vient d'être écrite — jamais exécutée. À valider au premier lancement réel : que `/move_action` accepte bien la pose transformée, que la pince s'ouvre correctement avant le mouvement, et que la comparaison TF `tag36h11:<id>` vs `fp3_hand_tcp` donne un résultat interprétable.

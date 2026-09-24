# CLAUDE.md — calib_bridge

## Rôle

Dérive la calibration eye-on-base du D455 (`fp3_link0 -> camera_color_optical_frame`) à partir de
la calibration eye-in-hand du D405 déjà obtenue (`calib_eye_in_hand`), **sans jamais relancer le
solveur AX=XB d'`easy_handeye2` sur le D455** — cf. `calib_eye_in_hand/CLAUDE.md` pour le contexte
(~10 tentatives de calibration D455 directe sans résultat physiquement correct, et le constat que
le test de vérification `fp3_apriltag_demo` était circulaire).

## Méthode

Les deux caméras observent le **même tag fixe** en même temps :

```
fp3_link0 -> tag  =  (fp3_link0 -> fp3_hand)        [FK, live]
                   x  (fp3_hand -> D405 optical)      [calib_eye_in_hand, .calib chargé]
                   x  (D405 optical -> tag)            [détection D405, live]

fp3_link0 -> D455 optical  =  (fp3_link0 -> tag) x (D455 optical -> tag)^-1
                                                        [détection D455, live, inversée]
```

Le D405 + FK donnent une mesure indépendante de "où est le tag dans `fp3_link0`" (indépendante car
elle ne dépend jamais de la calibration D455 en cours de dérivation) ; inverser la détection D455 du
même tag donne alors directement l'extrinsèque D455 — pas de solveur, juste de la composition de
transforms. Vérifié numériquement (test synthétique, écart ~1e-16) avant tout test réel.

## Topics

Les deux caméras tournent sous un namespace commun `franka` avec un `camera_name` distinct
(`camera_namespace:=franka camera_name:=d455`/`d405`) : topics sous `/franka/d455/...` et
`/franka/d405/...`, y compris les `/detections` de chaque `apriltag_node` (chacun namespacé
`franka/d455`/`franka/d405` en conséquence). **Le D455 reçoit en plus `tf_prefix:=camera`** :
sans ça, `camera_name:=d455` renommerait aussi ses frames TF en `d455_color_optical_frame`,
cassant la compatibilité avec le `.calib` D455 existant (`tracking_base_frame:
camera_color_optical_frame`, sauvé quand cette caméra était lancée sans namespace) et avec tout ce
qui relance le D455 seul ensuite (`handeye_tf_publisher`, `evaluate_calibration.launch.py
camera:=d455`, `fp3_apriltag_demo camera:=d455`). Le D405 n'a pas ce problème : son frame_id
(`d405_color_optical_frame`, dérivé de son propre `camera_name`) n'est jamais lu par
`bridge_calibration_node` (aucune lookup TF sur son nom, cf. section suivante), donc pas besoin de
le forcer à un nom particulier — et il ne le FAUT pas non plus : le forcer à `camera_...` comme le
D455 créerait une vraie collision (deux caméras vivantes revendiquant le même frame_id).

## Pourquoi pas le TF broadcast natif d'`apriltag_node`

Les deux instances `apriltag_node` (une par caméra) tournent **simultanément** ici (contrairement à
`calib_eye_in_hand`/`fp3_apriltag_demo`, une seule caméra à la fois) — si les deux publiaient TF pour
`tag36h11:0`, cette frame aurait deux parents différents (un par caméra), conflit TF réel. Solution :
`bridge_calibration_node` fait son propre `solvePnP` directement depuis `/detections` (coins 2D, pas
de pose 3D native dans `apriltag_msgs`, cf. `fp3_apriltag_demo/apriltag_move_once_node.py`) — jamais
de lookup TF sur la frame du tag. Le launch file donne quand même à l'instance D405 un
`tag.frames` différent (`tag36h11_d405:0`) pour éviter le conflit côté RViz/visuel, même si ce n'est
pas strictement nécessaire pour le calcul lui-même.

## Utilisation

```bash
source $FP3_ROOT/franka_ros2_ws/install/setup.bash
source $FP3_ROOT/franka_demo_ws/install/setup.bash
source $FP3_ROOT/calib_ws/install/setup.bash

# Trouver les numéros de série si besoin (obligatoire une fois les deux caméras branchées ensemble --
# contrairement à calib_eye_in_hand/fp3_apriltag_demo, "n'importe quel appareil" est ambigu ici) :
rs-enumerate-devices -s

ros2 launch calib_bridge calib_bridge.launch.py \
  use_fake_hardware:=false robot_ip:=192.168.1.1 \
  d455_serial_no:=<...> d405_serial_no:=<...> \
  d405_calibration_name:=fp3_hand_d405_camera_color_optical_frame_001
```

Puis, à **plusieurs poses différentes du bras** (tag visible par les deux caméras à chaque fois, bras
immobile pendant l'appel — cf. note FK ci-dessous) :

```bash
ros2 service call /calib_bridge/take_sample std_srvs/srv/Trigger {}
```

Le node loggue l'écart (translation/rotation) par rapport à la moyenne courante dès le 2e échantillon
— outil de qualité approximatif, même esprit que le "Maximum divergence" d'`easy_handeye2`. Viser
plusieurs poses **vraiment différentes** (pas la même pose répétée) : le bruit de détection ne
s'annule que si les erreurs sont décorrélées d'un échantillon à l'autre.

Une fois satisfait :

```bash
ros2 service call /calib_bridge/save_calibration std_srvs/srv/Trigger {}
```

Écrit `~/.ros2/easy_handeye2/calibrations/<output_calibration_name>.calib` (défaut
`fp3_link0_d455_camera_color_optical_frame_derived_001` — **nom différent de l'existant**, exprès,
pour pouvoir comparer les deux avant de remplacer). Utilise directement
`easy_handeye2.handeye_calibration.save_calibration()` (même mécanisme que `easy_handeye2` lui-même,
`message_to_yaml` sur un message `HandeyeCalibration`) — format de fichier garanti identique à un
`.calib` produit normalement, donc utilisable tel quel avec `handeye_tf_publisher`,
`evaluate_calibration.launch.py` (`camera:=d455 calibration_name:=<nom_dérivé>`), ou
`fp3_apriltag_demo` (`camera:=d455 calibration_name:=<nom_dérivé>`).

## Pourquoi la lookup FK utilise "latest available", pas le stamp de la détection

Comme dans `fp3_apriltag_demo` (même bug rencontré et corrigé), `fp3_link0 -> fp3_hand` est une
transform **vivante** (dépend de `/joint_states`), dont le taux de publication peut être en retard
par rapport au stamp d'une détection caméra — d'où `rclpy.time.Time()` (stamp zéro = dernière
transform disponible) plutôt que le stamp exact. Ça suppose implicitement que **le bras est immobile**
pendant l'appel à `take_sample` — cohérent avec l'usage prévu (un `take_sample` discret par pose, pas
une collecte continue pendant le mouvement).

## Champs de métadonnées du `.calib` produit

`robot_effector_frame` dans le fichier de sortie est repris du `robot_effector_frame` du `.calib`
D405 chargé (`fp3_hand` normalement) — uniquement pour que le schéma du fichier reste identique à un
`.calib` eye-on-base classique (cf. `fp3_link0_d455_camera_color_optical_frame_001.calib` existant,
mêmes champs). Ce node ne l'utilise jamais dans son propre calcul (qui ne fait aucun AX=XB, donc pas
besoin d'un tag monté sur l'effecteur).

## Non testé en conditions réelles

Écrit et vérifié uniquement par un test synthétique (maths de composition de transforms) dans cette
session — jamais exécuté contre les deux vraies caméras/le vrai bras. À valider au premier lancement
réel : que les deux flux RealSense démarrent correctement en simultané (bande passante USB, il se
peut qu'il faille les brancher sur des contrôleurs USB différents), que le TF `fp3_link0 -> fp3_hand`
reste assez frais entre deux `take_sample`, et que le résultat dérivé soit effectivement plus
précis que la calibration D455 directe existante (comparer les deux via
`evaluate_calibration.launch.py` et/ou `fp3_apriltag_demo`).

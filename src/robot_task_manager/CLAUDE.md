# CLAUDE.md — `robot_task_manager`

Ce fichier documente spécifiquement le package `robot_task_manager`. Pour la vue d'ensemble du pipeline complet (autres packages, flux ZMQ, roadmap), voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Orchestration du pick : bufferise les frames caméra, gère la déprojection/génération du pointcloud, et le node séquenceur (`pick_task_node`) qui appelle les autres services du pipeline dans l'ordre. Son launch file (`launch/robot_task_manager.launch.py`) démarre aussi les bridges d'autres packages (`gemini_er_bridge`, `sam3_bridge`, `graspgen_bridge`) — cf. `package.xml` (`exec_depend`) et le `CLAUDE.md` racine pour le détail.

**Note** : ni `visualize_segmentation_node` (overlay RViz de la segmentation) ni `visualize_grasps_node` (markers RViz des grasps) ne sont dans ce package — déplacés respectivement vers `sam3_bridge` et `graspgen_bridge`, toujours comme nodes/services séparés (`visualize_segmentation`, `visualize_grasps`). Dans les deux cas, ce n'est plus `pick_task_node` qui les appelle : c'est le node producteur du résultat correspondant (`sam3_bridge_node` pour le masque, `graspgen_bridge_node` pour les grasps) qui appelle le service en interne, dès que le résultat est prêt (fire-and-forget, sans bloquer). `pick_task_node` n'a plus aucune trace de ces deux services.

## Nodes

| Node | Fichier | Service exposé | Rôle |
|---|---|---|---|
| `camera_buffer` | `camera_buffer_node.py` | `get_frames` | dernier RGB+depth (aligné)+camera_info bufferisés |
| `pick_task_node` | `pick_task_node.py` | `execute_pick_task` | séquenceur du pick (appelle tous les autres services) |
| `create_pointcloud_node` | `create_pointcloud_node.py` | `create_pointcloud` | déprojette l'objet (et la scène) depuis depth+K, masqué par SAM3 |

## `camera_buffer_node`

S'abonne à 3 topics RealSense et bufferise en mémoire (`_last_rgb`, `_last_depth`, `_last_camera_info`) uniquement le dernier message reçu de chacun — pas d'historique, pas de queue métier. Le service `get_frames` renvoie l'instantané courant des 3 buffers en un seul appel, mais **sans vérifier qu'ils correspondent au même instant caméra** (cf. "Synchronisation RGB/depth" ci-dessous — régression à corriger).

Topics souscrits :
- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

**N'a plus de subscription au nuage natif RealSense** (`/camera/camera/depth/color/points`, `_last_cloud`/`_cloud_callback` retirés) — `GetFrames.srv` n'a plus de champ `cloud`. Ce topic n'est plus utilisé nulle part dans le pipeline : `pointcloud.enable` n'est même plus activé au launch (`franka_demo_bringup`), cf. "Pourquoi plus de nuage natif RealSense" plus bas.

### QoS

Les 3 subscriptions utilisent le QoS par défaut de `create_subscription` (profondeur `10`, `RELIABLE`) — **pas** de profil explicite (ex. `QoSPresetProfiles.SENSOR_DATA`, `BEST_EFFORT`) actuellement.

Point d'attention : en ROS2/DDS, le QoS du subscriber doit être *compatible* avec celui du publisher, sinon la souscription ne reçoit **silencieusement rien** (pas d'erreur, pas de warning — le topic existe dans `ros2 topic list`, mais rien n'arrive jamais). Un subscriber `RELIABLE` exige un publisher `RELIABLE` ; un subscriber `BEST_EFFORT` accepte les deux. Si le driver `realsense2_camera` publie un jour un de ces 3 topics en `BEST_EFFORT` (ou si le node est utilisé avec une autre source d'images utilisant ce profil), `camera_buffer_node` resterait bloqué à `_last_* = None` sans log d'erreur explicite — seul `handle_get_frames` renverrait `success=False` avec un message générique ("no RGB frame available", etc.), sans indiquer que la cause est un mismatch QoS.

À vérifier une fois la RealSense branchée : `ros2 topic info -v /camera/camera/color/image_raw` (et les 2 autres topics) pour confirmer la compatibilité réelle.

### Synchronisation RGB/depth

**⚠️ Régression : `handle_get_frames` ne vérifie plus aucune synchronisation temporelle.** Une version antérieure de ce node rejetait la requête (`success=False`) si `rgb` et `depth` étaient désynchronisés de plus de 100ms (`_SYNC_TOLERANCE_S`, comparaison de `header.stamp`) — ce check a disparu du code à un moment de ce chantier, sans qu'aucun changement d'architecture documenté ne l'explique (contrairement aux suppressions volontaires de ce repo, toutes tracées dans l'Historique du CLAUDE.md racine). `handle_get_frames` renvoie désormais `success=True` dès que les 3 buffers sont non-`None`, sans comparer leurs timestamps entre eux.

Risque concret : un `depth` arbitrairement plus vieux que le `rgb` envoyé à SAM3 (ex. caméra qui rame temporairement, republish partiel) passerait silencieusement jusqu'à la déprojection — masque et depth désalignés, sans aucun log d'alerte.

**Amélioration recommandée** : restaurer un check de gap temporel `rgb`/`depth` avant de considérer ce node terminé.

## `create_pointcloud_node`

Renommé depuis `filter_pointcloud_node` (qui avait lui-même été renommé depuis `create_pointcloud_node` dans un refactor précédent — cf. Historique CLAUDE.md racine pour le détail de cet aller-retour et pourquoi il était justifié dans les deux sens). **Déprojette manuellement** — n'utilise plus le nuage natif RealSense du tout.

### Pourquoi plus de nuage natif RealSense

Testé en direct sur ce setup, confirmé par recherche : avec `align_depth.enable:=true` + `pointcloud.enable:=true` combinés, `/camera/camera/depth/color/points` a un **bug non résolu côté `realsense-ros`** — sa géométrie est calculée indépendamment de l'alignement depth→couleur, et se retrouve décalée spatialement (quelques cm, en translation) par rapport à la vraie position des objets. Confirmé de deux façons : (1) `frame_id` du topic natif = `camera_depth_optical_frame`, alors que `aligned_depth_to_color`/`color/image_raw` ont `camera_color_optical_frame` — donc le nuage n'est *pas* dans le repère couleur malgré `align_depth.enable:=true` ; (2) documenté upstream sans fix officiel : [issue #2595](https://github.com/IntelRealSense/realsense-ros/issues/2595), [issue #3050](https://github.com/realsenseai/realsense-ros/issues/3050). Une simple correction TF n'aurait pas suffi : le nuage est organisé selon la grille du capteur *depth* (résolution/FOV différents du capteur couleur), donc l'indexation par le masque SAM3 (basé sur l'image couleur) sélectionnerait de toute façon les mauvais points, peu importe le repère dans lequel leurs coordonnées XYZ sont exprimées.

Cette découverte a mis fin à une chaîne de trois bugs de parsing successifs rencontrés en essayant de faire coller ce nuage natif au masque (nuage non organisé, reshape `(width,height)` inversé, padding `row_step` non lu — tous dans `sensor_msgs_py`, lib ROS2 Jazzy, pas ce repo, cf. Historique CLAUDE.md racine pour le détail complet de cette investigation) : ces trois correctifs étaient chacun de vrais bugs, mais ne pouvaient de toute façon jamais résoudre le décalage de fond, puisqu'il venait du driver RealSense, pas du parsing côté ROS2. `pointcloud.enable` n'est donc plus activé du tout au launch (`franka_demo_bringup/launch/franka_demo.launch.py`).

### Déprojection manuelle

`handle_create_pointcloud` prend en requête (`CreatePointcloud.srv`, renommé depuis `FilterPointcloud.srv`) `mask` (SAM3) + `rgb` + `depth` (`aligned_depth_to_color`, garanti pixel-aligné sur la couleur *par construction* — fonctionnalité mature et indépendante du bug pointcloud ci-dessus) + `camera_info` (intrinsèques `K`, `float64[9]` row-major : `fx=k[0]`, `cx=k[2]`, `fy=k[4]`, `cy=k[5]`). Déprojection pinhole standard, vectorisée numpy (pas de boucle par pixel) dans `_deproject_xyz` :
```
depth_m = depth_raw (uint16, mm) * 0.001, NaN où depth_raw == 0 (pas de mesure valide)
X = (u - cx) * depth_m / fx
Y = (v - cy) * depth_m / fy
Z = depth_m
```
Élimine toute la classe de bugs `PointCloud2`/`row_step`/reshape rencontrée avec l'ancienne approche, puisqu'aucun nuage externe n'est plus parsé — la géométrie est calculée directement depuis l'image depth, dont on contrôle nous-mêmes l'indexation `(height, width)`.

**Vérifié par test synthétique** (point principal `(cx,cy)` → `(0,0,Z)`, mise à l'échelle linéaire pour un pixel excentré, `depth=0` → `NaN`) — **non re-testé sur le vrai D455** après ce correctif (à confirmer au prochain lancement réel que le décalage précédemment rapporté a disparu).

### Nuage coloré (`x`/`y`/`z`/`rgb`)

Les deux nuages publiés — objet (`/pick/pointcloud`, `response.cloud`) **et** scène (`/pick/scene_pointcloud`, `response.scene_cloud`) — portent une couleur par point. `_pack_rgb_float32` convertit `request.rgb` (BGR8 via `cv_bridge`) en un tableau `(height, width)` de `float32` dont les bits encodent en réalité un entier 24 bits `(r<<16)|(g<<8)|b` (`.view(np.float32)`, **reinterprétation de bits, pas un cast numérique** — piège classique si on utilisait `.astype()` à la place), calculé une seule fois et réutilisé pour les deux nuages. `_xyzrgb_cloud` construit chaque `PointCloud2` via un dtype structuré `[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')]` + `PointField` custom (`_XYZRGB_FIELDS`), au lieu de `pc2.create_cloud_xyz32`. C'est la convention PCL/RViz standard pour les nuages colorés (color transformer `RGB8` de RViz) — la même que celle du nuage natif RealSense, d'où la demande initiale ("avoir le nuage créé en RGB8 comme celui publié par la caméra").

Le masque objet (`object_mask`) et le masque scène (`scene_mask`) combinent chacun leur condition sur `mask_raw` **et** la validité du depth (`np.isfinite(xyz).all(axis=2)`, calculée une fois dans `valid` et réutilisée par les deux) en une seule passe, pour garder `xyz`/`rgb` alignés élément par élément lors de l'indexation — remplace l'ancien filtrage en deux temps (indexer par le masque, puis filtrer les `NaN` séparément), qui aurait désynchronisé les deux tableaux si fait en deux passes indépendantes. Le sous-échantillonnage de la scène (`_MAX_SCENE_POINTS`) applique le **même** `idx` à `scene_points` et `scene_colors` pour rester alignés après le `np.random.choice`.

**Vérifié par tests synthétiques** : round-trip du packing RGB (bits reconstruits identiques après `.view()`, cross-check indépendant via `struct.pack`/`unpack` Python) et round-trip complet à travers un vrai message `PointCloud2` (`pc2.create_cloud` + `pc2.read_points`), sur l'objet **et** sur la scène (vérifié que `scene_points.shape[0]` == total moins objet moins pixels invalides, et que l'extraction xyz-only reste cohérente après coloration) ; confirmé que `graspgen_bridge_node._pointcloud2_to_numpy` (qui sélectionne explicitement `field_names=('x','y','z')`) n'est affecté par le champ `rgb` sur aucun des deux nuages envoyés à GraspGen. **Non testé avec une vraie image RealSense** (couleurs réelles, pas juste des valeurs synthétiques).

### Nuage de scène (collision context pour GraspGen)

`handle_create_pointcloud` calcule aussi `xyz[dilated_mask == 0]` — le complément du masque objet **dilaté**, c-à-d. tout ce qui n'est *pas* l'objet ciblé (ni sa marge de sécurité), donc table/sol/autres objets. Filtré des `NaN` (même logique que le nuage objet), sous-échantillonné à `_MAX_SCENE_POINTS = 20_000` points si besoin (`np.random.choice`, sans remise), publié sur le topic `/pick/scene_pointcloud` et renvoyé dans `response.scene_cloud` (champ de `CreatePointcloud.srv`). Sert de contexte de collision pour GraspGen — cf. CLAUDE.md racine, "Filtrage de collision GraspGen" pour le flux complet jusqu'au serveur externe.

Ce calcul est *best-effort* : un nuage de scène vide (`scene_points.shape[0] == 0`) n'est **pas** une erreur — `response.success` reste `True`, `response.scene_cloud` est juste un `PointCloud2` vide. C'est `graspgen_bridge_node` qui décide, en aval, d'inclure ou non `scene_point_cloud` dans la requête GraspGen selon que ce nuage contient des points ou non.

#### Dilatation du masque avant exclusion de la scène

**Bug corrigé (usage réel) : le filtre de collision GraspGen rejetait la totalité des grasps** (`0/N collision-free`). Cause : `mask_raw == 0` collait le nuage de scène directement au bord du masque SAM3 — or ce bord n'est précis qu'à quelques pixels près, et le bruit de profondeur RealSense ajoute encore un peu de flou pile à cet endroit. Résultat : des points de scène qui ne sont en réalité que des artefacts de bord de l'objet lui-même se retrouvaient à quelques mm des points de contact réels du grasp — exactement là où `filter_colliding_grasps_fast` (cf. `graspgen_bridge_node`) cherche des collisions — donc presque tout grasp physiquement valide était rejeté comme faux positif.

Corrigé : `_dilate_mask(mask_raw, margin_px)` (`cv2.dilate`, noyau `(2*margin_px+1)²`) dilate le masque objet d'une marge (`scene_exclusion_margin_px`, paramètre ROS2, défaut `15` px) **avant** de calculer `scene_mask` — le nuage objet lui-même (`object_mask`) reste construit sur le masque **non dilaté** (la marge ne sert qu'à donner de la marge de sécurité au filtrage de collision, pas à faire "grossir" l'objet segmenté). Réglable sans recompiler : `ros2 param set /create_pointcloud_node scene_exclusion_margin_px <valeur>`.

**Vérifié par test synthétique** (masque carré 20×20, marge 10px) : l'anneau de 10px autour de l'objet est bien exclu du nuage de scène après dilatation, le nuage objet reste inchangé. **Non re-testé avec le vrai pipeline GraspGen** — à confirmer au prochain lancement réel que `Collision filter : X/Y grasps collision-free` (log `graspgen_bridge_node`) redevient non-nul. `collision_threshold` (paramètre ROS2 de `graspgen_bridge_node`, cf. CLAUDE.md `graspgen_bridge`) a aussi été abaissé de `0.02` à `0.01`m en parallèle — plus petit = filtre plus permissif, contre-intuitif. Si `0/Y` persiste malgré les deux correctifs combinés, augmenter encore `scene_exclusion_margin_px` et/ou baisser encore `collision_threshold`.

**Vérifié par test synthétique** (déprojection sur un depth synthétique + masque partiel) : `object_points`/`scene_points` restent des complémentaires exacts (avant dilatation). **Non testé avec un vrai depth RealSense/une vraie scène** (table, objets, bruit capteur réel).

## Dette technique spécifique à `camera_buffer_node`

- **Aucune vérification de synchronisation temporelle** (cf. ci-dessus) — à restaurer.
- Même en restaurant un check RGB/depth simple (gap entre les deux stamps), il ne détecterait pas la staleness *globale* : si la caméra se déconnecte, `_last_rgb` et `_last_depth` resteraient tous les deux figés sur la dernière frame reçue — leur écart mutuel resterait ~0, donc le check passerait alors que les données peuvent être arbitrairement anciennes. Ajouter une vérification contre l'horloge courante (`self.get_clock().now()`) en plus du gap RGB/depth réglerait aussi ce problème.

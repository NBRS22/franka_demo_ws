# CLAUDE.md — `robot_task_manager`

Ce fichier documente spécifiquement le package `robot_task_manager`. Pour la vue d'ensemble du pipeline complet (autres packages, flux ZMQ, roadmap), voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Orchestration du pick : bufferise les frames caméra, gère la génération/filtrage du pointcloud, et le node séquenceur (`pick_task_node`) qui appelle les autres services du pipeline dans l'ordre. Son launch file (`launch/robot_task_manager.launch.py`) démarre aussi les bridges d'autres packages (`gemini_er_bridge`, `sam3_bridge`, `graspgen_bridge`) — cf. `package.xml` (`exec_depend`) et le `CLAUDE.md` racine pour le détail.

**Note** : ni `visualize_segmentation_node` (overlay RViz de la segmentation) ni `visualize_grasps_node` (markers RViz des grasps) ne sont dans ce package — déplacés respectivement vers `sam3_bridge` et `graspgen_bridge`, toujours comme nodes/services séparés (`visualize_segmentation`, `visualize_grasps`). Dans les deux cas, ce n'est plus `pick_task_node` qui les appelle : c'est le node producteur du résultat correspondant (`sam3_bridge_node` pour le masque, `graspgen_bridge_node` pour les grasps) qui appelle le service en interne, dès que le résultat est prêt (fire-and-forget, sans bloquer). `pick_task_node` n'a plus aucune trace de ces deux services.

## Nodes

| Node | Fichier | Service exposé | Rôle |
|---|---|---|---|
| `camera_buffer` | `camera_buffer_node.py` | `get_frames` | dernier RGB+depth+camera_info+cloud bufferisés |
| `pick_task_node` | `pick_task_node.py` | `execute_pick_task` | séquenceur du pick (appelle tous les autres services) |
| `filter_pointcloud_node` | `filter_pointcloud_node.py` | `filter_pointcloud` | filtre le nuage reçu en requête par le masque SAM3 |

## `camera_buffer_node`

S'abonne à 4 topics RealSense et bufferise en mémoire (`_last_rgb`, `_last_depth`, `_last_camera_info`, `_last_cloud`) uniquement le dernier message reçu de chacun — pas d'historique, pas de queue métier. Le service `get_frames` renvoie l'instantané courant des 4 buffers en un seul appel, mais **sans vérifier qu'ils correspondent au même instant caméra** (cf. "Synchronisation RGB/depth/pointcloud" ci-dessous — régression à corriger).

Topics souscrits :
- `/camera/camera/color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`)
- `/camera/camera/color/camera_info` (`sensor_msgs/CameraInfo`)
- `/camera/camera/depth/color/points` (`sensor_msgs/PointCloud2`, nécessite `pointcloud.enable:=true` en plus de `align_depth.enable:=true` au launch RealSense)

### QoS

Les 4 subscriptions utilisent le QoS par défaut de `create_subscription` (profondeur `10`, `RELIABLE`) — **pas** de profil explicite (ex. `QoSPresetProfiles.SENSOR_DATA`, `BEST_EFFORT`) actuellement.

Point d'attention : en ROS2/DDS, le QoS du subscriber doit être *compatible* avec celui du publisher, sinon la souscription ne reçoit **silencieusement rien** (pas d'erreur, pas de warning — le topic existe dans `ros2 topic list`, mais rien n'arrive jamais). Un subscriber `RELIABLE` exige un publisher `RELIABLE` ; un subscriber `BEST_EFFORT` accepte les deux. Si le driver `realsense2_camera` publie un jour un de ces 4 topics en `BEST_EFFORT` (ou si le node est utilisé avec une autre source d'images utilisant ce profil), `camera_buffer_node` resterait bloqué à `_last_* = None` sans log d'erreur explicite — seul `handle_get_frames` renverrait `success=False` avec un message générique ("no RGB frame available", etc.), sans indiquer que la cause est un mismatch QoS.

À vérifier une fois la RealSense branchée : `ros2 topic info -v /camera/camera/color/image_raw` (et les 3 autres topics) pour confirmer la compatibilité réelle.

### Synchronisation RGB/depth/pointcloud

**⚠️ Régression : `handle_get_frames` ne vérifie plus aucune synchronisation temporelle.** Une version antérieure de ce node rejetait la requête (`success=False`) si `rgb` et `depth` étaient désynchronisés de plus de 100ms (`_SYNC_TOLERANCE_S`, comparaison de `header.stamp`) — ce check a disparu du code actuel sans qu'aucun changement d'architecture documenté ne l'explique (contrairement aux suppressions volontaires de ce repo, qui sont toutes tracées dans l'Historique du CLAUDE.md racine). `handle_get_frames` renvoie désormais `success=True` dès que les 4 buffers (`_last_rgb`, `_last_depth`, `_last_camera_info`, `_last_cloud`) sont non-`None`, sans comparer leurs timestamps entre eux ni avec `cloud`.

Risque concret : un `depth` (ou `cloud`) arbitrairement plus vieux que le `rgb` envoyé à SAM3 (ex. caméra qui rame temporairement, republish partiel) passerait silencieusement jusqu'au filtrage du nuage — masque et géométrie désalignés, sans aucun log d'alerte.

**Amélioration recommandée** : restaurer un check de gap temporel `rgb`/`depth` (et envisager d'étendre à `cloud`, avec éventuellement une tolérance plus large vu le traitement supplémentaire du pointcloud RealSense) avant de considérer ce node terminé.

## Dette technique spécifique à `camera_buffer_node`

- **Aucune vérification de synchronisation temporelle** (cf. ci-dessus) — à restaurer.
- Même en restaurant un check RGB/depth simple (gap entre les deux stamps), il ne détecterait pas la staleness *globale* : si la caméra se déconnecte, `_last_rgb` et `_last_depth` resteraient tous les deux figés sur la dernière frame reçue — leur écart mutuel resterait ~0, donc le check passerait alors que les données peuvent être arbitrairement anciennes. Ajouter une vérification contre l'horloge courante (`self.get_clock().now()`) en plus du gap RGB/depth réglerait aussi ce problème pour `cloud`.

# CLAUDE.md — `franka_demo_bringup`

Ce fichier documente spécifiquement le package `franka_demo_bringup`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Package `ament_python` **launch-only** (aucun node ROS2, aucun `entry_points`) : point d'entrée racine du pipeline complet. Il embarque désormais un script utilitaire (`scripts/wait_for_zmq_health.py`, installé via `data_files` — pas un entry_point) mais reste sans node/exécutable ROS2 déclaré.

`launch/franka_demo.launch.py` :
1. Démarre les serveurs externes SAM3 et GraspGen (`ExecuteProcess` + `conda run -n <env> --no-capture-output`, cf. "Serveurs externes" plus bas), puis attend qu'ils répondent healthy sur leur port ZMQ (`wait_for_zmq_health.py`, gate séquentiel).
2. Une fois les deux serveurs healthy : démarre RealSense D455 via `scripts/launch_realsense_with_retry.sh` (`ExecuteProcess`, pas un `IncludeLaunchDescription` — cf. "Dépannage — RealSense a besoin d'un reset à chaque lancement" plus bas pour le pourquoi), avec `align_depth.enable:=true`, `initial_reset:=true` et un profil color/depth (`rgb_camera.color_profile`/`depth_module.depth_profile`) — cf. "Résolution RealSense" plus bas. **`pointcloud.enable` n'est volontairement pas activé** : bug non résolu de `realsense-ros` quand combiné à `align_depth.enable` (nuage spatialement décalé de quelques cm) — cf. CLAUDE.md racine, dette technique "Important" et `robot_task_manager/CLAUDE.md`. Le pipeline déprojette lui-même depuis `aligned_depth_to_color` (`create_pointcloud_node`) au lieu de dépendre du nuage natif RealSense.
3. Inclut (`IncludeLaunchDescription`) `robot_task_manager/launch/robot_task_manager.launch.py`, qui démarre en cascade tous les autres nodes ROS2 du pipeline (bridges compris).
4. En parallèle, tout le temps que le launch tourne : si `sam3_server` ou `graspgen_server` meurt (crash, pas seulement au démarrage), un `OnProcessExit` dédié émet un `Shutdown()` qui arrête tout l'arbre de launch.

```bash
ros2 launch franka_demo_bringup franka_demo.launch.py
```

## Serveurs externes (SAM3 / GraspGen) — auto-lancés depuis ce launch file

Ajouté pour que le pipeline complet (`ros2 launch franka_demo_bringup franka_demo.launch.py`) n'exige plus de lancer SAM3/GraspGen à la main avant, et pour que le launch s'arrête proprement si l'un des deux tombe (au lieu de continuer à tourner avec un bridge qui échouera silencieusement à chaque appel).

### Démarrage des process serveurs

`_conda_run_cmd(env_name, workdir, *command)` construit `['bash', '-c', f'cd {workdir} && exec conda run -n {env_name} --no-capture-output {command...}']` — `exec` remplace le process bash par `conda run` pour que les signaux (Ctrl+C, `Shutdown()`) atteignent le bon PID, et `--no-capture-output` évite que `conda run` bufferise/masque la sortie du process (nécessaire pour `output='screen'` et pour un forwarding correct des signaux/codes de retour).

Constantes en tête de fichier (`franka_demo.launch.py`), codées en dur pour ce poste de dev — **à adapter si le repo est cloné ailleurs** :

| Constante | Valeur | Rôle |
|---|---|---|
| `SAM3_DIR` | `/home/ngr/Documents/FP3/SAM3` | `cwd` avant `conda run` |
| `SAM3_CONDA_ENV` | `SAM3` | nom de l'env conda |
| `SAM3_HOST`/`SAM3_PORT` | `127.0.0.1`/`5557` | pour le health-check (doit matcher `sam3_bridge_host`/`sam3_bridge_port` de `sam3_bridge_node`) |
| `GRASPGEN_DIR` | `/home/ngr/Documents/FP3/GraspGen` | `cwd` avant `conda run` |
| `GRASPGEN_CONDA_ENV` | `GraspGen` | nom de l'env conda |
| `GRASPGEN_HOST`/`GRASPGEN_PORT` | `127.0.0.1`/`5558` | pour le health-check (doit matcher `graspgen_bridge_host`/`graspgen_bridge_port` de `graspgen_bridge_node`) |

Commandes lancées : `python -m sam3_server` (SAM3) et `python client-server/graspgen_server.py` (GraspGen) — telles que données par l'utilisateur pour un lancement manuel équivalent, sans argument de port (les serveurs sont supposés déjà écouter sur 5557/5558 par défaut, cohérent avec les défauts actuels des bridges).

### Health-check gate

`scripts/wait_for_zmq_health.py` (nouveau fichier, installé dans `share/franka_demo_bringup/scripts/` via `data_files` dans `setup.py`, appelé en `ExecuteProcess(cmd=['python3', <chemin installé>, '--name', ..., '--host', ..., '--port', ..., '--timeout', '180'])`) : socket ZMQ REQ, envoie `{'action': 'health'}`, retry toutes les 2s (`--retry-interval`) jusqu'à recevoir `{'status': 'ok'}` ou dépasser `--timeout` (180s par défaut). Exit code 0 si healthy, 1 sinon. Même protocole que `_check_sam3_server`/`_check_graspgen_server` dans les bridges, mais recréé ici en script standalone (pas de dépendance `rclpy`, juste `zmq`+`msgpack`) car ce n'est pas un node ROS2.

Gate séquentiel dans `generate_launch_description()` : `wait_for_sam3` démarre en même temps que les deux serveurs ; à sa sortie, un `OnProcessExit` (callback `_on_wait_exit`) regarde `event.returncode` — succès (`0`) → démarre `wait_for_graspgen` ; échec/timeout → émet `Shutdown()`. Même logique enchaînée sur `wait_for_graspgen`, qui à son succès démarre `realsense` + `robot_task_manager` (les deux `IncludeLaunchDescription`/`GroupAction` ne sont donc **pas** dans la liste initiale de `LaunchDescription`, seulement retournés dynamiquement par ce handler). Comme les deux serveurs ont déjà été lancés en parallèle dès le début, sérialiser les *checks* (SAM3 puis GraspGen) n'ajoute pas de latence significative — le temps d'attente total est dominé par le plus lent des deux chargements de modèle, pas par leur somme.

### Arrêt en cascade si un serveur meurt

Deux `RegisterEventHandler(OnProcessExit(target_action=<sam3_server|graspgen_server>, on_exit=[EmitEvent(event=Shutdown(...))]))` indépendants du gate de santé ci-dessus : ils surveillent le process serveur lui-même pendant toute la durée de vie du launch (pas seulement au démarrage), donc un crash de SAM3/GraspGen **en plein milieu d'un pick** déclenche aussi l'arrêt de tout l'arbre (RealSense, bridges, `robot_task_manager`). Un arrêt normal (Ctrl+C → `Shutdown()` global → tous les process reçoivent SIGINT, y compris `sam3_server`/`graspgen_server`) redéclenche ces mêmes handlers, qui ré-émettent un `Shutdown()` — sans effet néfaste, `launch` tolère plusieurs émissions.

### Non vérifié / limites connues

- **Jamais testé contre les vrais serveurs SAM3/GraspGen** (session de dev sans CUDA/hardware disponible pour ce chantier) — seul le protocole `wait_for_zmq_health.py` a été vérifié de bout en bout contre un serveur ZMQ REQ/REP factice, et `generate_launch_description()` vérifié pour ne pas lever d'exception à la construction. Rien ne garantit que `python -m sam3_server` / `client-server/graspgen_server.py` sont bien les bonnes commandes une fois les vrais repos SAM3/GraspGen à jour, ni qu'ils écoutent bien sur 5557/5558 sans argument.
- **`conda run` et propagation des signaux** : réputation connue de mal forwarder certains signaux à son sous-processus dans d'anciennes versions de conda, ce qui pourrait laisser un process serveur orphelin après un `Shutdown()`/Ctrl+C plutôt que de le tuer proprement. À vérifier (`ros2 launch franka_demo_bringup franka_demo.launch.py`, Ctrl+C, puis `ps aux | grep -E 'sam3_server|graspgen_server'`) dès que testable.
- Gemini ER (`gemini_er_simulator.py`, env conda `ER`) **n'est pas concerné** par ce changement — reste lancé manuellement, ce n'est pas un serveur ZMQ à health-checker de la même façon (script de test interactif).

## Résolution RealSense

`REALSENSE_COLOR_PROFILE`/`REALSENSE_DEPTH_PROFILE` (constantes en tête de `franka_demo.launch.py`, format `'WIDTHxHEIGHTxFPS'` — la valeur par défaut sentinelle `'0,0,0'` de `rs_launch.py` signifie "profil auto" du firmware, pas utilisée ici) valent `'1280x720x30'` par défaut — résolution native max du D455 commune aux profils color et depth (au-delà, 1920×1080 n'existe que côté color, pas depth). Passés à l'`IncludeLaunchDescription` de `rs_launch.py` via `launch_arguments['rgb_camera.color_profile']`/`['depth_module.depth_profile']`. Color et depth sont volontairement gardés au **même** profil : avec `align_depth.enable:=true`, le depth aligné (`aligned_depth_to_color`) suit la résolution couleur — `create_pointcloud_node` déprojette ce depth aligné en supposant `depth.shape` == résolution couleur (cf. CLAUDE.md racine, "Convention de coordonnées").

**Historique de cette valeur** : abaissée temporairement à `'848x480x30'` puis `'424x240x30'` en pensant réduire le débit pour stabiliser le flux, pendant un épisode de déconnexions USB de la caméra — la vraie cause s'est révélée être la gestion d'énergie USB (`power/control=auto`, cf. "Dépannage — déconnexions USB" plus bas), pas une histoire de débit. Une fois ce fix appliqué, remontée à la résolution native max.

**Aucun autre changement de code nécessaire pour que la résolution soit dynamique** : vérifié par recherche exhaustive (`grep` sur tout `src/`) qu'aucun node ROS2 du pipeline ne hardcode une largeur/hauteur — `create_pointcloud_node.handle_create_pointcloud` lit `depth_raw.shape` du message reçu en requête, `sam3_bridge_node._numpy_mask_to_image_msg` reconstruit le masque depuis `mask_shape` renvoyé par le serveur SAM3 (donc dérivé de la taille du JPEG envoyé, lui-même dérivé de l'image RGB reçue). Changer `REALSENSE_COLOR_PROFILE`/`REALSENSE_DEPTH_PROFILE` ici est donc suffisant — pas besoin de toucher `sam3_bridge`, `graspgen_bridge`, ni `robot_task_manager`.

**Testé sur hardware réel** (D455 branchée) : `'848x480x30'` puis `'1280x720x30'` sont des profils valides (`sam3_server` reçoit bien des images à la bonne résolution, cf. logs). L'exploration de `pointcloud.enable`/`pointcloud.ordered_pc` (essayée à ce stade pour obtenir un nuage natif RealSense organisé) s'est révélée être une fausse piste : `align_depth.enable` + `pointcloud.enable` combinés est un bug non résolu de `realsense-ros` (nuage spatialement décalé, indépendant de l'alignement depth→couleur) — cf. CLAUDE.md racine, dette technique "Important", et `robot_task_manager/CLAUDE.md` pour le détail de l'investigation. `pointcloud.enable` a été retiré des `launch_arguments` ; le pipeline déprojette désormais lui-même depuis `aligned_depth_to_color` (`create_pointcloud_node`), qui n'a besoin que d'`align_depth.enable:=true`.

## Dépannage — déconnexions USB de la D455 (fix système, hors repo)

Symptôme observé sur ce poste de dev : la RealSense se déconnecte de façon aléatoire pendant le streaming (`dmesg` : `usb 2-N: USB disconnect` suivi d'une réénumération sur un nouveau numéro de device), régulièrement accompagné d'un `segfault` du process `realsense2_camera` (le driver plante plutôt que de gérer proprement la perte du device en plein DMA). Le controller négocie bien du SuperSpeed (`speed: 5000`, bus `xhci_hcd` AMD 500 Series Chipset, 10 Gbps) — donc pas un repli USB2, pas un souci de câble/port en soi.

**Cause identifiée** : `power/control` valait `auto` (avec `autosuspend_delay_ms: 2000`) sur le device USB de la D455 (`/sys/bus/usb/devices/<bus-port>/power/control`) — la gestion d'énergie USB du kernel peut suspendre le device après ~2s d'inactivité perçue, y compris en plein streaming. Cause classique et documentée des déconnexions RealSense sous Linux.

**Fix appliqué** (système, pas dans ce repo) : `/etc/udev/rules.d/99-realsense-usb-power.rules`, une règle udev qui force `power/control=on` pour tout device `idVendor=="8086"` + `idProduct=="0b5c"` (le D455) :
```
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", TEST=="power/control", ATTR{power/control}="on"
```
Matche sur l'identité du device (vendor/product ID), **pas sur le port physique** — s'applique donc quel que soit le port USB utilisé, et à n'importe quelle D455 (pas seulement ce numéro de série précis, la règle ne teste pas `idSerial`). Vérifié actif : `cat /sys/bus/usb/devices/<bus-port>/power/control` → `on` (au lieu de `auto`) après `sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=usb`.

**Non re-testé en streaming prolongé** après ce fix — à confirmer (`dmesg -w` en parallèle d'un lancement complet) qu'aucune déconnexion ne revient sur une session plus longue qu'un simple test ponctuel.

## Dépannage — RealSense a besoin d'un reset à chaque lancement

Symptôme différent des deux ci-dessous (pas un fix système one-shot, un truc qui revenait **à chaque lancement** du node RealSense, même après les deux fix udev déjà en place) : `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true` démarrait, `RealSense Node Is Up!` s'affichait, puis `Hardware Notification: Depth stream start failure` suivi de `Frames didn't arrived within 5 seconds` en boucle — sauf juste après avoir physiquement débranché/rebranché la caméra, où ça repartait proprement.

**Cause identifiée** : un `Stop Sensor`/`Close Sensor` (Ctrl-C normal, confirmé propre dans les logs) n'arrête que le driver côté Linux — il ne réinitialise pas l'état interne du module stéréo/ASIC de profondeur (le "Vision Processor D4") de la caméra elle-même, qui est un état **firmware**, pas un état du driver. Le prochain `open()` du flux depth échoue parce que l'ASIC redémarre depuis un état déjà "streaming"/mal terminé.

**Testé et écarté** : `sudo usbreset 8086:0b5c` (reset au niveau du bus USB hôte — juste un signal de reset électrique sur la connexion existante) ne suffit pas, parce qu'il ne parle pas au firmware de la caméra. Un vrai débranchement physique fonctionne parce qu'il coupe l'alimentation (VBUS) et force une vraie ré-énumération complète, redémarrant le firmware à froid.

**Fix appliqué** (dans ce repo, cette fois) : `initial_reset:=true` (argument déjà exposé par `realsense2_camera`) déclenche exactement ce reset firmware côté SDK (`rs2::device::hardware_reset()`) — l'équivalent logiciel exact du débranchement physique. Le problème avec cet argument seul : le node ROS tente parfois de rouvrir le device *avant* que la ré-énumération USB déclenchée par ce reset soit terminée, et plante avec `Device or resource busy` → `No such device` quelques secondes après le démarrage. **Confirmé manuellement sur ce poste** : un simple relancement immédiat après ce crash (sans rien débrancher) repart proprement.

`scripts/launch_realsense_with_retry.sh` (nouveau, installé via `data_files` — pas un entry_point, un simple script bash appelé en `ExecuteProcess(cmd=[<chemin installé>, 'align_depth.enable:=true', 'initial_reset:=true', ...])`, même convention que `wait_for_zmq_health.py`) automatise ce retry : lance `ros2 launch realsense2_camera rs_launch.py "$@"`, et si le process sort en erreur moins de 20s après son démarrage (`STARTUP_GRACE_S`), le relance automatiquement (jusqu'à 4 tentatives, `RETRY_DELAY_S=3` entre chaque). Un arrêt normal (Ctrl-C, ou un crash survenant après 20s de fonctionnement) n'est **pas** retenté — le script forward juste SIGINT/SIGTERM à son enfant et sort avec son code de retour. Utilisé par `franka_demo.launch.py` (`realsense`, maintenant un `ExecuteProcess` au lieu d'un `IncludeLaunchDescription`/`GroupAction` — lancer `rs_launch.py` comme son propre process OS donne aussi gratuitement l'isolation de `LaunchConfiguration` que `GroupAction(scoped=True, forwarding=False)` fournissait avant, plus besoin de ce wrapper) et par `fp3_apriltag_demo/apriltag_move_once.launch.py` (référence ce script via `FindPackageShare('franka_demo_bringup')`, d'où le nouvel `exec_depend` correspondant dans son `package.xml`).

**Testé manuellement contre la vraie caméra** (ce chantier) : le script lance `rs_launch.py` avec `initial_reset:=true`, la caméra démarre et stream normalement, extinction propre (SIGTERM forwardé, aucun process orphelin après coup). Le chemin de retry lui-même (crash rapide → relance auto) n'a pas été ré-exercé lors de ce test précis (la caméra était déjà dans un état propre suite aux tests précédents de la session) — mais le comportement `initial_reset:=true` + un relancement manuel a été confirmé fonctionnel par l'utilisateur juste avant l'implémentation de ce script.

## Dépannage — "Frames didn't arrived within 5 seconds" en boucle (fix système, hors repo)

Symptôme différent du précédent (pas de `usb disconnect` dans `dmesg`, le device reste énuméré) : `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true` démarre normalement (`RealSense Node Is Up!`, les deux profils s'ouvrent sans erreur), mais **aucune frame n'arrive jamais** — `backend-v4l2.cpp:2044 Frames didn't arrived within 5 seconds` en boucle toutes les ~5s, indéfiniment. Observé sur cette même machine, `power/control=on` déjà actif (donc pas une régression du fix précédent), aucun autre process n'a la caméra ouverte (`fuser /dev/video*` propre), aucune contention temps-réel (même symptôme avec et sans `ros2_control_node`/gravity compensation Franka actif en parallèle — hypothèse testée et écartée), rien d'anormal dans `journalctl -k` pendant la tentative (pas d'erreur xhci/bande-passante).

**Cause identifiée** : aucune règle udev officielle RealSense n'était installée sur cette machine (le paquet `ros-jazzy-librealsense2` n'installe que la lib runtime, pas les règles udev — normalement posées par `scripts/setup_udev_rules.sh` du repo librealsense officiel, jamais exécuté ici). Conséquence vérifiée avec `rs-save-to-disk` (outil librealsense pur, sans ROS2 — installé avec `ros-jazzy-librealsense2` sous `/opt/ros/jazzy/bin/`) : échoue immédiatement avec `Permission denied` sur un `scan_element` IIO (accéléromètre). En creusant : `/dev/bus/usb/<bus>/<dev>` (le node USB brut de la caméra) appartenait à `root:root` mode `664` — `ngr` n'avait que lecture, malgré son appartenance au groupe `plugdev` (le *fichier* n'était pas dans ce groupe). Les flux couleur/depth passent par `uvcvideo`/V4L2 (`/dev/video*`, permissions correctes par défaut — d'où le démarrage "propre" en apparence), mais les séries D400 utilisent aussi des commandes de contrôle USB bas niveau (HWMON/vendor-specific, accès `libusb` direct au device brut) pour la synchro depth/couleur et le déclenchement effectif du flux — exactement l'accès qui était bloqué.

**Fix appliqué** (système, pas dans ce repo — même famille que le fix `power/control` ci-dessus) : `/etc/udev/rules.d/99-realsense-libusb.rules` :
```
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb_device", ATTR{idVendor}=="8086", MODE="0666", GROUP="plugdev"
KERNEL=="iio:device*", ATTRS{name}=="HID-SENSOR-200073*", MODE="0666", GROUP="plugdev"
KERNEL=="iio:device*", ATTRS{name}=="HID-SENSOR-200076*", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"
```
Matche sur `idVendor=="8086"` (Intel) sans filtrer sur `idProduct` — plus large que la règle `power/control` (qui cible spécifiquement le D455, `0b5c`), volontairement, pour couvrir tout device RealSense sans avoir à lister chaque product ID. Après `sudo udevadm control --reload-rules && sudo udevadm trigger` **et un débranchement/rebranchement physique de la caméra** (nécessaire ici, contrairement au fix `power/control` — un attribut sysfs relisible en direct — parce qu'il faut que le device node USB soit recréé avec les nouvelles permissions), le streaming a fonctionné immédiatement. **Persistant d'un reboot à l'autre** comme toute règle udev dans `/etc/udev/rules.d/` — pas besoin de la réappliquer à chaque démarrage, `systemd-udevd` la relit automatiquement à la détection du device.

**Non testé** : sur une machine sans aucune des deux règles udev (`power-control` et `libusb`), ni sur un scénario où seule celle-ci manquerait indépendamment de l'autre.

## Dépendances (`package.xml`)

- `exec_depend` sur `robot_task_manager` (référencé via `get_package_share_directory`) et `realsense2_camera` (référencé indirectement — plus via `get_package_share_directory` dans ce fichier Python depuis le passage à `launch_realsense_with_retry.sh`, mais toujours invoqué à l'exécution par ce script via `ros2 launch realsense2_camera rs_launch.py`, donc toujours un vrai `exec_depend`). Les bridges (`gemini_er_bridge`/`sam3_bridge`/`graspgen_bridge`) ne sont **pas** des `exec_depend` ici : ils sont démarrés transitivement par `robot_task_manager.launch.py`, qui a déjà ses propres `exec_depend` dessus (cf. CLAUDE.md racine, Historique — ces `exec_depend` redondants ont été retirés d'ici).
- `zmq`/`msgpack` (utilisés par `wait_for_zmq_health.py`) **ne sont pas déclarés** dans `package.xml` — même convention (absence de déclaration) que dans `sam3_bridge`/`graspgen_bridge`, qui utilisent ces mêmes libs sans `exec_depend` dédié non plus.

## Notes

- `license` (`package.xml`/`setup.py`) toujours à `TODO: License declaration` — jamais renseigné.
- Pas de tests fonctionnels, seulement les templates `ament_copyright`/`ament_flake8`/`ament_pep257` standards — pas de test non plus sur `wait_for_zmq_health.py`.

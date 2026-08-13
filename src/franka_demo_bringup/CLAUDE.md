# CLAUDE.md — `franka_demo_bringup`

Ce fichier documente spécifiquement le package `franka_demo_bringup`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Package `ament_python` **launch-only** (aucun node ROS2, aucun `entry_points`) : point d'entrée racine du pipeline complet. Il embarque désormais un script utilitaire (`scripts/wait_for_zmq_health.py`, installé via `data_files` — pas un entry_point) mais reste sans node/exécutable ROS2 déclaré.

`launch/franka_demo.launch.py` :
1. Démarre les serveurs externes SAM3 et GraspGen (`ExecuteProcess` + `conda run -n <env> --no-capture-output`, cf. "Serveurs externes" plus bas), puis attend qu'ils répondent healthy sur leur port ZMQ (`wait_for_zmq_health.py`, gate séquentiel).
2. Une fois les deux serveurs healthy : démarre RealSense D455 (`rs_launch.py` du package `realsense2_camera`, dans un `GroupAction(scoped=True, forwarding=False)` pour isoler son contexte de `LaunchConfiguration` du reste de l'arbre — cf. CLAUDE.md racine pour le détail sur `scoped`/`forwarding`), avec `align_depth.enable:=true` et un profil color/depth (`rgb_camera.color_profile`/`depth_module.depth_profile`) — cf. "Résolution RealSense" plus bas. **`pointcloud.enable` n'est volontairement pas activé** : bug non résolu de `realsense-ros` quand combiné à `align_depth.enable` (nuage spatialement décalé de quelques cm) — cf. CLAUDE.md racine, dette technique "Important" et `robot_task_manager/CLAUDE.md`. Le pipeline déprojette lui-même depuis `aligned_depth_to_color` (`create_pointcloud_node`) au lieu de dépendre du nuage natif RealSense.
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

## Dépendances (`package.xml`)

- `exec_depend` sur `robot_task_manager` et `realsense2_camera` — les seuls packages ROS2 référencés directement par le launch file (via `get_package_share_directory`). Les bridges (`gemini_er_bridge`/`sam3_bridge`/`graspgen_bridge`) ne sont **pas** des `exec_depend` ici : ils sont démarrés transitivement par `robot_task_manager.launch.py`, qui a déjà ses propres `exec_depend` dessus (cf. CLAUDE.md racine, Historique — ces `exec_depend` redondants ont été retirés d'ici).
- `zmq`/`msgpack` (utilisés par `wait_for_zmq_health.py`) **ne sont pas déclarés** dans `package.xml` — même convention (absence de déclaration) que dans `sam3_bridge`/`graspgen_bridge`, qui utilisent ces mêmes libs sans `exec_depend` dédié non plus.

## Notes

- `license` (`package.xml`/`setup.py`) toujours à `TODO: License declaration` — jamais renseigné.
- Pas de tests fonctionnels, seulement les templates `ament_copyright`/`ament_flake8`/`ament_pep257` standards — pas de test non plus sur `wait_for_zmq_health.py`.

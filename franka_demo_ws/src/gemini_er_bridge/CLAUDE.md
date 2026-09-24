# CLAUDE.md — `gemini_er_bridge`

Ce fichier documente spécifiquement le package `gemini_er_bridge`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Pont ZMQ entre Gemini ER (VLM tournant sur une machine séparée) et le reste du pipeline ROS2. Deux nodes indépendants, chacun avec son propre `zmq.Context`/socket (pas de couplage entre eux) :

| Node | Fichier | Rôle | Socket ZMQ |
|---|---|---|---|
| `camera_bridge` | `camera_bridge_node.py` | republie le flux RGB en JPEG vers Gemini ER | PUB, bind `0.0.0.0:5555` |
| `command_bridge` | `command_bridge_node.py` | reçoit les commandes de pick de Gemini ER, les relaie à `pick_task_node` | REP, bind `0.0.0.0:5556` |

Pas de launch file dédié pour l'instant (contrairement à `sam3_bridge`/`graspgen_bridge`) — les deux nodes sont listés directement en `Node()` dans `robot_task_manager/launch/robot_task_manager.launch.py`.

## `camera_bridge_node`

- S'abonne à `/camera/camera/color/image_raw`, encode chaque frame en JPEG (`camera_bridge_jpeg_quality`, param ROS2, défaut 80) et publie sur le socket PUB (topic ZMQ `rgb`, préfixe `b'rgb'` en `send_multipart`).
- `SNDHWM=1` : ne garde qu'1 message en attente d'envoi — si Gemini ER ne consomme pas assez vite, les frames intermédiaires sont perdues plutôt que mises en queue (comportement voulu pour un flux vidéo temps réel, pas de retransmission utile d'anciennes frames).
- Watchdog (`_watchdog`, timer 10s) : logue un `warn` si aucune frame n'a été reçue depuis plus de 10s (`_WATCHDOG_STALE_S`) — détecte une caméra déconnectée ou un topic qui ne publie plus.
- **QoS** : subscriber en profondeur `10`, profil par défaut (`RELIABLE`). Testé et fonctionnel avec la vraie RealSense D455 — ne pas changer sans raison (cf. CLAUDE.md racine).

### Amélioration possible
- **Pas de validation de `camera_bridge_jpeg_quality`** : si le paramètre est passé hors de la plage `0–100`, `cv2.imencode(..., [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])` peut échouer silencieusement côté OpenCV. Ajouter un clamp ou une vérification au démarrage.

## `command_bridge_node`

- Socket ZMQ **REP** (alternance stricte recv/send), écouté dans un thread dédié (`_zmq_loop`, `daemon=True`) — pas dans le thread ROS2 principal, donc ne bloque pas l'exécuteur `rclpy.spin(node)`.
- `_handle_pick_command` : valide les champs requis (`object_label`, `point_x`, `point_y`), vérifie `service_is_ready()` (non-bloquant) avant d'appeler `/execute_pick_task`, puis **bloque le thread ZMQ** (`event.wait(timeout=self.timeout)`, défaut 60s) jusqu'à la fin du pick ou le timeout.
- `destroy_node` : `_shutdown.set()` avant de fermer le socket, pour que `_zmq_loop` et tout `event.wait()` en cours se terminent proprement à l'arrêt du node.

### Dette technique (déjà trackée dans le CLAUDE.md racine, section "Critique")
- **`task_type: "stop"` ne peut rien interrompre** : le socket REP à alternance stricte + le blocage du thread ZMQ pendant tout un pick (`event.wait(timeout=self.timeout)`, jusqu'à 60s) signifient qu'un message `"stop"` envoyé par Gemini ER pendant un pick en cours ne peut être ni reçu ni traité avant la fin de ce pick (succès ou timeout). Problème de sécurité tant que le bras peut être en mouvement pendant l'attente (roadmap #2, exécution du mouvement, pas encore réécrite).

### Améliorations possibles
- **`task_type` absent vs non supporté** : si le champ manque complètement, `command.get('task_type')` retourne `None` et le log dit `"Unsupported task_type : 'None'"` — trompeur, à distinguer explicitement des cas où un `task_type` réel mais non géré est reçu.
- **Pas de log de connexion client** : un socket ZMQ REP ne notifie pas nativement les connexions/déconnexions — impossible de savoir si Gemini ER est effectivement connecté sans recevoir un premier message.

## Notes de cohérence avec le CLAUDE.md racine

- Les deux nodes sont packagés ensemble (même package Python) mais restent deux exécutables/nodes ROS2 distincts — voir CLAUDE.md racine, section "Ports ZMQ", pour la justification de ce découpage (pas de couplage de blocage entre eux malgré le partage du package).

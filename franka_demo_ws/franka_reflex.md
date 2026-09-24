# Reflex `communication_constraints_violation` — enquête et correctif

Document de référence sur le reflex libfranka qui faisait échouer les picks sur le FP3 (session du 2026-09-21). Cause trouvée : traitement tardif de la réception réseau sur ce poste (noyau `PREEMPT_RT`, carte Realtek). Correctif : passer le thread `ksoftirqd` de la carte réseau en temps réel, rendu persistant par un service systemd.

## 1. Symptôme

Les picks échouaient à l'exécution avec :

```
[fp3_arm_controller] Can't accept new action goals. Controller is not running.
[pick_place_node]    No pose was reachable/collision-free
```

Alors que MTC avait planifié les trajectoires sans problème (IK trouvée, pas de collision). Le problème n'était donc ni les filtres de grasp, ni la calibration.

Dans le log de `ros2_control_node`, juste avant :

```
libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]
Deactivating following hardware components as their read cycle resulted in an error: [ FrankaHardwareInterface ]
Deactivating following controllers ...: [ fp3_arm_controller joint_state_broadcaster ]
```

## 2. Mécanisme

1. La boucle temps réel libfranka (1 kHz) rate trop de cycles : le contrôleur du bras déclenche le reflex `communication_constraints_violation`, **même bras immobile**.
2. `ros2_control` désactive `FrankaHardwareInterface` (état `unconfigured`), puis tous les controllers qui en dépendent.
3. **Rien dans la pipeline ne les réactive.** Tous les picks suivants échouent à l'exécution, même avec des plans valides.
4. Après un reflex, il faut relancer tout le launch (et acquitter l'erreur sur le Desk si besoin).

Vérification de l'état à chaud :

```bash
ros2 control list_controllers            # fp3_arm_controller doit être `active`
ros2 control list_hardware_components    # FrankaHardwareInterface doit être `active`
```

Note : `ros2 node list` peut renvoyer un cache périmé du daemon. En cas de doute : `ros2 daemon stop && ros2 daemon start`.

## 3. Pistes écartées (toutes mesurées)

| Piste | Verdict |
|---|---|
| Governor CPU `schedutil` | Passé en `performance` (runtime + GRUB `cpufreq.default_governor=performance`), actif sur les 24 cœurs. Insuffisant : le reflex est revenu |
| Course au démarrage (controller pas encore actif) | Réelle sur un seul lancement (aussi `franka_ros2_ws` non sourcé), pas la cause générale |
| Charge de chargement des modèles SAM3/GraspGen | Écartée : un reflex est arrivé 51 s après que les serveurs étaient prêts |
| Mémoire non verrouillée (`memlock` 100 Mo, warning `Unable to lock the memory`) | Corrigée (`limits.d`, voir §6), sans effet sur le reflex |
| EEE (Energy Efficient Ethernet) sur `eno1` | Coupé (`ethtool --set-eee eno1 eee off`), sans effet |
| Erreurs/pertes de la carte réseau | 0 erreur, 0 `rx_missed`, 0 paquet perdu |
| ASPM de la carte réseau | `l1_aspm=0` |
| Tick du noyau | `CONFIG_HZ_1000`, `PREEMPT_RT`, `NO_HZ_FULL` |
| Collision d'interruptions | NIC → cpu21, USB `xhci_hcd` → cpu14, GPU `nvidia` → cpu20 : cœurs distincts |
| Priorité temps réel du processus de test (`chrt -f 90`) | Pas d'amélioration des retards |
| États de veille C2/C3 (C3 = 350 µs de latence de sortie) | Désactivés à chaud : gain faible (40 → 28 pertes), pas la cause |
| Câblage | Connexion directe : seul `192.168.1.1` apparaît dans la table ARP de `eno1` |

## 4. Mesures qui ont désigné la cause

Test officiel libfranka (**déplace le bras** vers une configuration articulaire fixe : zone dégagée, bouton d'arrêt à la main, tout le reste arrêté) :

```bash
$FP3_ROOT/franka_ros2_ws/install/libfranka/bin/communication_test 192.168.1.1
```

| Condition | États perdus (sur ~10 000) | `Min` success rate |
|---|---|---|
| Base | 40 | 0.90 |
| C2/C3 désactivés | 28 | 0.93 |
| + priorité temps réel `chrt -f 90` | 21 | 0.91 |
| **Après le correctif (§5)** | **~0** | **1.00** |

Test de latence brute vers le bras (sans mouvement, sans sudo) :

```bash
ping -i 0.002 -c 5000 192.168.1.1
```

- Avant : moyenne 0,096 ms, mais **3 à 20 pings > 1 ms sur 5 000, maximum ~4 ms**. Identique en priorité temps réel : le retard vient du chemin de réception, pas du processus.
- Après le correctif : **0 ping > 1 ms, maximum 0,2 ms** (0,5 ms sur le pire des 3 runs).

Un cycle de 1 ms qui attend 1 à 4 ms correspond à 1 à 4 cycles ratés d'affilée, d'où les rafales de pertes.

## 5. Cause racine et correctif

**Cause** : la carte réseau `eno1` (Realtek RTL8125, pilote `r8169`, IRQ 67, câblée en direct au bras) traite ses interruptions sur le cœur 21 (~1,41 M de `NET_RX` sur ce cœur contre ~50 000 ailleurs). Sur un noyau `PREEMPT_RT`, le traitement des paquets reçus est fait par `ksoftirqd/21`, un thread en **priorité normale** (`SCHED_OTHER`, nice 0). Les paquets du bras attendaient derrière l'ordonnanceur classique.

**Correctif** (test rapide, perdu au reboot) :

```bash
sudo chrt -f -p 80 $(pgrep -x ksoftirqd/21)
chrt -p $(pgrep -x ksoftirqd/21)      # SCHED_FIFO, priority 80
```

Résultat confirmé : pings propres et `communication_test` à 1.00.

## 6. Rendre le correctif persistant

Service systemd (retrouve l'interruption de `eno1` par son nom, la fixe au cœur 21, passe son `ksoftirqd` en `SCHED_FIFO 80`). **Installé et activé le 2026-09-21.**

```bash
sudo tee /usr/local/sbin/franka-net-rt.sh >/dev/null <<'EOF'
#!/bin/bash
CPU=21
IRQ=$(awk '$NF=="eno1" {sub(":","",$1); print $1}' /proc/interrupts | head -1)
[ -n "$IRQ" ] && echo $CPU > /proc/irq/$IRQ/smp_affinity_list
PID=$(pgrep -x ksoftirqd/$CPU | head -1)
[ -n "$PID" ] && chrt -f -p 80 $PID
EOF
sudo chmod +x /usr/local/sbin/franka-net-rt.sh

sudo tee /etc/systemd/system/franka-net-rt.service >/dev/null <<'EOF'
[Unit]
Description=Real-time priority for the NIC softirq thread (Franka FCI)
After=network-online.target
[Service]
Type=oneshot
ExecStart=/usr/local/sbin/franka-net-rt.sh
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now franka-net-rt.service
```

Vérification :

```bash
systemctl is-enabled franka-net-rt.service          # enabled
systemctl status franka-net-rt.service --no-pager   # active (exited), sans erreur
chrt -p $(pgrep -x ksoftirqd/21)                    # SCHED_FIFO, priority 80
cat /proc/irq/67/smp_affinity_list                  # 21
```

Si la carte réseau change ou si l'interface est renommée, le script est à adapter (`eno1`, cœur 21).

### Réglages complémentaires appliqués (pas la cause, gardés)

- **Verrouillage mémoire** : `/etc/security/limits.d/99-franka-rt.conf` avec `ngr - memlock unlimited` et `ngr - rtprio 99`. Effectif après reconnexion/reboot (`ulimit -l` → `unlimited`). Le fichier `/etc/systemd/user.conf.d/99-franka-rt.conf` n'a pas été nécessaire.
- **EEE** : à couper à la main après chaque reboot si on veut le garder désactivé (`sudo ethtool --set-eee eno1 eee off`), non persistant.
- **États C2/C3** : désactivés à chaud seulement, revenus au reboot (sans conséquence).

## 7. À faire / non fait

- **Vérifier après le prochain reboot** que le service persiste (deux dernières commandes du §6).
- **Non fait** : le message final `No pose was reachable/collision-free` est trompeur quand les plans réussissent mais que l'exécution échoue (controller inactif) ; `pick_place_node` rejoue en plus tous les candidats pour rien. À remplacer par un arrêt immédiat avec un message clair (« contrôleur du bras inactif »).
- **Non fait** : bug de nom `fr3`/`fp3` : `franka_robot_state_broadcaster` requiert `fr3/robot_state` et ne s'active jamais (sans impact sur les picks, mais `/franka_robot_state_broadcaster/robot_state` ne publie rien).
- **Non testé** : pipeline complète avec la nouvelle calibration D455 (tag 12,9 cm), maintenant que le controller ne devrait plus tomber. Vérifier dans RViz que les flèches de grasp tombent sur l'objet avant un pick avec exécution.

## 8. Autres changements de la session

- Offset **+5 mm** sur l'axe Z local de `fp3_hand_tcp` côté MTC (`mtc_tasks.cpp::planAndExecuteApproach`), URDF inchangé (compense la caméra D405 montée sur la main).
- Nouveau fichier de tag 12,9 cm : `calib_ws/src/handeye_tf_publisher/tags/36h11_0_0.129.yaml`. `sample_guard` a encore `TAG_SIZE_M = 0.04` codé en dur : ses métriques flat/reprojection sont faussées avec ce tag.
- Watchdog `launch_realsense_with_retry.sh` (plus de `realsense2_camera_node` orphelin), conversion Gemini 0-1000 → pixels dans `pick_task_node`, `rclpy.try_shutdown()` partout.

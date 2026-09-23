# camera.md — Dépannage RealSense D455 (déconnexions USB / échecs d'énumération)

Documente en détail un chantier de dépannage matériel mené le 18/09/2026 sur le poste de dev
(Dell Alienware Aurora Ryzen Edition R14, AMD). Concerne la caméra D455 utilisée par
`franka_demo_bringup`/`robot_task_manager` (ce repo) et par `calib_ws` (calibration eye-on-base/
eye-in-hand) — le problème est **matériel/OS**, pas spécifique à un des deux workspaces.

## Résumé exécutif

La D455 se déconnecte/échoue à s'énumérer de façon **intermittente** quand elle est branchée sur
un port câblé au contrôleur USB **fourni par le chipset AMD** (`0000:02:00.0`, "AMD 500 Series
Chipset USB 3.1 XHCI Controller"). C'est un bug matériel/firmware connu de cette génération de
chipset AMD (Ryzen + 500-series), pas un défaut de la caméra, pas un bug logiciel de ce projet.

**Seule solution 100% fiable trouvée** : brancher la caméra sur un port câblé à l'**autre**
contrôleur USB de la machine, fourni par le CPU (`0000:08:00.3`, "AMD Matisse USB 3.0 Host
Controller", Bus 003/004) — jamais reproduit d'échec dessus pendant tout ce chantier.

## Symptômes observés (dans l'ordre où ils sont apparus)

1. **`xioctl(VIDIOC_S_FMT) failed, errno=5 Input/output error`** dans `realsense-viewer`, au moment
   d'ouvrir le flux vidéo.
2. **Bandeau `realsense-viewer` "UDEV-Rules file ... is not up-to date! Version 1.1 can be
   applied"**.
3. Au niveau kernel (`journalctl -k`), la vraie cause visible : **`device descriptor read/8,
   error -110`** en boucle, avec parfois `usb usbX-portY: attempt power cycle` puis `unable to
   enumerate USB device` — le kernel n'arrive pas à lire les descripteurs USB de base, avant même
   qu'un logiciel RealSense intervienne.
4. Une fois l'énumération USB réussie (parfois), une **deuxième couche d'échec, différente** :
   `set_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed on control 1 Last Error: Connection timed out` —
   la caméra est visible sur le bus USB (`lsusb`, `uvcvideo` attaché) mais son firmware ne répond
   plus aux requêtes de contrôle (extension unit, utilisées pour la config profondeur/IMU).
5. **`xioctl(VIDIOC_S_FMT) failed, errno=16 Device or resource busy`** — piège différent, sans
   rapport avec le bug matériel : un process `realsense2_camera_node` orphelin (lancé par une
   commande de test précédente, mal tué) gardait les `/dev/video*` ouverts en exclusif. `fuser -v
   /dev/video0` (ou `lsof`) le révèle immédiatement ; `kill -9 <pid>` suffit.

## Root cause

- Machine : **Dell Alienware Aurora Ryzen Edition R14**, deux contrôleurs xHCI USB3 physiquement
  distincts :
  - `0000:02:00.0` — "AMD 500 Series Chipset USB 3.1 XHCI Controller" → root hubs **Bus 001**
    (USB2, 480M) et **Bus 002** (USB3, 10000M). **C'est celui qui bug.**
  - `0000:08:00.3` — "AMD Matisse USB 3.0 Host Controller" (fourni par le CPU, pas le chipset) →
    root hubs **Bus 003** (USB2) et **Bus 004** (USB3). **Fiable, jamais échoué.**
- Trouver quel bus correspond à quel contrôleur : `readlink -f /sys/bus/usb/devices/usbN` remonte
  jusqu'au device PCI (`.../0000:02:00.0/usbN` ou `.../0000:08:00.3/usbN`).
- C'est un bug **connu et documenté dans la communauté Linux** pour les chipsets AMD 500-series
  (X570/B550, plateforme Ryzen "Matisse") : déconnexions/échecs d'énumération USB aléatoires,
  indépendants du périphérique branché, généralement attribués à une interaction avec la gestion
  d'énergie PCIe (ASPM) ou les états C du processeur.
- **BIOS déjà à jour** — vérifié via `fwupdmgr get-updates` (l'outil Linux natif, plus fiable que la
  page Dell support générique trouvée par recherche web, qui affichait une version obsolète/non
  pertinente) : "System Firmware" apparaît dans "Devices with no available firmware updates". Donc
  pas une histoire de version manquante — le bug est présent même sur le dernier firmware Dell
  disponible pour ce modèle.
- **Intermittent, pas déterministe** : la caméra a fonctionné parfaitement sur `02:00.0` à plusieurs
  reprises pendant ce chantier (au boot initial de la session, une fois juste après un reboot avec
  les correctifs actifs, et une fois pendant un test délibérément SANS aucun correctif) — puis a
  replanté sans changement de configuration apparent. Aucun correctif testé ne garantit un
  fonctionnement stable à 100% sur ce contrôleur.

### Découverte importante : un bug apparenté déjà documenté avant ce chantier

`/etc/udev/rules.d/99-realsense-usb-power.rules` existait déjà (créé le **13 août 2026**, par une
session Claude Code précédente sur ce projet — documenté dans
`franka_demo_bringup/CLAUDE.md`, section "Dépannage — déconnexions USB aléatoires pendant le
streaming"). Ce fix visait exactement le même symptôme (`USB disconnect` + `realsense2_camera`
segfault en plein streaming), cause identifiée à l'époque : `power/control=auto` +
`autosuspend_delay_ms=2000` par défaut sur le device USB de la D455. Le fix : forcer
`power/control=on` via une règle udev matchant `idVendor=8086`/`idProduct=0b5c` (le D455).

Que ce fix pré-existant n'ait pas suffi à empêcher le problème de ce chantier suggère que
l'autosuspend seul n'est qu'**une partie** de la cause profonde (le vrai bug ASPM/contrôleur
étant plus large), et que ce fix de 13 août n'avait **jamais été re-testé en streaming
prolongé** (noté explicitement dans le CLAUDE.md d'origine comme non confirmé).

## Tous les correctifs essayés

| # | Correctif | Portée | Effet observé |
|---|---|---|---|
| 1 | Règle udev `99-realsense-libusb.rules` mise à jour (générique → v1.1, présente dans `~/.99-realsense-libusb.rules`, jamais installée avant) | Permissions (`0666`/`0777`) sur tous les devices RealSense, **y compris récursivement l'IMU** (`chmod -R /sys/%p`) | Corrige un vrai bug séparé (permission denied sur `scan_elements` de l'IMU quand seule l'ancienne règle générique est active) — **ne corrige pas** le bug d'énumération/déconnexion principal |
| 2 | `power/control=on` forcé à la main (runtime, `echo on \| sudo tee .../power/control`) | Root hubs `usb1`/`usb2` (contrôleur fautif) | Pas d'effet observé sur le bug principal ; **ne survit pas à un reboot** (réglage runtime, pas persistant) |
| 3 | `pcie_aspm.policy=performance usbcore.autosuspend=-1` (paramètres kernel, `/etc/default/grub` → `update-grub` → reboot) | Système entier | A semblé aider juste après le reboot (une énumération + un `rs-enumerate-devices` complet ont réussi), puis le bug est revenu peu après sans changement de config — **effet non concluant, probablement coïncidence avec le caractère intermittent du bug** |
| 4 | Débranchement/rebranchement physique du câble | Le device lui-même | Nécessaire pour sortir d'un état où le firmware de la caméra est bloqué (le control-channel étant lui-même bloqué, un reset logiciel — `initial_reset:=true`/`hardware_reset()` — ne peut pas passer par ce même canal) ; **pas garanti de récupérer l'énumération de base sur `02:00.0`** (a échoué certaines fois même après replug) |
| 5 | Vérification BIOS (`fwupdmgr`) | Firmware système | Déjà à jour — piste fermée |
| 6 | **Changer de contrôleur physique (port câblé sur `08:00.3` au lieu de `02:00.0`)** | Matériel | **Seule solution fiable à 100% observée pendant tout ce chantier** |

### Test A/B (retrait complet des correctifs 1+3)

Pour vérifier que les correctifs udev/GRUB n'étaient pas eux-mêmes la source du problème (ou
inversement, réellement la solution), un test contrôlé a été fait : retour à la règle udev
générique d'origine, désactivation de `99-realsense-usb-power.rules`, retrait des paramètres GRUB,
reboot. **Résultat : la caméra a fonctionné parfaitement sur `02:00.0` dans cet état "sans" aussi**
(un seul échec observé ensuite : permission denied sur l'IMU — attendu, cf. correctif #1). Ça
confirme :
- Nos changements n'ont jamais été la **cause** du bug (chronologie aussi vérifiée via
  `journalctl -k -b -N` : la toute première déconnexion a eu lieu à 11:01:21, **avant** toute
  modification de ce chantier, la première datant de 11:04:40).
- Nos changements ne sont pas non plus une solution **garantie** — le bug reste intermittent qu'ils
  soient actifs ou non.

## État actuel de la configuration (au 18/09/2026, fin de session)

- `/etc/udev/rules.d/99-realsense-libusb.rules` : version **v1.1** (réinstallée depuis
  `~/.99-realsense-libusb.rules`) — **actif**
- `/etc/udev/rules.d/99-realsense-usb-power.rules` : présent, **actif**
- Paramètres kernel GRUB (`pcie_aspm.policy=performance usbcore.autosuspend=-1`) : **retirés**, pas
  remis depuis le test A/B — `/proc/cmdline` actuel ne les contient pas
- Politique ASPM (`/sys/module/pcie_aspm/parameters/policy`) : repassée à `default` (conséquence du
  point précédent)
- `power/control` runtime des 4 root hubs : `auto` (état par défaut post-reboot)

## Souci connu : `realsense2_camera_node` reste orphelin après l'arrêt d'un launch

Récurrent avec tous les launch files de ce projet qui démarrent RealSense via
`launch_realsense_with_retry.sh` (`ExecuteProcess` → `ros2 launch realsense2_camera rs_launch.py`
→ `realsense2_camera_node`) : un `Ctrl-C`/SIGINT sur le launch principal (`calib_bringup`,
`evaluate_calibration`, `franka_demo_bringup`, etc. -- `fp3_apriltag_demo` a depuis été déplacé vers
`calib_ws`, où le même souci s'applique) arrête proprement tout le
reste, mais `realsense2_camera_node` lui-même survit comme process orphelin (le signal ne traverse
pas correctement la chaîne bash → `ros2 launch` imbriqué → node). Un simple SIGTERM ne suffit pas
non plus, il faut `-9`. Reproduit systématiquement pendant tout ce chantier (D455 et D405).

**Fix** : après chaque arrêt de pipeline, avant d'en relancer un autre (sinon le nouveau lancement
entre en conflit sur le même device caméra) :
```bash
pkill -9 -f realsense2_camera_node
```
Vérifier qu'il n'y a plus rien : `ps aux | grep realsense2_camera_node`.

## Recommandation pratique

1. **Utiliser un port branché sur le contrôleur `08:00.3`** (Bus 003/004) pour tout usage nécessitant
   une fiabilité — c'est le seul qui n'a jamais échoué. Si l'emplacement voulu pour la caméra est
   physiquement plus proche d'un port câblé sur `02:00.0`, passer par une **rallonge USB3
   active/alimentée** depuis un port `08:00.3` plutôt que de brancher directement sur le port proche
   mais peu fiable.
2. Garder les règles udev v1.1 + `usb-power` actives (correctifs #1 et pré-existant) — elles
   corrigent de vrais problèmes séparés (permissions IMU, autosuspend), même si insuffisantes seules
   contre le bug principal.
3. Les paramètres GRUB (`pcie_aspm.policy=performance usbcore.autosuspend=-1`) peuvent être remis si
   on veut maximiser les chances sur `02:00.0`, mais ne pas en attendre une fiabilité garantie — les
   remettre nécessite un reboot (`sudo nano /etc/default/grub` → ligne
   `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash pcie_aspm.policy=performance usbcore.autosuspend=-1"` →
   `sudo update-grub` → `sudo reboot`).
4. Si la caméra reste bloquée en échec d'énumération (`error -110` en boucle) : débrancher/rebrancher
   physiquement (pas juste relancer un logiciel).
5. Si la caméra énumère mais `set_xu`/tout accès timeout ("Connection timed out") : le firmware de
   la caméra lui-même est bloqué — un reset logiciel (`initial_reset:=true`) ne peut pas passer par
   le canal de contrôle déjà bloqué ; seul un débranchement physique réel (coupe VBUS, redémarre le
   firmware à froid) fonctionne.
6. Si `xioctl(...) errno=16 Device or resource busy` : un autre process a le device ouvert —
   `fuser -v /dev/video0` (ou toutes les `/dev/video*`) pour l'identifier, `kill` ce process.

## Commandes de diagnostic de référence

```bash
# Où la camera est branchee, a quelle vitesse, quel driver
lsusb -t
lsusb | grep -i intel

# A quel controleur PCI correspond un bus USB donne
readlink -f /sys/bus/usb/devices/usbN

# Evenements USB bas niveau (boot courant)
journalctl -k --no-pager -n 30 | grep -iE usb

# Lister les boots precedents / regarder un boot anterieur
journalctl --list-boots
journalctl -k -b -N --no-pager | grep -iE "usb|disconnect"

# Qui a les /dev/video* ouverts
for f in /dev/video*; do fuser -v "$f"; done

# Test complet caméra (énumération + communication de contrôle, pas juste USB)
source /opt/ros/jazzy/setup.bash
rs-enumerate-devices        # infos device -- si ça timeout, firmware bloqué
rs-hello-realsense           # ouvre un vrai flux -- test le plus complet

# Etat des regles udev actives
ls -la /etc/udev/rules.d/*realsense*
cat /etc/udev/rules.d/99-realsense-libusb.rules | head -1   # doit dire ##Version=1.1##

# Etat ASPM / autosuspend
cat /sys/module/pcie_aspm/parameters/policy
cat /sys/bus/usb/devices/usbN/power/control

# BIOS a jour ?
fwupdmgr get-updates
```

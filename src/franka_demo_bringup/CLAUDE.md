# CLAUDE.md — `franka_demo_bringup`

Ce fichier documente spécifiquement le package `franka_demo_bringup`. Pour la vue d'ensemble du pipeline complet, voir le `CLAUDE.md` à la racine du workspace.

## Rôle du package

Package `ament_python` **launch-only** (aucun node, aucun `entry_points`) : point d'entrée racine du pipeline complet.

`launch/franka_demo.launch.py` :
1. Démarre RealSense D455 (`rs_launch.py` du package `realsense2_camera`, dans un `GroupAction(scoped=True, forwarding=False)` pour isoler son contexte de `LaunchConfiguration` du reste de l'arbre — cf. CLAUDE.md racine pour le détail sur `scoped`/`forwarding`), avec `align_depth.enable:=true` et `pointcloud.enable:=true`.
2. Inclut (`IncludeLaunchDescription`) `robot_task_manager/launch/robot_task_manager.launch.py`, qui démarre en cascade tous les autres nodes ROS2 du pipeline (bridges compris).

```bash
ros2 launch franka_demo_bringup franka_demo.launch.py
```

## Dépendances (`package.xml`)

- `exec_depend` sur `robot_task_manager` et `realsense2_camera` — les seuls packages référencés directement par le launch file (via `get_package_share_directory`). Les bridges (`gemini_er_bridge`/`sam3_bridge`/`graspgen_bridge`) ne sont **pas** des `exec_depend` ici : ils sont démarrés transitivement par `robot_task_manager.launch.py`, qui a déjà ses propres `exec_depend` dessus (cf. CLAUDE.md racine, Historique — ces `exec_depend` redondants ont été retirés d'ici).

## Notes

- `license` (`package.xml`/`setup.py`) toujours à `TODO: License declaration` — jamais renseigné.
- Pas de tests fonctionnels, seulement les templates `ament_copyright`/`ament_flake8`/`ament_pep257` standards.

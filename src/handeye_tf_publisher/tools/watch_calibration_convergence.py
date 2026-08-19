#!/usr/bin/env python3
"""
Lit le log de `ros2 launch easy_handeye2 calibrate.launch.py` sur stdin,
extrait chaque matrice "Computed calibration: [[...]]" imprimee par
handeye_server apres chaque echantillon, et affiche l'ecart (delta) de
translation ET de rotation par rapport a l'echantillon precedent.

Usage:
    ros2 launch easy_handeye2 calibrate.launch.py \\
      calibration_type:=eye_on_base \\
      name:=<nom_calibration> \\
      robot_base_frame:=fp3_link0 robot_effector_frame:=fp3_hand \\
      tracking_base_frame:=camera_color_optical_frame tracking_marker_frame:=tag36h11:0 \\
      2>&1 | python3 src/handeye_tf_publisher/tools/watch_calibration_convergence.py

(rediriger en plus vers `tee /tmp/calib.log` est optionnel, juste pour
garder le log complet en parallele)

Les 5-8 premiers echantillons bougent enormement (deltas de dizaines de
cm / dizaines de degres) -- normal, Tsai-Lenz est tres instable avec peu de
donnees. Ne juger la convergence qu'a partir d'une dizaine d'echantillons.

IMPORTANT : "CONVERGE" ne veut dire quelque chose que si TRANSLATION et
ROTATION convergent toutes les deux. Une translation stable avec une
rotation encore instable/biaisee produit quand meme une grosse erreur de
position une fois evaluee a de nouvelles poses (bras de levier
camera<->effecteur) -- observe en pratique sur ce projet : convergence
"translation seule" affichee comme bonne, puis ~5.4cm de divergence a
l'evaluateur. D'ou le suivi des deux ci-dessous.
"""
import math
import re
import sys

FLOAT_RE = re.compile(r'[-+]?\d+\.\d*(?:[eE][-+]?\d+)?')

CONVERGED_WINDOW = 3              # nombre de deltas consecutifs a verifier
TRANSLATION_THRESHOLD = 0.01      # 1cm
ROTATION_THRESHOLD_DEG = 2.0      # 2 degres


def _rotation_matrix_from_flat(numbers):
    # matrice 4x4 row-major -> bloc de rotation 3x3 (indices 0,1,2 / 4,5,6 / 8,9,10)
    return [
        [numbers[0], numbers[1], numbers[2]],
        [numbers[4], numbers[5], numbers[6]],
        [numbers[8], numbers[9], numbers[10]],
    ]


def _rotation_angle_deg(r1, r2):
    """Angle (deg) de la rotation relative R1^T @ R2, via la trace."""
    # r_rel = r1^T @ r2
    r1t = [[r1[j][i] for j in range(3)] for i in range(3)]
    r_rel = [
        [sum(r1t[i][k] * r2[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    trace = r_rel[0][0] + r_rel[1][1] + r_rel[2][2]
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_angle))


def main():
    prev_translation = None
    prev_rotation = None
    recent_translation_deltas = []
    recent_rotation_deltas = []
    sample_count = 0
    buffer = ''
    collecting = False

    for line in sys.stdin:
        if 'Computed calibration:' in line:
            collecting = True
            buffer = line.split('Computed calibration:', 1)[1]
            continue

        if collecting:
            buffer += line
            if buffer.count(']]') >= 1:
                collecting = False
                numbers = [float(x) for x in FLOAT_RE.findall(buffer)]
                if len(numbers) < 12:
                    print(f'[watch] matrice incomplete ({len(numbers)} nombres trouves), ignoree', file=sys.stderr)
                    buffer = ''
                    continue

                translation = (numbers[3], numbers[7], numbers[11])
                rotation = _rotation_matrix_from_flat(numbers)
                sample_count += 1

                if prev_translation is None:
                    print(f'[sample {sample_count}] translation={tuple(round(v, 4) for v in translation)}  (premier echantillon, pas de delta)')
                else:
                    t_delta = sum((a - b) ** 2 for a, b in zip(translation, prev_translation)) ** 0.5
                    r_delta = _rotation_angle_deg(prev_rotation, rotation)

                    recent_translation_deltas.append(t_delta)
                    recent_translation_deltas[:] = recent_translation_deltas[-CONVERGED_WINDOW:]
                    recent_rotation_deltas.append(r_delta)
                    recent_rotation_deltas[:] = recent_rotation_deltas[-CONVERGED_WINDOW:]

                    t_converged = (len(recent_translation_deltas) >= CONVERGED_WINDOW
                                   and all(d < TRANSLATION_THRESHOLD for d in recent_translation_deltas))
                    r_converged = (len(recent_rotation_deltas) >= CONVERGED_WINDOW
                                   and all(d < ROTATION_THRESHOLD_DEG for d in recent_rotation_deltas))

                    status = ''
                    if t_converged and r_converged:
                        status = '  <-- CONVERGE (translation ET rotation stables sur {} echantillons)'.format(
                            CONVERGED_WINDOW)
                    elif t_converged and not r_converged:
                        status = '  <-- translation stable MAIS rotation encore instable, continuer'
                    elif r_converged and not t_converged:
                        status = '  <-- rotation stable MAIS translation encore instable, continuer'

                    print(f'[sample {sample_count}] translation={tuple(round(v, 4) for v in translation)}  '
                          f'delta_t={t_delta*100:.1f}cm  delta_r={r_delta:.2f}deg{status}')

                prev_translation = translation
                prev_rotation = rotation
                buffer = ''


if __name__ == '__main__':
    main()

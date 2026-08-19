#!/usr/bin/env python3
"""
Lit le log de `ros2 launch easy_handeye2 calibrate.launch.py` sur stdin,
extrait chaque matrice "Computed calibration: [[...]]" imprimee par
handeye_server apres chaque echantillon, et affiche l'ecart (delta) de
translation par rapport a l'echantillon precedent.

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
cm) -- normal, Tsai-Lenz est tres instable avec peu de donnees. Ne juger
la convergence qu'a partir d'une dizaine d'echantillons.
"""
import re
import sys

FLOAT_RE = re.compile(r'[-+]?\d+\.\d*(?:[eE][-+]?\d+)?')

CONVERGED_WINDOW = 3        # nombre de deltas consecutifs a verifier
CONVERGED_THRESHOLD = 0.01  # 1cm : sous ce seuil, on considere que ca a converge


def main():
    prev_translation = None
    recent_deltas = []
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

                # matrice 4x4 row-major : colonne de translation = indices 3, 7, 11
                translation = (numbers[3], numbers[7], numbers[11])
                sample_count += 1

                if prev_translation is None:
                    print(f'[sample {sample_count}] translation={tuple(round(v, 4) for v in translation)}  (premier echantillon, pas de delta)')
                else:
                    delta = sum((a - b) ** 2 for a, b in zip(translation, prev_translation)) ** 0.5
                    recent_deltas.append(delta)
                    recent_deltas[:] = recent_deltas[-CONVERGED_WINDOW:]

                    status = ''
                    if len(recent_deltas) >= CONVERGED_WINDOW and all(d < CONVERGED_THRESHOLD for d in recent_deltas):
                        status = '  <-- CONVERGE (derniers {} deltas < {}cm)'.format(
                            CONVERGED_WINDOW, int(CONVERGED_THRESHOLD * 100))

                    print(f'[sample {sample_count}] translation={tuple(round(v, 4) for v in translation)}  '
                          f'delta={delta*100:.1f}cm{status}')

                prev_translation = translation
                buffer = ''


if __name__ == '__main__':
    main()

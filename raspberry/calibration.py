# Essais de calibration des encodeurs et de la régulation de vitesse.
#
#   python3 calibration.py tours      [n_tours]   roues levées, moteurs coupés : ticks/tour et signes
#   python3 calibration.py vmax                   roues levées, firmware BOUCLE_FERMEE 0 : caractéristique PWM -> vitesse
#   python3 calibration.py echelon    [consigne]  roues levées, firmware BOUCLE_FERMEE 1 : réponse indicielle du PI
#   python3 calibration.py distance   [metres]    au sol, moteurs coupés, robot poussé à la main : ENCODER_SCALE
#
# Les mesures sont enregistrées en CSV dans essais/ (essai echelon : analyse avec ordinateur/analyse_echelon.py).
# La protection ultrason est désactivée pendant les essais (le capteur voit souvent la table).

import csv
import math
import os
import sys
import time

import config as cfg
from hardware.arduino import arduino as ard
from hardware.serial_protocol import ArduinoStop, retrCmdi, retrCmdl

ROUES = ("FR", "FL", "RR", "RL")
TICKS_TOUR_THEO = 11 * 21.3                         # front descendant voie B x réducteur
VMAX_THEO = 280 / 60 * TICKS_TOUR_THEO              # 280 tr/min à vide sous 6 V
DOSSIER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "essais")


# ---------------------------------------------------------------- outils
def connecter():
    a = ard(cfg.ARDUINO_PORT)
    if not a.connect():
        raise RuntimeError("L'Arduino ne répond pas")
    a.SetObstacleProtection(0)
    return a


def compteurs(a):
    """Compteurs bruts des 4 encodeurs (FR, FL, RR, RL), sans correction de signe."""
    return retrCmdl(a.serialport, b'N') + retrCmdl(a.serialport, b'n')


def vitesses(a):
    """Vitesses filtrées (ticks/s, signe corrigé par encSign) : commande P."""
    return retrCmdi(a.serialport, b'P')


def pwm(a):
    """PWM réellement appliquées : commande T."""
    return retrCmdi(a.serialport, b'T')


def consigne(a, c):
    try:
        a.Advance(c, c)
    except ArduinoStop as e:
        print(f"  (arrêt Arduino : {e.cause})")


def attendre(a, duree):
    """Attend en interrogeant l'Arduino (le chien de garde voit une liaison active)."""
    fin = time.monotonic() + duree
    while time.monotonic() < fin:
        vitesses(a)
        time.sleep(0.05)


def ecrire_csv(nom, entete, lignes):
    os.makedirs(DOSSIER, exist_ok=True)
    chemin = os.path.join(DOSSIER, time.strftime(f"{nom}_%Y%m%d_%H%M%S.csv"))
    with open(chemin, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(entete)
        w.writerows(lignes)
    print(f"Enregistré : {chemin}")


def diff(c0, c1):
    return [x1 - x0 for x0, x1 in zip(c0, c1)]


# ---------------------------------------------------------------- essai 1 : ticks par tour
def essai_tours(a, n_tours=10):
    print(f"Marquez chaque roue, puis faites-leur faire {n_tours} tours à la main,")
    print("dans le sens où elles feraient AVANCER le robot (toutes en même temps ou une par une).")
    input("Entrée pour commencer...")
    c0 = compteurs(a)
    input("Entrée quand c'est fini...")
    d = diff(c0, compteurs(a))
    print(f"\n{'roue':>4} {'ticks':>7} {'ticks/tour':>11} {'théorie':>8}  encSign")
    for r, x in zip(ROUES, d):
        print(f"{r:>4} {x:>7d} {abs(x) / n_tours:>11.1f} {TICKS_TOUR_THEO:>8.1f}  {1 if x >= 0 else -1:>4d}")
    print("\n-> recopier la colonne encSign dans parameters.h (const int encSign[4]).")
    print("   Un compteur à ~0 : encodeur mal branché. Un écart > 5 % : tours mal comptés ou ticks perdus.")


# ---------------------------------------------------------------- essai 2 : VMAX (boucle ouverte)
def essai_vmax(a, niveaux=(40, 60, 80, 100, 150, 200, 255), duree=2.0):
    print("Firmware avec BOUCLE_FERMEE 0, ROUES LEVÉES. Les moteurs vont tourner jusqu'à PWM 255.")
    input("Entrée pour commencer...")
    lignes = []
    try:
        for p in niveaux:
            consigne(a, p)
            attendre(a, 1.0)                                # régime établi
            c0, t0 = compteurs(a), time.monotonic()
            attendre(a, duree)
            d, dt = diff(c0, compteurs(a)), time.monotonic() - t0
            v = [x / dt for x in d]
            lignes.append([p] + [round(x) for x in v])
            print(f"PWM {p:>3} : " + "  ".join(f"{r} {x:7.0f}" for r, x in zip(ROUES, v)) + "  ticks/s")
    finally:
        consigne(a, 0)
    ecrire_csv("vmax", ["pwm"] + [f"v_{r}" for r in ROUES], lignes)
    vmax = sum(abs(x) for x in lignes[-1][1:]) / 4
    print(f"\nVMAX moyen à PWM 255 : {vmax:.0f} ticks/s (théorie à vide sous 6 V : {VMAX_THEO:.0f})")
    print(f"-> parameters.h : #define VMAX_TICKS {0.9 * vmax:.0f}.0f")
    print("   (90 % du max mesuré, pour garder de la marge au PI quand la batterie baisse et sous charge)")
    print("   La PWM en dessous de laquelle les roues ne tournent pas donne la zone morte.")


# ---------------------------------------------------------------- essai 3 : réponse indicielle (boucle fermée)
def essai_echelon(a, c=100, duree=2.0):
    print(f"Firmware avec BOUCLE_FERMEE 1, ROUES LEVÉES. Échelon de consigne 0 -> {c} -> 0.")
    input("Entrée pour commencer...")
    lignes, t0 = [], time.monotonic()

    def mesurer(sp, jusqu_a):
        while time.monotonic() - t0 < jusqu_a:
            t = time.monotonic() - t0
            lignes.append([round(t, 3), sp, *vitesses(a), *pwm(a)])
            time.sleep(0.01)

    try:
        mesurer(0, 0.3)
        consigne(a, c)
        mesurer(c, 0.3 + duree)
        consigne(a, 0)
        mesurer(0, 0.6 + 2 * duree)
    finally:
        consigne(a, 0)
    ecrire_csv(f"echelon{c}", ["t", "consigne", *[f"v_{r}" for r in ROUES], *[f"pwm_{r}" for r in ROUES]], lignes)

    # synthèse sur le palier haut
    k = 255 / VMAX_TICKS_FIRMWARE
    palier = [l for l in lignes if l[1] == c]
    fin = palier[len(palier) // 2:]
    print(f"\nvitesse attendue : {c / k:.0f} ticks/s  (= {c} x VMAX_TICKS / 255)")
    for i, r in enumerate(ROUES):
        v = [l[2 + i] for l in palier]
        v_fin = sum(l[2 + i] for l in fin) / len(fin)
        p_fin = sum(l[6 + i] for l in fin) / len(fin)
        depass = max(0.0, (max(v) - v_fin) / max(abs(v_fin), 1) * 100)
        t90 = next((l[0] - 0.3 for l in palier if l[2 + i] >= 0.9 * c / k), math.nan)
        print(f"{r} : v finale {v_fin:6.0f}  erreur {100 * (v_fin * k - c) / c:+5.1f} %  "
              f"dépassement {depass:4.1f} %  t90 {t90:4.2f} s  PWM {p_fin:5.0f}")
    print("\nÀ vérifier : erreur finale ~0 (intégrale), dépassement < 10-15 %, pas d'oscillation.")
    print("Une roue qui part à PWM ±255 sans suivre la consigne : encSign faux pour cette roue.")


# ---------------------------------------------------------------- essai 4 : ENCODER_SCALE au sol
def essai_distance(a, metres=3.0, repetitions=3):
    print(f"Au sol, moteurs coupés. Marquez {metres} m au mètre ruban.")
    print("Poussez le robot à la main en ligne droite, sans le soulever.")
    scales = []
    for k in range(repetitions):
        input(f"\nEssai {k + 1}/{repetitions} : robot sur le départ, Entrée...")
        c0 = compteurs(a)
        input("Robot sur l'arrivée, Entrée...")
        d = [abs(x) for x in diff(c0, compteurs(a))]
        moy = sum(d) / 4
        scales.append(metres / moy)
        print("  ticks : " + "  ".join(f"{r} {x}" for r, x in zip(ROUES, d))
              + f"  -> {metres / moy:.3e} m/tick")
    s = sum(scales) / len(scales)
    theo = math.pi * 0.065 / TICKS_TOUR_THEO
    print(f"\nENCODER_SCALE = {s:.3e} m/tick  (théorie Ø65 mm : {theo:.3e}, écart {100 * (s / theo - 1):+.1f} %)")
    print(f"diamètre effectif des roues : {s * TICKS_TOUR_THEO / math.pi * 1000:.1f} mm")
    print("-> recopier dans config.py. Des roues qui diffèrent de > 3 % : patinage ou poussée en biais.")


# ---------------------------------------------------------------- main
VMAX_TICKS_FIRMWARE = 1000.0          # à garder égal à VMAX_TICKS de parameters.h (pour l'essai echelon)

ESSAIS = {"tours": essai_tours, "vmax": essai_vmax, "echelon": essai_echelon, "distance": essai_distance}

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ESSAIS:
        print("usage : python3 calibration.py {tours|vmax|echelon|distance} [paramètre]")
        sys.exit(1)
    args = [float(x) if "." in x else int(x) for x in sys.argv[2:]]
    a = connecter()
    try:
        ESSAIS[sys.argv[1]](a, *args)
    except KeyboardInterrupt:
        print("\nInterrompu")
    finally:
        try:
            a.Stop()
        finally:
            a.disconnect()

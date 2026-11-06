# Analyse d'un essai "echelon" (raspberry/calibration.py echelon), à lancer sur l'ordinateur :
#
#   python analyse_echelon.py [fichier.csv] [--tau-bf 0.15] [--vmax 1000]
#
#   - trace consigne, vitesses et PWM des 4 roues
#   - identifie par roue un modèle du 1er ordre PWM -> vitesse avec zone morte :
#         tau * dv/dt + v = K * (u - u0)      (v en unités de consigne)
#   - propose des gains PI par compensation du pôle (Ti = tau), constante de temps tau_bf en boucle fermée
#
# Sans fichier, prend le dernier echelon*.csv de raspberry/essais/. Dépendances : numpy, matplotlib.

import argparse
import csv
import glob
import os

import matplotlib.pyplot as plt
import numpy as np

ROUES = ("FR", "FL", "RR", "RL")


def lire_csv(chemin):
    with open(chemin, newline="") as f:
        lignes = list(csv.reader(f))
    entete, donnees = lignes[0], np.array(lignes[1:], dtype=float)
    return {nom: donnees[:, i] for i, nom in enumerate(entete)}


def zone_morte(u, u0):
    return np.sign(u) * np.maximum(np.abs(u) - u0, 0.0)


def identifie(t, u, v):
    """Moindres carrés sur le modèle discret v[n] = a v[n-1] + b ue[n-1], u0 balayé de 0 à 80.
    Renvoie K, tau, u0."""
    dt = np.mean(np.diff(t))
    meilleur = None
    for u0 in np.arange(0.0, 80.5, 0.5):
        X = np.column_stack((v[:-1], zone_morte(u[:-1], u0)))
        (a, b), *_ = np.linalg.lstsq(X, v[1:], rcond=None)
        if not 0.0 < a < 1.0:
            continue
        err = np.sum((simule(t, u, b / (1 - a), -dt / np.log(a), u0) - v) ** 2)
        if meilleur is None or err < meilleur[0]:
            meilleur = (err, b / (1 - a), -dt / np.log(a), u0)
    if meilleur is None:
        raise ValueError("identification impossible (pas assez d'excitation ?)")

    # affinage sur l'erreur de simulation (les moindres carrés sont biaisés par le bruit de mesure)
    _, K0, tau0, u00 = meilleur
    for K in K0 * np.linspace(0.8, 1.2, 11):
        for tau in tau0 * np.linspace(0.7, 1.4, 11):
            for u0 in np.clip(u00 + np.linspace(-10, 10, 11), 0, None):
                err = np.sum((simule(t, u, K, tau, u0) - v) ** 2)
                if err < meilleur[0]:
                    meilleur = (err, K, tau, u0)
    return meilleur[1:]


def simule(t, u, K, tau, u0):
    ue = zone_morte(u, u0)
    v = np.zeros_like(t)
    for n in range(1, len(t)):
        dt = t[n] - t[n - 1]
        v[n] = v[n - 1] + dt / tau * (K * ue[n - 1] - v[n - 1])
    return v


def main():
    racine = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    p = argparse.ArgumentParser()
    p.add_argument("fichier", nargs="?")
    p.add_argument("--tau-bf", type=float, default=0.15, help="constante de temps souhaitée en boucle fermée (s)")
    p.add_argument("--vmax", type=float, default=1000.0, help="VMAX_TICKS de parameters.h")
    p.add_argument("--roues-hs", default="FL", help="roues à encodeur HS, ignorées (ex. FL ou FL,RR ; vide = aucune)")
    args = p.parse_args()

    fichier = args.fichier
    if fichier is None:
        candidats = sorted(glob.glob(os.path.join(racine, "raspberry", "essais", "echelon*.csv")))
        if not candidats:
            p.error("aucun fichier echelon*.csv trouvé dans raspberry/essais/")
        fichier = candidats[-1]
    print(f"Fichier : {fichier}")

    d = lire_csv(fichier)
    t, k = d["t"], 255.0 / args.vmax                     # ticks/s -> unités de consigne

    fig, (ax_v, ax_u) = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    fig.suptitle(os.path.basename(fichier))
    ax_v.step(t, d["consigne"], "k--", where="post", lw=1.5, label="consigne")

    print(f"\n{'roue':>4} {'K':>6} {'tau':>6} {'u0':>6}   {'KP_V':>6} {'KI_V':>6}")
    gains = []
    hs = {r.strip().upper() for r in args.roues_hs.split(",") if r.strip()}
    for r in ROUES:
        if r in hs:
            print(f"{r:>4}   encodeur HS, ignorée")
            continue
        v, u = k * d[f"v_{r}"], d[f"pwm_{r}"]
        K, tau, u0 = identifie(t, u, v)
        kp = tau / (K * args.tau_bf)
        ki = kp / tau
        gains.append((kp, ki))
        print(f"{r:>4} {K:6.2f} {tau:6.3f} {u0:6.1f}   {kp:6.2f} {ki:6.2f}")
        ligne, = ax_v.plot(t, v, label=r)
        ax_v.plot(t, simule(t, u, K, tau, u0), ":", color=ligne.get_color())
        ax_u.plot(t, u, label=r)

    kp, ki = np.mean(gains, axis=0)
    print(f"\nmoyenne : KP_V {kp:.2f}f   KI_V {ki:.2f}f   (à recopier dans parameters.h)")
    print("K ~ 1 confirme VMAX_TICKS ; sinon prendre KFF_V = 1/K.")

    ax_v.set_ylabel("vitesse (unités de consigne)")
    ax_v.legend(loc="best")
    ax_v.grid(True)
    ax_u.set_ylabel("PWM")
    ax_u.set_xlabel("t (s)")
    ax_u.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

# Véhicule autonome de livraison — ST5

Projet de véhicule autonome réalisé dans le cadre de la discipline ST5. Le robot suit une ligne, détecte les intersections et peut soit parcourir un circuit en huit, soit rejoindre une destination sur une grille.

## Architecture

Le système est réparti sur trois composants :

|---|---|
| `ordinateur/` | Interface opérateur et serveur de communication ZeroMQ. L’interface envoie une mission au robot et permet de lui transmettre une touche d’arrêt. |
| `raspberry/` | Programme principal du robot : communication avec le serveur, caméra, traitement d’image, suivi de ligne, odométrie, localisation et planification d’itinéraire. |
| `arduino/serial_link/` | Programme de la carte Arduino/Bluno : commande des quatre moteurs, lecture des encodeurs et communication série avec le Raspberry Pi. |

Le Raspberry Pi attend une mission avant d’initialiser l’Arduino et la caméra. Pour une mission sur grille, il détecte les branches aux intersections, met à jour la carte et recalcule l’itinéraire si nécessaire.

## Prérequis

- Un ordinateur et un Raspberry Pi connectés au même réseau.
- Un Raspberry Pi compatible avec `Picamera2` et `libcamera`, une caméra et la carte Arduino/Bluno.
- Python 3 et les modules Python `pyzmq`, `pyserial`, `numpy` et OpenCV (`cv2`).
- Les bibliothèques système `Picamera2` et `libcamera` installées sur le Raspberry Pi.
- Arduino IDE ou un outil compatible pour téléverser le sketch.

Le dépôt ne fournit pas de fichier de dépendances Python. Sur Raspberry Pi OS, privilégiez les paquets système compatibles avec la version de l’OS, en particulier pour `Picamera2` et `libcamera`.

## Configuration du réseau

Le serveur écoute sur le port TCP `5005`. Avant le démarrage :

1. Dans `ordinateur/server.py` et `ordinateur/control.py`, remplacez `server_ip` par l’adresse IP de l’ordinateur sur le réseau du robot.
2. Assurez-vous que cette adresse est disponible et que le pare-feu autorise les connexions TCP entrantes sur le port `5005`.
3. Le Raspberry Pi utilise `SERVER_IP` dans `raspberry/config.py`. Cette valeur peut aussi être fournie en argument au démarrage (voir ci-dessous).

Les valeurs réseau de ces fichiers sont des exemples issus du développement et ne sont pas toutes identiques : elles doivent correspondre à la configuration réelle du réseau.

## Préparation du robot

1. Téléversez `arduino/serial_link/serial_link.ino` sur la carte Arduino/Bluno. Le sketch utilise les autres fichiers `.h` et `.cpp` du même dossier.
2. Vérifiez le port série dans `raspberry/config.py` (`ARDUINO_PORT`, par défaut `/dev/ttyACM0`).
3. Vérifiez les paramètres de caméra, de suivi de ligne, de grille et d’odométrie dans ce même fichier. Les paramètres marqués « À CALIBRER » doivent être mesurés sur le robot et la piste réels.
4. Placez le robot sur une piste adaptée, avec une ligne visible par la caméra et des intersections compatibles avec la détection.

## Démarrage

Lancer chaque programme dans un terminal distinct, depuis le dossier indiqué.

Sur l’ordinateur, démarrer le serveur :

```bash
cd ordinateur
python3 server.py
```

Dans un autre terminal sur l’ordinateur, démarrer l’interface de commande :

```bash
cd ordinateur
python3 control.py
```

Sur le Raspberry Pi, démarrer le programme du robot en indiquant l’adresse IP de l’ordinateur et, facultativement, l’identifiant du robot :

```bash
cd raspberry
python3 robot_main.py <IP_SERVEUR> [ID_ROBOT]
```

Exemple :

```bash
python3 robot_main.py 192.168.0.101 bot001
```

Sans argument, le programme utilise `SERVER_IP` et `ROBOT_ID` de `raspberry/config.py`.

## Missions et commandes

Dans `control.py`, saisir `m` pour envoyer une mission :

- **Circuit en huit** : choisir `h`. Le robot suit la ligne et franchit les intersections en continuant tout droit lorsque cette branche est détectée.
- **Grille** : choisir `g`, puis saisir les coordonnées de départ `x y`, le cap initial (`N`, `E`, `S` ou `W`) et les coordonnées de destination `x y`.
- **Debug caméra** : choisir `d`. Le Raspberry Pi n’ouvre pas la liaison Arduino et n’envoie aucune commande moteur. Une fenêtre affiche côte à côte l’image annotée (ligne et branches détectées) et le masque traité, avec une nouvelle capture toutes les deux secondes. Appuyez sur `q` dans la fenêtre pour fermer le mode debug, ou envoyez `s` depuis `control.py`.

Les nœuds de la grille sont indexés à partir de zéro : `N` augmente `y` et `E` augmente `x`. La taille par défaut est `4 × 4` (`GRID_W` et `GRID_H` dans `raspberry/config.py`). Les coordonnées de départ et de destination doivent être dans la grille.

Pendant une mission, envoyer `q` ou `s` depuis l’interface pour demander l’arrêt du robot. `q` permet également de quitter l’interface. Gardez toujours le robot sous surveillance et prévoyez un moyen d’arrêt physique indépendant.

## Calibrage et limites

- Mesurez et ajustez `CELL_M`, `ENCODER_SCALE`, `ENCODER_SIGN` et `APPROACH_OFFSET_M` dans `raspberry/config.py` pour la piste et les encodeurs utilisés.
- Ajustez si nécessaire la vitesse, le gain de suivi de ligne et les seuils de détection dans `raspberry/config.py` et `raspberry/perception/image_processing.py`.
- La mission en grille s’appuie sur la détection visuelle des branches et sur l’odométrie ; une calibration insuffisante ou une piste différente peut entraîner une localisation imprécise ou l’absence d’itinéraire.
- Le programme arrête les moteurs à la fin ou en cas d’interruption gérée, mais l’arrêt logiciel ne remplace pas un arrêt d’urgence matériel.

## Dépannage

- **Le robot ne reçoit pas de mission** : vérifiez que le serveur est lancé, que les deux appareils utilisent la même adresse IP et que le port `5005` est accessible.
- **L’Arduino ne répond pas** : vérifiez l’alimentation, le câble USB, le port série configuré et que le sketch a bien été téléversé.
- **La caméra ne démarre pas** : vérifiez sa connexion et l’installation de `Picamera2`/`libcamera` sur Raspberry Pi OS.
- **Le suivi ou les intersections sont instables** : vérifiez l’éclairage, le cadrage de la caméra et les paramètres de traitement et de calibrage.

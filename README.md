# Véhicule autonome de livraison — ST5

Projet de véhicule autonome réalisé dans le cadre de la discipline ST5. Le robot suit une ligne, détecte les intersections et peut soit parcourir un circuit en huit, soit rejoindre une destination sur une grille.

## Architecture

Le système est réparti sur trois composants :

|---|---|
| `ordinateur/` | Interface opérateur et serveur de communication ZeroMQ. L’interface envoie une mission au robot et permet de lui transmettre une touche d’arrêt. |
| `raspberry/` | Programme principal du robot : communication avec le serveur, caméra, traitement d’image, suivi de ligne, odométrie, localisation et planification d’itinéraire. |
| `arduino/serial_link/` | Programme de la carte Arduino/Bluno : commande des quatre moteurs, lecture des encodeurs et communication série avec le Raspberry Pi. |

Le Raspberry Pi attend une mission avant d’initialiser l’Arduino et la caméra. Pour une mission sur grille, le serveur calcule l’itinéraire initial en tenant compte des obstacles connus et le transmet au robot. Pendant le déplacement, le Raspberry Pi met à jour la carte à partir des intersections et recalcule localement l’itinéraire lorsqu’un obstacle ou une nouvelle information modifie les passages disponibles.

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
- **Grille** : choisir `g`, saisir la taille (largeur et hauteur, par défaut `4 4`), les coordonnées de départ `x y`, le cap initial (`N`, `E`, `S` ou `W`) et les coordonnées de destination `x y`. Le contrôle propose ensuite d’ajouter des obstacles connus : cliquez sur un nœud pour bloquer toutes ses branches, ou sur un segment pour ne bloquer que celui-ci. Cliquez à nouveau pour retirer un obstacle, puis validez la sélection.
- **Debug caméra** : choisir `d`. Le Raspberry Pi n’ouvre pas la liaison Arduino et n’envoie aucune commande moteur. Une fenêtre affiche côte à côte l’image annotée (ligne et branches détectées) et le masque traité, en analysant chaque image disponible comme dans la boucle de conduite. Le temps de capture et les durées du prétraitement, de détection de ligne et de détection d’intersection sont envoyés au terminal du serveur. Appuyez sur `q` dans la fenêtre pour fermer le mode debug, ou envoyez `s` depuis `control.py`.

Les nœuds de la grille sont indexés à partir de zéro : `N` augmente `y` et `E` augmente `x`. La taille par défaut est `4 × 4` (`GRID_W` et `GRID_H` dans `raspberry/config.py`). Les coordonnées de départ et de destination doivent être dans la grille.

Pendant une mission, saisir `r` dans `control.py` pour abandonner la mission : le driver arrête les moteurs, passe en état `ARRET` et ne reprend plus de capture caméra; le programme libère ensuite la caméra et l’Arduino. `q` quitte l’interface de commande sans remplacer l’arrêt d’urgence matériel. Gardez toujours le robot sous surveillance et prévoyez un moyen d’arrêt physique indépendant.

Pendant une mission sur grille, le serveur ouvre une fenêtre de prévisualisation avec la grille, le départ, la destination, la position actuelle du robot, son prochain nœud cible, l’itinéraire planifié restant et sa trace parcourue. L’itinéraire planifié apparaît en orange pointillé, la trace parcourue en bleu et les segments bloqués par un obstacle Arduino persistant en rouge. Le Raspberry Pi transmet aussi les transitions d’état du driver et les événements de mission/intersection, qui restent visibles dans le terminal du serveur. Les mises à jour sont transportées avec les requêtes périodiques de touches afin que le thread propriétaire de la socket ZeroMQ reste le seul à l’utiliser.

La carte Arduino ne transmet pas de données spontanément : elle répond aux requêtes du Raspberry Pi. Toute activité série entrante (commande moteur ou lecture de capteurs) renouvelle le watchdog. Si aucune activité n’est reçue pendant 750 ms alors qu’une consigne moteur est non nulle, l’Arduino arrête et verrouille les moteurs; il ignore ensuite les commandes de mouvement jusqu’à une nouvelle connexion explicite. Les attentes d’acquittement série du Raspberry Pi expirent après 0,5 s et lèvent `TimeoutError`. Ce watchdog doit être validé sur banc, roues levées, avant tout essai sur piste; il ne remplace pas un arrêt d’urgence matériel.

### Arrêts de sécurité de l’Arduino

Au démarrage d’une mission, le Raspberry Pi active la protection ultrason de l’Arduino (commande `D`, seuil `ARDUINO_OBSTACLE_STOP_CM` dans `raspberry/config.py`, `0` pour la désactiver). L’Arduino bloque alors toute avance dans deux cas : obstacle plus proche que le seuil (`obst`), ou capteur ultrason qui ne répond plus (`muet`, 5 lectures muettes de suite, levé dès qu’une mesure valide revient). Le recul et la rotation restent autorisés.

L’acquittement `OB` est renvoyé lorsqu’une commande demande d’avancer alors qu’un obstacle est détecté. Les commandes d’arrêt, de recul et de rotation restent autorisées afin que le robot puisse s’orienter vers un autre itinéraire. `WD` signale un watchdog déclenché et `MU` un capteur muet (prioritaire sur `OB`). La requête `S` renvoie la distance et l’état (`0` libre, `1` obstacle, `2` capteur muet). Le driver gère `OB` par la procédure de récupération d’obstacle; `WD` et `MU` provoquent un arrêt verrouillé et l’événement `mission_interrupted`. Une absence d’acquittement série (`TimeoutError`) produit l’événement `emergency_stop` avec `cause` = `timeout_serie`. Avec la protection active, un capteur ultrason débranché empêche donc le robot d’avancer.

## Calibrage et limites

- Mesurez et ajustez `CELL_M`, `ENCODER_SCALE`, `ENCODER_SIGN` et `APPROACH_OFFSET_M` dans `raspberry/config.py` pour la piste et les encodeurs utilisés.
- Depuis `raspberry/`, lancez `python3 calibration.py rotation [consigne]` pour mesurer les rotations. Sur une surface plane et dégagée, marquez le cap de départ, faites chaque essai depuis le même repère, puis saisissez l’angle réellement parcouru. Le programme enregistre les vitesses et les deltas d’encodeur dans `raspberry/essais/` et estime les durées de 90° et 180° séparément pour chaque sens. Répétez à différentes consignes pour comparer les vitesses. Sans gyroscope, l’angle saisi reste une mesure manuelle; ces estimations dépendent du sol, de la batterie et de la consigne. **Attention : la calibration désactive la protection ultrason.** Gardez le robot sur une surface plane et dégagée, et restez près de l’arrêt d’urgence.
- Pour appliquer les temps mesurés aux virages, réglez `VITESSE_ROTATION` et `TEMPS_ROTATION` dans `raspberry/config.py`. Pour chaque direction (`gauche`, `droite`) et angle (`90`, `180`), indiquez `(temps_min, temps_max)` en secondes, mesurés à cette vitesse. Le robot tourne à vitesse normale jusqu’au minimum, puis cherche la ligne; s’il ne la trouve pas au maximum, il ralentit à `VITESSE_BALAYAGE_ROTATION` et continue à tourner jusqu’à ce que la ligne soit centrée pendant `IMAGES_ALIGNEES_ROTATION` images consécutives. Le balayage n’a pas de délai maximal : gardez le robot sous surveillance et utilisez l’arrêt opérateur si la ligne n’est pas retrouvée. Les demi-tours initiaux de 180° utilisent leurs propres durées.
- Ajustez si nécessaire la vitesse, le gain de suivi de ligne et les seuils de détection dans `raspberry/config.py` et `raspberry/perception/image_processing.py`.
- La mission en grille s’appuie sur la détection visuelle des branches et sur l’odométrie ; une calibration insuffisante ou une piste différente peut entraîner une localisation imprécise ou l’absence d’itinéraire.
- Le programme arrête les moteurs à la fin ou en cas d’interruption gérée, mais l’arrêt logiciel ne remplace pas un arrêt d’urgence matériel.

## Dépannage

- **Le robot ne reçoit pas de mission** : vérifiez que le serveur est lancé, que les deux appareils utilisent la même adresse IP et que le port `5005` est accessible.
- **L’Arduino ne répond pas** : vérifiez l’alimentation, le câble USB, le port série configuré et que le sketch a bien été téléversé.
- **La caméra ne démarre pas** : vérifiez sa connexion et l’installation de `Picamera2`/`libcamera` sur Raspberry Pi OS.
- **Le suivi ou les intersections sont instables** : vérifiez l’éclairage, le cadrage de la caméra et les paramètres de traitement et de calibrage.
- **Le robot refuse d’avancer (cause `muet`)** : vérifiez le câblage du capteur ultrason (sortie PWM sur la broche 2) ou mettez `ARDUINO_OBSTACLE_STOP_CM = 0` pour désactiver la protection.

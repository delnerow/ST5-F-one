import cv2
import numpy as np
import time 

# Paramètres (ligne)
TAILLE_FLOU = (5, 5)      
ECART = 60                
                          
NOYAU = np.ones((3, 3), np.uint8)   
ITERATIONS = 1            
HAUT_ZONE = 0.6           
AIRE_MIN = 100            
# Paramètres (intersection)
ZONE_HAUT = 0.2      # on cherche l'intersection entre 20 % et 90 % de la hauteur de l'image
ZONE_BAS = 0.9
RATIO = 1.5          # une rangée est transversale si son blanc continu depuis un bord dépasse 1,5 fois la ligne
RANGEES_MIN = 3      # il faut au moins 3 rangées transversales (sinon c'est du bruit)
BORD = 0.08          # une branche latérale doit arriver à moins de 8 % du bord de l'image


def traitement(image):
"""1er traitement de l'image, commun à la ligne et à l'intersection."""
    h, w = image.shape[:2]
    # Convert to gray scale image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Blur and threshold
    blur = cv2.blur(gray, TAILLE_FLOU)
    seuil = np.percentile(blur,10) + ECART
    ret, thresh = cv2.threshold(blur, seuil, 255, cv2.THRESH_BINARY)

    # Remove noise with morphological processing
    eroded_mask = cv2.erode(thresh, NOYAU, iterations=ITERATIONS)
    closed_mask = cv2.dilate(eroded_mask, NOYAU, iterations=ITERATIONS)
    return closed_mask, h
    


def find_line(closed_mask, h):
    # On ne garde que le bas de l'image 
    bas = closed_mask[int(HAUT_ZONE * h):, :]

    # Find the different contours
    contours, hierarchy = cv2.findContours(bas, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return None

    # Biggest area = the line
    biggest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(biggest) < AIRE_MIN:      # AJOUT : rejette les petites taches
        return None

    # Compute barycenter
    M = cv2.moments(biggest)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00']) + int(HAUT_ZONE * h)   # coordonnée dans l'image entière
    return cx, cy


def find_intersection(closed_mask, h):
    w = closed_mask.shape[1]          

    # 1. Nombre de pixels blancs sur chaque rangée de la zone étudiée
    y0, y1 = int(ZONE_HAUT * h), int(ZONE_BAS * h)
    blancs_par_rangee = np.count_nonzero(closed_mask[y0:y1, :], axis=1)
    rangees_avec_ligne = blancs_par_rangee[blancs_par_rangee > 0]
    if len(rangees_avec_ligne) < 10:
        return None                       # presque pas de ligne visible : on ne conclut rien

    # 2. Longueur de blanc CONTINU depuis le bord gauche et depuis le bord droit, rangée par rangée.
    #    Une branche fait 80 cm à 1 m : elle part toujours du bord de l'image et rejoint la ligne.
    #    Une tache ou un reflet près du bord donne un blanc court, la ligne seule aussi.
    zone = closed_mask[y0:y1, :] > 0
    blanc_depuis_gauche = np.where(zone.all(axis=1), w, np.argmin(zone, axis=1))
    blanc_depuis_droite = np.where(zone.all(axis=1), w, np.argmin(zone[:, ::-1], axis=1))
    touche_bord = (blanc_depuis_gauche > 0) | (blanc_depuis_droite > 0)

    # 3. Largeur habituelle de la ligne = médiane des rangées qui ne touchent pas un bord
    #    (la barre transversale est exclue, même quand elle occupe beaucoup de rangées)
    largeur_interieur = blancs_par_rangee[(blancs_par_rangee > 0) & ~touche_bord]
    if len(largeur_interieur) < 5:
        return None                       # pas assez de ligne "normale" pour comparer
    largeur_ligne = np.median(largeur_interieur)

    # 4. Rangées transversales : blanc continu depuis un bord, nettement plus long que la ligne.

    plus_long = np.maximum(blanc_depuis_gauche, blanc_depuis_droite)
    rangees = np.where(plus_long > RATIO * largeur_ligne)[0]
    if len(rangees) < RANGEES_MIN:
        return None                       # pas de ligne transversale : pas d'intersection
    y_trans = y0 + int(np.median(rangees))

    # 5. Position de la ligne sous la transversale (barycentre, comme find_line)
    dessous = closed_mask[y_trans + 5:y1, :]
    M = cv2.moments(dessous, binaryImage=True)
    if M['m00'] == 0:
        return None
    cx = M['m10'] / M['m00']

    # 6. Branches à gauche et à droite : la transversale atteint-elle le bord de l'image ?
  
    #    un bout de ruban qui dépasse de 2-3 cm, non. On prend toutes les rangées de la
    #    transversale (+/- 3) pour bien voir une ligne posée de travers.
    bande = closed_mask[max(0, y0 + rangees.min() - 3):y0 + rangees.max() + 4, :]
    colonnes_blanches = np.where(bande.max(axis=0) > 0)[0]
    gauche = colonnes_blanches.min() < BORD * w
    droite = colonnes_blanches.max() > (1 - BORD) * w

    # 7. Branche devant : la ligne continue-t-elle au-dessus de la transversale ?
  
    haut = max(0, y_trans - int(0.30 * h))
    bas = max(0, y_trans - int(0.15 * h))
    x_min = int(max(0, cx - 3 * largeur_ligne))
    x_max = int(min(w, cx + 3 * largeur_ligne))
    au_dessus = closed_mask[haut:bas, x_min:x_max]
    devant = au_dessus.shape[0] > 0 and np.mean(au_dessus.max(axis=1) > 0) > 0.5

   
    if not (gauche or droite or devant):
        return None

    return {'gauche': bool(gauche), 'droite': bool(droite), 'devant': bool(devant),
            'distance': round(1 - y_trans / h, 2)}

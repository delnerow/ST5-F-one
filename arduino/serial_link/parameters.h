#ifndef PARAMETERS_H
#define PARAMETERS_H

#define SERIAL_BAUD 115200
#define SERIAL_TIMEOUT 100
#define SERIAL_WATCHDOG_TIMEOUT_MS 750

/* full Bluno wiring */
const int ServofrontPin = 26;
const int IR_pin = A6;

const int dirMotor1 = 4;  // motor direction pin
const int vitMotor1 = 3;  // motor PWM pin
const int Encoder1B = 18;  // encoder speed pin -> white
const int Encoder1A = 31;  // encoder direction pin -> yellow

const int dirMotor2 = 12; // motor direction pin
const int vitMotor2 = 11; // motor PWM pin
const int Encoder2B = 19;  // encoder speed pin -> white
const int Encoder2A = 32;  // encoder direction pin -> yellow

const int dirMotor3 = 8;  // motor direction pin
const int vitMotor3 = 5;  // motor PWM pin
const int Encoder3B = 20;  // encoder speed pin -> white
const int Encoder3A = 33;  // encoder direction pin -> yellow

const int dirMotor4 = 7;  // motor direction pin
const int vitMotor4 = 6;  // motor PWM pin
const int Encoder4B = 21;  // encoder speed pin -> white
const int Encoder4A = 34;  // encoder direction pin -> yellow

// [PI] régulation de vitesse en boucle fermée (task2, toutes les 20 ms)
//   La consigne de la commande C (-255..255) devient une consigne de VITESSE :
//   255 <-> VMAX_TICKS ticks/s. Les vitesses mesurées sont renvoyées par la question P (ticks/s).
#define BOUCLE_FERMEE 1          // 0 = ancienne rampe sur la PWM (boucle ouverte)
#define VMAX_TICKS    1000.0f    // (À CALIBRER) ticks/s mesurés à PWM 255, roues levées, BOUCLE_FERMEE 0
                                 // théorique : 280 tr/min à vide sous 6 V x 234,3 ticks/tr roue / 60 = 1090
#define KFF_V         1.0f       // anticipation (1 si VMAX_TICKS est bien calibré)
#define KP_V          0.6f       // (À RÉGLER) gain proportionnel (PWM par unité de consigne)
#define KI_V          4.0f       // (À RÉGLER) gain intégral (1/s) ; Ti = KP_V/KI_V ~ constante de temps moteur
#define ALPHA_V       0.4f       // filtre de la mesure (0..1, 1 = pas de filtre)
// (À CALIBRER) -1 si le compteur de la roue diminue quand elle tourne en marche avant
//                     FR  FL  RR  RL
const int encSign[4] = { 1,  1,  1,  1 };

// [OBSTACLE] capteur ultrason URM37 en mode automatique (une mesure toutes les 25 ms)
const int URM_ECHO = 2;                  // sortie PWM du capteur (fil vert) -> broche 2 (INT0)
#define URM_TIMEOUT_MS   200             // sans mesure depuis 200 ms -> capteur considéré muet
#define DIST_HORS_PORTEE 999             // valeur renvoyée quand rien n'est détecté (> 5 m)
#define OBST_HYSTERESIS  5               // cm au-dessus du seuil pour lever l'obstacle (évite les oscillations)
#define OBST_PERIOD_MS   30              // période de la tâche de surveillance (ms)
#define OBST_N_MUET      10               // lectures "capteur muet" consécutives avant de déclarer l'obstacle
#define OBST_N_PROCHE    2               // lectures "d < seuil" consécutives avant de déclarer l'obstacle
#define OBST_DECEL       25              // baisse de PWM par période lors d'un arrêt d'urgence :
                                         // 100 -> 0 en 4 x 30 ms = 120 ms (env. 2 cm de plus)
                                         // à baisser si la batterie se coupe, à monter pour freiner plus court

#endif

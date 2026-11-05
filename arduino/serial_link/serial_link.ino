//////////////////////////////////////////////////////-
// Ce programme a été développé à CENTRALESUPELEC
// Merci de conserver ce cartouche
// Copyright  (c) 2026  CENTRALE-SUPELEC
// Département Electronique et électromagnétisme
// //////////////////////////////////////////////////-
//
// fichier : Arduino_commande_moteurs
// auteur  : P.BENABES
// éditeur : M.ROGER
// Copyright (c) 2026 CENTRALE-SUPELEC
// Revision: 2.1  Date: 01/11/2026
//
// [OBSTACLE] ajouts : mesure ultrason URM37 par interruption,
//            requête S (distance), commande D (protection), tâche 3 (arrêt de sécurité)
// [MUET] ajout : état "capteur ultrason muet" séparé de l'état obstacle
//
// //////////////////////////////////////////////////-
//
// SKETCH DESCRIPTION:
// program running on the Bluno board
// for the ST5 VAC mobile robot
//
//////////////////////////////////////////////////////

#include "parameters.h"   // paramètres de la carte (N° de broches, etc ...)
#include "Motor.h"        // pour utiliser les onjets Moteur
#include "MyServo.h"      // pour utiliser les objets servo moteur
//#include "Screen.h"       // pour utiliser les objets ecran
#include "Sercomm.h"      // toutes les fonctions de la liaison série
#include "Task.h"         // pour la gestion des taches
#include "Commandes.h"    // toutes les commandes reçues par la liaison série

///////////////////////////////////////////////////////////////////////////
//
//    DEFINITION DES VARIABLES GLOBALES ET OBJETS
//
/////////////////////////////////////////////////////////////////////////

bool obst=false ;    // obstacle détecté
bool muet=false ;    // [MUET] capteur ultrason muet (débranché, en panne)
bool ConnOn ;        // indique si la carte est connectée
char feedback ;      // si non nul indique que les commandes doivent renvoyer un acquitement
char commode=0 ;     // indique le mode de communication 0=tout en ASCII, 1 les commandes sont en binaire et les retours en ascii, 2 tout est en binaire


// variables de la gestion des moteurs
Motor FRmotor(vitMotor1,dirMotor1,0); // front right
Motor FLmotor(vitMotor2,dirMotor2,0); // front left
Motor RRmotor(vitMotor3,dirMotor3,0); // rear right
Motor RLmotor(vitMotor4,dirMotor4,0); // rear left

// les compteurs incrémentaux
long int volatile CoderCount[4] = {0,0,0,0};

// gestion du servomoteur
MyServo frontservo(ServofrontPin,30,150,10);

// [OBSTACLE] gestion du capteur ultrason
volatile unsigned long urmPulseStart = 0;     // instant (µs) du début de l'impulsion basse
volatile unsigned long urmPulseDuration = 0;  // durée (µs) de la dernière impulsion basse : 50 µs = 1 cm
volatile unsigned long urmLastEcho = 0;       // instant (ms) de la dernière mesure complète
int seuilObst = 0;                            // seuil d'arrêt en cm ; 0 = protection désactivée


///////////////////////////////////////////////////////////////////////////
//
//   INITIALISATION DE L'ARDUINO
//
/////////////////////////////////////////////////////////////////////////

void init_arduino() {

  FRmotor.setvalue(0);
  FLmotor.setvalue(0);
  RRmotor.setvalue(0);
  RLmotor.setvalue(0);
  initVitesse();   // [PI] remise à zéro des consignes, PWM, intégrales et compteurs de vitesse
  frontservo.start() ;
  frontservo.setpos((frontservo.getmin()+frontservo.getmax())/2);

  TaskOffAll();
  obst=false ;
  muet=false ;    // [MUET]
  seuilObst=0 ;   // [OBSTACLE] protection désactivée à chaque (dé)connexion
  TaskOn(0) ;     // on allume la tache permanente de lecture de la liaison série
  TaskOn(2) ;     // [PI] on allume la tache de régulation de vitesse des moteurs
  TaskOn(3) ;     // [OBSTACLE] on allume la tache de surveillance des obstacles
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT); // timeout pour attendre une fin de message

  init_arduino() ;

  attachInterrupt(digitalPinToInterrupt(Encoder1B), FRcoderISR, FALLING); // interrupt attachment for encoder routines
  attachInterrupt(digitalPinToInterrupt(Encoder2B), FLcoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder3B), RRcoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder4B), RLcoderISR, FALLING);

  // [OBSTACLE] capteur ultrason : on mesure la durée de chaque impulsion basse
  pinMode(URM_ECHO, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(URM_ECHO), URMechoISR, CHANGE);
}

////////////////////////////////////////////////////////////
//
// la boucle principale
//
/////////////////////////////////////////////////////////////


void loop() {
  launch_tasks() ;
}

////////////////////////////////////////////////////////////
//
// les routines d'interruption
//
////////////////////////////////////////////////////////////

void FRcoderISR() {
  CoderCount[0] += ((digitalRead(Encoder1A) == HIGH) ? +1 : -1);
}

void FLcoderISR() {
  CoderCount[1] += ((digitalRead(Encoder2A) == LOW) ? +1 : -1);
}

void RRcoderISR() {
  CoderCount[2] += ((digitalRead(Encoder3A) == HIGH) ? +1 : -1);
}

void RLcoderISR() {
  CoderCount[3] += ((digitalRead(Encoder4A) == LOW) ? +1 : -1);
}

// [OBSTACLE] front descendant = début de la mesure, front montant = fin de la mesure
void URMechoISR() {
  if (digitalRead(URM_ECHO) == LOW)
    urmPulseStart = micros();
  else {
    urmPulseDuration = micros() - urmPulseStart;
    urmLastEcho = millis();
  }
}

////////////////////////////////////////////////////////////
//
// [OBSTACLE] lecture de la distance
//   renvoie la distance en cm,
//   DIST_HORS_PORTEE si rien n'est détecté,
//   -1 si le capteur ne répond plus (débranché, en panne)
//
////////////////////////////////////////////////////////////

int getDistanceCm() {
  unsigned long duree, derniere;
  noInterrupts();                 // copie des variables partagées avec l'interruption
  duree = urmPulseDuration;
  derniere = urmLastEcho;
  interrupts();

  if ((derniere == 0) || (millis() - derniere > URM_TIMEOUT_MS))
    return -1;                    // aucune mesure récente : capteur muet
  unsigned long cm = duree / 50;  // 50 µs = 1 cm
  if (cm > 250)
    return DIST_HORS_PORTEE;      // mesure invalide (impulsion de 50 000 µs) : rien devant
  return (int)cm;
}

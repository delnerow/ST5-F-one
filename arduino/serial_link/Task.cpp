#include "Arduino.h"
#include "parameters.h"
#include "Motor.h"
#include "MyServo.h"
#include "Commandes.h"
#include "Task.h"

// on définit ici toutes les variables auxquelles doivent accéder les taches
void init_arduino();
extern Motor FRmotor;
extern Motor FLmotor;
extern Motor RRmotor;
extern Motor RLmotor;
extern long int volatile CoderCount[4] ;
extern bool obst;
extern bool muet;              // [MUET] capteur ultrason muet
extern MyServo frontservo;
extern int seuilObst;          // [OBSTACLE] seuil d'arrêt en cm (0 = protection désactivée)
extern int getDistanceCm();    // [OBSTACLE] défini dans serial_link.ino

int value[4] = {0,0,0,0};  // 'signed PWM' values between -255 and 255
int setpoint[4] = {0,0,0,0}; // PWM setpoints between -255 and 255
unsigned char deltaMax = 10;  // maximum gap between 2 successive values

// gestion du multitache. définition de la période de chaque tache
// definition du nombre de taches maximales
#define numtask 4                                    // [OBSTACLE] 3 -> 4
int deltask[numtask]={0,1000,20,OBST_PERIOD_MS} ;    // [OBSTACLE] période de la tâche 3
int timtask[numtask] ;
bool taskon[numtask]={true,false,true,true} ;
unsigned long lastSerialActivityMs = 0;
bool watchdogTripped = false;
void task0(); // tache permanente de lecture de la liaison série
void task1(); // arrête les moteurs si la liaison série est perdue
void task2(); // tache de transition douce pour les moteurs
void task3(); // [OBSTACLE] tache de surveillance des obstacles


void (*NameTask[numtask])() = {  task0,task1,task2,task3  };

///////////////////////////////////////////////////////////////////////////
//   mise en route et arret des taches périodiques
/////////////////////////////////////////////////////////////////////////
void TaskOn(int task)
{ taskon[task]=true;
  timtask[task] = (int)millis()+deltask[task];
}

void TaskOff(int task)
{ taskon[task]=false;
}

void TaskOffAll() {
  for (int k=1; k<numtask; k++)
    taskon[k]=false ;
}

bool TaskState(int task) {
  return(taskon[task]);
}

  // lancement des differentes taches périodiques
void launch_tasks() {
  for (int k=0;k<numtask;k++)
    if (taskon[k])
      if ((((int)millis()-timtask[k])>0) || (deltask[k]==0))   // si on a dépassé le temps du prochain lancement ou si tache permanente
      {
        NameTask[k]() ;                   // on lance la tache k
        timtask[k]=timtask[k]+deltask[k] ;
      }
}

////////////////////////////////////////////////////////////////////
//
// LES TACHES PERIODIQUES
//
////////////////////////////////////////////////////////////////////

extern bool ConnOn ;

// tache permanente de lecture de la liaison série
void task0() {
  char CharIn;
  if (Serial.available() > 0) {
    // on lit le premier caractère qui arrive
    CharIn = Serial.read();
    lastSerialActivityMs = millis();
    if ((CharIn=='A') || (ConnOn)) decod_serial(CharIn);  // on ne fait un decodage de commande que si on est connecté
  }

}

// tache du chien de garde
void task1() {
  if (!ConnOn || watchdogTripped) return;

  bool motorsCommanded = false;
  for (int i = 0; i < 4; i++) {
    if (setpoint[i] != 0) {
      motorsCommanded = true;
      break;
    }
  }

  if (motorsCommanded &&
      (unsigned long)(millis() - lastSerialActivityMs) > SERIAL_WATCHDOG_TIMEOUT_MS) {
    for (int i = 0; i < 4; i++) {
      setpoint[i] = 0;
      value[i] = 0;
    }
    FRmotor.setvalue(0);
    FLmotor.setvalue(0);
    RRmotor.setvalue(0);
    RLmotor.setvalue(0);
    watchdogTripped = true;  // Require an explicit reconnect before accepting motor commands.
  }
}

void task2() {
  for (int i=0; i<4; i++) {
    if (value[i] < setpoint[i])
      if (value[i] > (setpoint[i]-deltaMax) )
        value[i] = setpoint[i];
      else
        value[i] += deltaMax;
    else
      if (value[i] < (setpoint[i]+deltaMax) )
        value[i] = setpoint[i];
      else
        value[i] -= deltaMax;
  }

  FRmotor.setvalue(value[0]); // after updating the value to improve reactivity to new setpoints
  FLmotor.setvalue(value[1]);
  RRmotor.setvalue(value[2]);
  RLmotor.setvalue(value[3]);
}

////////////////////////////////////////////////////////////////////
// [OBSTACLE] tache 3 : surveillance des obstacles (toutes les OBST_PERIOD_MS)
//
//  - protection désactivée (seuilObst = 0)    : ne fait rien
//  - obstacle si distance < seuil sur OBST_N_PROCHE lectures consécutives
//  - fin d'obstacle si distance > seuil + hystérésis
//  - [MUET] capteur muet si le capteur ne répond plus sur OBST_N_MUET lectures
//    consécutives ; fin dès qu'une mesure valide revient. Pendant que le capteur
//    est muet, l'état obstacle est gelé (aucune mesure pour le lever ou le baisser).
//  - en présence d'obstacle OU de capteur muet, tout mouvement vers l'avant (somme
//    des 4 consignes > 0) est annulé et freiné rapidement (OBST_DECEL). Le recul,
//    la rotation sur place et la translation latérale restent autorisés pour se dégager.
////////////////////////////////////////////////////////////////////
void task3() {
  static unsigned char nbMuet = 0;             // lectures "capteur muet" consécutives
  static unsigned char nbProche = 0;           // lectures "d < seuil" consécutives

  if (seuilObst <= 0) {
    obst = false;
    muet = false;
    nbMuet = 0;
    nbProche = 0;
    return;
  }

  int d = getDistanceCm();

  // filtre : mise à jour des compteurs de lectures consécutives
  if (d < 0) {
    if (nbMuet < OBST_N_MUET) nbMuet++;
    nbProche = 0;
  }
  else if (d < seuilObst) {
    if (nbProche < OBST_N_PROCHE) nbProche++;
    nbMuet = 0;
  }
  else {
    nbMuet = 0;
    nbProche = 0;
  }

  // [MUET] capteur muet (OBST_N_MUET lectures de suite), levé dès une mesure valide
  if (nbMuet >= OBST_N_MUET)
    muet = true;
  else if (d >= 0)
    muet = false;

  // obstacle trop proche (OBST_N_PROCHE lectures de suite) / voie libre
  if (nbProche >= OBST_N_PROCHE)
    obst = true;
  else if (d > seuilObst + OBST_HYSTERESIS)
    obst = false;
  // sinon (entre seuil et seuil + hystérésis, lecture pas encore confirmée ou capteur muet) :
  // on garde l'état précédent

  bool arret = obst || muet;

  if (arret && (setpoint[0]+setpoint[1]+setpoint[2]+setpoint[3] > 0)) {
    for (int i=0; i<4; i++)                    // consigne d'arrêt
      setpoint[i] = 0;
  }

  // freinage rapide mais progressif : une coupure brutale de la PWM crée un pic de tension
  // qui peut éteindre ou abîmer la batterie moteurs (cf. Doc robot - Prise en main).
  // La valeur baisse de OBST_DECEL à chaque passage (toutes les OBST_PERIOD_MS),
  // au lieu de deltaMax toutes les 20 ms pour la rampe normale.
  if (arret && (value[0]+value[1]+value[2]+value[3] > 0)) {
    for (int i=0; i<4; i++) {
      if (value[i] > setpoint[i] + OBST_DECEL)       value[i] -= OBST_DECEL;
      else if (value[i] < setpoint[i] - OBST_DECEL)  value[i] += OBST_DECEL;
      else                                           value[i] = setpoint[i];
    }
    FRmotor.setvalue(value[0]);
    FLmotor.setvalue(value[1]);
    RRmotor.setvalue(value[2]);
    RLmotor.setvalue(value[3]);
  }
}

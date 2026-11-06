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
int setpoint[4] = {0,0,0,0}; // consignes entre -255 et 255 (vitesse si BOUCLE_FERMEE, PWM sinon)
unsigned char deltaMax = 20;  // pente max : de la référence de vitesse (PI) ou de la PWM (boucle ouverte)

// [PI] état de la régulation de vitesse (indices : 0 FR, 1 FL, 2 RR, 3 RL)
static float ref[4]    = {0,0,0,0};   // référence rampée, en unités de consigne
static float integ[4]  = {0,0,0,0};   // terme intégral, en PWM
static float vitMes[4] = {0,0,0,0};   // vitesse mesurée filtrée, en ticks/s (signe corrigé)
static long lastCount[4] = {0,0,0,0};
static unsigned long lastMicros = 0;

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
void task2(); // [PI] mesure de vitesse + régulation (ou rampe en boucle ouverte)
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
// [PI] RÉGULATION DE VITESSE
//
////////////////////////////////////////////////////////////////////

static void resetRegulateurs() {
  for (int i=0; i<4; i++) { ref[i] = 0; integ[i] = 0; }
}

// remise à zéro complète, appelée par init_arduino() (connexion, déconnexion, reset)
void initVitesse() {
  long c[4];
  noInterrupts();                       // un long n'est pas lu de façon atomique sur AVR
  for (int i=0; i<4; i++) c[i] = CoderCount[i];
  interrupts();
  for (int i=0; i<4; i++) {
    setpoint[i] = 0;
    value[i] = 0;
    lastCount[i] = c[i];
    vitMes[i] = 0;
  }
  resetRegulateurs();
  lastMicros = micros();
}

// vitesse mesurée (ticks/s, positive en marche avant) : question P
int getVitesseMesuree(int i) {
  return (int)vitMes[i];
}

// mesure des 4 vitesses ; renvoie dt en s (0 si la mesure n'est pas exploitable)
static float mesureVitesses() {
  long c[4];
  noInterrupts();
  for (int i=0; i<4; i++) c[i] = CoderCount[i];
  interrupts();

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  if (dt <= 0.0f || dt > 0.2f) {        // premier passage ou tâche longtemps suspendue
    for (int i=0; i<4; i++) lastCount[i] = c[i];
    return 0.0f;
  }
  for (int i=0; i<4; i++) {
    float v = encSign[i] * (float)(c[i] - lastCount[i]) / dt;
    lastCount[i] = c[i];
    vitMes[i] += ALPHA_V * (v - vitMes[i]);   // filtre passe-bas du 1er ordre
  }
  // encodeur HS : on renvoie la vitesse de la roue recopiée (estimation pour l'odométrie)
  for (int i=0; i<4; i++)
    if (copieDe[i] >= 0) vitMes[i] = vitMes[copieDe[i]];
  return dt;
}

static void appliquePWM() {
  FRmotor.setvalue(value[0]);
  FLmotor.setvalue(value[1]);
  RRmotor.setvalue(value[2]);
  RLmotor.setvalue(value[3]);
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
    resetRegulateurs();                 // [PI] sinon l'intégrale relancerait les moteurs
    FRmotor.setvalue(0);
    FLmotor.setvalue(0);
    RRmotor.setvalue(0);
    RLmotor.setvalue(0);
    watchdogTripped = true;  // Require an explicit reconnect before accepting motor commands.
  }
}

////////////////////////////////////////////////////////////////////
// [PI] tache 2 (toutes les 20 ms)
//   - mesure toujours les vitesses (sert aussi à l'odométrie, question P)
//   - BOUCLE_FERMEE = 1 : PI par roue avec anticipation, référence rampée (deltaMax)
//     et anti-windup par intégration conditionnelle ; une roue à encodeur HS
//     (copieDe dans parameters.h) recopie la PWM d'une autre roue
//   - BOUCLE_FERMEE = 0 : ancienne rampe sur la PWM (boucle ouverte)
////////////////////////////////////////////////////////////////////
void task2() {
  float dt = mesureVitesses();

  if (watchdogTripped) {                // moteurs verrouillés à 0 par task1
    resetRegulateurs();
    return;
  }

  // même règle que task3 : pas de consigne vers l'avant en cas d'obstacle / capteur muet
  bool arret = obst || muet;
  if (arret && (setpoint[0]+setpoint[1]+setpoint[2]+setpoint[3] > 0))
    for (int i=0; i<4; i++) setpoint[i] = 0;

  // freinage d'urgence en cours : task3 pilote directement la PWM
  if (arret && (value[0]+value[1]+value[2]+value[3] > 0)) {
    resetRegulateurs();
    return;
  }

#if BOUCLE_FERMEE
  if (dt == 0.0f) return;               // pas de mesure valide : on garde la PWM actuelle

  const float k = 255.0f / VMAX_TICKS;  // ticks/s -> unités de consigne

  for (int i=0; i<4; i++) {
    // rampe sur la référence (douceur pour la batterie, comme l'ancienne rampe PWM)
    float sp = setpoint[i];
    if (ref[i] < sp - deltaMax)      ref[i] += deltaMax;
    else if (ref[i] > sp + deltaMax) ref[i] -= deltaMax;
    else                             ref[i] = sp;

    // arrêt franc : pas d'intégrale qui fait bourdonner les moteurs à l'arrêt
    if (sp == 0 && ref[i] == 0) {
      integ[i] = 0;
      value[i] = 0;
      continue;
    }

    // encodeur HS : pas de PI pour cette roue (commande recopiée après la boucle)
    if (copieDe[i] >= 0) {
      integ[i] = 0;
      continue;
    }

    float e = ref[i] - k * vitMes[i];
    float iNew = integ[i] + KI_V * e * dt;
    float u = KFF_V * ref[i] + KP_V * e + iNew;

    // anti-windup : on n'intègre pas si cela pousse plus loin dans la saturation
    if (!((u > 255.0f && e > 0) || (u < -255.0f && e < 0)))
      integ[i] = constrain(iNew, -255.0f, 255.0f);

    u = KFF_V * ref[i] + KP_V * e + integ[i];
    value[i] = (int)constrain(u, -255.0f, 255.0f);
  }

  // encodeur HS : même consigne -> même PWM que la roue recopiée,
  // sinon (consignes différentes) anticipation seule, en boucle ouverte
  for (int i=0; i<4; i++) {
    int j = copieDe[i];
    if (j < 0 || (setpoint[i] == 0 && ref[i] == 0)) continue;
    if (setpoint[i] == setpoint[j]) value[i] = value[j];
    else value[i] = (int)constrain(KFF_V * ref[i], -255.0f, 255.0f);
  }
#else
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
#endif

  appliquePWM();
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
//  - [PI] pendant ce freinage, task2 ne régule pas (elle remet le PI à zéro).
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
    appliquePWM();
  }
}

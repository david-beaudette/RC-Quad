#include "PPMReader.h"

// RC receiver channel
#define RC_RX 0

// Pins associated to each motor signal
#define DRV_PWM 5
#define DIR_L_PWM 9
#define DIR_R_PWM 10
#define LED_PWM 11
#define DIR_SEN A0

const float moteur_av_gain_pid[3] = {0.05f, 0.01f, 0.0f};

PPMReader ppmReader(2, 0, false);

// Controle de la direction et de la propulsion
int pid(long &integrale,
        long &erreur_prec,
        int &commande_prec,
        const long commande_desiree, 
        const long lecture,
        const int integrale_erreur_min, 
        const int commande_min, 
        const int commande_max, 
        const float *gain_pid);

void setup()
{
  Serial.begin(115200);
  Serial.println("Ready to race.");
   
  ppmReader.start();
  
  pinMode(DRV_PWM, OUTPUT); 
  pinMode(DIR_L_PWM, OUTPUT); 
  pinMode(DIR_R_PWM, OUTPUT);
  pinMode(LED_PWM, OUTPUT); 
  
  pinMode(DIR_SEN, INPUT); 
  
}

void loop()
{

  // Reading the direction potentiometer value
  long dir_sen_val = analogRead(DIR_SEN);
  
  Serial.print("Direction sensor value: ");
  Serial.print(dir_sen_val);
  Serial.println(".");


  uint8_t ch_num = 0;
  int ppm_ch[8];
  while(ppmReader.get(ch_num) != 0){  
    // Print out the servo values
    Serial.print(ppmReader.get(ch_num));
    Serial.print("  ");
    ppm_ch[ch_num] = ppmReader.get(ch_num);
    ch_num++;
  }
  Serial.println("");

  /* Set motor PWM 
  byte dir_l, dir_r, drv, led;

  lf = constrain(map(ppm_ch[RC_CH_3], 1500, 2000, 0, 255), 0, 255);
  lr = constrain(map(ppm_ch[RC_CH_3], 1000, 1500, 255, 0), 0, 255);
  rf = constrain(map(ppm_ch[RC_CH_4], 1500, 2000, 0, 255), 0, 255);
  rr = constrain(map(ppm_ch[RC_CH_4], 1000, 1500, 255, 0), 0, 255);

  analogWrite(ML_FWD, lf);
  analogWrite(ML_REV, lr);
  analogWrite(MR_FWD, rf);
  analogWrite(MR_REV, rr);*/
}


int pid(long &integrale,
        long &erreur_prec,
        int &commande_prec,
        const long commande_desiree, 
        const long lecture,
        const int commande_min, 
        const int commande_max, 
        const float *gain_pid) {
  
  long erreur = commande_desiree - lecture;
  
  float commande_pid = gain_pid[0] * (float)erreur + 
                       gain_pid[1] * (float)(erreur - erreur_prec) + 
                       gain_pid[2] * (float)integrale;
    
  int commande_actuelle;
  int commande_non_saturee = static_cast<int>(commande_pid);
  
  bool saturation = true;
  if(commande_non_saturee < commande_min) 
    commande_actuelle = commande_min;
  else if(commande_non_saturee > commande_max) 
    commande_actuelle = commande_max;
  else {
    commande_actuelle = commande_non_saturee;
    saturation = false;
  }
  
  // L'integrale diverge sans mise a jour conditionnelle
  if(erreur * commande_non_saturee <= 0 || 
     !saturation) {
    integrale += erreur;      
  }
  
  // Mise a jour de l'erreur        
  erreur_prec = erreur;
  commande_prec = commande_non_saturee;
  return commande_actuelle;        
}

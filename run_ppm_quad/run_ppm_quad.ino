#include "PPMReader.h"

// RC receiver channel
#define RC_RX_PIN 2
#define RC_CH_IDX_DIR 3
#define RC_CH_IDX_VRA 4
#define RC_CH_IDX_VRB 5

// Pins associated to each motor signal
#define DRV_PWM 5
#define DIR_L_PWM 10
#define DIR_R_PWM 9
#define LED_PWM 11
#define DIR_SEN A0

#define DIR_CMD_CEN 1500
#define DIR_CMD_LEFT 1000
#define DIR_CMD_RIGHT 2000
#define DIR_SEN_CEN 550
#define DIR_SEN_LEFT 750
#define DIR_SEN_RIGHT 350
 
#define DEBUG_RC_CH 0
#define DEBUG 1

float dir_gain_pid[3] = {0.005f, 0.0f, 0.0f};
long integrale = 0;
long erreur_prec = 0;
int commande_prec = 0;
          
PPMReader ppmReader(RC_RX_PIN, 0, false);

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
  
  uint8_t ch_num = 0;
  int ppm_ch[8];
  while(ppmReader.get(ch_num) != 0){  
    if(DEBUG_RC_CH) {
      // Print out the servo values
      Serial.print(ppmReader.get(ch_num));
      Serial.print("  ");      
    }
    ppm_ch[ch_num] = ppmReader.get(ch_num);
    ch_num++;
  }
  if(DEBUG_RC_CH) {
    Serial.println("");
  }

  long gain_i32;
  gain_i32 = map(ppm_ch[RC_CH_IDX_VRA], 1000, 2000, 0, 1000);  
  dir_gain_pid[0] = (float)gain_i32 * 0.005;
  gain_i32 = map(ppm_ch[RC_CH_IDX_VRB], 1000, 2000, 0, 1000);  
  dir_gain_pid[2] = (float)gain_i32 * 0.001;

  if(DEBUG) {
    Serial.print("Gains: ");
    Serial.print(dir_gain_pid[0], 4);
    Serial.print(" ");
    Serial.print(dir_gain_pid[2], 5);
    Serial.println(".");
  }

  // Set direction PWM 
  // Centre RC command value around 0
  long setpoint = map(ppm_ch[RC_CH_IDX_DIR], DIR_CMD_LEFT, DIR_CMD_RIGHT,
                                             DIR_SEN_LEFT, DIR_SEN_RIGHT);
  int dir_cmd = pid(integrale,
                    erreur_prec,
                    commande_prec,
                    setpoint, 
                    dir_sen_val,
                    -255, 
                    255, 
                    dir_gain_pid);
        
  byte dir_l_pwm = constrain(map(dir_cmd, 0, 255, 0, 255), 0, 255);
  byte dir_r_pwm = constrain(map(dir_cmd, -255, 0, 255, 0), 0, 255);

  // Print out the pwm output values
  if(DEBUG) {
    Serial.print(integrale);
    Serial.print("  ");
    Serial.print(erreur_prec);
    Serial.print("  ");
    Serial.print(setpoint);
    Serial.print("  ");
    Serial.print(dir_cmd);
    Serial.print("  ");
    Serial.print(dir_l_pwm);
    Serial.print("  ");
    Serial.print(dir_r_pwm);
    Serial.print("  ");
    Serial.println("");
  }

  analogWrite(DIR_L_PWM, dir_l_pwm);
  analogWrite(DIR_R_PWM, dir_r_pwm);

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

  delay(50);
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
  // Check sign of integral term; if error and integral are 
  // not on the same side then reset the integral
  if(erreur * integrale < 0) {
    integrale = 0;
  }
  
  // Mise a jour de l'erreur        
  erreur_prec = erreur;
  commande_prec = commande_non_saturee;
  return commande_actuelle;        
}

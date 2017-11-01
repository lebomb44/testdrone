/* *****************************
 *  Test moteur drone rotation
 *  USING:
 *   ARDULIBS 1.1.0
 */
#include <Cmd.h>

/* *****************************
 *  Pin allocation
 * *****************************
 */
#define MOTOR_FL_pin 3
#define MOTOR_FR_pin 5
#define MOTOR_RL_pin 6
#define MOTOR_RR_pin 9


/* *****************************
 *  Global variables
 * *****************************
 */
uint8_t motorFLpwm = 0;
uint8_t motorFRpwm = 0;
uint8_t motorRLpwm = 0;
uint8_t motorRRpwm = 0;

/* *****************************
 *  Debug Macros
 * *****************************
 */

/* *****************************
 *  Command lines functions
 * *****************************
 */
void motorFLpwmPlus(int arg_cnt, char **args) { if(255 > motorFLpwm) { motorFLpwm++; } analogWrite(MOTOR_FL_pin, motorFLpwm); Serial.print("Motor FL: "); Serial.println(motorFLpwm); }
void motorFLpwmMinus(int arg_cnt, char **args) { if(0 < motorFLpwm) { motorFLpwm--; } analogWrite(MOTOR_FL_pin, motorFLpwm); Serial.print("Motor FL: "); Serial.println(motorFLpwm); }

void motorFRpwmPlus(int arg_cnt, char **args) { if(255 > motorFRpwm) { motorFRpwm++; } analogWrite(MOTOR_FR_pin, motorFRpwm); Serial.print("Motor FR: "); Serial.println(motorFRpwm); }
void motorFRpwmMinus(int arg_cnt, char **args) { if(0 < motorFRpwm) { motorFRpwm--; } analogWrite(MOTOR_FR_pin, motorFRpwm); Serial.print("Motor FR: "); Serial.println(motorFRpwm); }

void motorRLpwmPlus(int arg_cnt, char **args) { if(255 > motorRLpwm) { motorRLpwm++; } analogWrite(MOTOR_RL_pin, motorFLpwm); Serial.print("Motor RL: "); Serial.println(motorRLpwm); }
void motorRLpwmMinus(int arg_cnt, char **args) { if(0 < motorRLpwm) { motorRLpwm--; } analogWrite(MOTOR_RL_pin, motorFLpwm); Serial.print("Motor RL: "); Serial.println(motorRLpwm); }

void motorRRpwmPlus(int arg_cnt, char **args) { if(255 > motorRRpwm) { motorRRpwm++; } analogWrite(MOTOR_RR_pin, motorRRpwm); Serial.print("Motor RR: "); Serial.println(motorRRpwm); }
void motorRRpwmMinus(int arg_cnt, char **args) { if(0 < motorRRpwm) { motorRRpwm--; } analogWrite(MOTOR_RR_pin, motorRRpwm); Serial.print("Motor RR: "); Serial.println(motorRRpwm); }

void setup() {
  /* ****************************
   *  Pin configuration
   * ****************************
   */
  pinMode(MOTOR_FL_pin, OUTPUT); analogWrite(MOTOR_FL_pin, motorFLpwm);
  pinMode(MOTOR_FR_pin, OUTPUT); analogWrite(MOTOR_FR_pin, motorFRpwm);
  pinMode(MOTOR_RL_pin, OUTPUT); analogWrite(MOTOR_RL_pin, motorRLpwm);
  pinMode(MOTOR_RR_pin, OUTPUT); analogWrite(MOTOR_RR_pin, motorRRpwm);

  Serial.begin(115200);
  Serial.println("testdrone Starting...");

  cmdInit();
  cmdAdd("7", "Motor FL +", motorFLpwmPlus);
  cmdAdd("4", "Motor FL -", motorFLpwmMinus);
  cmdAdd("9", "Motor FR +", motorFRpwmPlus);
  cmdAdd("6", "Motor FR -", motorFRpwmMinus);
  cmdAdd("1", "Motor RL +", motorRLpwmPlus);
  cmdAdd("0", "Motor RL -", motorRLpwmMinus);
  cmdAdd("3", "Motor RR +", motorRRpwmPlus);
  cmdAdd(".", "Motor RR -", motorRRpwmMinus);
  cmdAdd("help", "List commands", cmdList);

  Serial.println("testdrone Init done");
}

void loop() {
  /* Poll for new command line */
  cmdPoll();
  delay(1);
}


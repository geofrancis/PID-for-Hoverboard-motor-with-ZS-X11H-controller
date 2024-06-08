#include "sbus.h"
#include <Arduino.h>


// used pins
#define SBUS_PIN 3  // D3

SBUS sbus;

#define SPD1 6
#define SPD2 7
#define DIR2 8
#define DIR1 9
#define HALL1 10
#define HALL2 11

int PWM1 = 0;
int PWM2 = 0;

int PWM_PIN_OUT = 3;  // 490Hz PWM Output
int POT_PIN_IN = A7;  // Analog potentiomer for speed


volatile float tic, tac, feedback = 0.00;                                                                                  // interrupt variables
bool dir = 0;                                                                                                              // direction, 0=clockwise, 1=counterclockwise
float now, prvTime, dt;                                                                                                    // time variables
float P = 0.00, I = 0.00, D = 0.00, error = 0.00, errDiff = 0.00, prevErr = 0.00, maxSum = 50, errSum = 0.00, pid = 0.00;  // PID variables

float kp = 0.15, ki = 0.7, kd = 0.001, target = 0, trgt_min = 12, trgt_max = 25, fb_min = 104, fb_max = 46;  // variables to be modified

void setup() {
  Serial.begin(115200);

  pinMode(SPD1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  digitalWrite(SPD1, LOW);
  digitalWrite(DIR1, HIGH);

  pinMode(SPD2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  digitalWrite(SPD2, LOW);
  digitalWrite(DIR2, HIGH);

  attachInterrupt(digitalPinToInterrupt(HALL1), intrupt, RISING);  // Attach Interrupt for motor Hall sensors
  attachInterrupt(digitalPinToInterrupt(HALL2), intrupt, RISING);  // Attach Interrupt for motor Hall sensors


  sbus.begin(SBUS_PIN, sbusNonBlocking);
  Serial.println("STARTUP");
}


void loop() {


  if (!sbus.waitFrame()) {

    Serial.println("Timeout!");

  } else {

    PWM1 = sbus.getChannel(0);
    PWM2 = sbus.getChannel(1);

    if (PWM1 > 1520) {

      now = millis();
      dt = (now - prvTime) / 1000.00;
      prvTime = now;
      target = map(PWM1, 1520, 2000, trgt_min, trgt_max);  // time between two loops
      pid = PID();
      digitalWrite(DIR1, HIGH);                                            // PID calculation
      analogWrite(SPD1, round(pid = constrain(pid, trgt_min, trgt_max)));  // output PWM PID - constrain speed for security
    }



    if (PWM1 < 1480) {

      now = millis();
      dt = (now - prvTime) / 1000.00;
      prvTime = now;
      target = map(PWM1, 1480, 1000, trgt_min, trgt_max);  // time between two loops
      pid = PID();
      digitalWrite(DIR1, LOW);                                             // PID calculation
      analogWrite(SPD1, round(pid = constrain(pid, trgt_min, trgt_max)));  // output PWM PID - constrain speed for security
    }




    if (PWM2 > 1520) {

      now = millis();
      dt = (now - prvTime) / 1000.00;
      prvTime = now;
      target = map(PWM2, 1520, 2000, trgt_min, trgt_max);  // time between two loops
      pid = PID();
      digitalWrite(DIR2, HIGH);                                            // PID calculation
      analogWrite(SPD2, round(pid = constrain(pid, trgt_min, trgt_max)));  // output PWM PID - constrain speed for security
    }



    if (PWM2 < 1480) {

      now = millis();
      dt = (now - prvTime) / 1000.00;
      prvTime = now;
      target = map(PWM2, 1480, 1000, trgt_min, trgt_max);  // time between two loops
      pid = PID();
      digitalWrite(DIR2, LOW);                                             // PID calculation
      analogWrite(SPD2, round(pid = constrain(pid, trgt_min, trgt_max)));  // output PWM PID - constrain speed for security
    }



    else {
      analogWrite(SPD1, 0);
      analogWrite(SPD2, 0);
    }

    if (sbus.signalLossActive())
      Serial.print("SIGNAL_LOSS ");
    analogWrite(SPD1, 0);
    analogWrite(SPD2, 0);

    if (sbus.failsafeActive())
      Serial.print("FAILSAFE");
    analogWrite(SPD1, 0);
    analogWrite(SPD2, 0);

    Serial.println();
  }
}




void intrupt() {
  tic = millis();
  feedback = tic - tac;
  tac = tic;                                                     // time between 2 Hall sensor detections
  feedback = map(feedback, fb_min, fb_max, trgt_min, trgt_max);  // convert feedback milliseconds to PWM value
}


float PID() {
  noInterrupts();
  error = target - feedback;
  interrupts();
  P = kp * error;
  I = ki * (errSum = errSum + (error * dt));
  errSum = constrain(errSum, -maxSum, maxSum);
  D = kd * (error - prevErr) / dt;
  prevErr = error;
  return P + I + D;
}




void Trace() {
  Serial.print(String() + "\n"
               + "  target: " + target
               + "  feedback: " + String(feedback, 3)
               + "  pid: " + String(pid, 3)
               + "  error: " + String(error, 3)
               + "  prevErr: " + String(prevErr, 3)
               + "  errSum: " + String(errSum, 3)
               + "  P: " + String(P, 3)
               + "  I: " + String(I, 3)
               + "  D: " + String(D, 3)
               + "  dir: " + dir
               + "  dt: " + String(dt, 3));
}
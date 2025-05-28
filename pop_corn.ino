#include <AccelStepper.h>
#include "max6675.h"

const int corn_push_step=2;
const int corn_push_dir=3;
const int corn_push_hmp=4;
const int rpwm=5;
const int lpwm=6;
const int heater=7;

int SO = 50;     // MISO
int CS = 8;     // Any digital pin
int CLK = 52;    // SCK

void all_axis_homing();
void corn_push();
void corn_pull();
void blower();
void heater_on();
void heater_off();

AccelStepper corn_push_stepper(AccelStepper::DRIVER, corn_push_step, corn_push_dir);

MAX6675 thermocouple(CLK, CS, SO);


void setup() {
  Serial.begin(9600);

pinMode(corn_push_step,OUTPUT);
pinMode(corn_push_dir,OUTPUT);
pinMode(corn_push_hmp,INPUT_PULLUP);
pinMode(rpwm, OUTPUT);
pinMode(lpwm, OUTPUT);
pinMode(heater, OUTPUT);

digitalWrite(heater,HIGH);

corn_push_stepper.setMaxSpeed(5000);
corn_push_stepper.setAcceleration(5000);

}

void loop() {
  /*
  all_axis_homing();
  delay(100);
  corn_push();
  delay(100);
  corn_pull();
  delay(100);
  blower();
  delay(100);
  heater_off();
  delay(2000);
  */
  digitalWrite(heater,LOW);
  analogWrite(rpwm, 250); // 50% speed
  digitalWrite(lpwm, LOW); // Ensure only one direction

  Serial.print("Temperature: ");
  Serial.print(thermocouple.readCelsius());
  Serial.println(" °C");
  delay(1000);

}

void all_axis_homing(){
  corn_pull();
  delay(100);
}

void corn_push(){
  if (digitalRead(corn_push_hmp) == HIGH )
  {
    delay(100); // Wait before start
    corn_push_stepper.moveTo(17250);
    while (corn_push_stepper.distanceToGo() != 0)
    {
      corn_push_stepper.run(); // Smooth ramped move
    }
  }
}

void corn_pull(){
  if (digitalRead(corn_push_hmp) == LOW)
  {
    corn_push_stepper.setSpeed(-3000); // Set speed for moving backward
    while (digitalRead(corn_push_hmp) == LOW)
    {
      corn_push_stepper.runSpeed(); // Continue moving backward
    }
    corn_push_stepper.stop();
    delay(500);
    corn_push_stepper.setCurrentPosition(0); // Reset position for puriCatcher after homing
  }
}

void blower(){
  analogWrite(rpwm, 80); // 50% speed
  digitalWrite(lpwm, LOW); // Ensure only one direction
  digitalWrite(heater, LOW); 

  delay(70000);           //40 seconds
  //analogWrite(rpwm, 0); // 0% speed

}

void heater_on(){
  digitalWrite(heater, LOW); 
}

void heater_off(){
  analogWrite(rpwm, 255); // 100% speed
  digitalWrite(lpwm, LOW); // Ensure only one direction
  digitalWrite(heater, HIGH); 
  delay(40000);
  analogWrite(rpwm, 0); // 0% speed
}
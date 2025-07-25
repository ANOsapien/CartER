#include <Servo.h>

Servo myServo;

const int electromagnetPin = 7;

void setup() {
  myServo.attach(12, 300, 2500); 

  pinMode(electromagnetPin, OUTPUT);
  digitalWrite(electromagnetPin, LOW); 
}

void loop() {
  digitalWrite(electromagnetPin, HIGH);

  for (int angle = 0; angle <= 200; angle++) {
    myServo.write(angle);
    delay(55);
  }

  digitalWrite(electromagnetPin, LOW);
  delay(5000);


  for (int angle = 180; angle >= 0; angle--) {
    myServo.write(angle);
    delay(55);
  }

  digitalWrite(electromagnetPin, LOW);
  delay(3000);
}

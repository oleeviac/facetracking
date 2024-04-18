#include <Servo.h>

const int numServos = 6;  // Number of servo pairs 

Servo servoX[numServos];
Servo servoY[numServos];

// Arrays to store servo pair indices
int selectedIndices[numServos];
int reset = 90;

void setupServoPairs() {
  for (int i = 0; i < numServos; i++) {
    servoX[i].attach(2 + i * 2);   // Attach servos starting from pin 2 with a step of 2
    servoY[i].attach(3 + i * 2);   // Attach servos starting from pin 3 with a step of 2
  }
}

void writePositionsToServos(int posX, int posX2, int posY, int posY2, int posY3) {
  // Update positions for all servo pairs
  for (int i = 0; i < numServos; i++) {
    int servoXpin = servoX[selectedIndices[i]].read();
    if (servoXpin < 7){
      servoX[selectedIndices[i]].write(posX);
    }
    else if (servoXpin > 7){
      servoX[selectedIndices[i]].write(posX2);
    }
    int servoYpin = servoY[selectedIndices[i]].read();
    if (servoYpin == 3 || servoYpin == 9 ){
      servoY[selectedIndices[i]].write(posY);
    }
    else if (servoYpin == 5 || servoYpin == 13 ){
      servoY[selectedIndices[i]].write(posY2);
    }
    else if (servoYpin == 7 || servoYpin == 13 ){
      servoY[selectedIndices[i]].write(posY3);
    }
  }
}

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  setupServoPairs(); // Initialize servo pairs
  Serial.println("Arduino Initialized");
}

void loop() {
  if (Serial.available()) {
    // Read the incoming string until newline character
    byte buffer[20];
    Serial.readBytes(buffer, 20);
    Serial.println("Received:");

    int rMotorInt;
    int lMotorInt;
    int firstyInt;
    int secyInt;
    int thirdyInt;

    memcpy(&rMotorInt, buffer, sizeof(int));
    memcpy(&lMotorInt, buffer + sizeof(int), sizeof(int));
    memcpy(&firstyInt, buffer + 2 * sizeof(int), sizeof(int));
    memcpy(&secyInt, buffer + 3 * sizeof(int), sizeof(int));
    memcpy(&thirdyInt, buffer + 4 * sizeof(int), sizeof(int));
    
        Serial.print("Parsed values: ");
        Serial.print(rMotorInt);
        Serial.print(", ");
        Serial.print(lMotorInt);
        Serial.print(", ");
        Serial.print(firstyInt);
        Serial.print(", ");
        Serial.print(secyInt);
        Serial.print(", ");
        Serial.println(thirdyInt);

        // Call function to control servos
        //writePositionsToServos(posX, posX2, posY, posY2, posY3);
      
    
  }
}

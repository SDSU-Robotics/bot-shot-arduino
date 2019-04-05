#include <Servo.h>
#define NUM_READINGS 10

Servo myservo;

int pos = 0;

int launchPotReadings[NUM_READINGS];
int readIndex = 0;
int total = 0;
int launchPotAverage = 0;

int launchPotPort = A0;

void setup()
{
  Serial.begin(115200);
  myservo.attach(11);

  for (int thisReading = 0; thisReading < NUM_READINGS; thisReading++) {
    launchPotReadings[thisReading] = 0;
  }
}

void loop()
{
  byte incomingByte[2];
  String msg;

  total = total - launchPotReadings[readIndex];
  launchPotReadings[readIndex] = analogRead(launchPotPort);
  total = total + launchPotReadings[readIndex];
  readIndex = readIndex++;      
  if (readIndex >= NUM_READINGS) 
  {
    readIndex = 0;
  }
  launchPotAverage = total / NUM_READINGS;   

  if (Serial.available() > 0)
  {
    incomingByte[1] = Serial.read() - 48;
    if (incomingByte[1] == 0)
    {
        msg = String(launchPotAverage, 2);
        Serial.println(msg);       
    }
    if (incomingByte[1] == 1)
    {
        incomingByte[2] = Serial.read();
        myservo.write(incomingByte[2] - 48);
    }
  }
  delay(20);
}

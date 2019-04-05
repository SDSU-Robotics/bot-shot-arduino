#include <Servo.h>

Servo myservo;

int pos = 0;

void setup()
{
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  myservo.attach(9);

}

void loop()
{
  byte incomingByte[2];
  String msg;

    if (Serial.available() > 0) 
    {
        incomingByte[1] = Serial.read() - 48;
        
        if ((incomingByte[1]) == 1)
        {
          incomingByte[2] = Serial.read();
          myservo.write(incomingByte[2]-48);              
        }
     }
    

}

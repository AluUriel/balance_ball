#include <Arduino.h>
#include <servo.h>

Servo ESC;

int vel = 1000; // Amplitud minima de pulso para tu ESC

void setup()
{

  ESC.attach(9); // Pin en el que funciona

  // Activar el ESC
  ESC.writeMicroseconds(1000); // 1000 = 1ms
  // Cambia el 1000 anterior por 2000 si
  // tu ESC se activa con un pulso de 2ms
  delay(5000); // Esperar 5 segundos para hacer la activacion

  Serial.begin(9600);
  Serial.setTimeout(10);
}

void loop()
{
  if (Serial.available() >= 1)
  {
    vel = Serial.parseInt(); // Leer un entero por serial
    if (vel != 0)
    {
      ESC.writeMicroseconds(vel); // Generar un pulso con el numero recibido
    }
  }
}


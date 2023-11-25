#include <Arduino.h>

// Define los pines para el sensor HC-SR04
const int triggerPin = 9;
const int echoPin = 10;
const int analogPin = A0;

// Tipo de sensor: "ULTRASONICO" o "INFRARROJO"
#define SENSOR_TYPE "ULTRASONICO"

// Prototipos de las funciones
float leerDistanciaInfrarrojo(int n);
float leerDistanciaUltrasonico();

void setup() {
  Serial.begin(115200);

  if (String(SENSOR_TYPE) == "ULTRASONICO") {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }
}

void loop() {
  float distancia;

  if (String(SENSOR_TYPE) == "ULTRASONICO") {
    distancia = leerDistanciaUltrasonico();
  } else if (String(SENSOR_TYPE) == "INFRARROJO") {
    distancia = leerDistanciaInfrarrojo(20);
  }

  if (distancia < 60) {

    // Envía la distancia al Serial Plotter
    Serial.print("0 ");
    Serial.print("60 ");
    Serial.println(distancia);
  }

  delay(10); // Espera un segundo para la siguiente medición
}

float leerDistanciaInfrarrojo(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma += analogRead(analogPin);
  }  
  float adc = suma / n;
  float distancia_cm = 9462.7 * pow(adc, -1.115) + 3.297;

  return constrain(distancia_cm, 5.0, 80.0);
}

float leerDistanciaUltrasonico() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

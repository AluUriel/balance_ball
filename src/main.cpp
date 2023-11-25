#include <Arduino.h>
#include <RunningAverage.h>

// Define los pines para el sensor HC-SR04
const int triggerPin = 9;
const int echoPin = 10;

#ifdef ARDUINO_AVR_UNO  // Define para Arduino UNO
const int analogPin = A0;
#elif defined(ESP32)  // Define para ESP32
const int analogPin = 36;
#else
#error "Plataforma no soportada"
#endif

// Tipo de sensor: "ULTRASONICO" o "INFRARROJO"
#define SENSOR_TYPE "INFRARROJO"

// Crea objetos RunningAverage
RunningAverage raInfrarrojo(10); // Ajusta el tamaño según tus necesidades
RunningAverage raUltrasonico(10);

float leerDistanciaInfrarrojo();
float leerDistanciaUltrasonico();

void setup() {
  Serial.begin(115200);

  if (String(SENSOR_TYPE) == "ULTRASONICO") {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }

  raInfrarrojo.clear(); // Inicializa el promedio móvil para infrarrojo
  raUltrasonico.clear(); // Inicializa el promedio móvil para ultrasonico
}

void loop() {
  float distancia;

  if (String(SENSOR_TYPE) == "ULTRASONICO") {
    distancia = leerDistanciaUltrasonico();
  } else if (String(SENSOR_TYPE) == "INFRARROJO") {
    distancia = leerDistanciaInfrarrojo();
  }

  Serial.print("0 ");
  Serial.print("60 ");
  Serial.println(distancia);

  delay(10); // Espera un tiempo para la siguiente medición
}

float leerDistanciaInfrarrojo() {
  float adc = analogRead(analogPin);
  float distancia_cm = 9462.7 * pow(adc, -1.115) + 3.297;
  raInfrarrojo.addValue(constrain(distancia_cm, 5.0, 80.0));
  return raInfrarrojo.getAverage();
}

float leerDistanciaUltrasonico() {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  float distancia = duration * 0.034 / 2;
  raUltrasonico.addValue(distancia);
  return raUltrasonico.getAverage();
}
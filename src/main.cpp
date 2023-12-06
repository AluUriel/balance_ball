#include <Arduino.h>
#include <RunningAverage.h>
#include "CommunicationHandler.h"
#include "HardwareSerialAdapter.h"
#include <PID_v1.h>
#include <vector>
#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager
#include "KalmanFilter.h"

KalmanFilter kalmanFilter(0.0054, 0.1873);


// Define the min and max output values for the ESC
#define MIN_ESC_OUTPUT 1100
#define MAX_ESC_OUTPUT 1600

const double MAX_PID_OUTPUT = 100;
const double MIN_PID_OUTPUT = 0;

// Define PID constants
double Kp = 2.0, Ki = 1, Kd = 0.2;
double sampleTimePID = 10; // Tiempo de muestreo en milisegundos

// Define Input, Output, and Setpoint variables
double Setpoint = 0, Input = 0, Output = 0;

// Create a PID object
PID bldcPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// Define los pines para el sensor HC-SR04
const int triggerPin = 9;
const int echoPin = 10;

#ifdef ARDUINO_AVR_UNO  // Define para Arduino UNO
const int analogPin = A0;
#elif defined(ESP32)  // Define para ESP32
const int analogPin = 36;
// ESC control pin
int escPin = 23; // Change as per your connection
#else
#error "Plataforma no soportada"
#endif

// Tipo de sensor: "ULTRASONICO" o "INFRARROJO"
#define SENSOR_TYPE "INFRARROJO"
ESC motorESC (escPin, 1000, 2000, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)

// Crea objetos RunningAverage
RunningAverage raInfrarrojo(10); // Ajusta el tamaño según tus necesidades
RunningAverage raUltrasonico(10);
RunningAverage raDistance(20);

double leerDistanciaInfrarrojo();
double leerDistanciaUltrasonico();
// Function that you want to run in a separate thread
void distanceThread(void *parameter);
void emit_data(void *parameter);

HardwareSerialAdapter serialAdapter;
CommunicationHandler serialHandler(serialAdapter);

std::vector<String> split(const String& str, char delimiter);

void setSetpoint(String payload) {
    Setpoint = payload.toDouble();

    Serial.print("Setpoint: ");
    Serial.println(Setpoint);
}

void setKalmanGains(String payload) {
    std::vector<String> gains = split(payload, ' ');

    if (gains.size() == 2) {
        kalmanFilter.setProcessNoise(gains[0].toFloat());
        kalmanFilter.setMeasurementNoise(gains[1].toFloat());

        Serial.print("Kalman process noise: ");
        Serial.print(gains[0]);
        Serial.print(" measurement noise: ");
        Serial.println(gains[1]);
    } else {
        Serial.println("Error: no se proporcionaron suficientes ganancias");
    }
}

std::vector<String> split(const String& str, char delimiter) {
    std::vector<String> tokens;
    int start = 0;
    int end = str.indexOf(delimiter);

    while (end != -1) {
        tokens.push_back(str.substring(start, end));
        start = end + 1;
        end = str.indexOf(delimiter, start);
    }

    tokens.push_back(str.substring(start)); // Agrega el último token
    return tokens;
}

void setGains(const String& payload) {
    std::vector<String> gains = split(payload, ' ');

    if (gains.size() == 3) {
        Kp = gains[0].toFloat();
        Ki = gains[1].toFloat();
        Kd = gains[2].toFloat();
        bldcPID.SetTunings(Kp, Ki, Kd);

        Serial.print("Kp: ");
        Serial.print(Kp);
        Serial.print(" Ki: ");
        Serial.print(Ki);
        Serial.print(" Kd: ");
        Serial.println(Kd);
    } else {
        Serial.println("Error: no se proporcionaron suficientes ganancias");
    }
}

bool pid_enabled = true;

void setup() {
    Serial.begin(115200);

    if (String(SENSOR_TYPE) == "ULTRASONICO") {
        pinMode(triggerPin, OUTPUT);
        pinMode(echoPin, INPUT);
    }

    raInfrarrojo.clear(); // Inicializa el promedio móvil para infrarrojo
    raUltrasonico.clear(); // Inicializa el promedio móvil para ultrasonico
    raDistance.clear(); // Inicializa el promedio móvil para la distancia

    serialHandler.attachCallback("s", setSetpoint);
    serialHandler.attachCallback("g", setGains);
    serialHandler.attachCallback("o", [](String payload) {
        // analogWrite(escPin, payload.toInt());
        motorESC.speed(payload.toInt());
        Serial.print("Output: ");
        Serial.println(payload);
    });
    serialHandler.attachCallback("e", [](String payload) {
        pid_enabled = payload.toInt();
        Serial.print("PID: ");
        Serial.println(pid_enabled);
    });
    serialHandler.attachCallback("k", setKalmanGains);



    // Initialize the ESC pin as an output
    pinMode(escPin, OUTPUT);
    motorESC.arm(); // Arm the ESC
    delay(5000); // Wait for the ESC to initialize
    // the following loop turns on the motor slowly, so get ready
    for (int i=0; i<350; i++){ // run speed from 840 to 1190
        motorESC.speed(1040-200+i); // motor starts up about half way through loop
        delay(10);
    }

    // Initialize the PID controller
    Setpoint = 50; // Set the desired speed
    Input = 0; // Read the distance from the sensor
    bldcPID.SetMode(AUTOMATIC);
    bldcPID.SetOutputLimits(MIN_PID_OUTPUT, MAX_PID_OUTPUT); // Depending on ESC and motor
    bldcPID.SetSampleTime(20); // Sample time in milliseconds

    // start thred to read IR sensor
    xTaskCreate(
        distanceThread,      /* Task function. */
        "distanceTask",/* String with name of task. */
        10000,          /* Stack size in words. */
        NULL,           /* Parameter passed as input of the task */
        1,              /* Priority of the task. */
        NULL);          /* Task handle. */

#if 1
    xTaskCreate(
        emit_data,      /* Task function. */
        "emitDataTask",/* String with name of task. */
        10000,          /* Stack size in words. */
        NULL,           /* Parameter passed as input of the task */
        1,              /* Priority of the task. */
        NULL);          /* Task handle. */
#endif

    serialHandler.start();
#if 0
    while (true)
    {
        // serialHandler.emit("distance", String(raDistance.getAverage()));
        // Serial.print("60 0 ");
        Serial.println(raDistance.getAverage());
        delay(100);
    }
#endif

    delay(1000); // Wait for the ESC to initialize
    
}

double output_motor = 0;
void loop() {
    Input = kalmanFilter.update(raDistance.getAverage()); // Read the distance from the sensor
    bldcPID.Compute(); // Compute the output

    // Convert the output to the ESC to a value between 0 and 255
    output_motor = map(Output, MIN_PID_OUTPUT, MAX_PID_OUTPUT, MAX_ESC_OUTPUT, MIN_ESC_OUTPUT);

    // Write the output to the ESC
    if (pid_enabled){
        motorESC.speed(output_motor);
    }

    delay(sampleTimePID); // Espera un tiempo para la siguiente medición
}

// Function that you want to run in a separate thread
void distanceThread(void *parameter){
    // Your function code here
    double distancia;
    const int sampleTime = 1; // Tiempo de muestreo en milisegundos

    while(1){

        if (String(SENSOR_TYPE) == "ULTRASONICO") {
            distancia = leerDistanciaUltrasonico();
        } else if (String(SENSOR_TYPE) == "INFRARROJO") {
            distancia = leerDistanciaInfrarrojo();
        }

        raDistance.addValue(distancia);

        vTaskDelay(pdMS_TO_TICKS(sampleTime));
    }
}

double leerDistanciaInfrarrojo() {
    double adc = analogRead(analogPin);
    //   double distance = 27898/adc;
    double distance = 129576*pow(adc,-1.2);
    // double distance = adc;
    raInfrarrojo.addValue(distance);
    // float distancia_cm = 9462.7 * pow(adc, -1.115) + 3.297;
    // raInfrarrojo.addValue(constrain(distancia_cm, 5.0, 80.0));
    return raInfrarrojo.getAverage();
}

double leerDistanciaUltrasonico() {
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

void emit_data(void *parameter){
    while (true)
    {
        Serial.print("Input: ");
        Serial.print(Input);
        Serial.print(",Pid_Output: ");
        Serial.print(Output);
        Serial.print(",Motor_Output: ");
        Serial.print(output_motor);
        Serial.print(",Setpoint: ");
        Serial.print(Setpoint);
        Serial.print(",kp_error: ");
        Serial.print(bldcPID.kp_error);
        Serial.print(",ki_error: ");
        Serial.print(bldcPID.ki_error);
        Serial.print(",kd_error: ");
        Serial.println(bldcPID.kd_error);

        vTaskDelay(pdMS_TO_TICKS(100));
    }    
}
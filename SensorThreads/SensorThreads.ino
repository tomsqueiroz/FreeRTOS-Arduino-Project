#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <math.h>

#define SEND_GANTT  1


//Pinos/Variáveis para sensor de Temperatura
const int pinoSensorTemperatura = A0;
const float temperaturaIncendio = 45.0;
float temperaturaLida;


//Pinos/Variáveis para sensor de Chamas
const int pinoSensorChamas = 7;
const int FOGO_DETECTADO = 0;
int valorDigitalSensorChamas = 1;


//Pinos/Variáveis para sensor PIR
const int ALARME = 1;
const int pinoSensorPIR = 6;
const int PRESENCA_DETECTADA = 1;
int presenca;

//Pinos/Variáveis para sensor Gás MQ9
const int pinoSensorGas = A1;
const int valorVazamento = 80;
int valorAnalogicoSensorGas;

//Pinos/Variáveis para módulo Buzzer
const int pinoBuzzer = 9;
const int tomBuzzer = 261;

//Variaveis simulacao dos motores
const int tempoMedioMotor = 10;
long erroAssociado;

// Definindo as tarefas
TaskHandle_t readTemperatureSensorTaskH;
TaskHandle_t temperatureActuatorTaskH;
TaskHandle_t readGasSensorTaskH;
TaskHandle_t gasActuatorTaskH;
TaskHandle_t readFlameSensorTaskH;
TaskHandle_t flameActuatorTaskH;
TaskHandle_t readPIRSensorTaskH;
TaskHandle_t PIRActuatorTaskH;

// Mutex
SemaphoreHandle_t temperaturaMutex;
SemaphoreHandle_t gasMutex;
SemaphoreHandle_t PIRMutex;
SemaphoreHandle_t flameMutex;
SemaphoreHandle_t SerialMutex;

// Queues
QueueHandle_t temperatureQueue;

void setup() {
  Serial.begin(9600);

  //SetUp Pino Analógico Sensor Temperatura
  pinMode(pinoSensorTemperatura, INPUT);
  
  //SetUp Pino Digital Sensor Chamas
  pinMode(pinoSensorChamas, INPUT);

  //SetUp Pino Digital Sensor PIR
  pinMode(pinoSensorPIR, INPUT);

  //SetUp Pino Analógico Sensor Gás
  pinMode(pinoSensorGas, INPUT);

  //SetUp Pino Digital Buzzer SAÍDA!!
  pinMode(pinoBuzzer, OUTPUT);

  //SetUp SeedRandomica com valor de ruido da porta analogica
  randomSeed(analogRead(A2)); 

  // Mutex
  temperaturaMutex = xSemaphoreCreateMutex();
  gasMutex = xSemaphoreCreateMutex();
  flameMutex = xSemaphoreCreateMutex();
  PIRMutex = xSemaphoreCreateMutex();
  SerialMutex = xSemaphoreCreateMutex();

  // Tarefas
  xTaskCreate(readTemperatureSensor, "readTemperatureSensor", 128, NULL, 1, &readTemperatureSensorTaskH);
  xTaskCreate(temperatureActuator, "temperatureActuator", 128, NULL, 1, &temperatureActuatorTaskH);
  
  //xTaskCreate(readGasSensor, "readGasSensor", 128, NULL, 2, &readGasSensorTaskH);
  //xTaskCreate(GasActuator, "gasActuator", 128, NULL, 2, &gasActuatorTaskH);
  
  //xTaskCreate(readFlameSensor, "readFlameSensor", 128, NULL, 3, &readFlameSensorTaskH);
  //xTaskCreate(FlameActuator, "flameActuator", 128, NULL, 3, &flameActuatorTaskH);
  
  //xTaskCreate(readPIRSensor, "readPIRSensor", 128, NULL, 4, &readPIRSensorTaskH);
  //xTaskCreate(PIRActuator, "PIRActuator", 128, NULL, 4, &PIRActuatorTaskH);
  
}

void readTemperatureSensor(){
  temperatureQueue = xQueueCreate( 5, sizeof(float) );
 
  while(true){
    temperaturaLida = (float(analogRead(pinoSensorTemperatura))*5/(1023))/0.01;
    xQueueSend( temperatureQueue, (void*)&temperaturaLida, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  vTaskDelete(NULL);
}

void readFlameSensor(){
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(5000));
    if(xSemaphoreTake(flameMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        valorDigitalSensorChamas = digitalRead(pinoSensorChamas);
        xSemaphoreGive(flameMutex);                            //Libera Mutex
     }
     if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print("SensorChamas: ");
        Serial.print(valorDigitalSensorChamas);
        Serial.print("\t");
        Serial.print("Tempo de inicio: ");
        Serial.print(startTime);
        Serial.print("\t");
        Serial.print("Tempo de fim: ");
        Serial.println(millis());
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    } 
  }
  vTaskDelete(NULL);
}

void readPIRSensor(){
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(5000));
    if(xSemaphoreTake(PIRMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        presenca = digitalRead(pinoSensorPIR);
        xSemaphoreGive(PIRMutex);                            //Libera Mutex
     }
     if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print("SensorPIR: ");
        Serial.print(presenca);
        Serial.print("\t");
        Serial.print("Tempo de inicio: ");
        Serial.print(startTime);
        Serial.print("\t");
        Serial.print("Tempo de fim: ");
        Serial.println(millis());
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    } 
  }
  vTaskDelete(NULL); 
}

void readGasSensor(){
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(5000));
    if(xSemaphoreTake(gasMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        valorAnalogicoSensorGas = analogRead(pinoSensorGas);
        xSemaphoreGive(gasMutex);                            //Libera Mutex
     }
     if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print("SensorGas: ");
        Serial.print(valorAnalogicoSensorGas);
        Serial.print("\t");
        Serial.print("Tempo de inicio: ");
        Serial.print(startTime);
        Serial.print("\t");
        Serial.print("Tempo de fim: ");
        Serial.println(millis());
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    } 
  }
  vTaskDelete(NULL);
}

void setBuzzerOn(){
  tone(pinoBuzzer, tomBuzzer);
}

void setBuzzerOff(){
  noTone(pinoBuzzer);
  digitalWrite(pinoBuzzer, LOW);
  delay(2000);
}

int getMotorProcessSimulationTime(){
  erroAssociado = random(0,3);
  return tempoMedioMotor + erroAssociado;
}
bool isFlame(){
  if(valorDigitalSensorChamas == FOGO_DETECTADO){ return true; }
  else{ return false; }
}

bool isPresence(){
  if(presenca == PRESENCA_DETECTADA){ return true; }
  else{ return false; }
}

bool isGas(){
  if(valorAnalogicoSensorGas >= valorVazamento){ return true; }
  else{ return false; }
}

void temperatureActuator(){
  float temperatura;
  while(true){
    unsigned int startTime = millis();
    if(temperatureQueue != 0){
        if(xQueueReceive( temperatureQueue, (void*) &temperatura, pdMS_TO_TICKS(1000))){

            if(temperatura > temperaturaIncendio){ //ATUA 
            }
            else{ //nao atua 
            }
            if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
                Serial.print("Temperatura: ");
                Serial.print(temperatura);
                Serial.print("\t");
                Serial.print("Tempo de inicio: ");
                Serial.print(startTime);
                Serial.print("\t");
                Serial.print("Tempo de fim: ");
                Serial.println(millis());
                xSemaphoreGive(SerialMutex);                            //Libera Mutex
            }
          } 
        }
     }
  vTaskDelete(NULL); 
}

void FlameActuator(){
  bool flameBool;
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(1000));
    if(xSemaphoreTake(flameMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        flameBool = isFlame();
        xSemaphoreGive(flameMutex);                            //Libera Mutex
    }
    if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print("Flame Bool: ");
        Serial.print(flameBool);
        Serial.print("\t");
        Serial.print("Tempo de inicio: ");
        Serial.print(startTime);
        Serial.print("\t");
        Serial.print("Tempo de fim: ");
        Serial.println(millis());
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    }
    if (flameBool){
    
    setBuzzerOn();

    //abrir portas e janelas
    }
  }
  vTaskDelete(NULL); 
  
}

void PIRActuator(){
  bool presenceBool;
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(1000));
    if(xSemaphoreTake(PIRMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        presenceBool = isPresence();
        xSemaphoreGive(PIRMutex);                            //Libera Mutex
    }
    if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print("PIR Bool: ");
        Serial.print(presenceBool);
        Serial.print("\t");
        Serial.print("Tempo de inicio: ");
        Serial.print(startTime);
        Serial.print("\t");
        Serial.print("Tempo de fim: ");
        Serial.println(millis());
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    }
    if (presenceBool && ALARME){
    
      setBuzzerOn();
      motorActuator();

    //fechar portas e janelas
    }
  }
  vTaskDelete(NULL); 
}

void GasActuator(){
  bool gasBool;
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(1000));
    if(xSemaphoreTake(gasMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        gasBool = isGas();
        xSemaphoreGive(gasMutex);                            //Libera Mutex
    }
    if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print("Gas Bool: ");
        Serial.print(gasBool);
        Serial.print("\t");
        Serial.print("Tempo de inicio: ");
        Serial.print(startTime);
        Serial.print("\t");
        Serial.print("Tempo de fim: ");
        Serial.println(millis());
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    }
    if (gasBool && ALARME){
    
      setBuzzerOn();
      //motorActuator();

    //abrir portas e janelas
    }
  }
  vTaskDelete(NULL);
}

void motorActuator(){
  int tempoMotor = getMotorProcessSimulationTime();
  Serial.print("Tempo Motor: ");
  Serial.println(tempoMotor);

  delay(tempoMotor*1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  //readTemperatureSensor();
  //readFlameSensor();
  //readPIRSensor();
  //readGasSensor();
  
  //temperatureActuator();
  //delay(2000);
  //FlameActuator();
  //delay(2000);
  //PIRActuator();
  //delay(2000);
  //GasActuator();
  //delay(2000);
  //setBuzzerOff();
  
}

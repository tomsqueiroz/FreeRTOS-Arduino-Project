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
TaskHandle_t readSensorsTaskH;
TaskHandle_t gasActuatorTaskH;
TaskHandle_t temperatureActuatorTaskH;

TaskHandle_t flameActuatorTaskH;
TaskHandle_t readPIRSensorTaskH;
TaskHandle_t PIRActuatorTaskH;

// Mutex
SemaphoreHandle_t SerialMutex;

// Queues
QueueHandle_t temperatureQueue;
QueueHandle_t gasQueue;
QueueHandle_t flameQueue;

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
  SerialMutex = xSemaphoreCreateMutex();

  // Tarefas
  xTaskCreate(readSensors, "readSensors", 128, NULL, 1, &readSensorsTaskH);
  xTaskCreate(temperatureActuator, "temperatureActuator", 128, NULL, 4, &temperatureActuatorTaskH);
  xTaskCreate(GasActuator, "gasActuator", 128, NULL, 3, &gasActuatorTaskH);
  xTaskCreate(FlameActuator, "flameActuator", 128, NULL, 4, &flameActuatorTaskH);
  

  //xTaskCreate(readPIRSensor, "readPIRSensor", 128, NULL, 4, &readPIRSensorTaskH);
  //xTaskCreate(PIRActuator, "PIRActuator", 128, NULL, 4, &PIRActuatorTaskH);
  
  vTaskStartScheduler();
}

void readSensors(){
  temperatureQueue = xQueueCreate( 5, sizeof(float) );
  flameQueue = xQueueCreate( 5, sizeof(float) );
  gasQueue = xQueueCreate( 5, sizeof(float) );

  while(true){
    temperaturaLida = (float(analogRead(pinoSensorTemperatura))*5/(1023))/0.01;
    valorDigitalSensorChamas = digitalRead(pinoSensorChamas);
    valorAnalogicoSensorGas = analogRead(pinoSensorGas);
    xQueueSend( gasQueue, (void*)&valorAnalogicoSensorGas, pdMS_TO_TICKS(1000));
    xQueueSend( flameQueue, (void*)&valorDigitalSensorChamas, pdMS_TO_TICKS(1000));
    xQueueSend( temperatureQueue, (void*)&temperaturaLida, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(NULL);
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
            if(xSemaphoreTake(SerialMutex, ( TickType_t ) 5) == pdTRUE) {  //Solicita Mutex
                Serial.print("Temperatura: ");
                Serial.print(temperatura);
                Serial.print("\t\t\t");
                Serial.print("Tempo de inicio: ");
                Serial.print(startTime);
                Serial.print("\t");
                Serial.print("Tempo de fim: ");
                Serial.println(millis());
                xSemaphoreGive(SerialMutex);                            //Libera Mutex
            }
          } 
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
     }
  vTaskDelete(NULL); 
}

void FlameActuator(){
  int flame;
  while(true){
    unsigned int startTime = millis();
    if(flameQueue != 0){
        if(xQueueReceive( flameQueue, (void*) &flame, pdMS_TO_TICKS(1000))){

            if(flame == FOGO_DETECTADO){ //ATUA 
            }
            else{ //nao atua 
            }
            if(xSemaphoreTake(SerialMutex, ( TickType_t ) 5) == pdTRUE) {  //Solicita Mutex
                Serial.print("Flame: ");
                Serial.print(flame);
                Serial.print("\t\t\t");
                Serial.print("Tempo de inicio: ");
                Serial.print(startTime);
                Serial.print("\t");
                Serial.print("Tempo de fim: ");
                Serial.println(millis());
                xSemaphoreGive(SerialMutex);                            //Libera Mutex
            }
          } 
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
     }
  vTaskDelete(NULL);
}

void GasActuator(){
  int gas;
  while(true){
    unsigned int startTime = millis();
    if(gasQueue != 0){
        if(xQueueReceive( gasQueue, (void*) &gas, pdMS_TO_TICKS(1000))){

            if(gas > valorVazamento){ //ATUA 
            }
            else{ //nao atua 
            }
            if(xSemaphoreTake(SerialMutex, ( TickType_t ) 5) == pdTRUE) {  //Solicita Mutex
                Serial.print("Gas: ");
                Serial.print(gas);
                Serial.print("\t\t\t");
                Serial.print("Tempo de inicio: ");
                Serial.print(startTime);
                Serial.print("\t");
                Serial.print("Tempo de fim: ");
                Serial.println(millis());
                xSemaphoreGive(SerialMutex);                            //Libera Mutex
            }
          } 
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
     }
  vTaskDelete(NULL);
}


/*
void readPIRSensor(){
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(5000));
    presenca = digitalRead(pinoSensorPIR);
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
*/
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


bool isPresence(){
  if(presenca == PRESENCA_DETECTADA){ return true; }
  else{ return false; }
}



/*
void PIRActuator(){
  bool presenceBool;
  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(1000));
    if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        presenceBool = isPresence();
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
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
*/
void motorActuator(){
  int tempoMotor = getMotorProcessSimulationTime();
  Serial.print("Tempo Motor: ");
  Serial.println(tempoMotor);

  delay(tempoMotor*1000);
}

void loop() {
}

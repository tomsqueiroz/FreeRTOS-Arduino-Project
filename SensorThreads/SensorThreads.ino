#include "Arduino_FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <math.h>


//Pino para LEDS
const int pinoLedPortaAberta = 13;
const int pinoLedPortaFechada = 12;
const int pinoLedAgua = 11;
int valorPinoLed = 0;

//Constantes para abrir e fechar portas
const int ABRE_PORTA = 1;
const int FECHA_PORTA = 2;


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
int valorDigitalSensorPIR;

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
TaskHandle_t logicControllerTaskH;
TaskHandle_t readPIRSensorTaskH;
TaskHandle_t PIRActuatorTaskH;
TaskHandle_t motorActuatorTaskH;

// Mutex
SemaphoreHandle_t SerialMutex;

// Queues
QueueHandle_t temperatureQueue;
QueueHandle_t gasQueue;
QueueHandle_t flameQueue;
QueueHandle_t pirQueue;

void sendGantt(const char *name, unsigned int stime, unsigned int etime) {
    if(xSemaphoreTake(SerialMutex, portMAX_DELAY) == pdTRUE) {  //Solicita Mutex
        Serial.print(name);
        Serial.print(": ");
        Serial.print(stime);
        Serial.print(", ");
        Serial.println(etime);
        xSemaphoreGive(SerialMutex);                            //Libera Mutex
    }
}


void setup() {
  Serial.begin(9600);

  //Pino para LEDS
  pinMode(pinoLedPortaAberta, OUTPUT);
  pinMode(pinoLedPortaFechada, OUTPUT);
  pinMode(pinoLedAgua, OUTPUT);

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
  xTaskCreate(readSensors, "readSensors", 128, NULL, 2, &readSensorsTaskH);
  xTaskCreate(logicController, "logicController", 128, NULL, 1, &logicControllerTaskH);
  //xTaskCreate(readPIRSensor, "readPIRSensor", 128, NULL, 4, &readPIRSensorTaskH);
  //xTaskCreate(PIRActuator, "PIRActuator", 128, NULL, 4, &PIRActuatorTaskH);
  
  vTaskStartScheduler();
}

void readSensors(){
  temperatureQueue = xQueueCreate( 1, sizeof(float) );
  flameQueue = xQueueCreate( 1, sizeof(float) );
  gasQueue = xQueueCreate( 1, sizeof(float) );
  pirQueue = xQueueCreate( 1, sizeof(float) );

  while(true){
    unsigned int startTime = millis();
    temperaturaLida = (float(analogRead(pinoSensorTemperatura))*5/(1023))/0.01;
    valorDigitalSensorChamas = digitalRead(pinoSensorChamas);
    valorAnalogicoSensorGas = analogRead(pinoSensorGas);
    valorDigitalSensorPIR = digitalRead(pinoSensorPIR);
    xQueueSend( gasQueue, (void*)&valorAnalogicoSensorGas, pdMS_TO_TICKS(100));
    xQueueSend( flameQueue, (void*)&valorDigitalSensorChamas, pdMS_TO_TICKS(100));
    xQueueSend( temperatureQueue, (void*)&temperaturaLida, pdMS_TO_TICKS(100));
    xQueueSend( pirQueue, (void*)&valorDigitalSensorPIR, pdMS_TO_TICKS(100));
    sendGantt("SensorTask", startTime, millis());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(NULL);
}

void logicController(){
  float temperatureValue;
  int flameValue;
  int gasValue;
  int pirValue;

  while(true){
    unsigned int startTime = millis();
    vTaskDelay(pdMS_TO_TICKS(300));
    if(temperatureQueue != 0){if(xQueueReceive( temperatureQueue, (void*) &temperatureValue, pdMS_TO_TICKS(100))){/*sendGantt("Temperature", startTime, millis());*/}}
    if(flameQueue != 0){if(xQueueReceive( flameQueue, (void*) &flameValue, pdMS_TO_TICKS(100))){/*sendGantt("Flame", startTime, millis());*/}}
    if(gasQueue != 0){if(xQueueReceive( gasQueue, (void*) &gasValue, pdMS_TO_TICKS(100))){/*sendGantt("Gas", startTime, millis());*/}}
    if(pirQueue != 0){if(xQueueReceive( pirQueue, (void*) &pirValue, pdMS_TO_TICKS(100))){/*sendGantt("PIR", startTime, millis());*/}}
    /*int state = eTaskGetState(motorActuatorTaskH);
    sendGantt("Estado " + state, startTime, millis());*/
       if((isFlame(flameValue) || isGas(gasValue) || isTemperatureHigh(temperatureValue)) && valorPinoLed == 0){  
         
                //ABRE JANELAS E Portas
                xTaskCreate(motorActuator, "motorActuator", 128, (void*) ABRE_PORTA, 3, &motorActuatorTaskH);
                //INDICA ACIONAMENTO DE DIFUSOR DE AGUA DO SENSOR
                valorPinoLed = 1;
                digitalWrite(pinoLedAgua, valorPinoLed);

            }
        else if(isPresence(pirValue) && ALARME){
          //FECHA PORTAS E ACIONA ALARME, PRENDE O LADRAO
          xTaskCreate(motorActuator, "motorActuator", 128, (void*) FECHA_PORTA, 3, &motorActuatorTaskH);
        }
      //make logic comparisons!
      sendGantt("LogicTask", startTime, millis());
      vTaskDelay(pdMS_TO_TICKS(2000));
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
//Flame detection logic
bool isFlame(int valorChamas){
  if(valorChamas == FOGO_DETECTADO){ return true; }
  else{ return false; }
}

//Gas detection logic 
bool isGas(int valorGas){
  if(valorGas >= valorVazamento){ return true; }
  else{ return false; }
}

//High Temperature detection Logic
bool isTemperatureHigh(float temperatureValue){
  if(temperaturaLida > temperaturaIncendio){ return true; }
  else{ return false; }
}

//Presence detection logic
bool isPresence(int presenca){
  if(presenca == PRESENCA_DETECTADA){ return true; }
  else{ return false; }
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

void motorActuator(void *p){
  int modoPorta = (int*) p;
  int tempoMotor = getMotorProcessSimulationTime();
  unsigned int startTime = millis();
  setBuzzerOn();

  if(modoPorta == ABRE_PORTA){
    digitalWrite(pinoLedPortaFechada, LOW);
    digitalWrite(pinoLedPortaAberta, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(pinoLedPortaAberta, LOW);
    vTaskDelay(pdMS_TO_TICKS(tempoMotor*1000));
    digitalWrite(pinoLedPortaAberta, HIGH);
    sendGantt("MotorTask", startTime, millis());
  } else if(modoPorta == FECHA_PORTA){
    digitalWrite(pinoLedPortaAberta, LOW);
    digitalWrite(pinoLedPortaFechada, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(pinoLedPortaFechada, LOW);
    vTaskDelay(pdMS_TO_TICKS(tempoMotor*1000));
    digitalWrite(pinoLedPortaFechada, HIGH);
    sendGantt("MotorTask", startTime, millis());
  }

  vTaskDelete(NULL);
  
}

void loop() {
}

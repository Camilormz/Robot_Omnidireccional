// Código para robot omnidireccional
// Grupo 6 ; EI2001-5 ; Otoño 2018 ; FCFM ; U. de Chile
// Taller de proyecto de robótica y mecatrónica

#include <PID_v1.h> // Librería de PID

// Declaración de variables y pines
// --------------------------------------------------
// Matriz de pines de control para los motores (La fila determina el numero de motor y la columna el de control), los pines para el control #1 son 30, 33, 35 para los motores 1, 2 y 3
// respectivamente en el diagrama y los pines para el control #2 son 32, 35 y 34 para los motores 1, 2 y 3 respeticamente el diagrama.
const byte MotorCtrl[3][2] = {{30,31},{33,32},{35,34}};
// Lista de pines PWM (Habilitadores) para los motores segun su posición (pines 2, 3 y 4 para los motores 1, 2 y 3 respectivamente en el diagrama)
const byte MotorPWM[3] = {2,3,4};
// Lista de pines interruptores para encoders segun su posición (pines 49, 51 y 53 para encoders 1, 2 y 3 respectivamente en el diagrama)
const byte Enc[3] = {49,51,53};
// Lista de interruptores para encoders segun su posición
const byte Int[3] = {digitalPinToInterrupt(Enc[0]),digitalPinToInterrupt(Enc[1]),digitalPinToInterrupt(Enc[2])};

// Lista de contadores para las interrupciones de los encoders segun su posicion en ella
volatile byte Contador[3] = {0,0,0};

// Variables de medición de tiempo para encoders
volatile unsigned long Tiempo_Ini[3] = {0,0,0}; // Lista que almacena los tiempos en que inicia una iteración de medida de velocidad en cada encoder segun su posición en la lista
volatile unsigned long Tiempo_Fin[3];           // Lista que almacena los tiempos en que termina una iteración de medida de velocidad en cada encoder segun su posición en la lista
volatile float RapidezAngMed[3];                // Lista de rapideces angulares medidas por cada encoder segun su posición en la lista en perforaciones/ms
volatile float RPMMed[3];                       // Lista de rapideces angulares medidas por cada encoder segun su posicion en la lista en RPM
double RapidezInPID[3] = {0,0,0};               // Lista de rapideces angulares medidas ajustadas al rango PWM requerido por los motores

// Constantes de funcionamiento del robot
const float MaxRPM = 60;  // Rapidez angular máxima que los motores son capaces de entregar en RPM

// Constantes y variables relacionadas a los encoders e interrupciones
const byte Nverificador = 5;              // Establece la resolución de medida de la velocidad angular
const byte TotalPerf = 40;                // Número total de perforaciones en el disco (Perforaciones en un ángulo 2pi)
const long TiempoParaInterrupcion = 5000; // Tiempo en el cual interrupciones sucesivas de menor tiempo se omitiran en el debouncing en microsegundos
volatile long UltimoT[3] = {0,0,0};       // Tiempos de control para debouncing para las interrupciones de los encoders segun su posición en la lista

// Variables de estado para el robot
float RapidezAngDeseada[3] = {0,0,0}; // Lista que almacena a que rapidez angular que desea que funcione el motor corrspondiente a la posición en la lista en RPM
double RapidezSetpointPID[3];         // Lista que almacena los setpoints de rapidez angular para cada motor en el rango PWM correspondiente
byte Avance[3] = {0,0,0};             // Variable que almacena el estado de mov. del motor en la pos. correspondiente (0 a retroceso, 1 a detención (libre o frenada) y 2 a avance)
bool Frenado[3] = {true,true,true};   // Variable que almacena el estado de frenado del robot para el motor de la posicion correspondietne en la lista

// Declaraciones y variables para control PID
double Kp = 1;                // Constante proporcional de los PID
double Ki = 1;                // Constante integral de los PID
double Kd = 1;                // Constante derivativa de los PID
double RapidezOutPID[3];      // Lista de rapideces angulares de salida post control PID ajustadas al rango PWM requerido por los motores
int TiempoDeMuestreo = 200;   // Tiempo de computo del PID en ms
PID PID1(&RapidezInPID[0],&RapidezOutPID[0],&RapidezSetpointPID[0],Kp,Ki,Kd,DIRECT,P_ON_M);
PID PID2(&RapidezInPID[1],&RapidezOutPID[1],&RapidezSetpointPID[1],Kp,Ki,Kd,DIRECT,P_ON_M);
PID PID3(&RapidezInPID[2],&RapidezOutPID[2],&RapidezSetpointPID[2],Kp,Ki,Kd,DIRECT,P_ON_M);

// --------------------------------------------------
// Creación de funciones útiles para encoders e interrupciones
// --------------------------------------------------
// Debounced, función destinada a ejecutarse en una interrupción, retorna true o false dependiendo si esta misma no se ha ejecutado en un tiempo menor a UltT.
// El objetivo es limpiar la lectura de interrupciones erráticas que ocurren en un tiempo muy pequeño.
bool Debounced(volatile long UltT){
  long Diferencia = micros() - UltT;
  return Diferencia >= TiempoParaInterrupcion;
}
// --------------------------------------------------
// RutinaEncoder, función que ejecuta la accioón que debe realizar cada encoder tomando como parámetro el número de este, cada ISR llama a esta función con el respectivo número del
// encoder que ejecutó la interrupción.
void RutinaEncoder(byte NEncoder){
  byte Num = NEncoder - 1;
  if (Debounced(UltimoT[Num])){
    Contador[Num]++;
    if (Contador[Num] == Nverificador){
      Tiempo_Fin[Num] = millis();
      RapidezAngMed[Num] = Contador[Num]/(Tiempo_Fin[Num]-Tiempo_Ini[Num]);
      RPMMed[Num] = RapidezAngMed[Num]*60000/TotalPerf;
      RapidezInPID[Num] = RPMMed[Num]*255/MaxRPM;
      Tiempo_Ini[Num] = millis();
      Contador[Num] = 0;
    }
    UltimoT[Num] = micros();
  }
  switch (NEncoder){
    case 1: PID1.Compute(); break;
    case 2: PID2.Compute(); break;
    case 3: PID3.Compute(); break;
  }
  if (Avance[Num] != 1) {
    analogWrite(MotorPWM[Num],byte(RapidezOutPID[Num]));
  }
}
// --------------------------------------------------
// Funciones de movimiento de los motores
// --------------------------------------------------
// Liberar, funcion que dado el numero de un motor ejecuta acciones necesarias para que no entregue torque y gire libremente
void Liberar(byte NMotor){
  byte indice = NMotor - 1;
  switch (NMotor){
    case 1: PID1.SetMode(MANUAL); break;
    case 2: PID2.SetMode(MANUAL); break;
    case 3: PID3.SetMode(MANUAL); break;
  }
  digitalWrite(MotorCtrl[indice][0],LOW);
  digitalWrite(MotorCtrl[indice][1],LOW);
  RapidezAngDeseada[indice] = 0;
  RapidezSetpointPID[indice] = 0;
  Avance[indice] = 1;
  Frenado[indice] = false;
}
// --------------------------------------------------
// Frenar, funcion que dado el numero de un motor ejecuta acciones necesarias para frenarlo
void Frenar(byte NMotor){
  byte indice = NMotor - 1;
  switch (NMotor){
    case 1: PID1.SetMode(MANUAL); break;
    case 2: PID2.SetMode(MANUAL); break;
    case 3: PID3.SetMode(MANUAL); break;
  }
  digitalWrite(MotorCtrl[indice][0],HIGH);
  digitalWrite(MotorCtrl[indice][1],HIGH);
  RapidezAngDeseada[indice] = 0;
  RapidezSetpointPID[indice] = 0;
  Avance[indice] = 1;
  Frenado[indice] = true;
}
// --------------------------------------------------
// Avanzar, funcion que dado el numero de un motor y una rapidez angular en RPM, ejecuta las acciones necesarias para que avance con las RPM indicadas
void Avanzar(byte NMotor,float RPM){
  byte indice = NMotor - 1;
  switch (NMotor){
    case 1: PID1.SetMode(AUTOMATIC); break;
    case 2: PID2.SetMode(AUTOMATIC); break;
    case 3: PID3.SetMode(AUTOMATIC); break;
  }
  RapidezAngDeseada[indice] = RPM;
  RapidezSetpointPID[indice] = 255*RPM/MaxRPM;
  digitalWrite(MotorCtrl[indice][0],HIGH);
  digitalWrite(MotorCtrl[indice][1],LOW);
  Avance[indice] = 2;
  Frenado[indice] = false;
  switch (NMotor){
    case 1: PID1.Compute(); break;
    case 2: PID2.Compute(); break;
    case 3: PID3.Compute(); break;
  }
  analogWrite(MotorPWM[indice],byte(RapidezOutPID[indice]));
}
// --------------------------------------------------
// Retroceder, funcion que dado el numero de un motor y una rapidez angular en RPM, ejecuta las acciones necesarias para que retroceda con las RPM indicadas
void Retroceder(byte NMotor,float RPM){
  byte indice = NMotor - 1;
  switch (NMotor){
    case 1: PID1.SetMode(AUTOMATIC); break;
    case 2: PID2.SetMode(AUTOMATIC); break;
    case 3: PID3.SetMode(AUTOMATIC); break;
  }
  RapidezAngDeseada[indice] = -RPM;
  RapidezSetpointPID[indice] = 255*RPM/MaxRPM;
  digitalWrite(MotorCtrl[indice][0],LOW);
  digitalWrite(MotorCtrl[indice][1],HIGH);
  Avance[indice]= 0;
  Frenado[indice] = false;
  switch (NMotor){
    case 1: PID1.Compute(); break;
    case 2: PID2.Compute(); break;
    case 3: PID3.Compute(); break;
  }
  analogWrite(MotorPWM[indice],byte(RapidezOutPID[indice]));
}
void setup() {
  // Pines de control y PWM
  pinMode(MotorCtrl[0][0],OUTPUT);
  pinMode(MotorCtrl[0][1],OUTPUT);
  pinMode(MotorCtrl[1][0],OUTPUT);
  pinMode(MotorCtrl[1][1],OUTPUT);
  pinMode(MotorCtrl[2][0],OUTPUT);
  pinMode(MotorCtrl[2][1],OUTPUT);
  pinMode(MotorPWM[0],OUTPUT);
  pinMode(MotorPWM[1],OUTPUT);
  pinMode(MotorPWM[2],OUTPUT);
  // Inicialización de interruptores y pines digitales
  attachInterrupt(Int[0],Encoder1,RISING);
  attachInterrupt(Int[1],Encoder2,RISING);
  attachInterrupt(Int[2],Encoder3,RISING);
  PID1.SetSampleTime(TiempoDeMuestreo);
  PID2.SetSampleTime(TiempoDeMuestreo);
  PID3.SetSampleTime(TiempoDeMuestreo);
}
void loop() {
  /* Rutina a realizar
  Retroceder(1,60);
  Avanzar(2,55);
  Frenar(3);
  delay(5000);
  Retroceder(2,20);
  Avanzar(3,50);
  Frenar(1);
  delay(5000);
  Retroceder(3,60);
  Avanzar(1,50);
  Frenar(2);
  delay(5000);
  Liberar(1);
  Liberar(2);
  Liberar(3);
  delay(5000);
  */
}
// Interruptores accionados por encoders
void Encoder1(){
  RutinaEncoder(1);
}
void Encoder2(){
  RutinaEncoder(2);
}
void Encoder3(){
  RutinaEncoder(3);
}

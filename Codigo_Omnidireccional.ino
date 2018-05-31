// Código para robot omnidireccional
// Grupo 6 ; EI2001-5 ; Otoño 2018 ; FCFM ; U. de Chile
// Taller de proyecto de robótica y mecatrónica

// Declaración de variables y pines
// Matriz de pines de control para los motores (La fila determina el numero de motor y la columna el de control), los pines para el control #1 son 30, 33, 35 para los motores 1, 2 y 3
// respectivamente en el diagrama y los pines para el control #2 son 32, 35 y 34 para los motores 1, 2 y 3 respeticamente el diagrama.
const byte MotorCtrl[3][2] = {{30,31},{33,32},{35,34}};
// Lista de pines PWM (Habilitadores) para los motores segun su posición (pines 2, 3 y 4 para los motores 1, 2 y 3 respectivamente en el diagrama)
const byte MotorPWM[3] = {2,3,4};
// Lista de pines interruptores para encoders segun su posición (pines 49, 51 y 53 para encoders 1, 2 y 3 respectivamente en el diagrama)
const byte Enc[3] = {49,51,53};
// Lista de interruptores para encoders segun su posición
const byte Int[3] = {digitalPinToInterrupt(Enc[0]),digitalPinToInterrupt(Enc[1]),digitalPinToInterrupt(Enc[3])};

// Lista de contadores para las interrupciones de los encoders segun su posicion en ella
volatile byte Contador[3] = {0,0,0};

// Variables de medición de tiempo para encoders
volatile unsigned long Tiempo_Ini[3] = {0,0,0}; // Lista que almacena los tiempos en que inicia una iteración de medida de velocidad en cada encoder segun su posición en la lista
volatile unsigned long Tiempo_Fin[3];           // Lista que almacena los tiempos en que termina una iteración de medida de velocidad en cada encodes segun su posición en la lista
volatile float RapidezAngMed[3];                // Lista de rapideces angulares medidas por cada encoder segun su posición en la lista en perforaciones/ms
volatile float RPMMed[3];                       // Lista de rapideces angulares medidas por cada encoder segun su posicion en la lista en RPM

// Constantes y variables relacionadas a los encoders e interrupciones
const byte Nverificador = 5;              // Establece la resolución de medida de la velocidad angular
const byte TotalPerf = 20;                // Número total de perforaciones en el disco (Perforaciones en un ángulo 2pi)
const long TiempoParaInterrupcion = 5000; // Tiempo en el cual interrupciones sucesivas de menor tiempo se omitiran en el debouncing en microsegundos.
volatile long UltimoT[3] = {0,0,0};       // Tiempos de control para debouncing para las interrupciones de los encoders segun su posición en la lista

// Creación de funciones útiles
// --------------------------------------------------
// Debounced, función destinada a ejecutarse en una interrupción, retorna true o false dependiendo si esta misma no se ha ejecutado en un tiempo menor a UltT.
// El objetivo es limpiar la lectura de interrupciones erráticas que ocurren en un tiempo muy pequeño.
bool Debounced(volatile long UltT){
  long Diferencia = micros() - UltT;
  return Diferencia >= TiempoParaInterrupcion;
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
}

void loop() {
  // Rutina a realizar
}

// Interruptores accionados por encoders
void Encoder1(){
  if (Debounced(UltimoT[0])){
    Contador[0]++;
    if (Contador[0] == Nverificador){
      Tiempo_Fin[0] = millis();
      RapidezAngMed[0] = Contador[0]/(Tiempo_Fin[0]-Tiempo_Ini[0]);
      Tiempo_Ini[0] = millis();
      Contador[0] = 0;
    }
    UltimoT[0] = micros();
  }
}
void Encoder2(){
  if (Debounced(UltimoT[1])){
    Contador[1]++;
    if (Contador[1] == Nverificador){
      Tiempo_Fin[1] = millis();
      RapidezAngMed[1] = Contador[1]/(Tiempo_Fin[1]-Tiempo_Ini[1]);
      Tiempo_Ini[1] = millis();
      Contador[1] = 0;
    }
    UltimoT[1] = micros();
  }
}
void Encoder3(){
  if (Debounced(UltimoT[2])){
    Contador[2]++;
    if (Contador[2] == Nverificador){
      Tiempo_Fin[2] = millis();
      RapidezAngMed[2] = Contador[2]/(Tiempo_Fin[2]-Tiempo_Ini[2]);
      Tiempo_Ini[2] = millis();
      Contador[2] = 0;
    }
    UltimoT[2] = micros();
  }
}

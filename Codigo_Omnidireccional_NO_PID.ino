// Código para robot omnidireccional
// Grupo 6 ; EI2001-5 ; Otoño 2018 ; FCFM ; U. de Chile
// Taller de proyecto de robótica y mecatrónica
// --------------------------------------------------
// Nota: El origen del robot estará en su centro de masa (ortocentro del triangulo truncado del armazón), el eje y apunta hacia la rueda 1 (dónde se ubica el motor 1), el eje x es
// perpendicular al y hacia la derecha (formando un plano cartesiano), los ángulos se miden en sentido antihorario, el sistema descrito es solidario al movimiento del robot.
// Todo el programa se basa en el sistema mencionado.
// --------------------------------------------------
// La interacción con el usuario se realiza mediante el puerto serial, mediante el ingreso de textos con una sintaxis de formato <Param1,Param2,Param3,...>, en dónde Param1
// corresponde a un identificador de la función a ejectar por el programa y los parámetros subsiguientes los argumentos de esta función.
// --------------------------------------------------
// NOTA: Versión con la implementación relativa al PID, controles de estado y velocidad deshabilitada con bloques /* */

/*#include <PID_v1.h> // Librería de PID*/
#include <math.h>   // Librería de funciones matemáticas

// --------------------------------------------------
// Funciones de conversión de unidades
// --------------------------------------------------
// GradosARad, función que dado un ángulo en grados sexagesimales, lo convierte a radianes
double GradosARad(double Grados) {
  return PI * Grados / 180;
}
// RadsARPM, función que dada una rapidez angular en rad/s, la convierte a RPM
double RadsARPM(double rads) {
  return 30 * rads / PI;
}
// GradsARads, función que dada una rapidez angular en grados/s, la convierte a rad/s
double GradsARads(double grads) {
  return PI * grads / 180;
}
// --------------------------------------------------
// Declaración de variables, pines y constantes
// --------------------------------------------------
// Matriz de pines de control para los motores (La fila determina el numero de motor y la columna el de control), los pines para el control #1 son 30, 33, 35 para los motores 1, 2 y 3
// respectivamente en el diagrama y los pines para el control #2 son 32, 35 y 34 para los motores 1, 2 y 3 respeticamente el diagrama.
const byte MotorCtrl[3][2] = {{A0, 31}, {33, 32}, {35, 34}}; // Cambiar A0 por 30 para uso en DUE
// Lista de pines PWM (Habilitadores) para los motores segun su posición (pines 2, 3 y 4 para los motores 1, 2 y 3 respectivamente en el diagrama)
const byte MotorPWM[3] = {2, 3, 4};
/*// Lista de pines interruptores para encoders segun su posición (pines 49, 51 y 53 para encoders 1, 2 y 3 respectivamente en el diagrama)
const byte Enc[3] = {49, 51, 53};
// Lista de interruptores para encoders segun su posición
const byte Int[3] = {digitalPinToInterrupt(Enc[0]), digitalPinToInterrupt(Enc[1]), digitalPinToInterrupt(Enc[2])};

// Lista de contadores para las interrupciones de los encoders segun su posicion en ella
volatile byte Contador[3] = {0, 0, 0};

// Variables de medición de tiempo para encoders
volatile unsigned long Tiempo_Ini[3] = {0, 0, 0}; // Lista que almacena los tiempos en que inicia una iteración de medida de velocidad en cada encoder segun su posición en la lista
volatile unsigned long Tiempo_Fin[3];           // Lista que almacena los tiempos en que termina una iteración de medida de velocidad en cada encoder segun su posición en la lista
volatile float RapidezAngMed[3];                // Lista de rapideces angulares medidas por cada encoder segun su posición en la lista en perforaciones/ms
volatile float RPMMed[3];                       // Lista de rapideces angulares medidas por cada encoder segun su posicion en la lista en RPM
double RapidezInPID[3] = {0, 0, 0};             // Lista de rapideces angulares medidas ajustadas al rango PWM requerido por los motores*/

// Constantes de funcionamiento y especificaciones del robot
const float MaxRPM = 60;                  // Rapidez angular máxima que los motores son capaces de entregar en RPM
const double RadioRuedas = 50;            // Radio de las ruedas en mm
const double DistanciaAlCM = 250;         // Distancia desde el plano de giro de la rueda al centro de masas del robot en mm
const double AngRuedas[3] = {0, 120, 240}; // Lista que almacena el angulo que forma el eje centro de masas - rueda con respecto al eje y en grados
const double AngRadRuedas[3] = {GradosARad(AngRuedas[0]), GradosARad(AngRuedas[1]), GradosARad(AngRuedas[2])}; // Lista que almacena lo mismo que AngRuedas pero en radianes

/*// Constantes y variables relacionadas a los encoders e interrupciones
const byte Nverificador = 5;              // Establece la resolución de medida de la velocidad angular
const byte TotalPerf = 40;                // Número total de perforaciones en el disco (Perforaciones en un ángulo 2pi)
const long TiempoParaInterrupcion = 5000; // Tiempo en el cual interrupciones sucesivas de menor tiempo se omitiran en el debouncing en microsegundos
volatile long UltimoT[3] = {0, 0, 0};     // Tiempos de control para debouncing para las interrupciones de los encoders segun su posición en la lista

// Variables de estado para el robot
double RapidezAngDeseada[3] = {0, 0, 0}; // Lista que almacena a que rapidez angular que desea que funcione el motor corrspondiente a la posición en la lista en RPM
double RapidezSetpointPID[3];           // Lista que almacena los setpoints de rapidez angular para cada motor en el rango PWM correspondiente
byte Avance[3] = {0, 0, 0};             // Variable que almacena el estado de mov. del motor en la pos. correspondiente (0 a retroceso, 1 a detención (libre o frenada) y 2 a avance)
bool Frenado[3] = {true, true, true};   // Variable que almacena el estado de frenado del robot para el motor de la posicion correspondietne en la lista

// Declaraciones y variables para control PID
double Kp = 1;                // Constante proporcional de los PID
double Ki = 1;                // Constante integral de los PID
double Kd = 1;                // Constante derivativa de los PID
double RapidezOutPID[3];      // Lista de rapideces angulares de salida post control PID ajustadas al rango PWM requerido por los motores
int TiempoDeMuestreo = 200;   // Tiempo de computo del PID en ms
PID PIDs[3] = {PID(&RapidezInPID[0], &RapidezOutPID[0], &RapidezSetpointPID[0], Kp, Ki, Kd, DIRECT, P_ON_M), // Lista de PIDs, donde cada cual controla la rapidez angular del motor
               PID(&RapidezInPID[1], &RapidezOutPID[1], &RapidezSetpointPID[1], Kp, Ki, Kd, DIRECT, P_ON_M), // correspondiente a su posición en la lista
               PID(&RapidezInPID[2], &RapidezOutPID[2], &RapidezSetpointPID[2], Kp, Ki, Kd, DIRECT, P_ON_M)
              };*/

// Variables de trabajo para funciones asociadas al movimiento completo del robot
double RPMMov[3];     // Lista que almacena los valores trabajados por MovimientoXYRot(), ver función para mas detalles
double RPMLimMov[3];  // Lista que almacena los valores trabajados por LimitarMov(), ver función para mas detalles

// Constantes y arreglos usados en la comunicacion serial
char Caracter;                            // Inicializa la variable Caracter, que irá almacenando uno a uno los caracteres recibidos por el serial para ser analizados
const char CaractIni = '<';               // Caracter que indica el inicio de una nueva instrucción
const char CaractFin = '>';               // Caracter que indica el final de la instruccion que se esté recibiendo
const char Delim[2] = {',', '\0'};        // String en formato C que indica el separador de parámetros en las instrucciones recibidas
const byte SizeBuffer = 128;              // Tamaño máximo del buffer que almacenará la instrucción a trabajar
const byte MxSizeParam = 32;              // Tamaño máximo del espacio donde se almacenarán los parámetros
const byte MxCantParam = 8;               // Cantidad máxima de parámetros que serán almacenados de una instrucción
byte IndiceCaract = 0;                    // Indice que recorre el Buffer escribiéndolo con la instrucción
byte IndiceParam = 0;                     // Indice que recorre el arreglo de parámetros para que sea escrito cada uno en su lugar correspondiente
char Buffer[SizeBuffer];                  // Variable buffer que almacena el texto recibido por el serial
char Param[MxSizeParam][MxCantParam];     // Matriz que almacena los parámetros extraidos dela lectura del Serial
char SizeParam = MxSizeParam * MxCantParam; // Tamaño total de la matriz Param

/*// --------------------------------------------------
// Funciones útiles para encoders e interrupciones
// --------------------------------------------------
// Debounced, función destinada a ejecutarse en una interrupción, retorna true o false dependiendo si esta misma no se ha ejecutado en un tiempo menor a UltT.
// El objetivo es limpiar la lectura de interrupciones erráticas que ocurren en un tiempo muy pequeño.
bool Debounced(volatile long UltT) {
  long Diferencia = micros() - UltT;
  return Diferencia >= TiempoParaInterrupcion;
}
// --------------------------------------------------
// RutinaEncoder, función que ejecuta la accioón que debe realizar cada encoder tomando como parámetro el número de este, cada ISR llama a esta función con el respectivo número del
// encoder que ejecutó la interrupción.
void RutinaEncoder(byte NEncoder) {
  byte Num = NEncoder - 1;
  if (Debounced(UltimoT[Num])) {
    Contador[Num]++;
    if (Contador[Num] == Nverificador) {
      Tiempo_Fin[Num] = millis();
      RapidezAngMed[Num] = Contador[Num] / (Tiempo_Fin[Num] - Tiempo_Ini[Num]);
      RPMMed[Num] = RapidezAngMed[Num] * 60000 / TotalPerf;
      RapidezInPID[Num] = RPMMed[Num] * 255 / MaxRPM;
      Tiempo_Ini[Num] = millis();
      Contador[Num] = 0;
    }
    UltimoT[Num] = micros();
  }
  PIDs[Num].Compute();
  if (Avance[Num] != 1) {
    analogWrite(MotorPWM[Num], byte(RapidezOutPID[Num]));
  }
}*/
// --------------------------------------------------
// Funciones de movimiento de los motores
// --------------------------------------------------
// Liberar, funcion que dado el numero de un motor ejecuta acciones necesarias para que no entregue torque y gire libremente
void Liberar(byte NMotor) {
  byte indice = NMotor - 1;
  /*PIDs[indice].SetMode(MANUAL);*/
  digitalWrite(MotorCtrl[indice][0], LOW);
  digitalWrite(MotorCtrl[indice][1], LOW);
  /*RapidezAngDeseada[indice] = 0;
  RapidezSetpointPID[indice] = 0;
  Avance[indice] = 1;
  Frenado[indice] = false;*/
}
// --------------------------------------------------
// Frenar, funcion que dado el numero de un motor ejecuta acciones necesarias para frenarlo
void Frenar(byte NMotor) {
  byte indice = NMotor - 1;
  /*PIDs[indice].SetMode(MANUAL);*/
  digitalWrite(MotorCtrl[indice][0], HIGH);
  digitalWrite(MotorCtrl[indice][1], HIGH);
  /*RapidezAngDeseada[indice] = 0;
  RapidezSetpointPID[indice] = 0;
  Avance[indice] = 1;
  Frenado[indice] = true;*/
}
// --------------------------------------------------
// Avanzar, funcion que dado el numero de un motor y una rapidez angular en RPM, ejecuta las acciones necesarias para que avance con las RPM indicadas
void Avanzar(byte NMotor, double RPM) {
  byte indice = NMotor - 1;
  /*PIDs[indice].SetMode(AUTOMATIC);
  RapidezAngDeseada[indice] = RPM;
  RapidezSetpointPID[indice] = 255 * RPM / MaxRPM;*/
  digitalWrite(MotorCtrl[indice][0], HIGH);
  digitalWrite(MotorCtrl[indice][1], LOW);
  analogWrite(MotorPWM[indice],255*RPM/MaxRPM);
  /*Avance[indice] = 2;
  Frenado[indice] = false;
  PIDs[indice].Compute();
  analogWrite(MotorPWM[indice], byte(RapidezOutPID[indice]));*/
}
// --------------------------------------------------
// Retroceder, funcion que dado el numero de un motor y una rapidez angular en RPM, ejecuta las acciones necesarias para que retroceda con las RPM indicadas
void Retroceder(byte NMotor, double RPM) {
  byte indice = NMotor - 1;
  /*PIDs[indice].SetMode(AUTOMATIC);
  RapidezAngDeseada[indice] = -RPM;
  RapidezSetpointPID[indice] = 255 * RPM / MaxRPM;*/
  digitalWrite(MotorCtrl[indice][0], LOW);
  digitalWrite(MotorCtrl[indice][1], HIGH);
  analogWrite(MotorPWM[indice],255*RPM/MaxRPM);
  /*Avance[indice] = 0;
  Frenado[indice] = false;
  PIDs[indice].Compute();
  analogWrite(MotorPWM[indice], byte(RapidezOutPID[indice]));*/
}
// --------------------------------------------------
// Funciones para movimiento global del robot
// --------------------------------------------------
// MovimientoXYRot, función que dada una velocidad en el eje x en mm/s, una en el eje y en mm/s y una rapidez angular en grados/s, modifica la lista RPMMov, con los valores
// correspondientes a las RPM que debe ejecutar cada motor según su posición en la lista para realizar el movimiento descrito
void MovimientoXYRot(double Vx, double Vy, double Rot) {
  double theta;
  if (Vy != 0) {
    theta = atan2(Vx, Vy);
  }
  else if (Vx > 0) {
    theta = PI / 2;
  }
  else {
    theta = -PI / 2;
  }
  double RotRads = GradsARads(Rot);
  double AngParaCal1 = theta + AngRadRuedas[0];
  double AngParaCal2 = theta + AngRadRuedas[1];
  double AngParaCal3 = theta + AngRadRuedas[2];
  double RapAng1_p1 = (cos(theta) * cos(AngParaCal1) - sin(theta) * sin(AngParaCal1)) * Vx;
  double RapAng2_p1 = (cos(theta) * cos(AngParaCal2) - sin(theta) * sin(AngParaCal2)) * Vx;
  double RapAng3_p1 = (cos(theta) * cos(AngParaCal3) - sin(theta) * sin(AngParaCal3)) * Vx;
  double RapAng1_p2 = -(sin(theta) * cos(AngParaCal1) + cos(theta) * sin(AngParaCal1)) * Vy;
  double RapAng2_p2 = -(sin(theta) * cos(AngParaCal2) + cos(theta) * sin(AngParaCal2)) * Vy;
  double RapAng3_p2 = -(sin(theta) * cos(AngParaCal3) + cos(theta) * sin(AngParaCal3)) * Vy;
  double RapAng1_p3 = DistanciaAlCM * RotRads;
  double RapAng2_p3 = DistanciaAlCM * RotRads;
  double RapAng3_p3 = DistanciaAlCM * RotRads;
  double RapAng1 = (RapAng1_p1 + RapAng1_p2 + RapAng1_p3) / RadioRuedas;
  double RapAng2 = (RapAng2_p1 + RapAng2_p2 + RapAng2_p3) / RadioRuedas;
  double RapAng3 = (RapAng3_p1 + RapAng3_p2 + RapAng3_p3) / RadioRuedas;
  double RPM1 = RadsARPM(RapAng1);
  double RPM2 = RadsARPM(RapAng2);
  double RPM3 = RadsARPM(RapAng3);
  RPMMov[0] = RPM1;
  RPMMov[1] = RPM2;
  RPMMov[2] = RPM3;
}
// --------------------------------------------------
// LimitarMov, funcion que dados 3 valores correspondientes a RPM que debieran tener los motores para mover el robot de cierta forma, revisa que no sobrepasen las RPM máximas del mismo
// motor, en caso de que lo hagan, disminuye las RPM de todos en igual medida, los valores para cada motor, se guardan en la posición correspondiente de la lista RPMLimMov, retora true
// en caso de modificar los valores, false en caso contrario.
bool LimitarMov(double RPM1, double RPM2, double RPM3) {
  double MaxPar = max(abs(RPM1), abs(RPM2));
  double Max = max(MaxPar, abs(RPM3));
  if (Max > MaxRPM) {
    double factor = MaxRPM / Max;
    RPMLimMov[0] = RPM1 * factor;
    RPMLimMov[1] = RPM2 * factor;
    RPMLimMov[2] = RPM3 * factor;
    return true;
  }
  else {
    RPMLimMov[0] = RPM1;
    RPMLimMov[1] = RPM2;
    RPMLimMov[2] = RPM3;
    return false;
  }
}
// --------------------------------------------------
// MoverMotores, función que dados tres valores en RPM, da ordenes a los motores para que se muevan a esa rapidez en el sentido dado por el signo de los argumentos de la función
void MoverMotores(double VMot1, double VMot2, double VMot3) {
  LimitarMov(VMot1, VMot2, VMot3);
  for (byte i = 0; i < 3; i++) {
    if      (RPMLimMov[i] < 0) {
      Retroceder(i + 1, -RPMLimMov[i]);
    }
    else if (RPMLimMov[i] == 0) {
      Liberar(i + 1);
    }
    else                      {
      Avanzar(i + 1, RPMLimMov[i]);
    }
  }
}
// --------------------------------------------------
// MoverXYRotRobot, funcion que ejecuta las ordenes necesarias para mover el robot con parámetros análogos a la función MovimientoXYRot
void MoverXYRotRobot(double Vx, double Vy, double Rot) {
  MovimientoXYRot(Vx, Vy, Rot);
  MoverMotores(RPMMov[0], RPMMov[1], RPMMov[2]);
}
// --------------------------------------------------
// FrenarRobot, funcion que ejecuta las ordenes necesarias para frenar el robot
void FrenarRobot() {
  Frenar(1);
  Frenar(2);
  Frenar(3);
}
// --------------------------------------------------
// MoverPolarRotRobot, funcion que ejecuta las ordenes necesarias para mover el robot usando un sistema polar, dónde los ángulos se miden desde del eje y cartesiano, el primer parámetro
// es la rapidez resultante, el segundo el ángulo y el tercero la rotación (Análoga a MoverXYRotRobot())
void MoverPolarRotRobot(double Rapidez, double Angulo, double Rot) {
  double Grados = GradosARad(Angulo);
  double VelX = -Rapidez * sin(Grados);
  double VelY = Rapidez * cos(Grados);
  MoverXYRotRobot(VelX, VelY, Rot);
}
// --------------------------------------------------
// Funciones para comunicación serial
// --------------------------------------------------
// ParseBuffer, función que al ser ejecutada trabaja el contenido ASCII almacenado en la variable buffer y realiza un anáslis gramatical de este para separar el string por el delimitador
// definido, almacenando los parámetros resultatantes (tokens) en la matriz Param.
void ParseBuffer() {
  memset(Param, '\0', SizeParam);
  char *Token = strtok(Buffer, Delim);
  while (Token != NULL) {
    strcpy(Param[IndiceParam], Token);
    Token = strtok(NULL, Delim);
    IndiceParam++;
  }
  IndiceParam = 0;
  memset(Buffer, '\0', SizeBuffer);
}
// --------------------------------------------------
// EjecutarComandos, función que lee los parámetros contenidos en la matriz Param y los trabaja en función de lo que sea necesario siguiendo la sintaxis <Funcion,Arg1,Arg2,Arg3,...>
// Es decir, el primer parámetro corresponde al identificador de la función y los siguientes a los argumentos que esta requiera, identificadores a funciones del robot corresponden a:
// XYR : MoverXYRotRobot(), recibe 3 argumentos
// POL : MoverPolarRotRobot(), recibe 3 argumentos
// MMT : MoverMotores(), recibe 3 argumentos
// FRN : FrenarRobot(), no recibe argumentos
void EjecutarComandos() {
  String Funcion = String(Param[0]);
  if      (Funcion.equals("XYR")) {
    double VxSR = atof(Param[1]);
    double VySR = atof(Param[2]);
    double RotSR = atof(Param[3]);
    MoverXYRotRobot(VxSR, VySR, RotSR);
  }
  else if (Funcion.equals("POL")) {
    double RapidezSR = atof(Param[1]);
    double AnguloSR = atof(Param[2]);
    double RotSR = atof(Param[3]);
    MoverPolarRotRobot(RapidezSR, AnguloSR, RotSR);
  }
  else if (Funcion.equals("MMT")) {
    double VelM1SR = atof(Param[1]);
    double VelM2SR = atof(Param[2]);
    double VelM3SR = atof(Param[3]);
    MoverMotores(VelM1SR, VelM2SR, VelM3SR);
  }
  else if (Funcion.equals("FRN")) {
    FrenarRobot();
  }
  // Caso para debugging y demostración, a eliminar en versión final
  else if (Funcion.equals("LED")) {
    byte LedAzul = 12;
    byte LedRojo = 11;
    pinMode(LedAzul, OUTPUT);
    pinMode(LedRojo, OUTPUT);
    if (0 == atoi(Param[1])) {
      digitalWrite(LedAzul, LOW);
    }
    if (1 == atoi(Param[1])) {
      digitalWrite(LedAzul, HIGH);
    }
    if (0 == atoi(Param[2])) {
      digitalWrite(LedRojo, LOW);
    }
    if (1 == atoi(Param[2])) {
      digitalWrite(LedRojo, HIGH);
    }
  }
  else {
    Serial.println("Función desconocida");
  }
}
// --------------------------------------------------
// RecepcionSerial, función principal que administra la comunicación con el puerto serial, validando los datos recibidos e invocándo funciones que los almacenen en Buffer, en caso de
// recibir el caracter especial definido como caracter inicial limpia el buffer, el caso de recibir el caracter especial definido como caracter final ejecuta el análisis de los datos
// recibidos, todo ocurre mientras hayan datos disponibles en el serial, en caso contrario la función no ejecuta acción.
void RecepcionSerial() {
  while (Serial.available() > 0) {
    Caracter = Serial.read();
    if (Caracter == CaractIni) {
      memset(Buffer, '\0', SizeBuffer);
      IndiceCaract = 0;
    }
    else if (Caracter == CaractFin) {
      ParseBuffer();
      EjecutarComandos();
    }
    else {
      Buffer[IndiceCaract] = Caracter;
      IndiceCaract++;
    }
  }
}
// --------------------------------------------------
// Funciones principales de iniciación y ciclo de Arduino
// --------------------------------------------------
void setup() {
  // Pines de control y PWM
  pinMode(MotorCtrl[0][0], OUTPUT);
  pinMode(MotorCtrl[0][1], OUTPUT);
  pinMode(MotorCtrl[1][0], OUTPUT);
  pinMode(MotorCtrl[1][1], OUTPUT);
  pinMode(MotorCtrl[2][0], OUTPUT);
  pinMode(MotorCtrl[2][1], OUTPUT);
  pinMode(MotorPWM[0], OUTPUT);
  pinMode(MotorPWM[1], OUTPUT);
  pinMode(MotorPWM[2], OUTPUT);
  /*// Inicialización de interruptores y pines digitales
  attachInterrupt(Int[0], Encoder1, RISING);
  attachInterrupt(Int[1], Encoder2, RISING);
  attachInterrupt(Int[2], Encoder3, RISING);
  // Ajuste de PIDs
  PIDs[0].SetSampleTime(TiempoDeMuestreo);
  PIDs[1].SetSampleTime(TiempoDeMuestreo);
  PIDs[2].SetSampleTime(TiempoDeMuestreo);*/
  // Inicialización de la comunicación serial
  Serial.begin(9600);
}
void loop() {
  RecepcionSerial();
  delay(10);
}
/*// --------------------------------------------------
// Interruptores accionados por encoders
// --------------------------------------------------
void Encoder1() {
  RutinaEncoder(1);
}
void Encoder2() {
  RutinaEncoder(2);
}
void Encoder3() {
  RutinaEncoder(3);
}*/

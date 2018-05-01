// Código para robot omnidireccional
// Grupo 6 ; EI2001-5 ; Otoño 2018 ; FCFM ; U. de Chile
// Taller de proyecto de robótica y mecatrónica

// Declaración de variables y pines
const byte M1_Ctrl1; // Pin #1 de control para el motor 1 (pin 30 en diagrama)
const byte M1_Ctrl2; // Pin #2 de control para el motor 1 (pin 31 en diagrama)
const byte M2_Ctrl1; // Pin #1 de control para el motor 2 (pin 33 en diagrama)
const byte M2_Ctrl2; // Pin #2 de control para el motor 2 (pin 32 en diagrama)
const byte M3_Ctrl1; // Pin #1 de control para el motor 3 (pin 35 en diagrama)
const byte M3_Ctrl2; // Pin #2 de control para el motor 3 (pin 34 en diagrama)

const byte M1_PWM; // Pin PWM (Habilitador) para el motor 1 (pin 2 en diagrama)
const byte M2_PWM; // Pin PWM (Habilitador) para el motor 2 (pin 3 en diagrama)
const byte M3_PWM; // Pin PWM (Habilitador) para el motor 3 (pin 4 en diagrama)

const byte Enc1; // Pin interruptor para encoder 1 (pin 49 en diagrama)
const byte Enc2; // Pin interruptor para encoder 2 (pin 51 en diagrama)
const byte Enc3; // Pin interruptor para encoder 3 (pin 53 en diagrama)

const byte Int1 = digitalPinToInterrupt(Enc1);  // Interruptor para encoder 1
const byte Int2 = digitalPinToInterrupt(Enc1);  // Interruptor para encoder 2
const byte Int3 = digitalPinToInterrupt(Enc1);  // Interruptor para encoder 3

// Contadores para los encoders
volatile byte Contador1 = 0; // Contador para las interrupciones del encoder 1
volatile byte Contador2 = 0; // Contador para las interrupciones del encoder 2
volatile byte Contador3 = 0; // Contador para las interrupciones del encoder 3

// Variables de medición de tiempo para encoders
volatile unsigned long Tiempo_Ini1 = 0; // Almacena el tiempo en que inicia una iteración de medida de velocidad en encoder 1
volatile unsigned long Tiempo_Ini2 = 0; // Almacena el tiempo en que inicia una iteración de medida de velocidad en encoder 2
volatile unsigned long Tiempo_Ini3 = 0; // Almacena el tiempo en que inicia una iteración de medida de velocidad en encoder 3
volatile unsigned long Tiempo_Fin1; // Almacena el tiempo en que termina una iteración de medida de velocidad en encoder 1
volatile unsigned long Tiempo_Fin2; // Almacena el tiempo en que termina una iteración de medida de velocidad en encoder 2
volatile unsigned long Tiempo_Fin3; // Almacena el tiempo en que termina una iteración de medida de velocidad en encoder 3

// Constantes relacionadas a los encoders
const byte Nverificador = 5; // Establece la resolución de medida de la velocidad angular

// Creación de funciones útiles

void setup() {
  // Pines de control y PWM
  pinMode(M1_Ctrl1,OUTPUT);
  pinMode(M1_Ctrl2,OUTPUT);
  pinMode(M2_Ctrl1,OUTPUT);
  pinMode(M2_Ctrl2,OUTPUT);
  pinMode(M3_Ctrl1,OUTPUT);
  pinMode(M3_Ctrl2,OUTPUT);
  pinMode(M1_PWM,OUTPUT);
  pinMode(M2_PWM,OUTPUT);
  pinMode(M3_PWM,OUTPUT);
  // Inicialización de interruptores y pines digitales
  attachInterrupt(Int1,Encoder1,RISING);
  attachInterrupt(Int2,Encoder2,RISING);
  attachInterrupt(Int3,Encoder3,RISING);
}

void loop() {
  // Rutina a realizar
}

// Interruptores accionados por encoders
void Encoder1(){
  Contador1++;
  if (Contador1 == Nverificador){
    Tiempo_Fin1 = millis();
    // Calcula diferencia
    // Calcula velocidad
    Tiempo_Ini1;
    }
  }
void Encoder2(){
  Contador2++;
  }
void Encoder3(){
  }

# Robot Omnidireccional
```
Proyecto de robot omnidireccional para taller de proyecto de robótica y mecatrónica
Curso EI2001-5 grupo 6, semestre de otoño 2018
Facultad de Ciencias Físicas y Matemáticas
Universidad de Chile
```
Repositorio del proyecto mencionado con la finalidad de exponer avances.

El proyecto consiste en un robot omnidireccional de 3 ruedas, capaz de moverse en tres grados de libertad (eje x, eje y y rotación),
aun está pendiente el método de control por el usuario (control remoto) del mismo.

Elementos finalizados
- [x] Diseño del robot
- [x] Control de motores con PID
- [x] Programación funcional del movimiento

Elementos por finalizar
- [ ] Construcción y ensamblado del robot
- [ ] Testeo de los códigos de movimiento
- [ ] Programación de control a distancia

## Estado de la aplicación de Android

Aplicación cuyo objetivo final es la comunicación por Bluetooth con el robot omnidireccional para utilizarse como control remoto del mismo.
Contiene 4 activities (ventanas de trabajo), dónde la organización de elementos en pantalla está realizada, no así la rotulación correcta de los elementos, en la actividad princial hay tres botones programados:
- El de la izquierda accede al menú que lleva a otras ventanas
- El central conecta el télefono por bluetooth a cualquier dispositivo llamado "HC-06", que corresponde al nombre del módulo de Arduino
- El derecho, una vez conectado envía un byte (0 o 1) que el Arduino interpreta para apagar o encender un led

Se ha logrado principalmente la conexión Bluetooth y el envío de bytes simples, también está programada la recepción de Strings para recibirlas de Arduino y mostrarlas en un TextView, pero se muestra de forma errática esta información. El principal desafío actual es crear un protocolo de comunicación entre ambos dispositivos para poder enviar instrucciones en partícular funciones del Android al Arduino.

Funciones que realizan las actividades mencionadas se encuentran en [MainActivity.java](https://github.com/Camilormz/Robot_Omnidireccional/blob/master/ControlAndroid/app/src/main/java/cl/uchile/ing/controlomnidireccional/MainActivity.java).

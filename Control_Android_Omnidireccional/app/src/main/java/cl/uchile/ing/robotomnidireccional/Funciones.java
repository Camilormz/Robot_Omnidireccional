// Clase para definir funciones de uso común por la aplicación
package cl.uchile.ing.robotomnidireccional;

// Importe de clases útilies para comunicación Bluetooth
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;

// Importe de paquetes android genéricos para uso de funciones
import android.content.Context;
import android.widget.Toast;

// Importe de clases de Java necesarias para la comunicación
import java.io.IOException;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

public class Funciones {

    private static OutputStream OS_Arduino;
    private static BluetoothSocket HC06Socket;

    public static void ConectarHC06(Context Contexto){
        BluetoothAdapter AdaptadorDisp = BluetoothAdapter.getDefaultAdapter();
            // Obtiene el adaptador del dispositivo actual
        Set<BluetoothDevice> dispsSinc = AdaptadorDisp.getBondedDevices();
            // Define una lista de dispostivos externos sincronizados con el dispositivo propio
        if (dispsSinc.size()>0){
            IntentarEnlazarHC06(dispsSinc,Contexto);
        }   // En caso de existir dispositivos pareados intenta enlazar con HC-06
    }

    public static void TransmitirSerial(String Informacion, Context Contexto){
        String FallaTransmision = "Falla de transmisión de datos";
        try{OS_Arduino.write(Informacion.getBytes());}
        catch(IOException e){MostrarToast(FallaTransmision,Contexto);}
    }

    private static void MostrarToast(CharSequence Texto, Context Contexto){
        int Duracion = Toast.LENGTH_LONG;   // Duración de toast de 3,5 segundos
        Toast MensajeToast = Toast.makeText(Contexto, Texto, Duracion);
        MensajeToast.show();                // Muestra el toast!
    }

    private static void IntentarEnlazarHC06(Set<BluetoothDevice> ListaDisp, Context Contexto){
        String NombreModulo = "HC-06";  // Nombre del módulo Bluetooth
        for (BluetoothDevice Dispositivo : ListaDisp){
            if (Dispositivo.getName().equals(NombreModulo)){
                IntentarConcretarConexion(Dispositivo,Contexto);
            }   // Si es dispositivo coincide en nombre con el módulo intenta concretar la conexión
        }       // Los dispositivos se buscan en la lista de dispositivos pareados
    }

    private static void IntentarConcretarConexion(BluetoothDevice Disp, Context Contexto){
        String UUIDTexto = "00001101-0000-1000-8000-00805F9B34FB";  // Código UUID del módulo (ID)
        String FallaAsignacion = "Falla en asignar al socket un servicio de comunicación";
        String Conectado = "¡Conexión establecida!";
        String FallaConexion = "Falla en conectar al servicio de comunicación";
        String FallaCierre = "Falla en cerrar el servicio de comunicación (REINICIAR APP)";
        String NoHayPareados = "No hay dispositivos pareados";
        String FallaStream = "Falla al establecer flujos de entrada y salida de datos";
        UUID UBT = UUID.fromString(UUIDTexto);                  // Almacenamiento como objeto UUID
        BluetoothSocket TempSocket = null;                      // Declaración de un socket temporal
        // Intento de asignar al socket a un servicio de comunicación
        try{TempSocket = Disp.createInsecureRfcommSocketToServiceRecord(UBT);}
        catch (IOException e){MostrarToast(FallaAsignacion,Contexto);}
        if (TempSocket != null){
            try {   // Si se logró asignar el socket intenta conectar con él
                TempSocket.connect();
                HC06Socket = TempSocket;
                MostrarToast(Conectado,Contexto);
            }       // Si no se logra la conexión con el se intenta cerrar el socket
            catch (IOException conectException){
                MostrarToast(FallaConexion,Contexto);
                try{TempSocket.close();}
                catch(IOException closeException){MostrarToast(FallaCierre,Contexto);}
            }
        }
        else{MostrarToast(NoHayPareados,Contexto);}
        try {   // Intenta definir flujos de entrada y salida de datos
            OS_Arduino = HC06Socket.getOutputStream();}
        catch(IOException e){MostrarToast(FallaStream,Contexto);}
    }
}

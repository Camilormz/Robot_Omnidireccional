package cl.uchile.ing.controlomnidireccional;

import android.content.Context;
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private static final UUID UBT = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private BluetoothSocket ArduinoSk;

        @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }
    public void AbrirMenu(View AM) {
        Intent pasarAMenu = new Intent(getApplicationContext(), Menu.class);
        startActivity(pasarAMenu);
    }

    public void ConectarBT(View CBT){
        BluetoothAdapter MiBT = BluetoothAdapter.getDefaultAdapter();
        Set<BluetoothDevice> dispsSinc = MiBT.getBondedDevices();
        if (dispsSinc.size()>0){
            for (BluetoothDevice disp : dispsSinc){
                if (disp.getName().equals("HC-06")){
                    BluetoothSocket Temp = null;
                    try{Temp = disp.createInsecureRfcommSocketToServiceRecord(UBT);}
                    catch (IOException e){}
                    try{Temp.connect();
                        ArduinoSk = Temp;
                        MostrarToast("¡Conectado!");}
                    catch(IOException conectException){
                        try{Temp.close();}
                        catch(IOException closeException){}
                        return;
                    }
                }
            }
        }
    }

    public void MostrarToast(CharSequence texto) {
        Context context = getApplicationContext();
        int duration = Toast.LENGTH_SHORT;
        Toast toast = Toast.makeText(context, texto, duration);
        toast.show();
    }

    private byte[] MsgRec = new byte[128];
    private int bytesLeidos;
    private int encendido = 0;
    private String AOut;
    private InputStream ISArduino;
    private OutputStream OSArduino;

    public void SwitchLed(View SL){
        setContentView(R.layout.activity_main);
        final TextView out = findViewById(R.id.textView);
        try{
            ISArduino = ArduinoSk.getInputStream();
            OSArduino = ArduinoSk.getOutputStream();
        }
        catch(IOException e){}
        if (encendido == 0){
            try{OSArduino.write(1);
                encendido = 1;}
            catch(IOException e){}
            try{Thread.sleep(50);}
            catch(InterruptedException e){}
            try{
                bytesLeidos = ISArduino.read(MsgRec);
                }
            catch(IOException e){}
            AOut = new String(MsgRec);
            out.setText(AOut);
        }
        else if(encendido == 1){
            try{OSArduino.write(0);
                encendido = 0;}
            catch(IOException e){}
            try{Thread.sleep(50);}
            catch(InterruptedException e){}
            try{
                bytesLeidos = ISArduino.read(MsgRec);
            }
            catch(IOException e){}
            AOut = new String(MsgRec);
            out.setText(AOut);
        }
    }
}
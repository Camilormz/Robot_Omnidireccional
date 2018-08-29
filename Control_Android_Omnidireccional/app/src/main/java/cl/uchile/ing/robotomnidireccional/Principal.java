package cl.uchile.ing.robotomnidireccional;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;

public class Principal extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_principal);
    }

    // Método para conexión Bluetooth con el módulo HC-06
    public void ConectarBT(View CBT){Funciones.ConectarHC06(getApplicationContext());}
    public void InstruccionAdelante(View IAD)        {EnviarASerial("<POL,1000,0,0>");}
    public void InstruccionAdelanteIzq(View IADI)    {EnviarASerial("<POL,1000,45,0>");}
    public void InstruccionIzquierda(View IZQ)       {EnviarASerial("<POL,1000,90,0>");}
    public void InstruccionAtrasIzq(View IATI)       {EnviarASerial("<POL,1000,135,0>");}
    public void InstruccionAtras(View IAT)           {EnviarASerial("<POL,1000,180,0>");}
    public void InstruccionAtrasDer(View IATD)       {EnviarASerial("<POL,1000,225,0>");}
    public void InstruccionDerecha(View DER)         {EnviarASerial("<POL,1000,270,0>");}
    public void InstruccionAdelanteDer(View IADD)    {EnviarASerial("<POL,1000,315,0>");}
    public void InstruccionRotacionHor(View HOR)     {EnviarASerial("<POL,0,0,-1000>");}
    public void InstruccionRotacionAntihor(View ANHR){EnviarASerial("<POL,0,0,1000>");}
    public void InstruccionFreno(View FRN)           {EnviarASerial("<FRN>");}

    // Método que resume las transmisiones seriales de los botones
    private void EnviarASerial(String Comando){
        Funciones.TransmitirSerial(Comando,getApplicationContext());
    }
}

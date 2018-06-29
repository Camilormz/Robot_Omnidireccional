package cl.uchile.ing.controlomnidireccional;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;

public class Menu extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_menu);
    }
    public void AbrirMain(View AMn) {
        Intent pasarAMain = new Intent(getApplicationContext(), MainActivity.class);
        startActivity(pasarAMain);
    }
    public void AbrirEntradaManual(View EntM) {
        Intent pasarAEM = new Intent(getApplicationContext(), EntradaManual.class);
        startActivity(pasarAEM);
    }
    public void AbrirMotores(View AMot) {
        Intent pasarAMot = new Intent(getApplicationContext(), Motores.class);
        startActivity(pasarAMot);
    }
}

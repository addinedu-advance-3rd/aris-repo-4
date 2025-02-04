package com.example.mimokioskapp;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;

public class E_PaymentActivity extends Activity {

    private RadioButton card_btn, app_btn;
    private TextView textPaymentAmount;
    private Button done;
    private String selectedFlavor = "";

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_e_payment);

        selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        if (selectedFlavor != null) {
            Toast.makeText(this, "선택된 맛:" + selectedFlavor, Toast.LENGTH_SHORT).show();
        }


        card_btn = findViewById(R.id.card_btn);
        app_btn = findViewById(R.id.app_btn);

        done = findViewById(R.id.done);

        textPaymentAmount = findViewById(R.id.textPaymentAmount);

        Intent intent = getIntent();
        int totalPrice = intent.getIntExtra("totalPrice", 4000); //기본값 4000원
        textPaymentAmount.setText("결제 금액 : " + totalPrice + "원");


        done.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (card_btn.isChecked()) {
                    Intent intent1 = new Intent(E_PaymentActivity.this, FinalActivity.class);
                    intent1.putExtra("selectedFlavor", selectedFlavor);
                    startActivity(intent);
                }
            }
        });


    }
}

package com.example.mimokioskapp;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.TextView;

import androidx.annotation.Nullable;

public class E_PaymentActivity extends Activity {

    private RadioButton card_btn, app_btn;
    private TextView textPaymentAmount;
    private Button done;
    private String selectedFlavor = "";
    private int totalPrice = 4000;
    //private ROSService rosService;
    private int waitingTime = -1;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_e_payment);

        // ROS 서비스 초기화

        //rosService.setROSListener(this);

        // 데이터 수신
        selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        totalPrice = getIntent().getIntExtra("totalPrice", 4000);


        initUI();
    }

    private void initUI() {
        card_btn = findViewById(R.id.card_btn);
        app_btn = findViewById(R.id.app_btn);
        done = findViewById(R.id.done);
        textPaymentAmount = findViewById(R.id.textPaymentAmount);

        textPaymentAmount.setText("결제 금액: " + totalPrice + "원");

        done.setOnClickListener(v -> {
            String paymentData = "결제완료|맛:" + selectedFlavor + "|금액:" + totalPrice;
            Intent intent = new Intent(E_PaymentActivity.this,SurveyPopActivity.class);
            startActivity(intent);
            finish();
            //goToFinalScreen(); //이거 한줄만 추가하면됨!!
        });
    }

//    private void goToFinalScreen() {
//        Intent intent = new Intent(this, FinalActivity.class);
//        intent.putExtra("selectedFlavor", selectedFlavor);
//        intent.putExtra("waitingTime", waitingTime);
//        startActivity(intent);
//    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

    }
}
package com.example.mimokioskapp;

import android.os.Bundle;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

public class FinalActivity extends AppCompatActivity {

    private TextView final_text, order_number;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_final);

        String selectedFlavor = getIntent().getStringExtra("selectedFlavor");

        //로봇에 요청처리 (여기에 실제 로봇 통신 로직 추가)
        if (selectedFlavor !=null){
            sendRequestToRobot(selectedFlavor);
        }

        final_text= findViewById(R.id.final_text);
        order_number = findViewById(R.id.order_number);

        final_text.setText("만드는 중...");
        order_number.setText("주문 번호: 12345");
    }

    private void sendRequestToRobot(String flavor) {
        Toast.makeText(this, "로봇에게 요청:" +flavor, Toast.LENGTH_SHORT).show();
    }
}

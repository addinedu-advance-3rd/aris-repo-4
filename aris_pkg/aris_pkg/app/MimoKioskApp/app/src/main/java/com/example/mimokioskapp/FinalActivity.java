package com.example.mimokioskapp;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

public class FinalActivity extends AppCompatActivity {

    private TextView final_text, order_number, time_text;
    private static int orderCounter =1;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_final);

        long orderEndTime = System.currentTimeMillis();
        new DatabaseHelper(this).calculateOrderDuration(orderEndTime);
        final_text= findViewById(R.id.final_text);
        order_number = findViewById(R.id.order_number);
        time_text = findViewById(R.id.time_text);

        final_text.setText("만드는 중...");
        order_number.setText("주문 번호: 12345");

        String selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        //로봇에서 예상대기시간 받아오기
        int waitingTime = getIntent().getIntExtra("waitingTime",0);

        //대기번호 설정 (주문 순서대로 증가)
        int orderNumber = orderCounter++;
        order_number.setText("대기 번호 : "+orderNumber);

        //예상 대기시간 설정
        time_text.setText("예상 대기 시간: "+waitingTime+"분");

        //일정 시간 후 첫 회면으로 돌아가기
        new Handler().postDelayed(()-> {
            Intent intent = new Intent(FinalActivity.this,A_WaitingActivity.class);
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
            startActivity(intent);
            finish();
        }, 10000);//10초 후 실행

        //주문번호 저장
        SharedPreferences prefs = getSharedPreferences("OrderPrefs", MODE_PRIVATE);
        int savedOrderNumber = prefs.getInt("orderNumber",1);
        orderCounter = savedOrderNumber;


        //로봇에 요청처리 (여기에 실제 로봇 통신 로직 추가)
        if (selectedFlavor !=null){
            sendRequestToRobot(selectedFlavor);
        }

    }

    private void sendRequestToRobot(String flavor) {
        Toast.makeText(this, "로봇에게 요청:" +flavor, Toast.LENGTH_SHORT).show();
    }


}

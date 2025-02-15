package com.example.mimokioskapp;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Handler;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

public class ClawActivity extends AppCompatActivity {
    private ROSService rosService;

    private Button start_btn;
    private TextView recognized_text;
    private ImageView iv_1,iv_2;
    private LinearLayout both_hand;
    private CountDownTimer countDownTimer;
    private String savedIp;

    @SuppressLint("WrongViewCast")
    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_claw_machine);
        SharedPreferences prefs = getSharedPreferences("AppPrefs", MODE_PRIVATE);
        savedIp = prefs.getString("saved_ip_address", "192.168.0.37"); // 기본값 설정
        // ROSService 초기화 (실제 IP 주소로 변경 필요)
        rosService = new ROSService(savedIp,7777, this);

        start_btn = findViewById(R.id.start_btn);
        recognized_text = findViewById(R.id.recognized_text);
        iv_1 = findViewById(R.id.iv_1);
        iv_2 = findViewById(R.id.iv_2);
        both_hand = findViewById(R.id.both_hand);
        start_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                start_btn.setVisibility(View.GONE);
                recognized_text.setText("본 인형뽑기는 손짓을 인식하여 동작합니다. 방법을 잘 듣고 시작하시길 바랍니다.");
                iv_1.setVisibility(View.VISIBLE);

                new Handler().postDelayed(() -> {
                    iv_1.setVisibility(View.GONE);
                    both_hand.setVisibility(View.VISIBLE);
                }, 5000);

                new Handler().postDelayed(() -> {
                    startCountdown1(5);
                }, 15000);

                new Handler().postDelayed(() -> {
                    both_hand.setVisibility(View.GONE);
                    startCountdown2(30); // 30초 카운트다운 시작

                    // ⚠ ROSService가 null이 아닌지 확인 후 전송
                    if (rosService != null) {
                        rosService.sendDataToROS2("True");
                    } else {
                        recognized_text.setText("ROS 연결 실패!");
                    }
                }, 20000);
                new Handler().postDelayed(() -> {
                    if (rosService != null) {
                        rosService.sendDataToROS2("False");
                    } else {
                        recognized_text.setText("ROS 연결 실패!");
                    }
                    // ⚠ rosService가 null이 아닐 때만 닫기
                    if (rosService != null) {
                        rosService.closeConnection();
                    }
                    goToNextScreen();
                }, 50000);

            }


        });
    }
    private void startCountdown1(int durationSeconds) {
        countDownTimer = new CountDownTimer(durationSeconds * 1000L, 1000) {
            @Override
            public void onTick(long millisUntilFinished) {
                int secondsRemaining = (int) (millisUntilFinished / 1000);
                int minute = secondsRemaining / 60;
                int second = secondsRemaining % 60;
                String formattedTime = String.format("%01d:%02d", minute, second);
                recognized_text.setText(formattedTime+"초 뒤에 시작합니다!");
            }

            @Override
            public void onFinish() {
                recognized_text.setText("게임 시작!");

            }
        }.start();
    }
    private void startCountdown2(int durationSeconds) {
        countDownTimer = new CountDownTimer(durationSeconds * 1000L, 1000) {
            @Override
            public void onTick(long millisUntilFinished) {
                int secondsRemaining = (int) (millisUntilFinished / 1000);
                int minute = secondsRemaining / 60;
                int second = secondsRemaining % 60;
                String formattedTime = String.format("%01d:%02d", minute, second);
                recognized_text.setText("남은 시간: " + formattedTime);
            }

            @Override
            public void onFinish() {
                recognized_text.setText("시간 종료!");
            }
        }.start();
    }
    private void goToNextScreen() {
        Intent intent = new Intent(ClawActivity.this, A_WaitingActivity.class);
        startActivity(intent);
        finish();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        // 타이머 취소
        if (countDownTimer != null) {
            countDownTimer.cancel();
        }

        // ⚠ rosService 종료 전에 null 체크
        if (rosService != null) {
            rosService.closeConnection();
        }
    }
}

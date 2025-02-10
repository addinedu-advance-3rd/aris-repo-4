package com.example.mimokioskapp;

import android.app.Activity;
import android.content.Intent;
import android.media.MediaPlayer;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.IdRes;
import androidx.annotation.Nullable;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;

public class E_PaymentActivity extends Activity {

    private static final String CLIENT_ID = "a4zgp0vwh8"; // 네이버 클라이언트 ID
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW"; // 네이버 클라이언트 Secret


    private RadioGroup rg;
    private RadioButton card_btn, app_btn;
    private TextView textPaymentAmount, tv_notice;
    private Button done;
    private String selectedFlavor = "";
    private int totalPrice = 4000;

    private MediaPlayer mediaPlayer;

    //private ROSService rosService;
    private int waitingTime = -1;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_e_payment);
        card_btn = findViewById(R.id.card_btn);
        app_btn = findViewById(R.id.app_btn);

        rg =findViewById(R.id.rg);
        rg.setOnCheckedChangeListener(radioGroupButtonChangeListener);


        // ROS 서비스 초기화

        //rosService.setROSListener(this);

        // 데이터 수신
        selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        totalPrice = getIntent().getIntExtra("totalPrice", 4000);


        initUI();
    }

    RadioGroup.OnCheckedChangeListener radioGroupButtonChangeListener = new RadioGroup.OnCheckedChangeListener() {
        @Override
        public void onCheckedChanged(RadioGroup group, @IdRes int i) {
            if(i==R.id.app_btn){
                Toast.makeText(E_PaymentActivity.this, "앱으로 결제합니다.",Toast.LENGTH_SHORT).show();
                new TTSAsyncTask().execute("앱으로 결제합니다");
            }
            else if (i==R.id.card_btn){
                Toast.makeText(E_PaymentActivity.this, "카드로 결제합니다.",Toast.LENGTH_SHORT).show();
                new TTSAsyncTask().execute("카드로 결제합니다");

            }
        }
    };

    private class TTSAsyncTask extends AsyncTask<String, Void, String> {
        @Override
        protected String doInBackground(String... params) {
            try {
                String text = URLEncoder.encode(params[0], "UTF-8"); // 텍스트 인코딩
                String apiURL = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts";
                URL url = new URL(apiURL);
                HttpURLConnection con = (HttpURLConnection) url.openConnection();
                con.setRequestMethod("POST");
                con.setRequestProperty("X-NCP-APIGW-API-KEY-ID", CLIENT_ID);
                con.setRequestProperty("X-NCP-APIGW-API-KEY", CLIENT_SECRET);

                // POST 요청 파라미터
                String postParams = "speaker=nara&volume=0&speed=0&pitch=0&format=mp3&text=" + text;
                con.setDoOutput(true);
                DataOutputStream wr = new DataOutputStream(con.getOutputStream());
                wr.writeBytes(postParams);
                wr.flush();
                wr.close();

                int responseCode = con.getResponseCode();
                BufferedReader br;
                if (responseCode == 200) { // 정상 호출
                    InputStream is = con.getInputStream();
                    int read;
                    byte[] bytes = new byte[1024];

                    // 랜덤한 이름으로 mp3 파일 생성
                    String tempname = String.valueOf(System.currentTimeMillis());
                    File file = new File(getExternalFilesDir(null), tempname + ".mp3");
                    file.createNewFile();
                    OutputStream outputStream = new FileOutputStream(file);
                    while ((read = is.read(bytes)) != -1) {
                        outputStream.write(bytes, 0, read);
                    }
                    is.close();
                    outputStream.close();

                    return file.getAbsolutePath(); // 파일 경로 반환
                } else {  // 오류 발생
                    br = new BufferedReader(new InputStreamReader(con.getErrorStream()));
                    StringBuilder response = new StringBuilder();
                    String inputLine;
                    while ((inputLine = br.readLine()) != null) {
                        response.append(inputLine);
                    }
                    br.close();
                    return response.toString();
                }
            } catch (IOException e) {
                e.printStackTrace();
                return e.getMessage();
            }
        }

        @Override
        protected void onPostExecute(String result) {
            super.onPostExecute(result);
            stopTTS();

            // 결과 처리
            if (result.endsWith(".mp3")) {
                stopTTS();
                // MP3 파일이 생성되었으므로 이를 재생하는 코드 추가
                mediaPlayer = new MediaPlayer();
                try {
                    mediaPlayer.setDataSource(result); // 생성된 MP3 파일 경로를 전달
                    mediaPlayer.prepare(); // 파일 준비
                    mediaPlayer.start(); // 음성 재생 시작
                    mediaPlayer.setOnCompletionListener(mp -> {
                        mp.release();
                        mediaPlayer = null;
                    });
                } catch (IOException e) {
                    e.printStackTrace();
                    Toast.makeText(E_PaymentActivity.this, "음성 파일 재생 오류: " + e.getMessage(), Toast.LENGTH_LONG).show();
                }
            } else {
                // 오류 메시지 처리
                Toast.makeText(E_PaymentActivity.this, "TTS 오류: " + result, Toast.LENGTH_LONG).show();
            }
        }
    }

    private void initUI() {

        done = findViewById(R.id.done);
        textPaymentAmount = findViewById(R.id.textPaymentAmount);
        tv_notice = findViewById(R.id.tv_notice);



        textPaymentAmount.setText("결제 금액: " + totalPrice + "원");

        done.setOnClickListener(v -> {
            if(card_btn.isChecked()|app_btn.isChecked()) {
                String paymentData = "결제완료|맛:" + selectedFlavor + "|금액:" + totalPrice;
                Intent intent = new Intent(E_PaymentActivity.this, SurveyPopActivity.class);
                startActivity(intent);
                finish();
                //goToFinalScreen(); //이거 한줄만 추가하면됨!!
            }
            else
                tv_notice.setVisibility(View.VISIBLE);
        });
    }
    private void stopTTS(){
        if(mediaPlayer!=null){
            if (mediaPlayer.isPlaying()){
                mediaPlayer.stop();
            }
            mediaPlayer.release();
            mediaPlayer=null;
        }
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
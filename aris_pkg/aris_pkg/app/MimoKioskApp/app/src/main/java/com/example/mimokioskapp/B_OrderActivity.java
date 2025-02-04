package com.example.mimokioskapp;

import android.content.Intent;
import android.media.MediaPlayer;
import android.os.AsyncTask;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.appcompat.app.AppCompatActivity;

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
import java.util.ArrayList;
import java.util.Locale;

public class B_OrderActivity extends AppCompatActivity {

    private static final String TAG = "B_OrderActivity"; // 로그 태그
    private static final String CLIENT_ID = "a4zgp0vwh8"; // 클라이언트 ID
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW"; // 클라이언트 시크릿

    private String languageMode = "";
    private ToggleButton language_btn;
    private Button next_btn;
    private TextView order_text;
    private SpeechRecognizer speechRecognizer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_b_order);

        language_btn = findViewById(R.id.language_btn);
        order_text = findViewById(R.id.order_text);
        next_btn = findViewById(R.id.next_btn);

        // Intent로 전달된 언어 정보 가져오기
        languageMode = getIntent().getStringExtra("languageMode");
        if (languageMode == null) {
            languageMode = "ko"; // 기본값 설정
        }

        // TTS로 안내 메시지 출력
        new TTSAsyncTask().execute(languageMode.equals("ko") ? "주문하시겠습니까?" : "Would you like to order?");

        // 언어 설정 버튼 클릭 리스너
        language_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                boolean checked = ((ToggleButton) v).isChecked();
                if (checked) {
                    language_btn.setBackgroundDrawable(getResources().getDrawable(R.drawable.eng_img));
                    languageMode = "eng";
                    order_text.setText("Let me help you with your order.");
                    next_btn.setText("next");
                } else {
                    language_btn.setBackgroundDrawable(getResources().getDrawable(R.drawable.kor_img));
                    languageMode = "ko";
                    order_text.setText("주문을 도와드릴게요.");
                    next_btn.setText("다음");
                }
                // 언어 변경 시 TTS로 안내 메시지 출력
                new TTSAsyncTask().execute(languageMode.equals("ko") ? "언어가 한국어로 설정되었습니다." : "Language has been set to English.");
            }
        });

        // 다음 버튼 클릭 리스너
        next_btn.setOnClickListener(v -> {
            new TTSAsyncTask().execute(languageMode.equals("ko") ? "주문을 도와드릴게요." : "Let me help you with your order.");
            goToNextScreen();
        });

        // 음성 인식 초기화
        startListeningForResponse();
    }

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

            // 결과 처리
            if (result.endsWith(".mp3")) {
                // MP3 파일이 생성되었으므로 이를 재생하는 코드 추가
                MediaPlayer mediaPlayer = new MediaPlayer();
                try {
                    mediaPlayer.setDataSource(result); // 생성된 MP3 파일 경로를 전달
                    mediaPlayer.prepare(); // 파일 준비
                    mediaPlayer.start(); // 음성 재생 시작

                    Toast.makeText(B_OrderActivity.this, "음성 파일이 생성되었고 재생되었습니다: " + result, Toast.LENGTH_LONG).show();
                } catch (IOException e) {
                    e.printStackTrace();
                    Toast.makeText(B_OrderActivity.this, "음성 파일 재생 오류: " + e.getMessage(), Toast.LENGTH_LONG).show();
                }
            } else {
                // 오류 메시지 처리
                Toast.makeText(B_OrderActivity.this, "TTS 오류: " + result, Toast.LENGTH_LONG).show();
            }
        }
    }

    private void startListeningForResponse() {
        if (speechRecognizer == null) {
            speechRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
            speechRecognizer.setRecognitionListener(new RecognitionListener() {
                @Override
                public void onReadyForSpeech(Bundle params) {
                    Log.d(TAG, "음성 인식 준비 완료");
                }

                @Override
                public void onBeginningOfSpeech() {
                    Log.d(TAG, "음성 인식 시작");
                }

                @Override
                public void onRmsChanged(float rmsdB) {
                }

                @Override
                public void onBufferReceived(byte[] buffer) {
                }

                @Override
                public void onEndOfSpeech() {
                    Log.d(TAG, "음성 인식 종료");
                }

                @Override
                public void onError(int error) {
                    Log.e(TAG, "음성 인식 오류: " + error);
                    new android.os.Handler().postDelayed(() -> startListeningForResponse(), 1000); // 1초 후 다시 시도
                }

                @Override
                public void onResults(Bundle results) {
                    ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                    if (matches != null && !matches.isEmpty()) {
                        String recognizedText = matches.get(0);
                        Log.d(TAG, "인식된 텍스트: " + recognizedText);

                        // "네" 또는 "예"를 인식한 경우
                        if (recognizedText.contains("네") || recognizedText.contains("예") || recognizedText.contains("yes") || recognizedText.contains("yeah")) {
                            new TTSAsyncTask().execute(languageMode.equals("ko") ? "주문을 도와드릴게요." : "Let me help you with your order.");
                            goToNextScreen();
                        }
                        // "아니오" 또는 "no"를 인식한 경우
                        else if (recognizedText.contains("아니오") || recognizedText.contains("no")) {
                            new TTSAsyncTask().execute(languageMode.equals("ko") ? "처음 화면으로 돌아갑니다." : "Returning to the main screen.");
                            finish(); // 현재 액티비티 종료
                        }
                    }
                    new android.os.Handler().postDelayed(() -> startListeningForResponse(), 1000); // 1초 후 다시 음성 인식 시작
                }

                @Override
                public void onPartialResults(Bundle partialResults) {
                }

                @Override
                public void onEvent(int eventType, Bundle params) {
                }
            });
        }

        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, languageMode.equals("eng") ? Locale.ENGLISH : Locale.KOREAN);
        intent.putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
        speechRecognizer.startListening(intent);
    }

    private void goToNextScreen() {
        // 선택된 언어로 다음 화면으로 이동
        Intent intent = new Intent(B_OrderActivity.this, C_CupConeActivity.class);
        intent.putExtra("languageMode", languageMode);
        startActivity(intent);
        finish();
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (speechRecognizer != null) {
            speechRecognizer.destroy();
            speechRecognizer = null;
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        startListeningForResponse(); // 앱이 다시 활성화되면 음성 인식 다시 시작
    }
}
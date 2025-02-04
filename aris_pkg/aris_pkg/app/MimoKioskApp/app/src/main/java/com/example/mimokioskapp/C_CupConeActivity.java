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
import android.widget.ImageButton;
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

public class C_CupConeActivity extends AppCompatActivity {

    private static final String TAG = "C_CupConeActivity"; // 로그 태그
    private static final String CLIENT_ID = "a4zgp0vwh8"; // 네이버 클라이언트 ID
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW"; // 네이버 클라이언트 Secret

    private String languageMode = "";
    private String selectedFlavor = "";
    private String cupcone = "";
    private ToggleButton language_btn;
    private ImageButton cup_btn, cone_btn;
    private TextView order_text;
    private SpeechRecognizer speechRecognizer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_cupcone);

        cup_btn = findViewById(R.id.cup_btn);
        cone_btn = findViewById(R.id.cone_btn);
        language_btn = findViewById(R.id.language_btn);
        order_text = findViewById(R.id.order_text);

        // Intent로 전달된 데이터 가져오기
        selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        languageMode = getIntent().getStringExtra("languageMode");

        if (selectedFlavor != null) {
            Toast.makeText(this, "선택된 맛: " + selectedFlavor, Toast.LENGTH_SHORT).show();
        }

        // 언어 설정 버튼 클릭 리스너
        language_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                boolean checked = ((ToggleButton) v).isChecked();
                if (checked) {
                    language_btn.setBackgroundDrawable(getResources().getDrawable(R.drawable.eng_img));
                    languageMode = "eng";
                    order_text.setText("Please choose between a cup and a cone.");
                } else {
                    language_btn.setBackgroundDrawable(getResources().getDrawable(R.drawable.kor_img));
                    languageMode = "ko";
                    order_text.setText("컵과 콘 중에서 하나만 선택해주세요.");
                }
                // 언어 변경 시 TTS로 안내 메시지 출력
                new TTSAsyncTask().execute(languageMode.equals("ko") ? "언어가 한국어로 설정되었습니다." : "Language has been set to English.");
            }
        });

        // TTS로 안내 메시지 출력
        if ("ko".equals(languageMode)) {
            new TTSAsyncTask().execute("컵과 콘 중에서 하나만 선택해주세요.");
        } else if ("eng".equals(languageMode)) {
            new TTSAsyncTask().execute("Please choose between a cup and a cone.");
        }

        // 버튼 클릭 리스너
        cup_btn.setOnClickListener(v -> {
            cupcone = "cup";
            selectCup();
        });

        cone_btn.setOnClickListener(v -> {
            cupcone = "cone";
            selectCone();
        });

        // 음성 인식 시작
        startListeningForCupConeSelection();
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

                    Toast.makeText(C_CupConeActivity.this, "음성 파일이 생성되었고 재생되었습니다: " + result, Toast.LENGTH_LONG).show();
                } catch (IOException e) {
                    e.printStackTrace();
                    Toast.makeText(C_CupConeActivity.this, "음성 파일 재생 오류: " + e.getMessage(), Toast.LENGTH_LONG).show();
                }
            } else {
                // 오류 메시지 처리
                Toast.makeText(C_CupConeActivity.this, "TTS 오류: " + result, Toast.LENGTH_LONG).show();
            }
        }
    }

    private void startListeningForCupConeSelection() {
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
                    new android.os.Handler().postDelayed(() -> startListeningForCupConeSelection(), 1000); // 1초 후 다시 시도
                }

                @Override
                public void onResults(Bundle results) {
                    ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                    if (matches != null && !matches.isEmpty()) {
                        String recognizedText = matches.get(0);
                        Log.d(TAG, "인식된 텍스트: " + recognizedText);

                        // "컵" 또는 "콘"을 인식한 경우
                        if (recognizedText.contains("컵") || recognizedText.contains("cup")) {
                            selectCup();
                        } else if (recognizedText.contains("콘") || recognizedText.contains("cone")) {
                            selectCone();
                        } else {
                            Toast.makeText(C_CupConeActivity.this, "인식하지 못했습니다. 다시 시도해주세요.", Toast.LENGTH_LONG).show();
                            startListeningForCupConeSelection(); // 재시도
                        }
                    }
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

    private void selectCup() {
        cupcone = "cup";
        new TTSAsyncTask().execute(languageMode.equals("ko") ? "컵으로 선택하셨습니다." : "You chose the cup.");
        goToNextScreen();
    }

    private void selectCone() {
        cupcone = "cone";
        new TTSAsyncTask().execute(languageMode.equals("ko") ? "콘으로 선택하셨습니다." : "You chose the cone.");
        goToNextScreen();
    }

    private void goToNextScreen() {
        Intent intent = new Intent(C_CupConeActivity.this, C_FlavorActivity.class);
        intent.putExtra("languageMode", languageMode);
        intent.putExtra("cupcone", cupcone);
        startActivity(intent);
        finish();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (speechRecognizer != null) {
            speechRecognizer.destroy(); // 리소스 해제
        }
    }
}
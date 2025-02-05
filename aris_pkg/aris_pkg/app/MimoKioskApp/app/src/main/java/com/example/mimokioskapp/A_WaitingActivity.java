package com.example.mimokioskapp;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
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
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

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
import java.net.Socket;
import java.net.URL;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class A_WaitingActivity extends Activity {
    private static final String ROS2_IP = "192.168.0.37";
    private static final int ROS2_PORT_VOICE = 8888;
    private Socket socket;
    private OutputStream outputStream;
    private InputStream inputStream;
    private static final String TAG = "A_WaitingActivity"; // 로그 태그
    private static final String CLIENT_ID = "a4zgp0vwh8"; // 네이버 클라이언트 ID
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW"; // 네이버 클라이언트 Secret
    private static final String ADMIN_PASSWORD = "1234";

    private String languageMode = "ko";
    private static final String ORDER_KEYWORD_KR = "주문";
    private static final String ORDER_KEYWORD_EN = "order";
    private ToggleButton language_btn;
    private FrameLayout touch_area;
    private TextView waiting_text;
    private TextView recognized_text;
    private ExecutorService executor;
    private boolean isListening = false;

    private SpeechRecognizer speechRecognizer;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_a_waiting);

        executor = Executors.newFixedThreadPool(2);

        touch_area = findViewById(R.id.touch_area);
        language_btn = findViewById(R.id.language_btn);
        waiting_text = findViewById(R.id.waiting_text);
        recognized_text = findViewById(R.id.recognized_text);
        Button voiceServiceButton = findViewById(R.id.voice_service_btn);
        voiceServiceButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showVoiceServiceDialog();
            }
        });

        // TTS 초기화 및 환영 메시지 출력
        new TTSAsyncTask().execute(languageMode.equals("ko") ? "어서 오세요. 주문을 시작하려면 화면을 터치하거나 음성으로 말씀해주세요." : "Welcome. Please touch the screen or speak to start your order.");

        // 언어 설정 버튼 클릭 리스너
        language_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                boolean checked = ((ToggleButton) v).isChecked();
                if (checked) {
                    language_btn.setBackgroundDrawable(getResources().getDrawable(R.drawable.eng_img));
                    languageMode = "eng";
                    waiting_text.setText("Please touch the screen or speak by voice to start your order");
                } else {
                    language_btn.setBackgroundDrawable(getResources().getDrawable(R.drawable.kor_img));
                    languageMode = "ko";
                    waiting_text.setText("주문을 시작하려면 화면을 터치하거나 음성으로 말씀해주세요");
                }
                // 언어 변경 시 TTS로 안내 메시지 출력
                new TTSAsyncTask().execute(languageMode.equals("ko") ? "언어가 한국어로 설정되었습니다." : "Language has been set to English.");
            }
        });

        // 음성 인식 시작
        startListeningForOrder();

        // 터치 영역 클릭 리스너
        touch_area.setOnClickListener(v -> goToNextScreen());
    }

    private void showVoiceServiceDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("음성 서비스로 주문하시겠습니까?")
                .setPositiveButton("확인", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        String data="True";
                        executor.execute(() -> {
                            try {
                                socket = new Socket(ROS2_IP, ROS2_PORT_VOICE);
                                outputStream = socket.getOutputStream();
                                inputStream = socket.getInputStream();
                                if (outputStream != null) {
                                    outputStream.write(data.getBytes());
                                    outputStream.flush();
                                    Log.d(TAG, "전송 성공: " + data);
                                }
                                if (outputStream != null) outputStream.close();
                                if (inputStream != null) inputStream.close();
                                if (socket != null) socket.close();
                                executor.shutdown();
                            } catch (IOException e) {
                                Log.e(TAG, "ROS 연결 실패: " + e.getMessage());
                            }
                        });
                        Intent intent = new Intent(A_WaitingActivity.this, FinalActivity.class);
                        startActivity(intent);
                    }
                })
                .setNegativeButton("취소", null)
                .show();
    }
    private class TTSAsyncTask extends AsyncTask<String, Void, String> {
        @Override
        protected String doInBackground(String... params) {
            try {
                String text = URLEncoder.encode(params[0], "UTF-8");
                String apiURL = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts";
                URL url = new URL(apiURL);
                HttpURLConnection con = (HttpURLConnection) url.openConnection();
                con.setRequestMethod("POST");
                con.setRequestProperty("X-NCP-APIGW-API-KEY-ID", CLIENT_ID);
                con.setRequestProperty("X-NCP-APIGW-API-KEY", CLIENT_SECRET);

                String postParams = "speaker=nara&volume=0&speed=0&pitch=0&format=mp3&text=" + text;
                con.setDoOutput(true);
                DataOutputStream wr = new DataOutputStream(con.getOutputStream());
                wr.writeBytes(postParams);
                wr.flush();
                wr.close();

                int responseCode = con.getResponseCode();
                if (responseCode == 200) {
                    InputStream is = con.getInputStream();
                    int read;
                    byte[] bytes = new byte[1024];

                    String tempname = String.valueOf(System.currentTimeMillis());
                    File file = new File(getExternalFilesDir(null), tempname + ".mp3");
                    file.createNewFile();
                    OutputStream outputStream = new FileOutputStream(file);
                    while ((read = is.read(bytes)) != -1) {
                        outputStream.write(bytes, 0, read);
                    }
                    is.close();
                    outputStream.close();

                    return file.getAbsolutePath();
                } else {
                    BufferedReader br = new BufferedReader(new InputStreamReader(con.getErrorStream()));
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
            if (result.endsWith(".mp3")) {
                MediaPlayer mediaPlayer = new MediaPlayer();
                try {
                    mediaPlayer.setDataSource(result);
                    mediaPlayer.prepare();
                    mediaPlayer.start();
                } catch (IOException e) {
                    e.printStackTrace();
                    Toast.makeText(A_WaitingActivity.this, "TTS 재생 오류: " + e.getMessage(), Toast.LENGTH_SHORT).show();
                }
            } else {
                Toast.makeText(A_WaitingActivity.this, "TTS 오류: " + result, Toast.LENGTH_LONG).show();
            }
        }
    }

    private void startListeningForOrder() {
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
                    isListening = false;
                }

                @Override
                public void onError(int error) {
                    Log.e(TAG, "음성 인식 오류: " + error);
                    isListening = false;
                    new android.os.Handler().postDelayed(() -> startListeningForOrder(), 1000); // 1초 후 다시 실행
                }

                @Override
                public void onResults(Bundle results) {
                    ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
                    if (matches != null && !matches.isEmpty()) {
                        String recognizedText = matches.get(0);
                        recognized_text.setText(recognizedText);
                        for (String result : matches) {
                            String normalizedResult = result.replaceAll("\\s", "").toLowerCase();
                            // 다양한 유사 단어 추가
                            if (normalizedResult.contains("주문") || normalizedResult.contains("주문이요") || normalizedResult.contains("주문할게요") || normalizedResult.contains("오더") || normalizedResult.contains("order") || normalizedResult.contains("i want to order")) {
                                goToNextScreen();
                                return;
                            }
                        }
                    }
                    new android.os.Handler().postDelayed(() -> startListeningForOrder(), 1000);
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
    // "관리자 모드" 버튼 클릭 시 실행되는 메서드
    public void showAdminPasswordDialog(View view) {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("비밀번호를 입력하세요");

        // 비밀번호 입력 필드 추가
        final EditText input = new EditText(this);
        builder.setView(input);

        builder.setPositiveButton("확인", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                String enteredPassword = input.getText().toString();
                if (enteredPassword.equals(ADMIN_PASSWORD)) {
                    // 비밀번호가 맞으면 관리자 화면으로 이동
                    Intent intent = new Intent(A_WaitingActivity.this, AdminActivity.class);
                    startActivity(intent);
                } else {
                    // 비밀번호가 틀리면 경고 메시지 표시
                    Toast.makeText(A_WaitingActivity.this, "비밀번호가 틀렸습니다.", Toast.LENGTH_SHORT).show();
                }
            }
        });

        builder.setNegativeButton("취소", null);
        builder.show();
    }
    private void goToNextScreen() {
        Intent intent = new Intent(A_WaitingActivity.this, B_OrderActivity.class);
        intent.putExtra("languageMode", languageMode);
        startActivity(intent);
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
        startListeningForOrder(); // 앱이 다시 활성화되면 음성 인식 다시 시작
    }
}
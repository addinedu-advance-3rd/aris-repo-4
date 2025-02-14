package com.example.mimokioskapp;

import android.content.Intent;
import android.media.MediaPlayer;
import android.os.AsyncTask;
import android.os.Bundle;
import android.speech.RecognitionListener;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
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

public class C_FlavorActivity extends AppCompatActivity {

    private static final String TAG = "C_FlavorActivity"; // 로그 태그
    private static final String CLIENT_ID = "a4zgp0vwh8";
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW";

    private String languageMode = "ko";
    private String selectedFlavor = "";
    private String cupcone = "";
    private ToggleButton language_btn;
    private ImageButton strawberry_btn, blueberry_btn;
    private TextView order_text;
    private SpeechRecognizer speechRecognizer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_c_flavor);

        initializeViews();
        setupIntentData();
        setupLanguageButton();
        setupFlavorButtons();
        playInitialTTS();
        startVoiceRecognition();
    }

    private void initializeViews() {
        strawberry_btn = findViewById(R.id.strawberry_btn);
        blueberry_btn = findViewById(R.id.blueberry_btn);
        language_btn = findViewById(R.id.language_btn);
        order_text = findViewById(R.id.order_text);
    }

    private void setupIntentData() {
        cupcone = getIntent().getStringExtra("cupcone");
        languageMode = getIntent().getStringExtra("languageMode");
        if (cupcone != null) {
            Toast.makeText(this, "선택된 종류: " + cupcone, Toast.LENGTH_SHORT).show();
        }
    }

    private void setupLanguageButton() {
        language_btn.setOnClickListener(v -> {
            boolean checked = ((ToggleButton) v).isChecked();
            languageMode = checked ? "eng" : "ko";
            updateUIForLanguage(checked);
            new TTSAsyncTask().execute(getLanguageChangeMessage());
        });
    }

    private void updateUIForLanguage(boolean isEnglish) {
        language_btn.setBackgroundDrawable(getResources().getDrawable(
                isEnglish ? R.drawable.eng_img : R.drawable.kor_img
        ));
        order_text.setText(isEnglish ?
                "Please choose the flavor you want" : "원하는 맛을 선택해주세요");
    }

    private String getLanguageChangeMessage() {
        return languageMode.equals("ko") ?
                "언어가 한국어로 설정되었습니다." : "Language has been set to English.";
    }

    private void setupFlavorButtons() {
        strawberry_btn.setOnClickListener(v -> handleFlavorSelection("strawberry"));
        blueberry_btn.setOnClickListener(v -> handleFlavorSelection("blueberry"));
    }

    private void handleFlavorSelection(String flavor) {
        selectedFlavor = flavor;
        playSelectionConfirmationTTS();
        goToNextScreen();
    }

    private void playInitialTTS() {
        String message = languageMode.equals("ko") ?
                "무슨 맛으로 선택하시겠어요? 딸기 맛과 블루베리 맛이 있어요." :
                "What flavor would you like to have? We have strawberry and blueberry flavors.";
        new TTSAsyncTask().execute(message);
    }

    private void playSelectionConfirmationTTS() {
        String message = languageMode.equals("ko") ?
                (selectedFlavor.equals("strawberry") ? "딸기맛으로 선택하셨습니다." : "블루베리맛으로 선택하셨습니다.") :
                (selectedFlavor.equals("strawberry") ? "You have chosen the strawberry flavor." : "You have chosen the blueberry flavor.");
        new TTSAsyncTask().execute(message);
    }

    private class TTSAsyncTask extends AsyncTask<String, Void, String> {
        @Override
        protected String doInBackground(String... params) {
            try {
                String text = URLEncoder.encode(params[0], "UTF-8");
                HttpURLConnection con = createTTSConnection(text);
                return processTTSResponse(con);
            } catch (IOException e) {
                return e.getMessage();
            }
        }

        private HttpURLConnection createTTSConnection(String text) throws IOException {
            URL url = new URL("https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts");
            HttpURLConnection con = (HttpURLConnection) url.openConnection();
            con.setRequestMethod("POST");
            con.setRequestProperty("X-NCP-APIGW-API-KEY-ID", CLIENT_ID);
            con.setRequestProperty("X-NCP-APIGW-API-KEY", CLIENT_SECRET);

            DataOutputStream wr = new DataOutputStream(con.getOutputStream());
            wr.writeBytes("speaker=nara&volume=0&speed=0&pitch=0&format=mp3&text=" + text);
            wr.flush();
            wr.close();
            return con;
        }

        private String processTTSResponse(HttpURLConnection con) throws IOException {
            if (con.getResponseCode() == 200) {
                return saveAudioFile(con.getInputStream());
            } else {
                return readErrorStream(con);
            }
        }

        private String saveAudioFile(InputStream is) throws IOException {
            File file = File.createTempFile("tts_", ".mp3", getExternalFilesDir(null));
            try (OutputStream os = new FileOutputStream(file)) {
                byte[] buffer = new byte[1024];
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    os.write(buffer, 0, bytesRead);
                }
            }
            return file.getAbsolutePath();
        }

        private String readErrorStream(HttpURLConnection con) throws IOException {
            BufferedReader br = new BufferedReader(new InputStreamReader(con.getErrorStream()));
            StringBuilder response = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) response.append(line);
            br.close();
            return response.toString();
        }

        @Override
        protected void onPostExecute(String result) {
            handleTTSResult(result);
        }
    }

    private void handleTTSResult(String result) {
        if (result.endsWith(".mp3")) {
            playAudioFile(result);
        } else {
            showErrorToast(result);
        }
    }

    private void playAudioFile(String path) {
        try {
            MediaPlayer mediaPlayer = new MediaPlayer();
            mediaPlayer.setDataSource(path);
            mediaPlayer.prepare();
            mediaPlayer.start();
        } catch (IOException e) {
            showErrorToast("음성 재생 오류: " + e.getMessage());
        }
    }

    private void showErrorToast(String message) {
        Toast.makeText(this, "TTS 오류: " + message, Toast.LENGTH_LONG).show();
    }

    private void startVoiceRecognition() {
        speechRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
        speechRecognizer.setRecognitionListener(new RecognitionListener() {
            @Override
            public void onReadyForSpeech(Bundle params) {
                Log.d(TAG, "음성 인식 준비 완료");
            }

            @Override
            public void onBeginningOfSpeech() {
                Log.d(TAG, "음성 입력 시작");
            }

            @Override
            public void onRmsChanged(float rmsdB) {

            }

            @Override
            public void onBufferReceived(byte[] buffer) {

            }

            @Override
            public void onEndOfSpeech() {

            }

            @Override
            public void onResults(Bundle results) {
                processRecognitionResults(results);
            }

            @Override
            public void onPartialResults(Bundle partialResults) {

            }

            @Override
            public void onEvent(int eventType, Bundle params) {

            }

            @Override
            public void onError(int error) {
                handleRecognitionError(error);
            }

            // 나머지 오버라이드 메서드 생략...
        });
        startRecognition();
    }

    private void processRecognitionResults(Bundle results) {
        ArrayList<String> matches = results.getStringArrayList(SpeechRecognizer.RESULTS_RECOGNITION);
        if (matches != null && !matches.isEmpty()) {
            String text = matches.get(0).toLowerCase();
            if (text.contains("딸기") || text.contains("strawberry")) {
                handleFlavorSelection("strawberry");
            } else if (text.contains("블루베리") || text.contains("blueberry")) {
                handleFlavorSelection("blueberry");
            } else {
                Toast.makeText(this, getErrorMessage(), Toast.LENGTH_LONG).show();
            }
        }
        startRecognition();
    }

    private String getErrorMessage() {
        return languageMode.equals("ko") ?
                "맛을 인식하지 못했습니다. 다시 시도해주세요." :
                "Flavor not recognized. Please try again.";
    }

    private void handleRecognitionError(int error) {
        Log.e(TAG, "음성 인식 오류: " + error);
        new android.os.Handler().postDelayed(this::startRecognition, 1000);
    }

    private void startRecognition() {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH)
                .putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                .putExtra(RecognizerIntent.EXTRA_LANGUAGE, getLanguageCode())
                .putExtra(RecognizerIntent.EXTRA_CALLING_PACKAGE, getPackageName());
        speechRecognizer.startListening(intent);
    }

    private String getLanguageCode() {
        return languageMode.equals("eng") ? Locale.ENGLISH.toString() : Locale.KOREAN.toString();
    }

    private void goToNextScreen() {
        startActivity(new Intent(this, D_ToppingActivity.class)
                .putExtra("selectedFlavor", selectedFlavor)
                .putExtra("languageMode", languageMode)
                .putExtra("cupcone", cupcone));
        finish();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (speechRecognizer != null) {
            speechRecognizer.destroy();
        }
    }
}
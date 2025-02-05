package com.example.mimokioskapp;

import android.content.ContentValues;
import android.content.Intent;
import android.database.sqlite.SQLiteDatabase;
import android.media.MediaPlayer;
import android.os.AsyncTask;
import android.os.Bundle;
import android.speech.RecognizerIntent;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.Locale;

public class D_ToppingActivity extends AppCompatActivity {

    private static final String TAG = "D_ToppingActivity";
    private static final String CLIENT_ID = "a4zgp0vwh8";
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW";

    // UI Components
    private CheckBox checkTopping1, checkTopping2, checkTopping3;
    private RadioButton yes_btn, none_btn;
    private Button topping_btn;
    private LinearLayout topping_ly;
    private RadioGroup rg;
    private TextView text_price;

    // Business Logic
    private String languageMode = "ko";
    private String selectedFlavor = "";
    private String cupcone = "";
    private int basePrice = 4000;
    private int toppingPrice = 300;
    private int totalPrice = 4000;

    // Services
    private ROSService rosService;
    private DatabaseHelper dbHelper;
    private SpeechRecognizer speechRecognizer;
    private MediaPlayer mediaPlayer; // MediaPlayer 추가

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_d_topping);
        initializeComponents();
        setupIntentData();
        setupUIListeners();
        initializeServices();
        //startVoiceRecognition();
        playInitialTTS();
    }

    // [추가] 데이터베이스 연동 메서드
    private void saveOrderToDatabase(String flavor, String toppings) {
        SQLiteDatabase db = dbHelper.getWritableDatabase();
        ContentValues values = new ContentValues();
        values.put(DatabaseHelper.COL_FLAVOR, flavor);
        values.put(DatabaseHelper.COL_TOPPING, toppings);
        values.put(DatabaseHelper.COL_CUP_CONE, cupcone);
        values.put(DatabaseHelper.COL_PRICE, totalPrice);
        values.put(DatabaseHelper.COL_TIMESTAMP, System.currentTimeMillis());

        try {
            long result = db.insert(DatabaseHelper.ORDER_TABLE, null, values);
            if (result == -1) {
                Log.e(TAG, "주문 저장 실패");
                showToast(languageMode.equals("ko") ?
                        "주문 저장에 실패했습니다" : "Failed to save order");
            } else {
                Log.d(TAG, "주문이 성공적으로 저장되었습니다.");
            }
        } catch (Exception e) {
            Log.e(TAG, "데이터베이스 저장 중 오류 발생", e);
            showToast(languageMode.equals("ko") ?
                    "데이터베이스 오류가 발생했습니다" : "Database error occurred");
        } finally {
            db.close(); // 데이터베이스 연결 종료
        }
    }

    private void initializeComponents() {
        checkTopping1 = findViewById(R.id.checkTopping1);
        checkTopping2 = findViewById(R.id.checkTopping2);
        checkTopping3 = findViewById(R.id.checkTopping3);
        yes_btn = findViewById(R.id.yes_btn);
        none_btn = findViewById(R.id.none_btn);
        topping_btn = findViewById(R.id.topping_btn);
        topping_ly = findViewById(R.id.topping_ly);
        text_price = findViewById(R.id.text_price);
        rg = findViewById(R.id.rg);
    }

    private void setupIntentData() {
        Intent intent = getIntent();
        selectedFlavor = intent.getStringExtra("selectedFlavor");
        cupcone = intent.getStringExtra("cupcone");
        languageMode = intent.getStringExtra("languageMode");

        updatePriceDisplay();
    }

    private void setupUIListeners() {
        rg.setOnCheckedChangeListener((group, checkedId) -> handleRadioGroupChange(checkedId));

        View.OnClickListener priceUpdateListener = v -> updatePrice();
        checkTopping1.setOnClickListener(priceUpdateListener);
        checkTopping2.setOnClickListener(priceUpdateListener);
        checkTopping3.setOnClickListener(priceUpdateListener);

        topping_btn.setOnClickListener(v -> handleConfirmation());
    }

    private void initializeServices() {
        dbHelper = new DatabaseHelper(this);
        rosService = new ROSService(this);
        speechRecognizer = SpeechRecognizer.createSpeechRecognizer(this);
    }

    private void handleRadioGroupChange(int checkedId) {
        if (checkedId == R.id.yes_btn) {
            topping_ly.setVisibility(View.VISIBLE);
            new TTSAsyncTask().execute(languageMode.equals("ko") ?
                    "추가 토핑을 선택해주세요" : "Please select additional toppings");
        } else {
            topping_ly.setVisibility(View.INVISIBLE);
            clearToppings();
        }
        updatePrice();
    }

    private void updatePrice() {
        totalPrice = basePrice;
        if (yes_btn.isChecked()) {
            totalPrice += (getCheckedToppingCount() * toppingPrice);
        }
        updatePriceDisplay();
    }

    private int getCheckedToppingCount() {
        int count = 0;
        if (checkTopping1.isChecked()) count++;
        if (checkTopping2.isChecked()) count++;
        if (checkTopping3.isChecked()) count++;
        return count;
    }

    private void updatePriceDisplay() {
        String priceText = languageMode.equals("ko") ?
                "금액: " + totalPrice + "원" : "Total: $" + (totalPrice / 1000.0);
        text_price.setText(priceText);
    }

    private void clearToppings() {
        checkTopping1.setChecked(false);
        checkTopping2.setChecked(false);
        checkTopping3.setChecked(false);
    }

    private void handleConfirmation() {
        String toppings = getSelectedToppings();
        saveOrderToDatabase(selectedFlavor, toppings);
        if (rosService != null) {
            String orderData = "Topping: " + toppings + " Cup/Cone: " + cupcone;
            rosService.sendDataToROS2(orderData);
            dbHelper.processOrder(
                    selectedFlavor,
                    toppings,
                    cupcone
            );
        }
        new TTSAsyncTask().execute(languageMode.equals("ko") ?
                "결제 화면으로 이동합니다" : "Proceeding to payment");

        startActivity(new Intent(this, E_PaymentActivity.class)
                .putExtra("totalPrice", totalPrice)
                .putExtra("selectedFlavor", selectedFlavor)
                .putExtra("cupcone", cupcone)
                .putExtra("languageMode", languageMode));
    }

    /*private void checkROSConnection() {
        if (!rosService.isConnected()) {
            showToast(languageMode.equals("ko") ?
                    "서버 연결 실패" : "Server connection failed");
            Log.e(TAG, "ROS 서비스 연결 실패");
        }
    }*/

    private String getSelectedToppings() {
        if (!yes_btn.isChecked()) return "None";

        StringBuilder toppings = new StringBuilder();
        if (checkTopping1.isChecked()) toppings.append("죠리퐁 ");
        if (checkTopping2.isChecked()) toppings.append("코코볼 ");
        if (checkTopping3.isChecked()) toppings.append("해바라기씨 ");
        return toppings.toString().trim();
    }

    /*private void startVoiceRecognition() {
        speechRecognizer.setRecognitionListener(new G_CustomSTT());
        restartRecognition();
    }*/

    private void processVoiceResults(ArrayList<String> results) {
        if (results == null || results.isEmpty()) return;

        String voiceCommand = results.get(0).toLowerCase();
        if (voiceCommand.contains("yes") || voiceCommand.contains("네")) {
            rg.check(R.id.yes_btn);
        } else if (voiceCommand.contains("no") || voiceCommand.contains("아니오")) {
            rg.check(R.id.none_btn);
        } else {
            handleToppingSelection(voiceCommand);
        }
        restartRecognition();
    }

    private void handleToppingSelection(String command) {
        if (command.contains("1") || command.contains("죠리퐁")) checkTopping1.toggle();
        if (command.contains("2") || command.contains("코코볼")) checkTopping2.toggle();
        if (command.contains("3") || command.contains("해바라기")) checkTopping3.toggle();
        updatePrice();
    }

    private void handleRecognitionError(int error) {
        String errorMsg;
        switch (error) {
            case SpeechRecognizer.ERROR_AUDIO:
                errorMsg = "오디오 오류";
                break;
            case SpeechRecognizer.ERROR_CLIENT:
                return; // 클라이언트 오류는 무시
            case SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS:
                errorMsg = "권한 부족";
                break;
            case SpeechRecognizer.ERROR_NETWORK:
                errorMsg = "네트워크 오류";
                break;
            case SpeechRecognizer.ERROR_NO_MATCH:
                errorMsg = "인식 불가";
                break;
            default:
                errorMsg = "알 수 없는 오류";
        }

        showToast(languageMode.equals("ko") ?
                "음성 인식 오류: " + errorMsg : "Voice Error: " + errorMsg);
        restartRecognition();
    }

    private void restartRecognition() {
        speechRecognizer.startListening(new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH)
                .putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM)
                .putExtra(RecognizerIntent.EXTRA_LANGUAGE, getLanguageCode()));
    }

    private String getLanguageCode() {
        return languageMode.equals("ko") ? Locale.KOREAN.toString() : Locale.ENGLISH.toString();
    }

    private void playInitialTTS() {
        String message = languageMode.equals("ko") ?
                "토핑을 추가하시겠습니까? 네 또는 아니오로 대답해주세요" :
                "Would you like to add toppings? Please say yes or no";
        new TTSAsyncTask().execute(message);
    }

    private class TTSAsyncTask extends AsyncTask<String, Void, String> {
        @Override
        protected String doInBackground(String... texts) {
            try {
                HttpURLConnection conn = createTTSConnection(texts[0]);
                return processTTSResponse(conn);
            } catch (Exception e) {
                return e.getMessage();
            }
        }

        private HttpURLConnection createTTSConnection(String text) throws IOException {
            URL url = new URL("https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("X-NCP-APIGW-API-KEY-ID", CLIENT_ID);
            conn.setRequestProperty("X-NCP-APIGW-API-KEY", CLIENT_SECRET);

            DataOutputStream wr = new DataOutputStream(conn.getOutputStream());
            wr.writeBytes("speaker=nara&text=" + URLEncoder.encode(text, "UTF-8"));
            wr.close();
            return conn;
        }

        private String processTTSResponse(HttpURLConnection conn) throws IOException {
            if (conn.getResponseCode() == 200) {
                return saveAudioFile(conn.getInputStream());
            }
            return readErrorStream(conn);
        }

        private String saveAudioFile(InputStream is) throws IOException {
            File outputFile = File.createTempFile("tts_", ".mp3", getExternalFilesDir(null));
            try (FileOutputStream fos = new FileOutputStream(outputFile)) {
                byte[] buffer = new byte[1024];
                int bytesRead;
                while ((bytesRead = is.read(buffer)) != -1) {
                    fos.write(buffer, 0, bytesRead);
                }
            }
            return outputFile.getAbsolutePath();
        }

        private String readErrorStream(HttpURLConnection conn) throws IOException {
            BufferedReader br = new BufferedReader(new InputStreamReader(conn.getErrorStream()));
            StringBuilder errorResponse = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) errorResponse.append(line);
            br.close();
            return errorResponse.toString();
        }

        @Override
        protected void onPostExecute(String result) {
            if (result.endsWith(".mp3")) {
                playAudio(result);
            } else {
                showToast("TTS Error: " + result);
            }
        }

        private void playAudio(String path) {
            try {
                if (mediaPlayer != null) {
                    mediaPlayer.release();
                }
                mediaPlayer = new MediaPlayer();
                mediaPlayer.setDataSource(path);
                mediaPlayer.prepare();
                mediaPlayer.start();
            } catch (IOException e) {
                Log.e(TAG, "Audio playback error", e);
            }
        }
    }

    private void showToast(String message) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (speechRecognizer != null) {
            speechRecognizer.destroy();
        }
        if (rosService != null) {
            rosService.closeConnection();
        }
        if (mediaPlayer != null) {
            mediaPlayer.release();
            mediaPlayer = null;
        }
    }
}
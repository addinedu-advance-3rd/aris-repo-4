package com.example.mimokioskapp;

import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;

import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

public class G_CustomTTS {

    private static final String TAG = "CustomTTS";
    private static final String API_URL = "https://naveropenapi.apigw.ntruss.com/tts-premium/v1/tts";
    private static final String CLIENT_ID = "a4zgp0vwh8"; // 네이버 클라이언트 ID
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW"; // 네이버 클라이언트 Secret

    private Context context;

    public G_CustomTTS(Context context) {
        this.context = context;
    }

    public void speak(String text) {
        new Thread(() -> {
            try {
                String encodedText = java.net.URLEncoder.encode(text, "UTF-8");
                URL url = new URL(API_URL);
                HttpURLConnection connection = (HttpURLConnection) url.openConnection();
                connection.setRequestMethod("POST");
                connection.setRequestProperty("X-NCP-APIGW-API-KEY-ID", CLIENT_ID);
                connection.setRequestProperty("X-NCP-APIGW-API-KEY", CLIENT_SECRET);
                connection.setRequestProperty("Content-Type", "application/x-www-form-urlencoded");
                connection.setDoOutput(true);
                connection.getOutputStream().write(("speaker=mijin&speed=0&text=" + encodedText).getBytes());

                InputStream inputStream = connection.getInputStream();
                // 여기서 음성 출력 처리
                playAudio(inputStream);

            } catch (Exception e) {
                Log.e(TAG, "TTS error: " + e.getMessage());
            }
        }).start();
    }

    private void playAudio(InputStream inputStream) {
        // 음성 데이터를 재생하는 코드 (AudioTrack 또는 MediaPlayer 사용)
        Handler handler = new Handler(Looper.getMainLooper());
        handler.post(() -> {
            // TODO: UI에서 음성 재생 알림 추가 가능
            Log.d(TAG, "Audio playback started");
        });
    }
}

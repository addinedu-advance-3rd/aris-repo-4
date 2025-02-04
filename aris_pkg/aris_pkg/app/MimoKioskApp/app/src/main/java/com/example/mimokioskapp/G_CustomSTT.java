package com.example.mimokioskapp;

import android.util.Log;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Scanner;

public class G_CustomSTT {

    private static final String TAG = "CustomSTT";
    private static final String API_URL = "https://naveropenapi.apigw.ntruss.com/stt/v1/recognize";
    private static final String CLIENT_ID = "a4zgp0vwh8"; // 네이버 클라이언트 ID
    private static final String CLIENT_SECRET = "IB5GVyMe4O255EB2u61vHfZMla9GVZ87GcWEuMhW"; // 네이버 클라이언트 Secret

    public interface STTCallback {
        void onResult(String result);
        void onError(String error);
    }

    public void startListening(InputStream audioStream, STTCallback callback) {
        new Thread(() -> {
            HttpURLConnection connection = null;
            OutputStream outputStream = null;
            InputStream responseStream = null;

            try {
                URL url = new URL(API_URL);
                connection = (HttpURLConnection) url.openConnection();
                connection.setRequestMethod("POST");
                connection.setRequestProperty("X-NCP-APIGW-API-KEY-ID", CLIENT_ID);
                connection.setRequestProperty("X-NCP-APIGW-API-KEY", CLIENT_SECRET);
                connection.setRequestProperty("Content-Type", "application/octet-stream");
                connection.setDoOutput(true);

                outputStream = connection.getOutputStream();
                byte[] buffer = new byte[4096];
                int bytesRead;
                while ((bytesRead = audioStream.read(buffer)) != -1) {
                    outputStream.write(buffer, 0, bytesRead);
                }
                outputStream.flush();

                responseStream = connection.getInputStream();
                String result = new Scanner(responseStream).useDelimiter("\\A").next();
                Log.d(TAG, "STT Result: " + result);

                callback.onResult(result);
            } catch (Exception e) {
                Log.e(TAG, "STT error: " + e.getMessage());
                callback.onError(e.getMessage());
            } finally {
                try {
                    if (responseStream != null) responseStream.close();
                    if (outputStream != null) outputStream.close();
                    if (connection != null) connection.disconnect();
                } catch (IOException e) {
                    Log.e(TAG, "Error closing streams: " + e.getMessage());
                }
            }
        }).start();
    }
}
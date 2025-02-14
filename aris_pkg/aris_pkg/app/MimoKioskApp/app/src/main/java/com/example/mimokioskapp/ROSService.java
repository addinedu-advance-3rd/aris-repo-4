package com.example.mimokioskapp;

import android.content.Context;
import android.util.Log;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ROSService {
    private static final String TAG = "ROSService";

    //private static final String ROS2_IP = "192.168.219.184";
    private static final String ROS2_IP = "192.168.0.61";
    private static final int ROS2_PORT = 6789;

    private Socket socket;
    private OutputStream outputStream;
    private InputStream inputStream;
    private ExecutorService executor;
    private ROSListener listener;

    public interface ROSListener {
        void onDataReceived(String data);
    }

    public ROSService(Context context) {
        executor = Executors.newFixedThreadPool(2);
        connect();
    }

    private void connect() {
        executor.execute(() -> {
            try {
                socket = new Socket(ROS2_IP, ROS2_PORT);
                outputStream = socket.getOutputStream();
                inputStream = socket.getInputStream();
                startListening();
                Log.d(TAG, "ROS 연결 성공");
            } catch (IOException e) {
                Log.e(TAG, "ROS 연결 실패: " + e.getMessage());
            }
        });
    }

    private void startListening() {
        executor.execute(() -> {
            byte[] buffer = new byte[1024];
            try {
                while (socket != null && !socket.isClosed()) {
                    int bytesRead = inputStream.read(buffer);
                    if (bytesRead > 0) {
                        String data = new String(buffer, 0, bytesRead);
                        if (listener != null) {
                            listener.onDataReceived(data);
                        }
                    }
                }
            } catch (IOException e) {
                Log.e(TAG, "데이터 수신 오류: " + e.getMessage());
            }
        });
    }

    public void sendDataToROS2(String data) {
        executor.execute(() -> {
            try {
                if (outputStream != null) {
                    outputStream.write(data.getBytes());
                    outputStream.flush();
                    Log.d(TAG, "전송 성공: " + data);
                }
            } catch (IOException e) {
                Log.e(TAG, "전송 실패: " + e.getMessage());
            }
        });
    }

    public void setROSListener(ROSListener listener) {
        this.listener = listener;
    }

    public void closeConnection() {
        try {
            if (socket != null) socket.close();
            if (outputStream != null) outputStream.close();
            if (inputStream != null) inputStream.close();
            executor.shutdown();
            Log.d(TAG, "연결 종료");
        } catch (IOException e) {
            Log.e(TAG, "연결 종료 오류: " + e.getMessage());
        }
    }
}
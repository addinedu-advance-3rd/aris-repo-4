package com.example.mimokioskapp;

import android.os.Handler;
import android.os.Looper;
import android.content.Context;
import android.util.Log;
import android.app.Application;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ROSService extends Application{
    private static final String TAG = "ROSService";

    private Socket socket;
    private OutputStream outputStream;
    private InputStream inputStream;

    private ExecutorService executor ;

    public ROSService(){

    }
    public ROSService(String IP_adress, int port_number, Context context) {
        executor = Executors.newFixedThreadPool(5);
        connect(IP_adress, port_number, context);
    }
    private void ensureExecutorIsActive() {
        if (executor == null || executor.isShutdown() || executor.isTerminated()) {
            executor = Executors.newFixedThreadPool(5);
        }
    }

    private void connect(String IP_adress, int port_number, Context context) {
        ensureExecutorIsActive();
        executor.execute(() -> {
            try {
                socket = new Socket(IP_adress, port_number);

                outputStream = socket.getOutputStream();
                inputStream = socket.getInputStream();
                startListening(context);
                Log.d(TAG, "소켓 연결 성공");
            } catch (IOException e) {
                Log.e(TAG, "소켓 연결 실패: " + e.getMessage());

                new Handler(Looper.getMainLooper()).post(() ->
                        Toast.makeText(context, "서버 연결 실패. 네트워크 상태를 확인하세요. 현재 연결하려는 IP:"+IP_adress, Toast.LENGTH_SHORT).show()
                );

            }
        });
    }

    private void startListening(Context context) {
        ensureExecutorIsActive();
        executor.execute(() -> {
            byte[] buffer = new byte[1024];
            try {
                while (socket != null && !socket.isClosed()) {
                    int bytesRead = inputStream.read(buffer);
                    if (bytesRead > 0) {
                        String data = new String(buffer, 0, bytesRead);
                        Toast.makeText(context, "서버로부터 메세지: " + data, Toast.LENGTH_SHORT).show();
                    }
                }
            } catch (IOException e) {
                Log.e(TAG, "데이터 수신 오류: " + e.getMessage());
            }
        });
    }

    public void sendDataToROS2(String data) {
        ensureExecutorIsActive();
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

    public void closeConnection() {
        ensureExecutorIsActive();
        executor.execute(() -> {
            try {
                if (socket != null) { // ✅ Null 체크 추가
                    socket.close();
                }
                if (outputStream != null) { // ✅ Null 체크 추가
                    outputStream.close();
                }
                if (inputStream != null) { // ✅ Null 체크 추가
                    inputStream.close();
                }

                Log.d(TAG, "소켓 종료");
                executor.shutdown();
                Log.d(TAG, "ROSService 종료");
            } catch (IOException e) {
                Log.e(TAG, "연결 종료 오류: " + e.getMessage());
            }
        });
    }

}
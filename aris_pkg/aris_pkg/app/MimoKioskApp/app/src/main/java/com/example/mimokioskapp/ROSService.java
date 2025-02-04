package com.example.mimokioskapp;

import android.content.Context;
import android.util.Log;

import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ROSService {
    private static final String TAG = "ROSService";
    private static final String ROS2_IP = "192.168.0.61"; // ROS2 서버 IP 주소
    private static final int ROS2_PORT = 6789; // ROS2 서버 포트 번호

    private Socket socket=null;
    private OutputStream outputStream;

    public ROSService(Context context) {
        ExecutorService executorService = Executors.newSingleThreadExecutor();
        executorService.execute(new Runnable() {
            @Override
            public void run() {
                // 소켓 초기화
                try {
                    socket = new Socket(ROS2_IP, ROS2_PORT);
                    outputStream = socket.getOutputStream();
                } catch (IOException e) {
                    Log.e(TAG, "소켓 연결 실패: " + e.getMessage());
                }
            }
        });
    }

    // 데이터를 ROS2로 전송하는 메서드
    public void sendDataToROS2(String data) {
        ExecutorService executorService = Executors.newSingleThreadExecutor();
        executorService.execute(new Runnable() {
            @Override
            public void run() {
                if (outputStream != null) {
                    try {
                        outputStream.write(data.getBytes());
                        outputStream.flush();
                        Log.d(TAG, "데이터 전송 성공: " + data);
                    } catch (IOException e) {
                        Log.e(TAG, "데이터 전송 실패: " + e.getMessage());
                    }
                }
            }
        });
    }

    // 소켓 연결 종료
    public void closeConnection() {
        try {
            if (socket != null) {
                socket.close();
            }
            if (outputStream != null) {
                outputStream.close();
            }
            Log.d(TAG, "소켓 연결 종료");
        } catch (IOException e) {
            Log.e(TAG, "소켓 연결 종료 실패: " + e.getMessage());
        }
    }
}
package com.example.mimokioskapp;

import android.Manifest;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.database.Cursor;
import android.os.Bundle;
import android.text.InputFilter;
import android.text.Spanned;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class AdminActivity extends AppCompatActivity {
    private static final int REQUEST_PERMISSION = 100;
    private DatabaseHelper dbHelper;
    private TextView tvInventoryData;
    private TableLayout clientDataTable;
    private Button exit_btn,export_DB_btn;
    private TextView tvIpAddress;
    private Button btnEditIp;
    private static final String PREFS_NAME = "AppPrefs";
    private static final String KEY_IP_ADDRESS = "saved_ip_address";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_admin_mode);

        dbHelper = new DatabaseHelper(this);

        tvInventoryData = findViewById(R.id.tv_inventory_data);
        clientDataTable = findViewById(R.id.client_data);
        exit_btn = findViewById(R.id.exit_btn);
        export_DB_btn = findViewById(R.id.export_DB_btn);
        tvIpAddress = findViewById(R.id.tv_ip_address);
        btnEditIp = findViewById(R.id.btn_edit_ip);

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, REQUEST_PERMISSION);
        }
        // 데이터베이스 내용 가져와서 화면에 표시
        displayInventoryData();
        displayOrderTable();
        loadSavedIpAddress();

        //나가기 버튼
        exit_btn.setOnClickListener(v -> {
            Intent intent = new Intent(AdminActivity.this, A_WaitingActivity.class);
            startActivity(intent);
            finish();
        });
        export_DB_btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showVoiceServiceDialog();
            }
        });
        btnEditIp.setOnClickListener(v -> showEditIpDialog());
    }

    // 📌 저장된 IP 주소 불러오기
    private void loadSavedIpAddress() {
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        String savedIp = prefs.getString(KEY_IP_ADDRESS, "192.168.0.37"); // 기본값 설정
        tvIpAddress.setText(savedIp);
    }

    // 📌 IP 주소 변경 다이얼로그
    private void showEditIpDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle("IP 주소 수정");

        // EditText 생성 (IP 입력용)
        final EditText input = new EditText(this);
        input.setText(tvIpAddress.getText().toString());
        input.setFilters(new InputFilter[]{new IPAddressInputFilter()});

        LinearLayout layout = new LinearLayout(this);
        layout.setPadding(50, 20, 50, 20);
        layout.addView(input);

        builder.setView(layout);

        // 확인 버튼
        builder.setPositiveButton("저장", (dialog, which) -> {
            String newIp = input.getText().toString();
            if (isValidIpAddress(newIp)) {
                saveIpAddress(newIp);
                tvIpAddress.setText(newIp);
                Toast.makeText(this, "IP 주소가 저장되었습니다.", Toast.LENGTH_SHORT).show();
            } else {
                Toast.makeText(this, "올바른 IP 주소를 입력하세요.", Toast.LENGTH_SHORT).show();
            }
        });

        // 취소 버튼
        builder.setNegativeButton("취소", null);
        builder.show();
    }

// 📌 IP 주소 저장
private void saveIpAddress(String ipAddress) {
    SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
    SharedPreferences.Editor editor = prefs.edit();
    editor.putString(KEY_IP_ADDRESS, ipAddress);
    editor.apply();
}

// 📌 IP 주소 유효성 검사
private boolean isValidIpAddress(String ip) {
    String ipPattern =
            "^((25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])\\.){3}" +
                    "(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])$";
    return ip.matches(ipPattern);
}

// 📌 IP 입력 필터 (입력할 때부터 형식 제한)
public class IPAddressInputFilter implements InputFilter {
    private final Pattern pattern = Pattern.compile(
            "^\\d{0,3}(\\.\\d{0,3}){0,3}$"
    );

    @Override
    public CharSequence filter(CharSequence source, int start, int end,
                               Spanned dest, int dstart, int dend) {
        String result = dest.subSequence(0, dstart) + source.toString() +
                dest.subSequence(dend, dest.length());
        Matcher matcher = pattern.matcher(result);
        return matcher.matches() ? null : "";
    }
}

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == REQUEST_PERMISSION) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // 권한이 승인되었을 때 할 일
            } else {
                // 권한 거부 시
            }
        }
    }

    private void displayInventoryData() {
        StringBuilder inventoryData = new StringBuilder();

        // 아이스크림 재고
        inventoryData.append("📌 [아이스크림 재고]\n");
        appendCursorData(inventoryData, dbHelper.getAllData(DatabaseHelper.ICE_CREAM_TABLE));

        // 토핑 재고
        inventoryData.append("\n📌 [토핑 재고]\n");
        appendCursorData(inventoryData, dbHelper.getAllData(DatabaseHelper.TOPPING_TABLE));

        // 컵/콘 재고
        inventoryData.append("\n📌 [컵/콘 재고]\n");
        appendCursorData(inventoryData, dbHelper.getAllData(DatabaseHelper.CUP_CONE_TABLE));

        tvInventoryData.setText(inventoryData.toString());
    }


    private void displayOrderTable() {
        String query = "SELECT "+ DatabaseHelper.COL_ID + ","
                + DatabaseHelper.COL_CUSTOMER_TYPE + ","
                + DatabaseHelper.COL_GENDER + ","
                + DatabaseHelper.COL_AGE_GROUP + ","
                + DatabaseHelper.COL_USER_ID + ","
                + DatabaseHelper.COL_SATISFACTION + ","
                + DatabaseHelper.COL_ORDER_DURATION + ","
                + DatabaseHelper.COL_FLAVOR + ","
                + DatabaseHelper.COL_ORDER_DATE + ","
                + DatabaseHelper.COL_CUP_CONE + ","
                + DatabaseHelper.COL_TOPPING
                + " FROM " + DatabaseHelper.CLIENT_MANAGEMENT_TABLE;


        //2. 쿼리 실행
        Cursor cursor = null;
        try {
            cursor = dbHelper.getReadableDatabase().rawQuery(query, null);
            if (cursor != null) {
                addTableHeaders(clientDataTable, new String[]{"ID", "고객 유형", "성별", "연령대", "전화번호(사용자ID)", "서비스 만족도", "주문 소요시간(초)", "맛", "주문 일자","컵/콘", "토핑"});
                while (cursor.moveToNext()) {
                    TableRow row = new TableRow(this);
                    for (int i = 0; i < cursor.getColumnCount(); i++) {
                        String value = cursor.getString(i);
                        row.addView(createTextView(value, false));
                    }
                    clientDataTable.addView(row);
                }
            }
        } catch (Exception e) {
            Log.e("AdminActivity", "Error in displayOrderTable", e);
        } finally {
            if (cursor != null) cursor.close();
        }
    }

    private void showVoiceServiceDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setMessage("고객 관리 정보를 내보내시겠습니까?")
                .setPositiveButton("확인", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        dbHelper.exportDataToCSV(AdminActivity.this);

                    }
                })
                .setNegativeButton("취소", null)
                .show();
    }

    private void addTableHeaders(TableLayout table, String[] headers) {
        TableRow headerRow = new TableRow(this);
        for (String header : headers) {
            headerRow.addView(createTextView(header, true));
        }
        table.addView(headerRow);
    }


    // TextView를 생성하여 테이블 셀로 사용
    private TextView createTextView(String text, boolean isHeader) {
        TextView tv = new TextView(this);
        tv.setText(text);
        tv.setPadding(10, 5, 10, 5);
        tv.setTextSize(isHeader ? 16 : 14);
        tv.setBackgroundColor(isHeader ? getColor(R.color.logo_purple3) : getColor(R.color.logo_pink1));
        tv.setTextColor(isHeader ? getColor(android.R.color.white) : getColor(android.R.color.black));
        return tv;
    }

    // Cursor 데이터를 StringBuilder에 추가하는 함수
    private void appendCursorData(StringBuilder builder, Cursor cursor) {
        if (cursor != null) {
            while (cursor.moveToNext()) {
                for (int i = 0; i < cursor.getColumnCount(); i++) {
                    builder.append(cursor.getColumnName(i)).append(": ").append(cursor.getString(i)).append("\n");
                }
                builder.append("--------------------\n");
            }
            cursor.close();
        }
    }
}
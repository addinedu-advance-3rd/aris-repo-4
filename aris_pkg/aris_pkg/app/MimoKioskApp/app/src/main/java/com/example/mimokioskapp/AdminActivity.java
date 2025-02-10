package com.example.mimokioskapp;

import android.content.Intent;
import android.database.Cursor;
import android.os.Bundle;
import android.util.Log;
import android.widget.Button;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

public class AdminActivity extends AppCompatActivity {

    private DatabaseHelper dbHelper;
    private TextView tvInventoryData;
    private TableLayout orderTable, surveyTable, autoTable;
    private Button exit_btn;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_admin_mode);

        dbHelper = new DatabaseHelper(this);
        tvInventoryData = findViewById(R.id.tv_inventory_data);
        orderTable = findViewById(R.id.order_table);
        surveyTable = findViewById(R.id.survey_table);
        autoTable = findViewById(R.id.auto_table);
        exit_btn = findViewById(R.id.exit_btn);

        // 데이터베이스 내용 가져와서 화면에 표시
        displayInventoryData();
        displayOrderTable();
        displaySurveyTable();
        displayAutoTable();

        //나가기 버튼
        exit_btn.setOnClickListener(v -> {
            Intent intent = new Intent(AdminActivity.this, A_WaitingActivity.class);
            startActivity(intent);
            finish();
        });
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

    // ORDER 테이블 데이터를 TableLayout에 표시
    private void displayOrderTable() {
        // ORDERS_TABLE과 ORDER_TABLE 조인 쿼리
        String query = "SELECT o." + DatabaseHelper.COL_ID + ", "
                + "o." + DatabaseHelper.COL_FLAVOR + ", "
                + "o." + DatabaseHelper.COL_TOPPING + ", "
                + "o." + DatabaseHelper.COL_CUP_CONE + ", "
                + "o." + DatabaseHelper.COL_PRICE + ", "
                + "ord." + DatabaseHelper.COL_ORDER_DATE + ", "
                + "ord." + DatabaseHelper.COL_ORDER_DURATION + "/1000 " // 밀리초 -> 초 변환
                + "FROM " + DatabaseHelper.ORDER_TABLE + " o "
                + "INNER JOIN " + DatabaseHelper.ORDERS_TABLE + " ord "
                + "ON o." + DatabaseHelper.COL_ID + " = ord." + DatabaseHelper.COL_ID;

        //2. 쿼리 실행
        Cursor cursor = null;
        try {
            cursor = dbHelper.getReadableDatabase().rawQuery(query, null);
            if (cursor != null) {
                addTableHeaders(orderTable, new String[]{"ID", "맛", "토핑", "컵/콘", "가격", "주문일시", "소요시간(초)"});
                while (cursor.moveToNext()) {
                    TableRow row = new TableRow(this);
                    for (int i = 0; i < cursor.getColumnCount(); i++) {
                        String value = cursor.getString(i);
                        if (cursor.getColumnName(i).equals(DatabaseHelper.COL_ORDER_DATE)) {
                            try {
                                value = DatabaseHelper.converMillisToDate(Long.parseLong(value));
                            } catch (Exception e) {
                                value = "날짜 오류";
                            }
                        }
                        row.addView(createTextView(value, false));
                    }
                    orderTable.addView(row);
                }
            }
        } catch (Exception e) {
            Log.e("AdminActivity", "Error in displayOrderTable", e);
        } finally {
            if (cursor != null) cursor.close();
        }
    }

    private void displaySurveyTable() {
        Cursor cursor = null;
        try {
            cursor = dbHelper.getAllData(DatabaseHelper.SURVEY_TABLE);
            if (cursor != null) {
                addTableHeaders(surveyTable, new String[]{"ID", "고객 유형", "성별", "연령대", "전화번호", "만족도"});
                while (cursor.moveToNext()) {
                    TableRow row = new TableRow(this);
                    for (int i = 0; i < cursor.getColumnCount(); i++) {
                        row.addView(createTextView(cursor.getString(i), false));
                    }
                    surveyTable.addView(row);
                }
            }
        } finally {
            if (cursor != null) cursor.close();
        }
    }

    private void displayAutoTable() {
        Cursor cursor = dbHelper.getAllData(DatabaseHelper.ORDER_TABLE);

        if (cursor != null) {
            // 테이블 헤더 추가
            TableRow headerRow = new TableRow(this);
            String[] columnNames = {"ID", "고객 유형", "성별", "연령대", "전화번호", "서비스 만족도", "대기 시간"};
            for (String column : columnNames) {
                TextView textView = createTextView(column, true);
                headerRow.addView(textView);
            }
            orderTable.addView(headerRow);

            // 데이터 추가
            while (cursor.moveToNext()) {
                TableRow row = new TableRow(this);
                for (int i = 0; i < cursor.getColumnCount(); i++) {
                    TextView textView = createTextView(cursor.getString(i), false);
                    row.addView(textView);
                }
                orderTable.addView(row);
            }
            cursor.close();
        }
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
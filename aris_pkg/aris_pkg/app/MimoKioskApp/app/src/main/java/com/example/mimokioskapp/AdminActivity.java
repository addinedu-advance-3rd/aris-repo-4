package com.example.mimokioskapp;

import android.database.Cursor;
import android.os.Bundle;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

public class AdminActivity extends AppCompatActivity {
    private DatabaseHelper dbHelper;
    private TextView tvInventoryData;
    private TableLayout orderTable;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_admin_mode);

        dbHelper = new DatabaseHelper(this);
        tvInventoryData = findViewById(R.id.tv_inventory_data);
        orderTable = findViewById(R.id.order_table);

        // 데이터베이스 내용 가져와서 화면에 표시
        displayInventoryData();
        displayOrderTable();
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
        Cursor cursor = dbHelper.getAllData(DatabaseHelper.ORDER_TABLE);

        if (cursor != null) {
            // 테이블 헤더 추가
            TableRow headerRow = new TableRow(this);
            String[] columnNames = {"ID", "맛", "토핑", "컵/콘", "가격", "대기 시간"};
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

    // TextView를 생성하여 테이블 셀로 사용
    private TextView createTextView(String text, boolean isHeader) {
        TextView textView = new TextView(this);
        textView.setText(text);
        textView.setPadding(10, 5, 10, 5);
        textView.setTextSize(isHeader ? 16 : 14);
        textView.setTextColor(isHeader ? getResources().getColor(android.R.color.black) : getResources().getColor(android.R.color.darker_gray));
        return textView;
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

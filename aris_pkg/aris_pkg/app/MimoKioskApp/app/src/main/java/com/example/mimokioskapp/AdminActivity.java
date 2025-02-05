package com.example.mimokioskapp;

import android.database.Cursor;
import android.os.Bundle;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;

public class AdminActivity extends AppCompatActivity {
    private DatabaseHelper dbHelper;
    private TextView tvDatabaseContent;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_admin_mode);

        dbHelper = new DatabaseHelper(this);
        tvDatabaseContent = findViewById(R.id.tv_database_content);

        // 데이터베이스 내용 가져와서 화면에 표시
        displayDatabaseContent();
    }

    private void displayDatabaseContent() {
        StringBuilder dbContent = new StringBuilder();

        // 아이스크림 테이블 조회
        dbContent.append("📌 [아이스크림 재고]\n");
        Cursor cursor = dbHelper.getAllData(DatabaseHelper.ICE_CREAM_TABLE);
        appendCursorData(dbContent, cursor);

        // 토핑 테이블 조회
        dbContent.append("\n📌 [토핑 재고]\n");
        cursor = dbHelper.getAllData(DatabaseHelper.TOPPING_TABLE);
        appendCursorData(dbContent, cursor);

        // 컵/콘 테이블 조회
        dbContent.append("\n📌 [컵/콘 재고]\n");
        cursor = dbHelper.getAllData(DatabaseHelper.CUP_CONE_TABLE);
        appendCursorData(dbContent, cursor);

        // 주문 테이블 조회
        dbContent.append("\n📌 [주문 내역]\n");
        cursor = dbHelper.getAllData(DatabaseHelper.ORDER_TABLE);
        appendCursorData(dbContent, cursor);

        tvDatabaseContent.setText(dbContent.toString());
    }

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

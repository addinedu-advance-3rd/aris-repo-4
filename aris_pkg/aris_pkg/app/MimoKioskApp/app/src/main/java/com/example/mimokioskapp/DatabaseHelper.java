package com.example.mimokioskapp;

import android.annotation.SuppressLint;
import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.os.Environment;
import android.util.Log;

import com.opencsv.CSVWriter;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Date;
import java.text.SimpleDateFormat;
import java.util.Locale;

public class DatabaseHelper extends SQLiteOpenHelper {
    // 테이블 및 컬럼 이름 설정
    public static final String DATABASE_NAME = "IceCreamShop.db";
    public static final int DATABASE_VERSION = 2;  // 2에서 3으로 변경


    public static final String ICE_CREAM_TABLE = "ice_cream_inventory";
    public static final String TOPPING_TABLE = "topping_inventory";
    public static final String CUP_CONE_TABLE = "cup_cone_inventory";
    public static final String CLIENT_MANAGEMENT_TABLE = "client_management_data";

    // 공통 컬럼
    public static final String COL_ID = "ID";
    public static final String COL_STOCK = "stock";

    // 주문 테이블 컬럼
    public static final String COL_FLAVOR = "flavor";
    public static final String COL_TOPPING = "topping";
    public static final String COL_CUP_CONE = "cup_cone";
    public static final String COL_PRICE = "price";
    public static final String COL_TIMESTAMP = "timestamp";
    public static final String COL_CUSTOMER_TYPE = "customer_type";
    public static final String COL_GENDER = "gender";
    public static final String COL_AGE_GROUP = "age_group";
    public static final String COL_USER_ID = "user_id";
    public static final String COL_SATISFACTION = "satisfaction";
    public static final String COL_ORDER_DURATION = "order_duration";
    public static final String COL_ORDER_DATE = "order_date";

    // 가격 설정
    private static final int ICE_CREAM_PRICE = 4000;  // 아이스크림 가격
    private static final int TOPPING_PRICE = 300;     // 토핑 가격
    private static final String TAG = "DatabaseHelper";  // 클래스명이나 원하는 태그명 지정


    //private ROSService rosService;

    public DatabaseHelper(Context context) {
        super(context, DATABASE_NAME, null, DATABASE_VERSION);
        //rosService = new ROSService(context); // ROSService 초기화
    }

    @Override
    public void onCreate(SQLiteDatabase db) {

        // 재고 테이블 생성
        db.execSQL("CREATE TABLE " + ICE_CREAM_TABLE + " (" +
                COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COL_FLAVOR + " TEXT, " +
                COL_STOCK + " INTEGER)");

        db.execSQL("CREATE TABLE " + TOPPING_TABLE + " (" +
                COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COL_TOPPING + " TEXT, " +
                COL_STOCK + " INTEGER)");

        db.execSQL("CREATE TABLE " + CUP_CONE_TABLE + " (" +
                COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COL_CUP_CONE + " TEXT, " +
                COL_STOCK + " INTEGER)");

        db.execSQL("CREATE TABLE " + CLIENT_MANAGEMENT_TABLE + " (" +
                COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COL_CUSTOMER_TYPE + " TEXT, " +
                COL_GENDER + " TEXT, "+
                COL_AGE_GROUP + " TEXT, "+
                COL_USER_ID + " TEXT, "+
                COL_SATISFACTION + " INTEGER, "+
                COL_ORDER_DURATION+ " INTEGER, "+
                COL_FLAVOR + " TEXT, " +
                COL_ORDER_DATE + " TEXT,"+
                COL_CUP_CONE+ " TEXT, " +
                COL_TOPPING + " TEXT)");


        // 초기 재고 입력
        insertInitialStock(db);
    }


    //시간 변환 유틸리티 메서드
    public static String converMillisToDate(long millis) {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());
        return sdf.format(new Date(millis));
    }



    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        Log.d(TAG, "onUpgrade 호출됨: " + oldVersion + " -> " + newVersion);

        // 기존 테이블 삭제 (데이터 초기화)
        db.execSQL("DROP TABLE IF EXISTS " + ICE_CREAM_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + TOPPING_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + CUP_CONE_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + CLIENT_MANAGEMENT_TABLE);

        // 다시 테이블 생성
        onCreate(db);
    }

    public void resetDatabase() {
        SQLiteDatabase db = this.getWritableDatabase();

        // 기존 데이터 삭제
        db.execSQL("DROP TABLE IF EXISTS " + ICE_CREAM_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + TOPPING_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + CUP_CONE_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + CLIENT_MANAGEMENT_TABLE);

        // 다시 테이블 생성
        onCreate(db);
        Log.d(TAG, "데이터베이스가 초기화되었습니다.");
    }


    // 초기 재고 입력
    private void insertInitialStock(SQLiteDatabase db) {
        // 아이스크림 재고 입력 (딸기, 블루베리)
        ContentValues flavorValues = new ContentValues();
        flavorValues.put("flavor", "strawberry"); //원래 딸기로 되어있었음!!! 딸기->strawberry로 바꿔주세요!!
        flavorValues.put("stock", 100);  // 초기 재고 수량
        db.insert(ICE_CREAM_TABLE, null, flavorValues);

        flavorValues.put("flavor", "blueberry"); //원래 블루베리로 되어있었음!!! 블루베리->blueberry로 바꿔주세요!!
        db.insert(ICE_CREAM_TABLE, null, flavorValues);

        // 토핑 재고 입력 (죠리퐁, 코코볼, 해바라기씨)
        ContentValues toppingValues = new ContentValues();
        toppingValues.put("topping", "죠리퐁");
        toppingValues.put("stock", 50);
        db.insert(TOPPING_TABLE, null, toppingValues);

        toppingValues.put("topping", "코코볼");
        db.insert(TOPPING_TABLE, null, toppingValues);

        toppingValues.put("topping", "해바라기씨");
        db.insert(TOPPING_TABLE, null, toppingValues);

        // 컵/콘 재고 입력
        ContentValues cupConeValues = new ContentValues();//원래 컵으로 되어있었음!!! 컵-> cup로 바꿔주세요!!
        cupConeValues.put("cup_cone", "cup");
        cupConeValues.put("stock", 100);
        db.insert(CUP_CONE_TABLE, null, cupConeValues);

        cupConeValues.put("cup_cone", "cone"); //원래 콘으로 되어있었음!!! -> cone로 바꿔주세요!!
        db.insert(CUP_CONE_TABLE, null, cupConeValues);
    }

    // 주문 처리 및 재고 차감
    public void processOrder(String flavor, String topping, String cupCone) {
        SQLiteDatabase db = this.getWritableDatabase();

        // 아이스크림 재고 차감
        updateStock(db, ICE_CREAM_TABLE, flavor);

        // 토핑 재고 차감
        if (topping != null && !topping.equals("None")) {
            String[] toppings = topping.split(" ");
            for (String top : toppings) {
                updateStock(db, TOPPING_TABLE, top);
            }
        }

        // 컵/콘 재고 차감
        if (cupCone != null) {
            updateStock(db, CUP_CONE_TABLE, cupCone);
        }

        // 주문 정보 저장 (가격 계산 포함)
//        int totalPrice = calculatePrice(flavor, topping);
//        saveOrder(db, flavor, topping, cupCone, totalPrice);

        // ROS2로 주문 정보 전송
        //String orderData = "Topping: " + topping + " Cup/Cone: " + cupCone;
        //rosService.sendDataToROS2(orderData);
    }

    // 재고 차감 메서드
    private void updateStock(SQLiteDatabase db, String tableName, String item) {
        Cursor cursor = db.rawQuery("SELECT stock FROM " + tableName + " WHERE " + getColumnName(tableName) + " = ?", new String[]{item});
        if (cursor.moveToFirst()) {
            @SuppressLint("Range") int currentStock = cursor.getInt(cursor.getColumnIndex("stock"));
            if (currentStock > 0) {
                // 재고가 있으면 차감
                ContentValues values = new ContentValues();
                values.put("stock", currentStock - 1);
                db.update(tableName, values, getColumnName(tableName) + " = ?", new String[]{item});
            }
        }
        cursor.close();
    }

    // 테이블에 따른 컬럼 이름 반환
    private String getColumnName(String tableName) {
        switch (tableName) {
            case ICE_CREAM_TABLE:
                return COL_FLAVOR;
            case TOPPING_TABLE:
                return COL_TOPPING;
            case CUP_CONE_TABLE:
                return COL_CUP_CONE;
            default:
                return "";
        }
    }
    // 설문 데이터 저장 메서드


    // 가격 계산 (아이스크림 가격 + 토핑 가격)
    private int calculatePrice(String flavor, String topping) {
        int price = ICE_CREAM_PRICE;  // 기본 아이스크림 가격
        if (topping != null && !topping.equals("None")) {
            price += TOPPING_PRICE * topping.split(" ").length;  // 토핑 추가 가격
        }
        return price;
    }

    // 주문 정보 저장
    public void saveOrder(SQLiteDatabase db, String flavor, String topping, String cupCone, int orderDuration) {
        ContentValues orderValues = new ContentValues();
        orderValues.put(COL_FLAVOR, flavor);
        orderValues.put(COL_TOPPING, topping);
        orderValues.put(COL_CUP_CONE, cupCone);
        orderValues.put(COL_ORDER_DURATION, orderDuration);

        try {
            long result = db.insert(CLIENT_MANAGEMENT_TABLE, null, orderValues);
            if (result == -1) {
                Log.e(TAG, "주문 저장 실패");
//                Toast.makeText(getContext(), "주문 저장에 실패했습니다 : Failed to save order", Toast.LENGTH_SHORT).show();
            } else {
                Log.d(TAG, "주문이 성공적으로 저장되었습니다.");
            }
        } catch (Exception e) {
            Log.e(TAG, "데이터베이스 저장 중 오류 발생", e);
//            Toast.makeText(getContext(), "데이터베이스 오류가 발생했습니다 : Database error occurred", Toast.LENGTH_SHORT).show();
        } finally {
            db.close(); // 데이터베이스 연결 종료
        }
    }

    public void insertClientData(ContentValues values){
        SQLiteDatabase db = this.getWritableDatabase();
        try {
            long result = db.insert(CLIENT_MANAGEMENT_TABLE, null, values);
            if (result == -1) {
                Log.e(TAG, "고객정보 저장 실패");
//                Toast.makeText(getContext(), "주문 저장에 실패했습니다 : Failed to save order", Toast.LENGTH_SHORT).show();
            } else {
                Log.d(TAG, "고객정보가 성공적으로 저장되었습니다.");
            }
        } catch (Exception e) {
            Log.e(TAG, "데이터베이스 저장 중 오류 발생", e);
//            Toast.makeText(getContext(), "데이터베이스 오류가 발생했습니다 : Database error occurred", Toast.LENGTH_SHORT).show();
        } finally {
            db.close(); // 데이터베이스 연결 종료
        }
    }

    public void exportDataToCSV(Context context) {
        SQLiteDatabase db = this.getReadableDatabase();
        Cursor cursor = db.rawQuery("SELECT * FROM client_management_data", null);

        // 외부 저장소에 CSV 파일 생성
        File exportDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), "SQLiteExports");
        if (!exportDir.exists()) {
            exportDir.mkdirs();
        }

        File file = new File(exportDir, "client_data.csv");
        try {
            FileWriter fileWriter = new FileWriter(file);
            CSVWriter writer = new CSVWriter(fileWriter);

            // 컬럼 이름을 CSV 첫 줄에 작성
            String[] header = {"ID", "customer_type", "gender", "age_group", "user_id", "satisfaction", "order_duration", "flavor", "order_date", "cup_cone", "topping"};
            writer.writeNext(header);

            // 데이터를 한 줄씩 읽어서 CSV에 기록
            while (cursor.moveToNext()) {
                String[] row = new String[header.length];
                for (int i = 0; i < cursor.getColumnCount(); i++) {
                    row[i] = cursor.getString(i);
                }
                writer.writeNext(row);
            }

            writer.close();
            cursor.close();

            // 파일 내보내기 완료 메시지
            System.out.println("CSV export completed successfully.");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    // 특정 테이블의 모든 데이터 가져오기
    public Cursor getAllData(String tableName) {
        SQLiteDatabase db = this.getReadableDatabase();
        return db.rawQuery("SELECT * FROM " + tableName, null);
    }


    // 재고 정보 가져오기 (주문 후 확인용)
    public String getInventoryInfo() {
        SQLiteDatabase db = this.getReadableDatabase();
        StringBuilder inventoryInfo = new StringBuilder();

        // 아이스크림 재고 확인
        Cursor flavorCursor = db.rawQuery("SELECT * FROM " + ICE_CREAM_TABLE, null);
        while (flavorCursor.moveToNext()) {
            inventoryInfo.append("Flavor: ").append(flavorCursor.getString(1)).append(", Stock: ")
                    .append(flavorCursor.getInt(2)).append("\n");
        }
        flavorCursor.close();

        // 토핑 재고 확인
        Cursor toppingCursor = db.rawQuery("SELECT * FROM " + TOPPING_TABLE, null);
        while (toppingCursor.moveToNext()) {
            inventoryInfo.append("Topping: ").append(toppingCursor.getString(1)).append(", Stock: ")
                    .append(toppingCursor.getInt(2)).append("\n");
        }
        toppingCursor.close();

        // 컵/콘 재고 확인
        Cursor cupConeCursor = db.rawQuery("SELECT * FROM " + CUP_CONE_TABLE, null);
        while (cupConeCursor.moveToNext()) {
            inventoryInfo.append("Cup/Cone: ").append(cupConeCursor.getString(1)).append(", Stock: ")
                    .append(cupConeCursor.getInt(2)).append("\n");
        }
        cupConeCursor.close();

        return inventoryInfo.toString();
    }
}
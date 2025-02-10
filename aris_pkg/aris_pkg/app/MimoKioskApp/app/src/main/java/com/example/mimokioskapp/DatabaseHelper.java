package com.example.mimokioskapp;

import android.annotation.SuppressLint;
import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

public class DatabaseHelper extends SQLiteOpenHelper {
    // 테이블 및 컬럼 이름 설정
    public static final String DATABASE_NAME = "IceCreamShop.db";
    public static final int DATABASE_VERSION = 2; // 버전 변경 (기존 1 -> 2)

    public static final String ICE_CREAM_TABLE = "ice_cream_inventory";
    public static final String TOPPING_TABLE = "topping_inventory";
    public static final String CUP_CONE_TABLE = "cup_cone_inventory";
    public static final String ORDER_TABLE = "ice_cream_order";

    // 공통 컬럼
    public static final String COL_ID = "ID";
    public static final String COL_STOCK = "stock";

    // 주문 테이블 컬럼
    public static final String COL_FLAVOR = "flavor";
    public static final String COL_TOPPING = "topping";
    public static final String COL_CUP_CONE = "cup_cone";
    public static final String COL_PRICE = "price";
    public static final String SURVEY_TABLE = "survey_data";
    public static final String COL_TIMESTAMP = "timestamp";
    public static final String COL_CUSTOMER_TYPE = "customer_type";
    public static final String COL_GENDER = "gender";
    public static final String COL_AGE_GROUP = "age_group";
    public static final String COL_USER_ID = "user_id";
    public static final String COL_SATISFACTION = "satisfaction";
    public static final String COL_ORDER_START_TIME = "order_start_time";
    public static final String COL_ORDER_END_TIME = "order_end_time";
    public static final String COL_ORDER_DURATION = "order_duration";

    // 가격 설정
    private static final int ICE_CREAM_PRICE = 4000;  // 아이스크림 가격
    private static final int TOPPING_PRICE = 300;     // 토핑 가격
    public static String COLUMN_FLAVOR;
    public static String COLUMN_TOPPINGS;
    public static String COLUMN_CUPCONE;
    public static String COLUMN_PRICE;
    public static String COLUMN_TIMESTAMP;

    //private ROSService rosService;

    public DatabaseHelper(Context context) {
        super(context, DATABASE_NAME, null, 1);
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

        db.execSQL("CREATE TABLE " + ORDER_TABLE + " (" +
                COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COL_FLAVOR + " TEXT, " +
                COL_TOPPING + " TEXT, " +
                COL_CUP_CONE + " TEXT, " +
                COL_PRICE + " INTEGER, " +
                COL_TIMESTAMP + " INTEGER DEFAULT CURRENT_TIMESTAMP)");

        db.execSQL("CREATE TABLE " + SURVEY_TABLE + " ("
                + COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, "
                + COL_CUSTOMER_TYPE + " TEXT, "
                + COL_GENDER + " TEXT, "
                + COL_AGE_GROUP + " TEXT, "
                + COL_USER_ID + " TEXT, "
                + COL_SATISFACTION + " INTEGER, "
                + COL_TIMESTAMP + " DATETIME DEFAULT CURRENT_TIMESTAMP)");

        db.execSQL("CREATE TABLE orders ("
                + COL_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, "
                + COL_ORDER_START_TIME + " DATETIME, "
                + COL_ORDER_END_TIME + " DATETIME, "
                + COL_ORDER_DURATION + " INTEGER)");
        // 초기 재고 입력
        insertInitialStock(db);
    }

    //주문 시작시간 기록
    public void recordOrderStartTime(long startTime){
        SQLiteDatabase db=this.getWritableDatabase();
        ContentValues values = new ContentValues();
        values.put(COL_ORDER_START_TIME,startTime);
        db.insert("orders",null,values);
    }

    //주문 소요 시간 계산
    public void calculateOrderDuration(long endTime){
        SQLiteDatabase db=this.getWritableDatabase();
        Cursor cursor = db.rawQuery("SELECT " + COL_ORDER_START_TIME + " FROM orders ORDER BY " + COL_ID + " DESC LIMIT 1", null);

        if(cursor.moveToFirst()){
            long startTime = cursor.getLong(0);
            long duration = endTime - startTime;

            ContentValues values = new ContentValues();
            values.put(COL_ORDER_END_TIME, endTime);
            values.put(COL_ORDER_START_TIME, duration);

            db.update("orders", values, COL_ID + " = (SELECT MAX(" + COL_ID + ") FROM orders)", null);
        }
        cursor.close();
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        if (oldVersion < 2) {
            db.execSQL("ALTER TABLE " + ORDER_TABLE + " ADD COLUMN " + COL_TIMESTAMP + " INTEGER DEFAULT CURRENT_TIMESTAMP");
        }
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
        int totalPrice = calculatePrice(flavor, topping);
        saveOrder(db, flavor, topping, cupCone, totalPrice);

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
    public void insertSurveyData(ContentValues values) {
        SQLiteDatabase db = this.getWritableDatabase();
        try {
            db.insert(SURVEY_TABLE, null, values);
            Log.d("Database", "설문 데이터 저장 성공");
        } catch (Exception e) {
            Log.e("Database", "설문 데이터 저장 실패: " + e.getMessage());
        } finally {
            db.close();
        }
    }

    // 가격 계산 (아이스크림 가격 + 토핑 가격)
    private int calculatePrice(String flavor, String topping) {
        int price = ICE_CREAM_PRICE;  // 기본 아이스크림 가격
        if (topping != null && !topping.equals("None")) {
            price += TOPPING_PRICE * topping.split(" ").length;  // 토핑 추가 가격
        }
        return price;
    }

    // 주문 정보 저장
    private void saveOrder(SQLiteDatabase db, String flavor, String topping, String cupCone, int price) {
        ContentValues orderValues = new ContentValues();
        orderValues.put(COL_FLAVOR, flavor);
        orderValues.put(COL_TOPPING, topping);
        orderValues.put(COL_CUP_CONE, cupCone);
        orderValues.put(COL_PRICE, price);
        orderValues.put(COL_TIMESTAMP, System.currentTimeMillis()); // 현재 시간 저장
        db.insert(ORDER_TABLE, null, orderValues);
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
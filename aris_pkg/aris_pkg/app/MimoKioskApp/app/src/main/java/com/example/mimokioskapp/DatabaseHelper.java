package com.example.mimokioskapp;

import android.annotation.SuppressLint;
import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;

public class DatabaseHelper extends SQLiteOpenHelper {
    // 테이블 및 컬럼 이름 설정
    public static final String DATABASE_NAME = "IceCreamShop.db";
    public static final String ICE_CREAM_TABLE = "ice_cream_inventory";
    public static final String TOPPING_TABLE = "topping_inventory";
    public static final String CUP_CONE_TABLE = "cup_cone_inventory";
    public static final String ORDER_TABLE = "ice_cream_order";

    public static final String COL_ID = "ID";
    public static final String COL_FLAVOR = "flavor";
    public static final String COL_TOPPING = "topping";
    public static final String COL_CUP_CONE = "cup_cone";
    public static final String COL_PRICE = "price";
    public static final String COL_STOCK = "stock";  // 재고를 저장할 컬럼

    // 가격 설정
    private static final int ICE_CREAM_PRICE = 4000;  // 아이스크림 가격
    private static final int TOPPING_PRICE = 300;     // 토핑 가격
    public static String COLUMN_FLAVOR;
    public static String COLUMN_TOPPINGS;
    public static String COLUMN_CUPCONE;
    public static String COLUMN_PRICE;
    public static String COLUMN_TIMESTAMP;

    private ROSService rosService;

    public DatabaseHelper(Context context) {
        super(context, DATABASE_NAME, null, 1);
        rosService = new ROSService(context); // ROSService 초기화
    }

    @Override
    public void onCreate(SQLiteDatabase db) {
        // 재고 테이블 생성
        db.execSQL("CREATE TABLE " + ICE_CREAM_TABLE + " (ID INTEGER PRIMARY KEY AUTOINCREMENT, flavor TEXT, stock INTEGER)");
        db.execSQL("CREATE TABLE " + TOPPING_TABLE + " (ID INTEGER PRIMARY KEY AUTOINCREMENT, topping TEXT, stock INTEGER)");
        db.execSQL("CREATE TABLE " + CUP_CONE_TABLE + " (ID INTEGER PRIMARY KEY AUTOINCREMENT, cup_cone TEXT, stock INTEGER)");

        // 주문 테이블 생성
        db.execSQL("CREATE TABLE " + ORDER_TABLE + " (ID INTEGER PRIMARY KEY AUTOINCREMENT, flavor TEXT, topping TEXT, cup_cone TEXT, price INTEGER)");

        // 초기 재고 입력
        insertInitialStock(db);
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        db.execSQL("DROP TABLE IF EXISTS " + ICE_CREAM_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + TOPPING_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + CUP_CONE_TABLE);
        db.execSQL("DROP TABLE IF EXISTS " + ORDER_TABLE);
        onCreate(db);
    }

    // 초기 재고 입력
    private void insertInitialStock(SQLiteDatabase db) {
        // 아이스크림 재고 입력 (딸기, 블루베리)
        ContentValues flavorValues = new ContentValues();
        flavorValues.put("flavor", "딸기");
        flavorValues.put("stock", 100);  // 초기 재고 수량
        db.insert(ICE_CREAM_TABLE, null, flavorValues);

        flavorValues.put("flavor", "블루베리");
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
        ContentValues cupConeValues = new ContentValues();
        cupConeValues.put("cup_cone", "컵");
        cupConeValues.put("stock", 100);
        db.insert(CUP_CONE_TABLE, null, cupConeValues);

        cupConeValues.put("cup_cone", "콘");
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
        String orderData = "Topping: " + topping + " Cup/Cone: " + cupCone;
        rosService.sendDataToROS2(orderData);
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
        db.insert(ORDER_TABLE, null, orderValues);
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
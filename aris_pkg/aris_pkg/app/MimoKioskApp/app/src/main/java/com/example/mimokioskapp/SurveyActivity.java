package com.example.mimokioskapp;

import android.content.ContentValues;
import android.content.Intent;
import android.database.sqlite.SQLiteDatabase;
import android.os.Bundle;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RatingBar;
import android.widget.Spinner;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.time.LocalDate;

public class SurveyActivity extends AppCompatActivity {

    // UI 컴포넌트
    private Spinner spinnerCustomerType, spinnerGender, spinnerAgeGroup;
    private EditText etPhoneNumber;
    private RatingBar ratingSatisfaction;
    private DatabaseHelper dbHelper;
    private Button btnSubmit, btnCancle;
    private String selectedFlavor = "";
    private int totalPrice = 4000;
    private String selectedTopping="";
    private String cupcone = "";
    private String languageMode = "ko";
    private SQLiteDatabase db;
    private int orderDuration;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.survey_activity);
        btnSubmit = findViewById(R.id.btnSubmit);
        btnCancle = findViewById(R.id.btnCancle);

        // 데이터베이스 헬퍼 초기화
        dbHelper = new DatabaseHelper(this);

        selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        totalPrice = getIntent().getIntExtra("totalPrice", 4000);
        selectedTopping = getIntent().getStringExtra("selectedTopping");
        cupcone = getIntent().getStringExtra("cupcone");
        languageMode = getIntent().getStringExtra("languageMode");
        orderDuration = getIntent().getIntExtra("orderDuration",0);

        // UI 초기화
        initUIComponents();
        setupSpinners();
        setupSubmitButton();
        btnSubmit.setOnClickListener(v -> {
            saveSurveyData();
            Intent intent = new Intent(SurveyActivity.this, FinalActivity.class);
            startActivity(intent);
            finish();
        });
        btnCancle.setOnClickListener(v -> {
            saveClientData();
            Intent intent = new Intent(SurveyActivity.this, FinalActivity.class);
            startActivity(intent);
            finish();
        });
    }

    private void initUIComponents() {
        spinnerCustomerType = findViewById(R.id.spinnerCustomerType);
        spinnerGender = findViewById(R.id.spinnerGender);
        spinnerAgeGroup = findViewById(R.id.spinnerAgeGroup);
        etPhoneNumber = findViewById(R.id.etPhoneNumber);
        ratingSatisfaction = findViewById(R.id.ratingSatisfaction);
    }

    private void setupSpinners() {
        // 고객 유형 스피너
        ArrayAdapter<CharSequence> customerAdapter = ArrayAdapter.createFromResource(
                this, R.array.customer_types, android.R.layout.simple_spinner_item);
        customerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerCustomerType.setAdapter(customerAdapter);

        // 성별 스피너
        ArrayAdapter<CharSequence> genderAdapter = ArrayAdapter.createFromResource(
                this, R.array.gender_options, android.R.layout.simple_spinner_item);
        genderAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerGender.setAdapter(genderAdapter);

        // 연령대 스피너
        ArrayAdapter<CharSequence> ageAdapter = ArrayAdapter.createFromResource(
                this, R.array.age_groups, android.R.layout.simple_spinner_item);
        ageAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerAgeGroup.setAdapter(ageAdapter);
    }

    private void setupSubmitButton() {
        Button btnSubmit = findViewById(R.id.btnSubmit);
        btnSubmit.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (validateInputs()) {

                    finish(); // 설문 완료 후 화면 닫기
                }
            }
        });
    }

    private boolean validateInputs() {
        // 전화번호 유효성 검사
        if (etPhoneNumber.getText().toString().trim().isEmpty()) {
            Toast.makeText(this, "전화번호를 입력해주세요", Toast.LENGTH_SHORT).show();
            return false;
        }
        return true;
    }

    private void saveSurveyData() {
        int ratingNumber =(int) ratingSatisfaction.getRating();
        ContentValues values = new ContentValues();
        values.put(DatabaseHelper.COL_ORDER_DATE, String.valueOf(LocalDate.now()));
        values.put(DatabaseHelper.COL_FLAVOR, selectedFlavor);
        values.put(DatabaseHelper.COL_TOPPING, selectedTopping);
        values.put(DatabaseHelper.COL_CUP_CONE, cupcone);
        values.put(DatabaseHelper.COL_ORDER_DURATION, orderDuration);
        values.put(DatabaseHelper.COL_CUSTOMER_TYPE,
                spinnerCustomerType.getSelectedItem().toString());
        values.put(DatabaseHelper.COL_GENDER,
                spinnerGender.getSelectedItem().toString());
        values.put(DatabaseHelper.COL_AGE_GROUP,
                spinnerAgeGroup.getSelectedItem().toString());
        values.put(DatabaseHelper.COL_USER_ID,
                etPhoneNumber.getText().toString().trim());
        values.put(DatabaseHelper.COL_SATISFACTION,
                ratingNumber);

        // 데이터베이스 저장
        dbHelper.insertClientData(values);

        Toast.makeText(this, "설문이 제출되었습니다!", Toast.LENGTH_SHORT).show();
        Intent intent = new Intent(SurveyActivity.this, FinalActivity.class);
        startActivity(intent);
    }
    private void saveClientData() {
        dbHelper = new DatabaseHelper(this);
        dbHelper.saveOrder(db=dbHelper.getWritableDatabase(),
                selectedFlavor,
                selectedTopping,
                cupcone,
                orderDuration);
    }
}
package com.example.mimokioskapp;

import android.content.Intent;
import android.database.sqlite.SQLiteDatabase;
import android.os.Bundle;
import android.view.Window;
import android.widget.Button;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

public class SurveyPopActivity extends AppCompatActivity {


    Button yes_btn, no_btn;
    private String selectedFlavor = "";
    private int totalPrice = 4000;
    private String selectedTopping="";
    private String cupcone = "";
    private String languageMode = "ko";
    private DatabaseHelper dbHelper;
    private SQLiteDatabase db;
    private int orderDuration;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        supportRequestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.survey_popup);

        selectedFlavor = getIntent().getStringExtra("selectedFlavor");
        totalPrice = getIntent().getIntExtra("totalPrice", 4000);
        selectedTopping = getIntent().getStringExtra("selectedTopping");
        cupcone = getIntent().getStringExtra("cupcone");
        languageMode = getIntent().getStringExtra("languageMode");
        orderDuration = getIntent().getIntExtra("orderDuration",0);


        yes_btn = findViewById(R.id.yes_btn);
        no_btn = findViewById(R.id.no_btn);
        yes_btn.setOnClickListener(v -> {
            Intent intent = new Intent(SurveyPopActivity.this, SurveyActivity.class);
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
            intent.putExtra("totalPrice", totalPrice);
            intent.putExtra("selectedFlavor", selectedFlavor);
            intent.putExtra("languageMode", languageMode);
            intent.putExtra("cupcone", cupcone);
            intent.putExtra("selectedTopping", selectedTopping);
            intent.putExtra("orderDuration", orderDuration);
            startActivity(intent);
            finish();
        });

        no_btn.setOnClickListener(v -> {
            saveClientData();
            Intent intent = new Intent(SurveyPopActivity.this, FinalActivity.class);
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
            startActivity(intent);
            finish();
        });
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

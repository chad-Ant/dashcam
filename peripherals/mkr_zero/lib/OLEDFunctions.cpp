/*
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

//Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGT, &Wire, OLED_RESET);

bool initializeOLED(Adafruit_SSD1306 display){
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)){
        display.display();
        delay(1000);
        display.clearDisplay();
        return true;
    }
    return false;
}

void OLED_drawText(Adafruit_SSD1306 &display,char16_t displayString[128]){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.cp437(true);

    for(int16_t i = 0; i < 12; i++){
        display.write(displayString[i]);
    }

    display.display();
}

void OLED_clearScreen(Adafruit_SSD1306 &display){
    display.clearDisplay();
}

void OLED_drawGraphic(){

}
*/
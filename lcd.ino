#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

float n = 1.01;
char a[10];

void setup(){
    lcd.init();
    lcd.backlight();
}


void loop(){

    // lcd.setCursor(1, 0);
    // lcd.print("So n la: ");
    // lcd.print(n);
    // n+=0.8;
    // if (n>101) n = 101;
    // delay(200);
    // sprintf(a, "so n la: %.02f" ,n);
    dtostrf(n, 4, 2, a);
    lcd.setCursor(1, 0);
    lcd.print("So n la: ");
    lcd.print(a);
}
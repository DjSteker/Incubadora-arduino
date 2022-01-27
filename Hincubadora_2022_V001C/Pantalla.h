






byte solar[8] = //icon for solar panel
{
  0b11111, 0b10101, 0b11111, 0b10101, 0b11111, 0b10101, 0b11111, 0b00000
};
byte battery[8] =  //icon for battery
{
  0b01110, 0b11011, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111
};

byte energy[8] =  // icon for power
{
  0b00010, 0b00100, 0b01000, 0b11111, 0b00010, 0b00100, 0b01000, 0b00000
};
byte alarm[8] =  // icon for alarm
{
  0b00000,0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100
};
byte temperature[8] = //icon for termometer
{
  0b00100, 0b01010, 0b01010, 0b01110, 0b01110, 0b11111, 0b11111, 0b01110
};
byte charge[8] = // icon for battery charge
{
  0b01010, 0b11111, 0b10001, 0b10001, 0b10001, 0b01110, 0b00100, 0b00100,
};
byte not_charge[8] =
{
  0b00000, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b00000, 0b00000,
};
byte pwm [8] =
{
  0b11101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10111,
};




void setupPantalla() {

   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    //for (;;); // Don't proceed, loop forever
  }
  display.display();
  delay(200);
  display.clearDisplay();
  //display.drawPixel(10, 10, SSD1306_WHITE);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.print("Hincubadora V001");
  display.display();

  
}

void initDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3D (for the 128x64) // init done // Show image buffer on the display hardware.

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(15, 9);
  display.print("ITSHARSH");
  display.display();
  delay(2000);
}

void printOnScreen(byte a, byte b, String data) {
  display.setTextSize(1);
  display.setCursor(a, b);
  display.print(data);
}

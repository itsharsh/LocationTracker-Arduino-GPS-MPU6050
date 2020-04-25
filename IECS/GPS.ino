void initGPS() {
  GPS.begin(9600);
  GPS.sendCommand("$PGCMD, 33, 0*6D");          //Antenna update turn off
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //update speed for gps at 1HZ
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //only show GGA and RMC data
}

void getGPSData() {
  while (!GPS.newNMEAreceived()) {
    data = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived()) {
    data = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA1 = GPS.lastNMEA();

  while (!GPS.newNMEAreceived()) {
    data = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();
}
void printGPSData(byte printType) {
  GPS.hour += 5;            //Changing Time Zone
  GPS.minute += 30;
  if (GPS.minute >= 60) {
    GPS.minute -= 60;
    GPS.hour++;
  }
  if (GPS.hour >= 24) {
    GPS.hour -= 24;
  }

  switch (printType) {
    case 1:
      printGPSDataonSerial();
      break;

    case 2:
      printGPSDataOnLCD();
      break;
    default:
      printGPSDataonSerial();
      printGPSDataOnLCD();
  }
}

void printGPSDataonSerial() {
  Serial.print(NMEA1);
  Serial.println(NMEA2);

  Serial.print("Sat: ");
  Serial.print(GPS.satellites);
  Serial.print("\tLat: ");
  Serial.print(GPS.latitude_fixed);
  Serial.print("\tLong: ");
  Serial.print(GPS.longitude_fixed);
  Serial.print("\tAlt: ");
  Serial.print(GPS.altitude);
  Serial.print("\tTime: ");
  Serial.print(GPS.hour);
  Serial.print(":");
  Serial.print(GPS.minute);
  Serial.print(":");
  Serial.println(GPS.seconds);
}

void printGPSDataOnLCD() {
  display.clearDisplay();
  printOnScreen(0, 0, (String)"T:" + GPS.hour + ":" + GPS.minute + ":" + GPS.seconds);
  printOnScreen(70, 0, (String)"Sat: " + GPS.satellites);
  printOnScreen(0, 8, (String)"Alt: " + GPS.altitude);
  printOnScreen(70, 8, (String)"Fix: " + GPS.fixquality);

  printOnScreen(0, 16, (String)"Lat: " + GPS.latitude_fixed + GPS.lat);
  printOnScreen(0, 24, (String)"Lon: " + GPS.longitude_fixed  + GPS.lon);
}

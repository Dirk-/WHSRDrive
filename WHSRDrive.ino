/*
  WHSRDrive

  This example for the Westphalian University's WHSR lerning robot shows how to
  controll the robot via BLE.

  This example code is in the public domain. For more information, see
  https://github.com/Dirk-/WHSR
*/

#include <WHSR.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

// Farben zum Blinken
#define SCHWARZ 1
#define ROT 2
#define GELB 3
#define GRUEN 4
#define BLAU 5
#define WEISS 6

// Befehle, die man über BLE schicken kann
#define BEFEHL_ERLEDIGT 0
#define BEFEHL_SPEICHERN 1
#define BEFEHL_WEISSNICHT 2

// We need an instance of the WHSR class to handle the robot
WHSR robo = WHSR();

// Testvariablen für BLE
int farbe = GRUEN;
int basiszeit = 500; // Blinkzeit bei horizontaler Lage [ms]
int blinkzeit = 500; // Lageabhängige Blinkzeit [ms]
int motorLinks = 0;
int motorRechts = 0;

// Befehlscode, der über BLE gesendet wird
int befehl = BEFEHL_ERLEDIGT;

// Zeit für Integration
unsigned long letzteZeit = 0;
// Neigungswinkel um y-Achse
float winkelAlt = 0;


// UUIDs für den Service und die Charakteristiken (übertragbare Werte)
// Die UUIDs für den Service und die Charakteristiken kann man sich z.B. auf https://www.uuidgenerator.net generieren lassen

// BLE-Service des WHSR
BLEService whsrService("3f8e8bbb-e0a0-4ed9-8fed-12ab21af7656");

// Parameter, die über die BLE-Verbindung übermittelt werden
// 2901 steht beim BLEDescriptor immer für die Beschreibung

// Farbe (lesen und schreiben)
BLEByteCharacteristic farbeCharacteristic("089e4562-8f38-4ddd-9cc7-39c7070474d9", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEDescriptor farbeDescriptor("2901", "Farbe");

// Basis-Blinkzeit (lesen und schreiben)
BLEIntCharacteristic zeitCharacteristic("8063c28b-fe4d-4493-8c15-5a518040e32c", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEDescriptor zeitDescriptor("2901", "Basiszeit");

// Geschwindigkeit Motor links (lesen und schreiben)
BLEIntCharacteristic motorLinksCharacteristic("91d54e09-1373-400d-bb97-c01e068e6056", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEDescriptor motorLinksDescriptor("2901", "Geschwindigkeit Motor links");

// Geschwindigkeit Motor rechts (lesen und schreiben)
BLEIntCharacteristic motorRechtsCharacteristic("08eea80b-ea5c-415b-8073-03a92a01aaf2", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEDescriptor motorRechtsDescriptor("2901", "Geschwindigkeit Motor rechts");

// Neigungswinkel (nur lesbar)
BLEFloatCharacteristic phiCharacteristic("f5219e83-5dac-4adb-8418-44fe81777c2f", BLERead | BLENotify);
BLEDescriptor phiDescriptor("2901", "Neigungswinkel");

// Batteriespannung (nur lesbar)
BLEFloatCharacteristic batterieCharacteristic("c55fe8d0-4d14-4588-8463-f5c0c191d539", BLERead | BLENotify);
BLEDescriptor batterieDescriptor("2901", "Batteriespannung");

// Befehl (schreiben)
BLEIntCharacteristic befehlCharacteristic("a5690458-f22d-48db-8901-f6c6f3d6bf38", BLEWrite | BLEWriteWithoutResponse);
BLEDescriptor befehlDescriptor("2901", "Befehl");


void setup()
{
	// Initialize all functional modules of the robot
	robo.Init();

	// Wait some time to give the user a chance to program the robot before it starts moving
	robo.setStatusLED(COLOR_RED);
	delay(1000);
	robo.setStatusLED(COLOR_YELLOW);
	delay(1000);
	robo.setStatusLED(COLOR_GREEN);
	delay(1000);
  robo.setStatusLED(COLOR_OFF);

  if (!IMU.begin()) {
      Serial.println("Trägheitsmessung konnte nicht initialisiert werden!");
      while (1);
  }

  // BLE-Hardware initialisieren
  if (!BLE.begin()) {
      Serial.println("BLE konnte nicht gestartet werden!");
      while (1);
  }

  // So soll das BLE-Gerät heißen
  BLE.setDeviceName("WHSRController");
  // So soll das BLE-Gerät abgekürzt heißen
  BLE.setLocalName("WHSRController");
  // UUID setzen
  BLE.setAdvertisedService(whsrService);

  // Damit man auf dem BLE-Gerät zur Not eine vernünftige Bezeichnung der Parameter hat 
  farbeCharacteristic.addDescriptor(farbeDescriptor);
  zeitCharacteristic.addDescriptor(zeitDescriptor);
  motorLinksCharacteristic.addDescriptor(motorLinksDescriptor);
  motorRechtsCharacteristic.addDescriptor(motorRechtsDescriptor);
  phiCharacteristic.addDescriptor(phiDescriptor);
  befehlCharacteristic.addDescriptor(befehlDescriptor);
  batterieCharacteristic.addDescriptor(batterieDescriptor);

  // Charakteristiken zum Service hinzufügen
  whsrService.addCharacteristic(farbeCharacteristic);
  whsrService.addCharacteristic(zeitCharacteristic);
  whsrService.addCharacteristic(motorLinksCharacteristic);
  whsrService.addCharacteristic(motorRechtsCharacteristic);
  whsrService.addCharacteristic(phiCharacteristic);
  whsrService.addCharacteristic(befehlCharacteristic);
  whsrService.addCharacteristic(batterieCharacteristic);

  // Service hinzufügen
  BLE.addService(whsrService);

  // Event Handler zum Verbinden und Trennen bekanntgeben
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Event Handler für das Schreiben der Charakteristiken bekannt geben
  // Für phi brauchen wir keinen Event-Handler, das wird ja gelesen
  farbeCharacteristic.setEventHandler(BLEWritten, characteristicWritten);
  zeitCharacteristic.setEventHandler(BLEWritten, characteristicWritten);
  motorLinksCharacteristic.setEventHandler(BLEWritten, characteristicWritten);
  motorRechtsCharacteristic.setEventHandler(BLEWritten, characteristicWritten);
  befehlCharacteristic.setEventHandler(BLEWritten, characteristicWritten);

  // Startwerte setzen
  farbeCharacteristic.writeValue(farbe);
  zeitCharacteristic.writeValue(basiszeit);
  motorLinksCharacteristic.writeValue(motorLinks);
  motorRechtsCharacteristic.writeValue(motorRechts);
  befehlCharacteristic.writeValue(BEFEHL_ERLEDIGT);
  
  // Service anbieten
  BLE.advertise();

  // Zeitbasis für Integration der Winkelgeschwindigkeit initialisieren
  letzteZeit = millis();
  // Neigungswinkel mit reinem Beschleunigungswert initialisieren
  winkelAlt = neigungswinkelAcc();
}


void loop() 
{
  // Nach BLE-Events fragen
  BLE.poll(5);  // timeout 5ms

  int dirLinks = motorLinks < 0 ? BWD : FWD;
  int dirRechts = motorRechts < 0 ? BWD : FWD;
  // Serial.println(dirRechts);
  motorLinks = abs(motorLinks) < 50 ? 0 : motorLinks;
  motorRechts = abs(motorRechts) < 50 ? 0 : motorRechts;
  robo.setMotorDirection(dirLinks, dirRechts);
  robo.setMotorSpeed(motorLinks, motorRechts);

  phiCharacteristic.writeValue(neigungswinkel());
  batterieCharacteristic.writeValue(robo.readBattery());
	delay (100);
}


// ------------------------------------ Berechnungsroutinen --------------------------------

/* Berechnung des Neigungswinkels durch die Beschleunigungssensor-Messwerte */
float neigungswinkelAcc(){
     
     float beschlMesswertX;                                                 // Beschleunigungswert (X-Achse)
     float beschlMesswertY;                                                 // Beschleunigungswert (Y-Achse)
     float beschlMesswertZ;                                                 // Beschleunigungswert (Z-Achse)
     #define RAD_ZU_GRAD 57.295779513082321                                // Umrechnungsfaktor zur Umrechnung des Bogenmaß in das Gradmaß

     IMU.readAcceleration(beschlMesswertX,beschlMesswertY,beschlMesswertZ);  
    
     float beschlNeigungswinkel = atan(beschlMesswertX/beschlMesswertZ);          // Neigungwinkel aus der Beschleunigung berechnen. Ergebnis in Radiant 
     beschlNeigungswinkel = beschlNeigungswinkel*RAD_ZU_GRAD;             // Umrechnung des Winkels in das Gradmaß
  
     if(isnan(beschlNeigungswinkel))                                        // Die Fehlermeldung beim Start wird hier abgefangen
        beschlNeigungswinkel = 0.0;
     
     return beschlNeigungswinkel;                                           // Gesamtneigungswinkel wird durch die Methode zurückgegeben
}


/* Methode zur Berechnung des Neigungswinkels durch die Beschleunigungssensor Messwerte */
float neigungswinkel(){
     float beschlNeigungswinkel = neigungswinkelAcc();          // Neigungwinkel aus der Beschleunigung berechnen. Ergebnis in Radiant 
  
     float gyroMesswertX;                                                   // Gyroskopwert (X-Achse)
     float gyroMesswertY;                                                   // Gyroskopwert (Y-Achse)
     float gyroMesswertZ;                                                   // Gyroskopwert (Z-Achse)
     float abweichungRuheLage = 0.15;                                      // Spannungsausgabe bei Ruhelage
     
     IMU.readGyroscope(gyroMesswertX,gyroMesswertY,gyroMesswertZ); // degrees per second

    float deltaZeit = (millis() - letzteZeit)/1000.0; // in Sekunden
    letzteZeit = millis();
     float deltaNeigungswinkel = gyroMesswertY * deltaZeit;                        // Integration der Winkelgeschwindigkeit über die Zeit
     deltaNeigungswinkel = deltaNeigungswinkel - abweichungRuheLage;    // Abweichung bei Ruhelage

    // Aufteilung der Winkelberechnung, Gyro-Anteil
    const float k = 0.95;                            

    // Komplementärfilter gleicht Drift des Gyrosensors und Beschleunigungen des Fahrzeugs aus
    // Low Pass für Beschleunigung, High Pass für Gyro
    // Siehe https://aip.scitation.org/doi/pdf/10.1063/1.5018520
    float gefilterterNeigungswinkel = k * (winkelAlt+deltaNeigungswinkel) + (1.0-k) * beschlNeigungswinkel; // Komplementärfilter
    winkelAlt = gefilterterNeigungswinkel;

     return gefilterterNeigungswinkel;
}


// ------------------------------------ BLE Handler --------------------------------


void blePeripheralConnectHandler(BLEDevice device) {
  Serial.print("Neue Verbindung, Central: ");
  Serial.println(device.address());
  Serial.print("Gerätename: ");
  Serial.println(device.deviceName());
}


void blePeripheralDisconnectHandler(BLEDevice device) {
  Serial.print("Verbindung beendet, Central: ");
  Serial.println(device.address());
}


// Union, um ein Byte-Array in Integer oder Float umwandeln zu können. Wird von characteristicWritten() gebraucht.
union ByteArrayToValue {
  byte array[4];
  uint32_t integerWert;
  float floatWert;
};


// Callback, das gerufen wird, wenn eine Charakteristik ihren Wert geändert hat.
void characteristicWritten(BLEDevice device, BLECharacteristic characteristic) {
  if (characteristic.uuid() == farbeCharacteristic.uuid()) {
    // value() ist ein Zeiger, hier auf ein Byte (Byte ist einfach zu lesen, nicht wie Integer o.ä. (siehe unten)
    farbe = *characteristic.value();
    Serial.print("Farbe: ");
    Serial.println(farbe);
    return;
  }

  // In allen anderen Charakteristiken kommt der Wert als ein Byte-Array mit vier Byte an
  ByteArrayToValue buffer;
  characteristic.readValue(&buffer, 4);
  
  if (characteristic.uuid() == zeitCharacteristic.uuid()) {
    basiszeit = buffer.integerWert;
    blinkzeit = basiszeit;    // Wir machen noch nichts vom Fahrzustand abhängig, kommt noch
    Serial.print("Zeit: ");
    Serial.println(basiszeit);
  } else if (characteristic.uuid() == motorLinksCharacteristic.uuid()) {
    motorLinks = buffer.integerWert;
    Serial.print("MotorLinks: ");
    Serial.println(motorLinks);
  } else if (characteristic.uuid() == motorRechtsCharacteristic.uuid()) {
    motorRechts = buffer.integerWert;
    Serial.print("MotorRechts: ");
    Serial.println(motorRechts);
  } else if (characteristic.uuid() == befehlCharacteristic.uuid()) {
    // Hier wird der Befehl nur gesetzt, nicht ausgeführt.
    // Wenn das Speichern der Parameter im Callback durchgeführt wird,
    // kommt es zu Abstürzen.
    befehl = buffer.integerWert;
    Serial.print("Befehl: ");
    Serial.println(befehl);
  } else {
    Serial.println("KENN ICH NICHT!!");
  }
}


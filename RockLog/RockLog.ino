// --------------------------------------
// Rocker Logger
//
// Version 1
//
// Uses Arduino mini, IMU, BP, EEPROM
// Quick&Dirty first Version...
//

// ============================================== LIBRARIES
#include <Wire.h>
#include <Bme280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#define __DEBUG__ 0

// ==============================================  STATE DEFINITION
#define S_BOOT        0
#define S_LAUNCHPAD   1
#define S_ASCENT      2
#define S_COAST       3
#define S_DESCENT     4
#define S_FALLING     5
#define S_LANDED      6
#define S_RECOVERY    7
#define S_PROGRAMMING 8

int STATE = S_BOOT;      // CURRENT STATE - Start in S_BOOT 


// ============================================== ARDUINO PINS 
#define PIN_TONE      9 // should be 9, 8 to disarm beeper
#define PIN_LAUNCHPAD 2

// ============================================== HELPER MAKROS
#define PAD_FUSE !(digitalRead(PIN_LAUNCHPAD))



// ============================================== GLOBAL DEFINITIONS

Adafruit_MPU6050 mpu;     // IMU Sensor
Bme280TwoWire sensor;     // Pressure & TEMP Sensor


sensors_event_t a, g, temp;

struct dataset { // Dataset of our Datalogger
    unsigned int ts_delta; // Delta Timestamp
    float ax;
    float ay;
    float az;
    float rx;
    float ry;
    float rz;
    unsigned int h;  // Höhe
    unsigned int t;  // t 
};

struct dataset logdata;   // One Dataset for processsing

// global Variables for calculations
unsigned long  ts_last;       // Last Timestamp
float          p_start = 0;   // Barometric Pressure (average after Boot)
float          t_start = 0;   // Start Temperature (average after Boot)

unsigned int   _addr = 0;     // Pointer to current EEPROM Location




// ************************************************************
// Function data_get()
//      Poll Sensore and Store data in global logdata Variable 
void data_get() {
  mpu.getEvent(&a, &g, &temp);  
  
  logdata.ts_delta = a.timestamp - ts_last; 
  logdata.ax = a.acceleration.x; 
  logdata.ay = a.acceleration.y; 
  logdata.az = a.acceleration.z; 

  logdata.rx = g.gyro.x;
  logdata.ry = g.gyro.y;
  logdata.rz = g.gyro.z;

  logdata.h = getAltitude(p_start, sensor.getPressure(), sensor.getTemperature()); 
  logdata.t = sensor.getTemperature() * 100;  
}


// ************************************************************
// Function setup()
//      Arduino Startup
void setup() {
    Wire.begin();
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(38400);
    while (!Serial); // Leonardo: wait for serial monitor
    Serial.println("\nRocket Datalogger Version 1");
    pinMode(PIN_LAUNCHPAD, INPUT_PULLUP);    // set pint to input with pullup
    beep_welcome();
}

// ************************************************************
// Function Beep-Tones
void beep_welcome() {
    tone(PIN_TONE, 2000, 20);
    delay(40);
    tone(PIN_TONE, 2600, 20);
    delay(40);
    tone(PIN_TONE, 3000, 40);
}

// ************************************************************
// Function Init_MPU()
//      Initialization of the IMU Sensor 
void Init_MPU() {
    if (!mpu.begin()) {
    Serial.println("### FAILED ###");
      while (1) {
        delay(10);
      }
    }
    Serial.print("OK  :");

    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.print("  +-2G  /");
      break;
    case MPU6050_RANGE_4_G:
      Serial.print("  +-4G  /");
      break;
    case MPU6050_RANGE_8_G:
      Serial.print("  +-8G  /");
      break;
    case MPU6050_RANGE_16_G:
      Serial.print("  +-16G  /");
      break;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.print("  +- 250 deg/s  /");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.print("  +- 500 deg/s  /");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.print("  +- 1000 deg/s  /");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.print("  +- 2000 deg/s  /");
      break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("  260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("  184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("  94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("  44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("  21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("  10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("  5 Hz");
      break;
    }
    // give it some time to stabilize
    Serial.print("                             IMU Runup ");
    for (int i = 0; i<10; i++) {
          tone(PIN_TONE, 300, 50);
          Serial.print(".");
          delay(400);
    }
    Serial.println("done");
}



// ************************************************************
// Function Calculate Barometric Altitue
//      Initialization of the IMU Sensor 
float getAltitude(float p0, float p1, float t) {
  return ((pow((p0 / p1), 1/5.257) - 1.0) * (t + 273.15)) / 0.0065;
}


// #############################################################
// ##############  STATE PROCESSING ############################
// #############################################################
// --------------      BOOT         ----------------------------
// Pre-Flight initialization
// -------------------------------------------------------------
void boot() {
    
    Serial.println("Booting...");
    Serial.print("Sensor 1 / IMU MPU6050  >>>  ");
     Init_MPU();
    
    Serial.print("Sensor 2 / P            >>>  ");
     sensor.begin(Bme280TwoWireAddress::Primary);
     sensor.setSettings(Bme280Settings::indoor());

     // Start Ambient Pressue - average
     p_start = sensor.getPressure(); 
     for (int i=0; i<10; i++) { delay(50); p_start = (p_start + sensor.getPressure()) / 2; }
    Serial.print("P_avg: "); Serial.print(p_start); 
     // Start Temperatur - average
     t_start = sensor.getTemperature(); 
     for (int i=0; i<10; i++) { delay(50); t_start = (t_start + sensor.getTemperature()) / 2; }
    Serial.print(" T_avg: "); Serial.print(t_start); 
    Serial.print(" >> ALT: "); Serial.print(getAltitude(p_start, sensor.getPressure(), sensor.getTemperature())); Serial.println(" m");
      
    Serial.print("EEPROM                  >>>  ");
      Serial.println("not implemented");

    // Save t_start & p_start to EEPROM
    // Save Dataset 0 to EEPROM     
    Serial.println("\n*** Preflight bootup completed ***");
    delay(400); Change_State(S_LAUNCHPAD);
}

// ------- Ready for Take-off -------------------------------
void launchpad() {
   //tone(PIN_TONE, 2500, 20);
   delay(400);
   logdata_write(); 
    // wait for high a value -> go to ascent / fast logging
}

// ------- Power Ascent -------------------------------------
void ascent() {
     // leave if a<treshhold or FUSE detected
    delay(400); Change_State(S_COAST);
}

// ------- Costing to apogee --------------------------------
void coast() {
   // leave if a shock, speec > treshhold or FUSE detected
    delay(400); Change_State(S_DESCENT);
}

// ------- Descending on Parachute --------------------------
void descent() {
    delay(400); Change_State(S_LANDED);
}

// ------- Falling down / no Parachute ----------------------
void falling() {
    delay(400); Change_State(S_LANDED);
}

// ------- Landed / Impact on Ground ------------------------
void landed() {
    delay(400); Change_State(S_RECOVERY);
}

// ------- Recovery / Picked up from ground -----------------
void recovery() {
    delay(400); Change_State(S_PROGRAMMING);
}


// ------- Programming Mode ----------------------------------
void programming() {
    char cmd;
    if ( !PAD_FUSE ) {  Change_State(S_LAUNCHPAD); return; }
    // wait for 
     if (Serial.available())  {
      cmd = Serial.read(); //reads serial input
    }
    switch ( cmd) {
      case 'r': Serial.println("\nREAD FLIGHT DATA\n"); logdata_read(); break;
      case 'd': Serial.println("\nDELETE / RESET DATA WRITE\n"); logdata_reset(); break;
      case 'b': Serial.println("\nREBOOT\n"); boot(); break;
    }
    delay(200); 
}

void logdata_read() {

  unsigned int _r_addr = 0;
  // Read init data
  for (unsigned int i=32; i<1000; i+=32) {
    Serial.print(" "); Serial.print(i);
    // Read data from i
    // Show Data on Serial
  }
 Serial.println("not implemented\n Test only\n");

  data_get();
 
  Serial.println("\nD Timestamp |   Ax   |   Ay   |   Az   |   Rx   |   Ry   |   Rz   |   h   |   T   "); 
  Serial.print(logdata.ts_delta); Serial.print(" | ");
  Serial.print(logdata.ax); Serial.print(" | ");
  Serial.print(logdata.ay); Serial.print(" | ");
  Serial.print(logdata.az); Serial.print(" | ");
  Serial.print(logdata.rx); Serial.print(" | ");
  Serial.print(logdata.ry); Serial.print(" | ");
  Serial.print(logdata.rz); Serial.print(" | ");
  Serial.print(logdata.h); Serial.print(" | ");
  Serial.print(logdata.t); Serial.print(" \n\n "); 

}

void logdata_write() {
  _addr+=32;
  if (_addr > 5000 ) { // EEPROM FULL
     return;
  }
  // Write logdata -> EEPROM on Addr _addr
  Serial.print(_addr);
}

void logdata_reset() {
  _addr=32;
 Serial.println("NOT IMPLEMENTED");

}



// ------- Change to new State ----------------------------------
void Change_State(int newState) {
    STATE = newState;
    switch (STATE) {
      case S_BOOT:             break;
      case S_LAUNCHPAD:        break;
      case S_ASCENT:           break;
      case S_COAST:            break;
      case S_DESCENT:          break;
      case S_FALLING:          break;
      case S_LANDED:           break;
      case S_RECOVERY:         break;
      case S_PROGRAMMING:  beep_welcome(); Serial.println("\nPROGRAMMING MODE - use [d] delete [r] read   [b] reboot \n"); break;

      // ERROR STATE?
    }
    if ( __DEBUG__ ) { Serial.print("S["); Serial.print(newState);Serial.print("]\n"); }
}


// #################################################################
// #################################################################
void loop() {
 if ( __DEBUG__ ) { Serial.print(STATE); }
    switch (STATE) {
      case S_BOOT:        boot();         break;
      case S_LAUNCHPAD:   launchpad();    break;
      case S_ASCENT:      ascent();       break;
      case S_COAST:       coast();        break;
      case S_DESCENT:     descent();      break;
      case S_FALLING:     falling();      break;
      case S_LANDED:      landed();       break;
      case S_RECOVERY:    recovery();     break;
      case S_PROGRAMMING: programming();  break;

      // ERROR STATE?
    }

    // Fill up cycle time
    // 0 = no fixed cycle, just proceed 
    
      // If Launch Pad Pin (Fuse) set to to programming mode
    //if ( PAD_FUSE & !( STATE = S_PROGRAMMING)) {
    if ( PAD_FUSE & !(STATE == S_PROGRAMMING) ) {
       Change_State(S_PROGRAMMING); // if fuse set go to programming mode
    }

}

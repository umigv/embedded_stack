// file pulled from https://github.com/odriverobotics/ODrive/tree/master/Arduino/ODriveArduino
// this file is flashed to the Arduino Mega to control the ODrive via serial/uart
// after flashing set the PC's baud rate to 115200
// for more information read this repo's readme

// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial1;
HardwareSerial& odrive_serial2 = Serial2;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(8, 9);


// ODrive object
//ODriveArduino odrive(odrive_serial);
ODriveArduino odrive2(odrive_serial2);

void setupODrive(ODriveArduino& odrive);
void setupODriveParams(ODriveArduino& odrive);
void closedLoopControl(ODriveArduino& odrive);

void setup() {
  // ODrive uses 115200 baud
  //odrive_serial.begin(115200);
  odrive_serial2.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  //while (!Serial) ; // wait for Arduino Serial Monitor to open
  while (!Serial2); // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  //setupODriveParams(odrive);
  setupODriveParams(odrive2);
  int requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  //odrive.run_state(0, requested_state, false);
  odrive2.run_state(0, requested_state, false);
  delay(19000);
  //odrive.run_state(1, requested_state, false);
  odrive2.run_state(1, requested_state, false);
  delay(19000);
  Serial.println("Finished calibration!");
  //closedLoopControl(odrive);
  closedLoopControl(odrive2);
  

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop() {
  static uint32_t counter = 1;
  uint8_t stop = digitalRead(6);
  static uint8_t tuning = 0;
  if (counter == 500) {
    Serial.print("From pin 6: ");
    Serial.println(stop);
  }
//
//  if (Serial.available()) {
//    //char c = Serial.read();
//    byte vel = Serial.read();
//
//    odrive.SetVelocity(0, (int)vel);
//    odrive.SetVelocity(1, (int)vel);
//    return;
//  }

String readString = "";
char c = '\0';
while (Serial.available()) {
    c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the String readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  // Don't update until user types a new value or if button is pressed
  if (readString == "q") {
    return;
  }
  else if (stop) {
    if (counter == 500) Serial.println("Input velocity: 0" + readString);  //so you can see the captured String
    //odrive.SetVelocity(0, 0);
    //odrive.SetVelocity(1, 0);
    odrive2.SetVelocity(0, 0);
    odrive2.SetVelocity(1, 0);
  }
//    else if (readString  == "t") { // tuning
//    tuning = !tuning;
//    //serial << "w axis" << motor << ".controller.config.vel_gain = " << val << "\n";
//    //serial << "w axis" << motor << ".controller.config.vel__integrator_gain = " << val << "\n";
//  }
//  else if (tuning && c == 'v') {
//    // TODO fix it so it reads floats
//    odrive_serial2 << "r axis" << 0 << ".controller.config.vel_gain\n";
//    Serial.print("Current vel_gain for axis0: ");
//    Serial.println(odrive2.readFloat());
//    odrive_serial2 << "r axis" << 1 << ".controller.config.vel_gain\n";
//    Serial.print("Current vel_gain for axis1: ");
//    Serial.println(odrive2.readFloat());
//    String readVal = "";
//    while (!Serial.available()) {
//      readVal = "";
//      while(Serial.available()) {
//        char ch = Serial.read();  //gets one byte from serial buffer
//        readVal += ch; //makes the String readString
//        delay(2);  //slow looping to allow buffer to fill with next character
//      }
//      if (readVal.length() > 0) break;
//    }
//    odrive_serial2 << "w axis" << 0 << ".controller.config.vel_gain = " << readVal << "\n";
//    odrive_serial2 << "w axis" << 1 << ".controller.config.vel_gain = " << readVal << "\n";
//    Serial.println("Vel gains set to " + readVal + ".\n");
//  }
  else if (readString.length() > 0 || stop) {
    if (stop && counter == 500) Serial.println("Input velocity: 0" + readString);  //so you can see the captured String 
    else if (!stop) Serial.println("Input velocity: " + readString);  //so you can see the captured String 
    float n = digitalRead(6) ? 0 : readString.toFloat();  //convert readString into a number
    //Serial.println(n); //so you can see the integer

    //odrive.SetVelocity(0, n);
    //odrive.SetVelocity(1, n);
    odrive2.SetVelocity(0, n);
    odrive2.SetVelocity(1, n);
    readString = ""; // reset string
  }


  else if (readString.length() > 0) {
    float n = readString.toFloat();
    //odrive.SetVelocity(0, n);
    //odrive.SetVelocity(1, n);
    odrive2.SetVelocity(0, n);
    odrive2.SetVelocity(1, n);
  }
  if (counter == 500) {
    Serial.print("Tuning: ");
    Serial.println(tuning);
    //Serial.print("ODrive 1 Position: ");
    //Serial.print(odrive.GetPosition(0));
    //Serial.print("\t");
    //Serial.println(odrive.GetPosition(1));
    Serial.print("ODrive 2 Position: ");
    Serial.print(odrive2.GetPosition(0));
    Serial.print("\t");
    Serial.println(odrive2.GetPosition(1));
    Serial.println();
    //Serial.print("ODrive 1 Velocity: ");
    //Serial.print(odrive.GetVelocity(0));
    //Serial.print("\t");
    //Serial.println(odrive.GetVelocity(1));
    Serial.print("ODrive 2 Velocity: ");
    Serial.print(odrive2.GetVelocity(0));
    Serial.print("\t");
    Serial.println(odrive2.GetVelocity(1));
    Serial.println();
//    Serial.println("ODrive 1 CPR: ");
//    odrive_serial << "r axis" << 0 << ".encoder.pos_cpr_counts\n";
//    Serial.println(odrive.readFloat());
//    odrive_serial << "r axis" << 1 << ".encoder.pos_cpr_counts\n";
//    Serial.println(odrive.readFloat());
//    Serial.println("ODrive 2 CPR: ");
//    odrive_serial2 << "r axis" << 0 << ".encoder.pos_cpr_counts\n";
//    Serial.println(odrive2.readFloat());
//    odrive_serial2 << "r axis" << 1 << ".encoder.pos_cpr_counts\n";
//    Serial.println(odrive2.readFloat());
    counter = 0;
  }
  delay(1);
  readString = ""; // reset
  c = '\0'; // reset
  ++counter;
}

void setupODriveParams(ODriveArduino& odrive) {
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 30.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 60.0f << '\n';
  }
}

void closedLoopControl(ODriveArduino& odrive) {
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);
  
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);
}
//
//void setVelGain(int motor, HardwareSerial& serial, float val) {
//  serial << "w axis" << motor << ".controller.config.vel_gain = " << val << "\n";
//}
//
//float readVelGain(int motor, HardwareSerial& serial) {
//  serial << "w axis" << motor << ".controller.config.vel_gain";
//  
//}

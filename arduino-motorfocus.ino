#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <EEPROM.h>

SoftwareSerial debugSerial(2, 3);

const int stepsPerRevolution = 32*64;  // change this to fit the number of steps per revolution
const int maxSpeed = 10;
const int maxCmd = 8;

// initialize the stepper library on pins 8 through 11:
AccelStepper stepper(AccelStepper::FULL4WIRE, 6, 4, 5, 3, false);

// multiplier of SPEEDMUX, currently max speed is 480.
int speedFactor = 16;
int speedFactorRaw = 2;
int speedMult = 30;

long steps = 8;
long currentPosition = 0;
long targetPosition = 0;
long lastSavedPosition = 0;
long millisLastMove = 0;
const long millisDisableDelay = 15000;
bool isRunning = false;

// read commands
bool eoc = false;
String line;

void setup() {
  Serial.begin(9600);
  debugSerial.begin(9600);


  // initalize motor
  stepper.setMaxSpeed(speedFactor * speedMult);
  stepper.setAcceleration(10);
  millisLastMove = millis();
    
  // read saved position from EEPROM
  //EEPROM.put(0, (long)0);
  EEPROM.get(0, currentPosition);
  stepper.setCurrentPosition(currentPosition);
  lastSavedPosition = currentPosition;
  debugSerial.print("Load last position from EEPROM...");
  debugSerial.println(lastSavedPosition);
}

void loop() {
 
  // process the command we got
  if (eoc) {
    //Serial.println(line);
    String cmd, param;
    int len = line.length();
    if (len >= 2) {
      cmd = line.substring(0, 2);
    }
    if (len > 2) {
      param = line.substring(2);
    }
    
    //debugSerial.print(cmd);
    //debugSerial.print(":");
    //debugSerial.println(param);
    //debugSerial.println(line);
    line = "";
    eoc = false;

    // LED backlight value, always return "00"
    if (cmd.equalsIgnoreCase("GB")) {
      Serial.print("00#");
    }
    // home the motor, hard-coded, ignore parameters since we only have one motor
    if (cmd.equalsIgnoreCase("PH")) { 
      stepper.setCurrentPosition(8000);
      stepper.moveTo(0);
      isRunning = true;
    }
    // firmware value, always return "10"
    if (cmd.equalsIgnoreCase("GV")) {
      Serial.print("10#");
    }
    // get the current motor position
    if (cmd.equalsIgnoreCase("GP")) {
      currentPosition = stepper.currentPosition();
      char tempString[6];
      sprintf(tempString, "%04X", currentPosition);
      Serial.print(tempString);
      Serial.print("#");

      debugSerial.print("current motor position: 0x");
      debugSerial.print(tempString);
      debugSerial.print(" = ");
      debugSerial.println(currentPosition);
    }
    // get the new motor position (target)
    if (cmd.equalsIgnoreCase("GN")) {
      //pos = stepper.targetPosition();
      char tempString[6];
      sprintf(tempString, "%04X", targetPosition);
      Serial.print(tempString);
      Serial.print("#");
      
      debugSerial.print("target motor position: ");
      debugSerial.println(tempString);
    }
    // get the current temperature, hard-coded
    if (cmd.equalsIgnoreCase("GT")) {
      Serial.print("0020#");
    }

    // get the temperature coefficient, hard-coded
    if (cmd.equalsIgnoreCase("GC")) {
      Serial.print("02#");
    }
    
    // get the current motor speed, only values of 02, 04, 08, 10, 20
    if (cmd.equalsIgnoreCase("GD")) {
      char tempString[6];
      sprintf(tempString, "%02X", speedFactorRaw);
      Serial.print(tempString);
      Serial.print("#");

      debugSerial.print("current motor speed: ");
      debugSerial.println(tempString);
    }
    
    // set speed, only acceptable values are 02, 04, 08, 10, 20
    if (cmd.equalsIgnoreCase("SD")) {
      speedFactorRaw = hexstr2long(param);

      // SpeedFactor: smaller value means faster
      speedFactor = 32 / speedFactorRaw;
      stepper.setMaxSpeed(speedFactor * speedMult);
    }
    
    // whether half-step is enabled or not, always return "00"
    if (cmd.equalsIgnoreCase("GH")) {
      Serial.print("00#");
    }
    
    // motor is moving - 01 if moving, 00 otherwise
    if (cmd.equalsIgnoreCase("GI")) {
      if(abs(targetPosition - currentPosition) > 0){
        Serial.print("01#");
      } else {
        Serial.print("00#");
      }
    }  
    // set current motor position
    if (cmd.equalsIgnoreCase("SP")) {
      currentPosition = hexstr2long(param) * 16;
      stepper.setCurrentPosition(currentPosition);      
    }
    // set new motor position
    if (cmd.equalsIgnoreCase("SN")) {
      //Serial.println(param);
      debugSerial.print("new target position ");
      debugSerial.print(targetPosition);
      debugSerial.print(" -> ");
      targetPosition = hexstr2long(param) * 16;
      debugSerial.println(targetPosition);
      //Serial.println(targetPosition);
      //stepper.moveTo(pos);
    }
    // initiate a move
    if (cmd.equalsIgnoreCase("FG")) {
      //isRunning = 1;
      //stepper.enableOutputs();
      //running = true;
      stepper.enableOutputs();
      stepper.moveTo(targetPosition);
    }
    // stop a move
    if (cmd.equalsIgnoreCase("FQ")) {
      //isRunning = 0;
      //stepper.moveTo(stepper.currentPosition());
      //stepper.run();
      //running = false;
      stepper.stop();
    }
  }

  // move motor if not done
  if (stepper.distanceToGo() != 0) {
    isRunning = true;
    millisLastMove = millis();
    stepper.run();
  }
  else {
    isRunning = false;
    // @todo: turn stepper off, save point to EEPROM
    if(millis() - millisLastMove > millisDisableDelay){
      // Save current location in EEPROM
      if (lastSavedPosition != currentPosition) {
        EEPROM.put(0, currentPosition);
        lastSavedPosition = currentPosition;
        debugSerial.println("Save last position to EEPROM");
      }
    }
  }

  //delay(20);
}

// read the command until the terminating # character
void serialEvent () {
  // read the command until the terminating # character
  while (Serial.available() && !eoc) {
    char c = Serial.read();
    if (c != '#' && c != ':') {
      line = line + c;
    }
    else {
      if (c == '#') {
        eoc = true;
      }
    }
  }
}

long hexstr2long(String line) {
  char buf[line.length()+1];
  line.toCharArray(buf, line.length());
  long ret = 0;

  ret = strtol(buf, NULL, 16);
  return (ret);
}

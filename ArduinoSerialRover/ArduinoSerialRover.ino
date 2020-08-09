/*
   Make header of packet == 255
   IF SPEED/STEERING/BRAKE NOT CHANGED DO NOT SEND  TO AUTOPILOT A NEW COMMAND

   try to send new cordinates using G serial command try one point then multible points
   check error using E command
   stop/start debugging using D command

  COMMANDS THAT I CAN SEND TO YOU:
  1)  'S' Start navigation using the latest cordinates that i sent you
  2)  'B' stope any navigation and do not send me any things
  3)  'F' Finish the Navigation
  4)  "51.9228757,4.4800224" or more points can be sended as follow any time so you adjust the end points accordiganlly
  if the robot was controlled from your side then i expect brake command untill you do the new routing


  COMMANDS THAT I RECIVE FROM YOU:
 ** BRAKE VALUES:
  0 STOPE THE DROID BECASUE OF WEEK SIGNAL FROM GPS
  1 STOPE THE DROID BECASUE YOU ARE AWAY FROM THE ORIGINAL PATH AND CAN NOT BE CORRECTED
  2 STOPE THE DROID BECASUE YOU ARE DOING RE ROUTING
  3 - 253 Error from 0 to 10 meter 3 mean less than one meter and 253 mean higher than 100 meter
  254 End of mission
  .
  .

  255 THE POINTS THAT I SENT YOU ARE NOT CORRECT AND THEY SHOULD BE CHANGED OR SENDED AGAIN

  **Steering command -45.0 to +45.0 degrees for now
  if you need a steering angle of zero you will send me 4500 command
  if you need +5.55 degrees then x=+5.55
  steering to send=(x*100.0)+4500.0
  steering to send=5055

  Speed command 0 to 62000 degrees for now
  if you need a zero speed you will send me 31000 command
  +values for forwared speed and negative ones for reverse

  
  ** Fix Mode
  
  0=Invalid
  1=2D/3D
  2=DGNSS
  4=Fixed RTK
  5=Float RTK
  6=Dead Reckoning

*/

unsigned long G_Error[3];
String E_String = "";
int GPS_DEBUG = 0;

long timeBefore = 0;

uint16_t SteeringAngle = 0;
uint16_t RobotSpeed = 0;
uint16_t BrakeValue = 0;
uint16_t FixModeValue = 0;
float angle_steering = 0.0;

byte byteStruct[7];
String cordinates;
void setup() {
  

  
  Serial.begin(115200);
  Serial1.begin(115200);

}

void loop() {

  if (Serial.available() > 0) {
    int inChar = Serial.read();

    if (inChar == 'G') { //GPS cordinates
      Serial.print("GPS_Cordinates=");
      Serial.println(E_String);
      sendNumberOfLocations(E_String);
      E_String = "";
    }
    else  if (inChar == 'E') { //GPS ERRORS
      Serial.print("GPS_ERRORS=");
      Serial.print(G_Error[0]);
      Serial.print("\t");
      Serial.print(G_Error[1]);
      Serial.print("\t");
      Serial.println(G_Error[2]);
      E_String = "";
    }
    else  if (inChar == 'D') { //GPS_DEBUG
      if(GPS_DEBUG==0) {
            GPS_DEBUG = 1;
          }else if(GPS_DEBUG==1) {
            GPS_DEBUG = 0;
          }
      
      Serial.print("GPS_DEBUG=");
      Serial.println(GPS_DEBUG);
      E_String = "";
    }
    else  if (inChar == 'S') { //START
      sendNumberOfLocations("SS");
      Serial.println("START");
      E_String = "";
    }
    else  if (inChar == 'B') { //STOPE
      sendNumberOfLocations("BB");
      Serial.println("STOPE");
      E_String = "";
    }else  if (inChar == 'F') { //STOPE
      sendNumberOfLocations("FF");
      Serial.println("Finish");
      E_String = "";
    }else{
      E_String += (char) inChar;
    }
  }

  receiveEvent();
}



void receiveEvent() {
  
  timeBefore = micros();
  if (Serial1.available() == 7)
  {
    for (int i = 0; i < 7; i++){
      byteStruct[i] = Serial1.read();
    }
    
    if (byteStruct[0] == 0xFF)
    {
      SteeringAngle = (byteStruct[1] << 8) | byteStruct[2];
      angle_steering = float(SteeringAngle - 4500) / 100.0;
      RobotSpeed = (byteStruct[3] << 8) | byteStruct[4];
      RobotSpeed -= 31000;
      BrakeValue = (byteStruct[5]);
    FixModeValue = (byteStruct[6]);

      if (micros() - timeBefore > 15)
      {
        Serial.print("ERROR TIME!!!!!!=");
        Serial.println(micros() - timeBefore);
        G_Error[2]++;
      }
      if (GPS_DEBUG)
      {
        Serial.print(micros() - timeBefore);
        Serial.print("\t");
        Serial.print(angle_steering);
        Serial.print("\t");
        Serial.print(RobotSpeed);
        Serial.print("\t");
        Serial.print(BrakeValue);
        Serial.print("\t");
        Serial.print(FixModeValue);
        Serial.print("\t");
        Serial.println(7);
      }
    }
    else
      G_Error[0]++;//wrong gps header
  }
  else if(Serial1.available()>7){
    //why i am here
    G_Error[1]++;//more bytes than expected
    Serial.println(Serial1.read());
  }
    
}

void sendNumberOfLocations(String da){
  Serial1.println(da);
}


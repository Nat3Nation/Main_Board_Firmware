/*
  Multple Serial test

 Receives from the main serial port, sends to the others.
 Receives from serial port 1, sends to the main serial (Serial 0).

 This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc

 The circuit:
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:

 created 30 Dec. 2008
 modified 20 May 2012
 by Tom Igoe & Jed Roach
 modified 27 Nov 2015
 by Arturo Guadalupi

 This example code is in the public domain.

 */

String preamble = "radio tx ";
/*
"action:energy_record"
"id:ABC123"
"v_mag:0.00"
"i_mag:0.00"
"pow_factor:1.00"
*/
String transmissions[] = {"616374696F6E3A656E657267795F7265636F7264", "69643A414243313233",
                          "765F6D61673A302E303030", "695F6D61673A302E303030",
                          "706F775F666163746F723A312E3030"};


void setup() {
  // power RN2483
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  
  // initialize both serial ports:
  Serial.begin(57600);
  Serial1.begin(57600);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  //Transmit Data Packet
  delay(5000);
  readPacket();
  delay(1000);
  Serial1.println("mac pause");
  delay(1000);
  readPacket();
  delay(1000);
  Serial1.println("radio set freq 915000000");
  delay(1000);
  readPacket();
  delay(1000);
  Serial1.println("radio set cr 4/8");
  delay(1000);
  readPacket();
  delay(1000);
  /*
  String message;
  char *send;
  for(int i = 0; i < 5; i++) {
    message = preamble + transmissions[i];
    //send = message.c_str();
    Serial1.println(message);
    delay(2000);
    readPacket();
    delay(1000);
  }
  */
  String message;
  char *send;
  message = preamble + "626F6172642D3036392C3132302E302C3131382E352C3131392E322C352E322C342E382C352E302C302E302C3132302E302C2D3132302E302C33302E352C3135302E322C2D39302E382C302E39322C3632342E302C3537342E30382C3235352E33362C4E61746573277320686F7573652C343230303030303030";
  Serial1.println(message);
  delay(2000);
  readPacket();
  delay(1000);
}

void readPacket() {
  while (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // read from port 0, send to port 1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }
}

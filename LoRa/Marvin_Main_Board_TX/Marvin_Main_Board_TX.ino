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
  message = preamble + "7265636F72642C3132333435393837362C3132302C302C302C31302C302C302C39302C302C302C34352C302C302C312E30302C302C312C30";
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

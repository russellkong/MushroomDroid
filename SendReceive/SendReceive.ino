/*
  Simple example for receiving

  https://github.com/sui77/rc-switch/
*/

#include <RCSwitch.h>

RCSwitch myReceiver = RCSwitch();

RCSwitch mySender = RCSwitch();
int myRound = 0;

unsigned long vfan[2][2] = {{1117011, 1117810}, {2217011, 2217810}};
unsigned long mist[2][2] = {{1717200, 1817218}, {2218221, 2218230}};
//byte cooler = 5;
unsigned long light[2][2] = {{1330303, 1331818}, {2219331, 2219440}};
unsigned long cfan[2][2] = {{17400, 17418}, {2216441, 2216450}};
unsigned long heat[2][2] = {{17500, 17518}, {2215551, 2215560}};

int room = 9;
int target = 9;
int action = 0;
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(999999);
  myReceiver.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
  mySender.enableTransmit(10);
  Serial.println("Select Room [0;1]");
  room = Serial.parseInt();
  Serial.print("Room "); Serial.print(room); Serial.println(" selected");
}

void loop() {

  Serial.print(myRound++);
  if ( myRound % 2 == 0) {
    while (myReceiver.available()) {
      Serial.println("Receiving...");
      int value = myReceiver.getReceivedValue();

      if (value == 0) {
        Serial.print("Unknown encoding");
      } else {
        Serial.print("Received ");
        Serial.print( myReceiver.getReceivedValue() );
        Serial.print(" / ");
        Serial.print( myReceiver.getReceivedBitlength() );
        Serial.print("bit ");
        Serial.print("Protocol: ");
        Serial.println( myReceiver.getReceivedProtocol() );
      }
      myReceiver.resetAvailable();
    }
  } else {
    Serial.println("Send");
    Serial.println("Select target [1:light; 2:vfan 3:mister]");
    target = Serial.parseInt();
    Serial.println("Select target opertation [0:On; 1:Off]");
    action = Serial.parseInt();
    switch (target) {
      case 1:
        mySender.send(light[room][action],24);
        Serial.println(light[room][action]);
        break;
      case 2:
        mySender.send(vfan[room][action],24);
        Serial.println(vfan[room][action]);
        break;
      case 3:
        mySender.send(mist[room][action],24);
        Serial.println(mist[room][action]);
//        delay(3000);
//        Serial.println(mist[room][1]);
        break;
    }
    //mySender.sendTriState("00000FFF0F0F");
  }
  delay(1000);
}

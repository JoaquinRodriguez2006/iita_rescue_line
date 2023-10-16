#include "HC05.h"

#ifdef HC05_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
HC05 btSerial = HC05(A2, A5, A3, A4);  // key, state, rx, tx
#else
HC05 btSerial = HC05(3, 2);  // cmd, state
#endif

#ifdef DEBUG_HC05
#ifdef DEBUG_SW_PORT
  extern SoftwareSerial DEBUG_PORT;      //This will allow me to print to serial without sending to BT
#endif
#endif

String BTs;

void setup() {
 DEBUG_BEGIN(38400);
 DEBUG_PRINTLN("Setup");
  delay(3000);  // this delay is for debugging convenience only
 DEBUG_PRINTLN("DelayComplete");

  //Let's startup the device and clear it from past searching
  btSerial.findBaud();
  delay(2000);
  btSerial.setTimeout(1000);
  btSerial.cmd("AT+ORGL"); //factory reset
  btSerial.setBaud(38400);
  btSerial.cmd("AT+INQC");

  //Setup the bluetooth as a master
  btSerial.cmd("AT+ROLE=1"); //Master
  btSerial.cmd("AT+NAME=CHILD"); //Just a name given to the device
  btSerial.cmd("AT+PSWD=1234"); //Password shared with Slave device
  btSerial.cmd("AT+CMODE=0");
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");

  //Let's setup a connection
  DEBUG_PRINTLN(" ");
  btSerial.cmd("AT+INIT"); //initialize SPP 
  DEBUG_PRINTLN(" ");
  btSerial.cmd("AT+INQM=0,5,9"); //search for 5 devices, timeout 9 sec
  DEBUG_PRINTLN(" ");
  btSerial.cmd("AT+INQ"); //Search for devices
    BTs+=btSerial.readString(); 
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN(" ");
  DEBUG_PRINTLN("Here come BT addresses:");
  DEBUG_PRINTLN(BTs);
}

void loop() {
}
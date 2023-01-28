
// The serial TX and RX pins used to communicate with the vehicle.
const byte RX_PIN = 0;
const byte TX_PIN = 1;

const byte LED_PIN = 13;

long lastActivityStamp = 0;
int state_A = 0;
int state_B = 0;

// Number of received byte to ignore - used to eliminate
// process of bytes that we send on the K-Line (i.e. echo
// ignore).
int ignoreCount = 0;

// This is the buffer that we use to accumulate received data.
const int MAX_RX_MSG_LEN = 256;
byte rxMsg[MAX_RX_MSG_LEN];
int rxMsgLen = 0;

// Inspired by SternOBDII\code\checksum.c
static byte iso_checksum(byte *data, byte len) {
  byte crc = 0;
  for (byte i = 0; i < len; i++)
    crc = crc + data[i];
  return crc;
}

/**
 * Handles a byte received on the OBD2 port.
 */
static void processByte(byte b) {
  
  rxMsg[rxMsgLen] = b;
  
  if (rxMsgLen < MAX_RX_MSG_LEN) {
    rxMsgLen++;
  }

  if (state_B == 1) {
    if (b == ~0x08) {
      state_B = 2;
      rxMsgLen = 0;
      Serial1.write(0xcc);
    }
  }
  else if (state_B == 2) {
    {
      char buf[16];
      sprintf(buf,"%x",(int)b);
      Serial.println(buf);
    }
  }
}

void setup() {

  // Console
  Serial.begin(9600);

  // Define pin modes for TX and RX
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  // Idle state for TX
  digitalWrite(TX_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  // Idle state for the LED pin
  digitalWrite(LED_PIN, LOW);

  // Stobe LED so that we know things are running
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);

  Serial.println("OBD2 Diagnostic Scanner V1.3");
 
  delay(2000);

  // Set the baud rate for the ISO9141 serial port
  Serial1.begin(10400);
        
  state_A = 0;
  state_B = 0;
  lastActivityStamp = millis();
}

void loop() {

  long now = millis();

  if (state_A == 0) {
    // Look for the start of a long low
    if (digitalRead(RX_PIN) == 0) {
      state_A = 1;
      lastActivityStamp = now;
    }
  }
  else if (state_A == 1) {
    // Look for the case where the line stays low
    if (digitalRead(RX_PIN) == 0) {
      // Low for more than 1 bit in a 5-baud interval?
      if (now - lastActivityStamp > 190) {
        Serial.println("INFO: 5-Baud init detected");
        state_A = 2;
        state_B = 0;
        lastActivityStamp = now;
      }
    } 
    // If line went high then go back to idle
    else {
      state_A = 0;
      lastActivityStamp = now;
    }
  }
  else if (state_A == 2) {

    // Look for a timeout
    if ((now - lastActivityStamp) > 5000) {
      state_A = 0;
      Serial.println("INFO: Initialization timed out (0)");
    }
    // Process state machine
    else {

      // Delay during initialization
      if (state_B == 0) {
        // After delay send the 
        if (now - lastActivityStamp > 2000) {
          Serial1.write(0x55);
          Serial1.write(0x08);
          Serial1.write(0x08);
          state_B = 1;
          lastActivityStamp = now;
        }
      }
      // Now we are waiting for the ~KW1 from the scanner
      else if (state_B == 1) {
        if (now - lastActivityStamp > 2000) {
          // Initialization failed - go back
          state_A = 0;
          Serial.println("INFO: Initialization timed out (1)");
        }  
      }      
  
      // Check to see if we have any data from the K-Line
      if (Serial1.available() > 0) {
    
        // Read a byte from the K-Line
        int r = Serial1.read();
    
        // The K-Line has transmit and receive data so check to see 
        // if we should ignore our own transmission.
        if (ignoreCount > 0) {
          ignoreCount--;
        } 
        else {
          processByte(r);
          lastActivityStamp = now;
        }    
      }
    }
  }
}

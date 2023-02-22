
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

// Supported
static int generate_01_00(unsigned long x, byte pid, Stream& str) {
  byte a, b, c, d;
  a = (byte)((x >> 24) & 0xff);  
  b = (byte)((x >> 16) & 0xff);  
  c = (byte)((x >> 8) & 0xff);  
  d = (byte)((x) & 0xff);    
  byte out[10];
  out[0] = 0x48;
  out[1] = 0x6b;
  out[2] = 0x10;
  out[3] = 0x41;
  out[4] = pid;
  out[5] = a;
  out[6] = b;
  out[7] = c;
  out[8] = d;
  out[9] = iso_checksum(out, 9);
  str.write(out, 10);
  return 10;
}

// Engine speed
static int generate_01_0c(unsigned int rpm, Stream& str) {
  unsigned int r = rpm * 4;
  byte a = r / 256;
  byte b = r % 256;
  byte out[8];
  out[0] = 0x48;
  out[1] = 0x6b;
  out[2] = 0x10;
  out[3] = 0x41;
  out[4] = 0x0c;
  out[5] = a;
  out[6] = b;
  out[7] = iso_checksum(out, 7);
  str.write(out, 8);
  return 8;
}

// Vehicle speed
static int generate_01_0d(unsigned char speed, Stream& str) {
  byte a = speed;
  byte out[7];
  out[0] = 0x48;
  out[1] = 0x6b;
  out[2] = 0x10;
  out[3] = 0x41;
  out[4] = 0x0d;
  out[5] = a;
  out[6] = iso_checksum(out, 6);
  str.write(out, 7);
  return 7;
}

/**
 * Handles a byte received on the OBD2 port.
 */
static void processByte(byte b) {

  // Buffer the data
  rxMsg[rxMsgLen++] = b;
  rxMsgLen = rxMsgLen % MAX_RX_MSG_LEN;

  // In this state we are waiting for ~KW1
  if (state_B == 1) {
    if (b == 0xf7) {
      // Send ACK
      Serial1.write(0xcc);
      ignoreCount++;
      // Go into normal receive mode
      state_B = 2;
      rxMsgLen = 0;
    }
  }
  // In this state we process commands normally
  else if (state_B == 2) {
    // Look for a complete message
    if (rxMsgLen == 6) {
      // RPM
      if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x0c) {
          ignoreCount += generate_01_0c((unsigned int)random(850, 950), Serial1); 
          rxMsgLen = 0;
       }
       // Speed
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x0d) {
          ignoreCount += generate_01_0d(45, Serial1); 
          rxMsgLen = 0;
       }
       // Status
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x01) {
          ignoreCount += generate_01_00(0x0000L, 0x01, Serial1); 
          rxMsgLen = 0;
       }
       // Supported
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x00) {
          ignoreCount += generate_01_00(0x8080L, 0x00, Serial1); 
          rxMsgLen = 0;
       }
       // Supported
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x20) {
          ignoreCount += generate_01_00(0x9090L, 0x20, Serial1); 
          rxMsgLen = 0;
       }
       // Supported
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x40) {
          ignoreCount += generate_01_00(0xA0A0L, 0x40, Serial1); 
          rxMsgLen = 0;
       }
       // Supported
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x60) {
          ignoreCount += generate_01_00(0xB0B0L, 0x60, Serial1); 
          rxMsgLen = 0;
       }
       // Supported
       else if (rxMsg[0] == 0x68 &&
          rxMsg[1] == 0x6a &&
          rxMsg[2] == 0xf1 &&
          rxMsg[3] == 0x01 &&
          rxMsg[4] == 0x80) {
          ignoreCount += generate_01_00(0xC0C0L, 0x80, Serial1); 
          rxMsgLen = 0;
       }
    } else if (rxMsgLen > 32) {
      Serial.println("INFO: Resetting receive buffer");
      rxMsgLen = 0;
    }
  }
}

void setup() {

  // Console
  Serial.begin(9600);  
  // Set the baud rate for the ISO9141 serial port
  Serial1.begin(10400);

  // Define pin modes for TX and RX
  //pinMode(RX_PIN, INPUT);
  //pinMode(TX_PIN, OUTPUT);
  // Idle state for TX
  //digitalWrite(TX_PIN, HIGH);

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

  Serial.println("OBD2 ECU Simulator V1.3");
 
  delay(2000);

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
        Serial.println("INFO: Start of 5-Baud init detected");
        state_A = 8;
        lastActivityStamp = now;
      }
    } 
    // If line went high then go back to idle
    else {
      state_A = 0;
      lastActivityStamp = now;
    }
  }
  // In this state we are waiting for the entire 5-baud init to complete before 
  // doing anything else. 
  else if (state_A == 8) {
    if ((now - lastActivityStamp) > (2000 - 190)) {
      // Make sure the signal has gone back high
      if (digitalRead(RX_PIN) == 1) {
        Serial.println("INFO: 5-Baud init complete");
        state_A = 2;
        state_B = 0;
        lastActivityStamp = now;
        // Clear any junk from the serial port
        while (Serial1.available()) {
          Serial1.read();
        }
        ignoreCount = 0;
      } else {
        Serial.println("ERROR: 5-Baud init error");
        state_A = 0;
      }
    }
  }
  // In this state we process normal serial data
  else if (state_A == 2) {

    // Look for a timeout
    if ((now - lastActivityStamp) > 5000) {
      state_A = 0;
      Serial.println("INFO: Inactivity timeout");
    }
    // Process state machine
    else {

      // Delay during initialization
      if (state_B == 0) {
        // After W1 delay send the initial connection message 0x55 0x08 0x08
        if (now - lastActivityStamp > 25) {
          Serial1.write(0x55);
          Serial1.write(0x08);
          Serial1.write(0x08);
          state_B = 1;
          lastActivityStamp = now;
          ignoreCount = 3;
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
          /*
          // TEMP
          {
            char buf[16];
            sprintf(buf,"%x",(int)r);
            Serial.println(buf);
          }
          */
          processByte(r);
          lastActivityStamp = now;
        }    
      }
    }
  }
}

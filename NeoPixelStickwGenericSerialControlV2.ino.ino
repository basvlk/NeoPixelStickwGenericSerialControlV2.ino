/**
  This is NeoPixelStick control program with the  generic serial control V2 (non-blocking), created for defining and testing a standard method of serial communication that allows both
  1. realtime, quick-changing control, as well as
  2. (slow) transfer of larger amounts of data.

  At the bottom of this sketch is a paste of the MAX/MSP code that goes with this sketch

**************************
***** SERIAL COMMUNICATION
**************************
  The main objectives are:
  - Robust for messages being too short or too long, or invalid
  - ability to send error messages back over serial
  - a Diagnostic on/off function, sending a lot more information back
  - When large amounts of data is sent, for the program not to 'block' but to continue running.
    It does so by not waiting for all Bytes to arrive, but instead only reading the available in the serial buffer,
    then continuing the main loop until more Bytes arrive

  Format:
  (1)"INITIAL READ" : the first three Bytes
  1 the first Byte of any message must be '255'
  2 the second byte is 'Mode' which is used further down in the program for 'cases'
  3 the third Byte is 'DataLength" and says how many bytes will follow: if '0' it means no further Bytes follow
  As a result, messages are always a minimum of THREE Bytes: 1:Begin ("255") 2:Mode 3:DataLength
  - the program waits with reading in Bytes until there are a minimum of 3 Bytes in the buffer.
  - If the first Byte is not '255', it is discarded, and the next Byte is read, until a '255' is found
  - if DataLength > 0, there's a check whether the DataLength doesn't exceed the ReadInBuffer size set by MaxInputSize
  (2)"BULK READ" : Any additional Bytes, of any length
  SerialBulkRead will read all Bytes in the serial buffer into the ReadInBuffer, until:
  (1) there are no more Bytes in the serial buffer: the program will continue it's normal loop until more Bytes arrive. during this time ReadingBulkData = 1
  (2) All expected Bytes have arrived (as many as Datalength)
  - Reading will time out if not all the Bytes have arrived before CommsTimeout
  - If too many Bytes are in the serial buffer, the program assumes these belong to the next message which will start being read in the next loop

  Error scenarios:
  Short Messages:
  - first Byte is not 255 => Byte is discarded, loop starts again at the top with the next Byte
  - less then 3 Bytes received => program does nothing until at least 3 Bytes in buffer
  - More thant 3 Bytes are received => only the first 3 are read, following Bytes are treated as a new message
  - DataLength (intended amount of Bytes to send) exceeds the input buffer: all bytes in Serial buffer are dumped, BulkRead does not get started
  Bulk Messages:
  - less Bytes than DataLength after waiting for CommsTimeout ms: BulkRead aborted, ready for new data. ReadInBuffer may be INVALID if it was only partly overwritten
  - more Bytes than DataLength: the ReadInBuffer is filled until DataLenght, subsequent Bytes are considered to be the next incoming message

  Feedback to Host program:
  data prepended "[" (char 91) : Arduino comment
  data prepended "]" (char 93) : Bytes available in Buffer
  data prepended "{" (char 123): Array output
  data prepended "{" (char 125): Clear Array output


  2 kinds of Modes:
  0-99 are "OnceModes": they make the Arduino 'do' something once, like:
     (a) Do things like switch all LEDs on and off (1=Off, 2=White, 3=Red. 4=Green, 5=Blue
     (b) Mode 10-98 is to activate presets 10-98
     (c) Mode 9 sets the variable preset StateX, based on incoming bytes
     (d) Mode 99 is for changing the Diagnostic settings
  100-199  Continuous modes (ContMode): Modes that have time-based effects and need to be visited every loop. until the mode is changed, we want the program to keep executing this mode.

**/

//LED SETUP
#include <Adafruit_NeoPixel.h>
#define PIN 11
const  int nLEDs = 12; // Number of RGB LEDs:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDs, PIN, NEO_GRB + NEO_KHZ800);

//PROGRAM CONTROL
const byte ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
const byte  BusyLedPin = 12 ; //optional pin to indicate reading is in progress
unsigned long LoopStartMillis = 0;  // start time current main loop
unsigned long CommsTimeout = 1000;    // How long to wait for expected bytes to arrive

//DIAGNOSTIC TOOLS
byte Diagnostic = 10;                // switches on all kinds of diagnostic feedback from various locations in the program
unsigned long Slowdown = 0;      // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
byte LoopIteration = 0;             // to track loop iterations
unsigned long PrevLoopMillis = 0;  // start time previous main loop, allows calculating how long loops take to finish
int LooptimeDiag = 0;              // minimal feedback for checking efficiency: only feeds back looptime
int ArrayDiag = 0;                 // if switched on, prints all arrays every cycle                 // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
unsigned long msTable[10] = {0, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000}; //Delay values in ms to 'Slow Down' program for diagnostic purposes

// SERIAL Comms- required for the core functionality of the Serial communication
const int MaxInputSize = 36;     // the maximum length of input data the sketch accepts. Keep it to the minimum required to save memory
byte ReadInBuffer[MaxInputSize]; // Buffer to read data in to
byte ReadInBufferValid = 0;     // flag to be set when full valid data is read
byte PrevBytesInBuffer = 0;     // previous number of unread Bytes waiting in the serial buffer
byte BytesInBuffer = 0;             // current number of unread Bytes waiting in the serial buffer
int NextReadIndex = 0;
unsigned long ReadStartMillis = 0;
byte ReadingBulkData = 0; //When reading is in progress - can take several loops for long inputs
byte DiscardedBytes = 0;            // number of Bytes discarded (read out of the serial buffer but not used) since the last start of a read operation. Indicates something is wrong
byte ReadRuns = 0; // if not all data is read in in one single main loop iteration, 'ReadRuns' keeps track of how many times were required to read all data
byte Mode = 0; // Mode for the program - only updated when data is validated.
byte TempMode = 0; // temporary storage for 'Mode' until data is validated
int DataLength = 0;

// PRESETS
int STATE10[nLEDs * 3] = {100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0}; // 8 x 3 (8x RGB)
int STATE11[nLEDs * 3] = {100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0}; //
int STATE12[nLEDs * 3] = {0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0}; //
int STATE13[nLEDs * 3] = {0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 100}; //
int STATE14[nLEDs * 3] = {100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100,}; //

//**********************************************************
//*************      S E T U P       ***********************
//**********************************************************
void setup() {
  pinMode(ArduinoLedPin, OUTPUT);
  pinMode(BusyLedPin, OUTPUT);
  Serial.begin(115200);
  // Serial.setTimeout(2000); // Optional, wait longer for all data to arrive

  Serial.println("Hello.");
  Serial.println(F("[ ERROR: none - initialised OK"));
  memset(ReadInBuffer, 0, MaxInputSize); // Initialise ReadInBuffer, with all zeroes. Valid flag is already set to 0

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  Serial.println("]0"); // start value for host program: 0 bytes in buffer
}
//**********************************************************
//  *************         M A I N       **********************
//**********************************************************
void loop()
{
  //  Start of loop housekeeping, and feedback
  ++LoopIteration;
  LoopBlink(LoopIteration);
  PrevLoopMillis = LoopStartMillis;
  LoopStartMillis = millis();
  BytesInBuffer = Serial.available();

  FeedbackToHost();
  delay(Slowdown);
  // End of op housekeeping, and feedback

  //Serial handling
  SerialReadInitial();
  if (ReadingBulkData) {
    SerialReadBulkData();
  }

  /* Action! When building new cases don't forget:
      Check BulkData for validity before using it
      if the mode doesn't require timing but is 'set and forget' (OnceMode), end the case with Mode = 0
      DO NOT FORGET to add the "break;" at the end
  */
  switch (Mode) {
    case 99: {
        SetDiagnostic();
        Mode = 0;
        break;
      }
    case 98: {// colorByte calculatro - feedback to Host
        int pix = 0 ;
        byte colorByte = 0;
        int r = ReadInBuffer[0];
        int g = ReadInBuffer[1];
        int b = ReadInBuffer[2];
        colorByte = strip.Color(r, g, b);


        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(r, g, b)); // Erase pixel, but don't refresh!
        }   strip.show();              // Refresh LED states
        Mode = 0;
        break;

      }
    case 0: { //do nothing
        // THIS IS A SIMPLE TEST RUN, UNCOMMENT TO HAVE ALL LEDS (up to 60) blinking
        //if (LoopIteration % 2)
        //        {
        //          for (int i = 0; i < 60; i++) {
        //            strip.setPixelColor(i, 255); // Erase pixel, but don't refresh!
        //          }
        //        }
        //        else
        //        {
        //          for (int i = 0; i < 60; i++) {
        //            strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        //          }
        //        }
        //        delay(200);
        //        strip.show();
        break;
      }
    case 1: {// All off
        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 0); // Erase pixel, but don't refresh!
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }
    case 2: {// Blue
        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, 255);
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }
    case 3: {// Every LED different

        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < 12; i++) {
          strip.setPixelColor(i, strip.Color((i % 2) * 255, i * 10, (255 - i * 20)));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }
    case 4: {// blink in 4's

        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < 12; i++) {
          strip.setPixelColor(i, strip.Color((i % 2) * 255, i * 10, (255 - i * 20)));
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }
    case 5: { //manual every LED different


        SetBunchOfLeds( ReadInBuffer[0], ReadInBuffer[1],  ReadInBuffer[2],  ReadInBuffer[3], ReadInBuffer[4], ReadInBuffer[5]);
        //        SetBunchOfLeds( 1, 6,  2,  0, 150, 0);


        //        StartLed = 4;
        //        for ( i = StartLed; i < (StartLed + NrLeds); i++) {
        //          strip.setPixelColor(i * skip, strip.Color(150, 150, 0));
        //        }
        //
        //        StartLed = 8;
        //        for ( i = StartLed; i < (StartLed + NrLeds); i++) {
        //          strip.setPixelColor(i, strip.Color(0, 0, 150));
        //        }
        strip.show();
        break;
      }


    case 11: {// 12 RGB values

        int r = 0;
        int g = 0;
        int b = 0 ;
        //one-off mode: so something, then set mode to '0' so no further changes
        for (int i = 0; i < strip.numPixels(); i++) {
          r = ReadInBuffer[i * 3];
          g = ReadInBuffer[(i * 3) + 1];
          b = ReadInBuffer[(i * 3) + 2];
          strip.setPixelColor(i, strip.Color(r, g, b)); // Set new pixel 'on'
        }
        strip.show();              // Refresh LED states
        Mode = 0;
        break;
      }

    case 12: { //manual every LED different


        SetBunchOfLeds( ReadInBuffer[0], ReadInBuffer[1],  ReadInBuffer[2],  ReadInBuffer[3], ReadInBuffer[4], ReadInBuffer[5]);

        strip.show();
        break;
      }

    case 200:
      { // CHECK validity of ReadInBuffer flag
        // do something continuously, do NOT change the Mode so it keeps running through this cycle}
        break;
      }
  } //END modes


}

//**********************************************************
//*************        F U N C T I O N S       *************
//**********************************************************

void SerialReadInitial() { //SerialReadInitial checks if data is available, if it is valid, reads to first three Bytes and decides whether a subsequent BulkRead is required
  if (ReadingBulkData) {
    // ******* Existing Bulk Read cycle *****
    if ((LoopStartMillis - ReadStartMillis) > CommsTimeout) { //Reading Cycle Timed Out
      Serial.print(F("[ ERROR: Timeout, ReadInBuffer may be invalid. Waited "));
      Serial.print(CommsTimeout);
      Serial.print(F(" for incoming Bytes. Expected: "));
      Serial.print(DataLength);
      Serial.print(F(" Received: "));
      Serial.println((NextReadIndex)); //this is the next Index that would be read, so not the actual latest read index. but indexing starts 0 so NextReadIndex = #Bytes read)
      digitalWrite(BusyLedPin, LOW);
      ReadingBulkData = 0;
      ReadInBufferValid = 0;
    } // end of Reading Cycle Timed Out
  } //End Existing read cycle

  if (!ReadingBulkData) {
    //  ****** New reading cycle ******
    if (BytesInBuffer > 2)  {// there are 3 or more Bytes in Buffer
      if (Serial.read() == 255) {//And first is 255, ready to start reading
        ReadStartMillis = millis();
        DiscardedBytes = 0;
        TempMode = Serial.read(); // Second Byte = Mode, stored temporarily
        DataLength = int(Serial.read()); // Third Byte is DataLength: #Bytes to follow

        // SHORT MESSAGE
        if ((DataLength == 0)) {//short 3-Byte message only
          Mode = TempMode;
          if (Diagnostic) {
            Serial.print(F("[ INFO: the 3 initial Bytes have been read. After 255, Mode: "));
            Serial.print(TempMode);
            Serial.print(F(" Datalength: "));
            Serial.println(DataLength);
          }
        } // End first byte = 255

        //TOO LONG MESSAGE
        if ((DataLength > 0) && (DataLength > MaxInputSize)) {
          ReadingBulkData = 0;
          Serial.print(F("[ ERROR: intending to send DataLength =   "));
          Serial.print(DataLength);
          Serial.print(F(" bytes of data, but MaxInputSize =  "));
          Serial.print(MaxInputSize);
          Serial.println(F(". Dumping data. "));
          byte dump[DataLength];
          Serial.readBytes(dump, DataLength); // will read the bytes and dump until timeout, or DataLength bytes are  being read
        } // End DataLength > MaxInputSize

        // LONG MESSAGE
        if ((DataLength > 0) && (DataLength <= MaxInputSize)) { // DataLengh >0 and within limit
          ReadingBulkData = 1;
          digitalWrite(BusyLedPin, HIGH);
          NextReadIndex = 0;
          ReadRuns = 0;
          if (Diagnostic) {
            Serial.println(F("[ Read prechecks: going into bulk read mode"));
          }
        } // End long message
      } // End 'first Byte == 255

      else { // more than 3 Bytes, but the first isn't 255. It's now taken out of the buffer so in the next loop, the second Byte will be evaluated. Error feedback to host:
        ++DiscardedBytes;
        Serial.print(F("[ ERROR: first Byte is not '255', Bytes Discarded: "));
        Serial.println(DiscardedBytes);
      } // end 'first byte isnt 255'
    } // end '>2 Bytes'
  } // End of"New reading cycle"
} //End SerialReadInitial


void SerialReadBulkData() { //SerialReadBulkData is only called when SerialReadInitial sets ReadingBulkData to '1'. It is called in every loop until all DataLenght bytes are read
  ReadRuns++;

  while ( ((Serial.available() > 0) && (NextReadIndex < DataLength)) ) {
    ReadInBuffer[NextReadIndex] = Serial.read();
    NextReadIndex++;
  }
  if (NextReadIndex == DataLength) { // valid data
    ReadingBulkData = 0; // reset 'Reading Data' flag, ready for new data
    digitalWrite(BusyLedPin, LOW);
    Mode = TempMode ;
    ReadInBufferValid = 1;
    if (Diagnostic) {
      Serial.println(F("[ Done Bulk reading, Buffer complete "));
    }
  }// end valid data
}


void LoopBlink(int Loop) // Blink ArduinoLED to show program is running. Toggles On/Off every loop
{
  if (Loop % 2)
  {
    digitalWrite(ArduinoLedPin, HIGH);
  }
  else
  {
    digitalWrite(ArduinoLedPin, LOW);
  }
}// End LoopBlink


void FeedbackToHost()
{
  //  if (PrevBytesInBuffer != BytesInBuffer) {//Only update BytesInBuffer value to host if there has been a change
  Serial.print("]");
  Serial.println(BytesInBuffer);
  //  }

  if ((Diagnostic) && (!LooptimeDiag)) {// Only give full diagnostic feedback if LooptimeDiag is disabled
    Serial.print(F("[ **** BEGIN start-of-loop feedback for loop#: "));
    Serial.println(LoopIteration); Serial.print(F("[ Previous looptime: "));
    Serial.println((LoopStartMillis) - (PrevLoopMillis));
    Serial.print(F("[ LoopStartMillis: "));
    Serial.println(LoopStartMillis);
    Serial.print(F("[ ReadStartMillis: "));
    Serial.println(ReadStartMillis);
    Serial.print(F("[ Slowdown: "));
    Serial.println(Slowdown);
    Serial.print(F("[ Bytes in Buffer: "));
    Serial.println(BytesInBuffer);
    Serial.print(F("[Bytes Discarded: "));
    Serial.println(DiscardedBytes);
    Serial.print(F("[ TempMode: "));
    Serial.println(TempMode);
    Serial.print(F("[ DataLength: "));
    Serial.println(DataLength);
    Serial.print(F("[ Reading Bulk data in progress: "));
    Serial.println(ReadingBulkData);
    Serial.print(F("[ ReadingBulk: "));
    Serial.println(ReadingBulkData);
    Serial.print(F("[ ReadRuns: "));
    Serial.println(ReadRuns);
    Serial.print(F("[ Mode: "));
    Serial.println(Mode);
    Serial.println(F("[ **** END  start-of-loop feedback"));
  } // end of full giagnostic feedback
  if (LooptimeDiag) {// lean feedback of looptime only
    Serial.print(F("[ Looptime: "));
    Serial.println((LoopStartMillis) - (PrevLoopMillis));
  }
  if (ArrayDiag) {// Full ReadInBuffer printout to host
    ArrayToSerial(ReadInBuffer, MaxInputSize);
  } // END FeedbackToHost

}

// PrintArrayToSerial: A diagnostic printing out full arrays to the serial port
void ArrayToSerial(byte Array[], int N) {
  Serial.println("}"); // CLEARS array in Host
  for (int i = 0; i < N ; i++)
  {
    Serial.print("{ ");
    Serial.println(Array[i], DEC);
  }
} // END PrintArrayToSerial

void SetDiagnostic() //Mode 99 is CONFIG. Bytes set: Diagnostic, Delay
{ Serial.println("[ Entering Diagnostic change");
  Diagnostic = ReadInBuffer[0];
  int i = ReadInBuffer[1];
  if (i < 10) {
    Slowdown = msTable[i];
  }
  else
  { Serial.print("[ ERROR: Slowdown value > 9");
  }
  LooptimeDiag = ReadInBuffer[2];
  ArrayDiag = ReadInBuffer[3];
  CommsTimeout = msTable[4];
  Serial.setTimeout(CommsTimeout);
  Serial.print("[ Diagnostic set to: ");
  Serial.println(Diagnostic);
  Serial.print("[ Slowdown set to: ");
  Serial.println(Slowdown);
  Serial.print("[ CommsTimeout set to: ");
  Serial.println(CommsTimeout);
} // END SetDiagnostic


// ******* LED FUNCTIONS

void SetBunchOfLeds(int StartLed, int NrLeds, int skip, int r, int g, int b) {
  int i = 0;
  for ( i = StartLed; i < (StartLed + NrLeds); i++) {
    strip.setPixelColor(i * (skip + 1), strip.Color(r, g, b));
  }
} //end SetBunchOfLeds

/* MAX/MSP control program code

  <pre><code>
  ----------begin_max5_patcher----------
  15601.3oc680+baaqrn+b6eEn57dWa2ZqR.9s6KmYbSba8bRhyjjdamotSGZ
  IZadiLouRT0ImN8+8GVrfRjRjTPTjvz9D2owVhjfKVreiE69We4WL3xjOFNa
  .4XxuQ9hu3u9xu3KDeE7Eeg7yewfaC93nIAyD21failFL714SRiRSlO5lAGh
  2SZvc+A7+iRlGmB2ng7BiRljLEeAFCcs7MXGV5eXP9c4SL69n6B+iv3fKmDN
  N+PMMIMHs7qDdaB+JihlNZBNcxtxjjQASJ6B3aQLGVGnuKJl+0k7hhmeaT7j
  vTwPQK91uNbV57oEeK2w+bXLGriRh+iogiRQLguu2PluuOy8Phsu0PC3WNCc
  sOj34kCU.Xzxghj4oqBF2EjN5ln3qy+drcGx3CpkiyP9unLWw6v2dk2QEnA7
  kj9o6BwgavfEOU9IVdfHR.nCRt7+4HW5.3696u7Kg+4PEoulcOLMxHq9yfow
  A2FVxUtKXJ+BogSkHnJWkLUeof55OjwbMscNj3XXMz0vvv0ji93+8RDVQrOq
  ZrO0xCWZMsQruG7ISVdZ8.NIyRj3vpw7GRFb0jjfzsZIvzsIKAWNOMkOjYfX
  5mPr6fA0wDnLMI0DI3c3zj7ewrx92ESr0l7WFDe8x4ct4miWSlewg2ye3ERt
  B+XJReENgCjDCEm3rslbvzwVLwo9.4fMSLwYabheXd9tbSdaVKN4Qw5oCKHT
  uQK8N0fAXdTACgq3WdlaFCHl8q9+kgMX1ZiTuBIITGpXIkZ4Oz21xkxJk5dK
  XTntvSaIwSaMihBBHXlMAqca3rYAWGtFQDy1l36QLITCFA9.eJjcSShhCWnf
  wZWXwpAkYIDoXwYv.pLCA2V8jWkQJQY8QolTjVfs6BMajRgzjqudRXY5kwq7
  aN+dcplocDdwzVviXhq79FY+a03kn3zxQK1KV6B9yvw+QPZ5zH9hZ3x+ZlD+
  HQP.ZXx7vjqx95ruOOJXRR70UhpJbmytIYZpZ2Z1zwrjqEE+mQyhxP630Eq0
  Y+pkX1WZ5B2tkAcDCM0zVxKKVlccUPiQYKtlz1zPAx6SdW3znfIYW5pj3kqb
  mj+JqiPf6cVz+FWdXCMTmMvXy3IOWg7Am00rl2vbmNQ..8AQ..iaFovEGWgQ
  1lBSrLsahD.SCsIAfptD.ZuUBfAgRf+2g+uq8eclDAGpvGVaTIOixZnHAm1z
  xGK032E5sqAKTgkklddBZazwAzDfBj3aM1Sxsv7y3YTA4of8jzFImc9sgwyK
  SxBedcY3zeyqMkrTAN11GwwbiG3+xwZcAnog2JiTF+RCNj6IBM6OXY+g8xKs
  7ZKuXtql6x4tdUKolUuj5PohkRCl.vE+a8rCf7u0bkRgEWGZ2Keb0U7ZjOJu
  UyFHeLJkqhdcRnxtKIei8xklNUVakrBhKzt5XyQQS2D051SWlIpwTXYtMcyh
  oKktLG8mEq6o+VAMWC4Wc2oN0NOJ4VNjjtty3jODEOdFI4JxqRFGN63KhIeM
  w3HeeRvzPxECNOdTn3JWL3XR5MgehbavGBg+hbxzwyihSH6MNYOxrDNfCKtj
  D9SbHYRzGBgAC94qI6Gb.4EIDwMLSbMxr6i3TCjfISHu7zWvgfXRP7XNfbEY
  e5yN+pq3DBO6WtgS9woNd1aCGOjX8rebZXXLmN4YeOeAa4ne4ABnmSCdjuGI
  ZFIMgDLJM5OCRCIBYVoyvKt7YFIeFeh3pvDhyIEAbIxGg7NXmD90CIWFLKbL
  .fQwb7HLGu7Sbpnki03rwxGd4WkLkL5Fti0vcBi6KhBtNNYVZzH3ckB3.AZl
  yxbDk+Ljmy09GEOOY9Lxs.tlrO7MvPdvw35BebBRI2vonIoQ2FdDBRgWcEmS
  Zl.wEGx+B979R97fS5jBW9OCm9Ixjjj6FRlyeCSDPC7F.vT.hgiOjbeH49f3
  TwEuaZx0bZPXf9PX3cjvOFNZdJNQhPna3EwkFtFucw18Z7ZwEMngxLEADECw
  AkZUtaKVsoOb2Qd94u9GN6GUzLU6scp43HrifYKTN63Vm+XldCxOPgSWUH1U
  QSfE7YEUM+ECBt6tbecQ4a2F7+f66l6BYP74C9UKU2MMDnnvm2Xw2FLcDvcN
  B1NKAT9QOmkh4.JkowyiJH+JCjxswJBSnLohEUGKCL7U4BMCek65IIi9Pg81
  hqV3tv3n3UMGYwkGGdUv7Io+Q4tQW75WELJrxGtT+0+hAWOMZbRL.DEP0vWm
  85.yADTp14mLh6HN3tRdX9hLvLW9EmwmjymcYvzbpEXYWLMIYRwKs34lDdUp
  7x2EEGuBVLM4tpu3znquolm8xD9EustwVbkY+w7X7p+AmaK8O.kxEuOtF.I6
  Wwg+iAwQ2xE.Cx6DSWiEWDMl4lYillLYRg4Kdk+rjqLlSDOJ79nwo2HdQ4IF
  32dzcYDQCVrJONB1s1heWZv0yJ9MqITf+UyuTxj9GbqmtaBeVT7FJrg544Hy
  Knpv2WqKlkDfcehMAc81f3j+1JHy1L2EVedTkGnU6MYkAfkJsrycYL2s7ywX
  TuSkqsgDCx9ZoMQYx5aBprfr+MDCu5iiWMnwJDDUk1wpUiTEFFwoTzkz0ir2
  ZaKnlvf2E7AIQHkXMX6n1raJ0lDWXf+Bi44ZHC0o1nFV5BaMMjK3eLIiCdKQ
  XzcDgggEkZXuiHLeu1DeIleaIlvX2vDncHlK925wC4iT7ZRpX4gwLuvDv9iU
  Tjmc6hhVDP8mNnHJqkIiVrC4OgvQ1sMcj0SObjeaSG4rM3nkts.FMVg8pBvF
  td4HuYIymNJCzjJGNjTbJvM.OMJdgWb+VlIJqde2DMdbX7pXzwQyVl6iJsR2
  YfLChCZu.jgEZk.Yv3FNEV+.ns1Jf1re.zzsBnY8Cf1Xq.59AMMXK0V.z8CY
  GkAKkC0kHkoY.M42+x7gaStOHX3kpYSP1pvfb8jjKClHCxwBmgqKlHKCbRy2
  EipBSqzsoYKzL29YVmMZtByP3ZsqUCym.peqFeZMLws7o4hrvNLwMdjMwM8k
  gTwX2l3d9sYVkMJXRHWJADg.igpmScNaOAuqfR22e4+1nYu2ixYeFUOtaS6v
  z28Q4zOi1m5siS+VM+oxkn3JlvXUlD99x7ADNeETWpGMKMmJLKUGgQMPwiHS
  SIC0NlwTdVsbJ32VHRKAsQd7n45Iwg53QCjuy0xDykssgvSI7XiNJCXRDUSt
  m41lYbC+UGc67aEne6r8TleeYe4FN1fTWCQxzX55sbyVpY4ntCijsIRVypXj
  J8j3r0muDNYi1xYLW0yYL1CdV6rQ5Nm9Dcmi7TVzBzcVXPl6Z5NWWsQ247jh
  ti0qn6vMrpMn6j18z4zc1Ziti8XhtqprT7GN+sjWe54u4re8zWRd26O64+Kx
  a+Qx69kSdyaN8Ee0WUZ5nUpkKkdG65A5DOmc9l3tMiQo2rRxuZRGLOLC9yR3
  sRFJkHtrZSGsC9.4J3+TzdQys2SK4weGCrhbS6ahuFtds77d4AUE9utyUSDA
  33gtbgm++lf.bnsuulfWlnGmc872d2m+FspylFr1xGIT+Tq4qogqv9doXh12
  WS2FEvpYShFWtMC3UT0lgrjjZwYfZcr+GhStrPEiw3P7enx+YwzUTLL3ni6l
  ml+kb40EdbSv8c5gh+xx12G+KOSa2hi11oVvAIjMYC8cwpEBlU.TKiFsra6H
  yUVbbKYr100cM3E1pzB0XURABpGLSR1HYMsSIqo4os6Ej0XxHzhj0L6tkr1w
  Saj0zGMj0UYoMdvWVdp8un0J6I3I7JeYOQZfKynI1JSYn7UKNgnsGyChlqu6
  ZimRDHtcg9tNRpvRAB8JoBxCXcqIUPZTX2IUfpMoBOVDITW85gRs6tCqOWL.
  bDPcwsRvwrg1+a8PWZmLMvR2lmUEUemsvDewtSIwHcRMchZOn0oQr5dZDTRP
  ioQn98EhDY5jzBDIHJoaJ7WFsOQhYmSj3f6gXiIRXz9BQByokHRjnjtgHwo8
  IRXcOQhCa2HR75IDILoUus.QhCq6HR7aehDZ2SjXZlUc.aVAEi0WHRjAipEH
  RL6t5LoeGXShQ2SjXXsSRR7c6KDIl9sEQhgU2Y3JsQ5apJfGPg1ncvhLW4VW
  IJlCX3NZXrMPrnsq77XZhktosMrF9sIdBpCLsDdhgE8BIdxeGvSlhQxCK6ik
  ENIkvSsuTGCmNWpiOtcYNMNsje3cEh4hT.sl+xRbR2H1oC7Exn6CphuMc2HS
  d3cFJiLo07XVhS5HxDq1mLo6iqhOisajIO7V5lQlzZ9LKwIcCYhoY6Slz8QV
  wCSxocfLwqmPlzddMKwIcCYhUGHMo6ishmrJzzXxDK69BYRq42rDmzQwxm19
  jIcezU7v5MSyISrs5KjIslmyRbRGQlzpdD9imuaky5Mtmx1bAqV1LOOtucbe
  DryxH6lkbsTaaWQ105HFJKuFkbsVzG9FHigXdjuMTAmVJIlZwe1nTwAy+XGw
  u5HCeZyrSdd7cAi9.IB+uc5j.aoPF5lUHR8TtJEu9uJ2rH5CRC1o0RQdlZoG
  hkuntiild0vRSuzMPOCZEC0NlWHls5wTGqOBj2GNKk7pSe26N4GO8cJRlZrs
  B3fxlp3HsZgIKiScUSV+mhESVaOLwnEJicwfZ63Y94hI6mKlretXxVSMMqnE
  dqbLHrI+6voI7WPU0P17EE1MpIoIkxxp10JWYMJPd5JV0dtsuFpZZYjkipUO
  hUnDohM7tkp5bapb+ZQfDzqt+SuKerMu7wXxDBlsndtWTSdCpxv9nAZ35GEC
  y6ZC4FpioJt9JsSnkVeqiC7UISCIiCRCfd.PL4pnzYjnXxkyu5JtoEAww70n
  QPc7m6kAgKnjHRtTkJ4yUud629rq3Q5lZtn3OisZ3cfekhl33KaawHWqaC4W
  c0I+Jkyv5SnDF2JcKhMwg3R7H7uwfyIvmJbzj3d32oC2SOBcGYeMae1WK7vV
  gm4+Lgv96B6K0xK+5IV2l6H1WlmtXe+uClDIYNuE3jKgIdGWbczhpUu1fUEi
  ofrrB0XUqVZkU0bKYU832e+iccEsssA6poW90TWVWxtZqqk7rUyAs+RfsSND
  kzfm0px5aW0J2xBSkF7DT61j51tpq.lZaEPxXMny0Y0dq.TzxSamtbEfoKUV
  WMexDopItNp2+Su8zSGnjbKasnIx0pMbxC6skTJa0jqaqVSLzopHagVlU0D8
  eBd1AA0zd4hlQW5YmLimz.e1raCVvn8eNgWgY5t6gWg5oSFOGRN9tA5ggY2z
  KY5istWbW.jLLcjhIp+Cfho9G+hsctBXTaFNR4lmsa7KZM7FlkpnZcWl5etI
  g8oLJKuEh6laRYAkDWEo3VT0QptjGMBcXjdIKtCdvr2.KARx98UEFGrcKZxV
  thriXUgD4VZQyVmrltBtv9zpGEMsSFd31b0SVR4b5zUOKco7aQn8Iyht8tIx
  F4K1EeGz5pyv.14hxAcoqdxZZZD.qe.UEmq03+YHp.fcP3Hn94QIlzc2pOFt
  4J3H51kgCxTa69Et7PFmDNKdONseZ.mC39nzaDEKVQSqli.RfrRo2YSnLgwj
  dRgD86lMgTYLDqcDUkMRe7QbkM1DaGLazXjGLkMx3zkg+ZCkMYlcjMjcIWGU
  ag.7DtiVSCG295Uj5l8xEUgcSuhJCnpnWCchdSt5pN.8hm6XLPzVFs.5UgAT
  Qzq1BSv+8Iu7rWzA3VYLUv5NmUKfaUX.UD2pssy9rWKvtjiH76li+tl7pfOd
  V7cySeWz+Nj7rU7pW+ZaQrYIEO6Fuo031bR2IQK15Z847W+7Se04u3ztxi.a
  VEm09lhdqe.UD8pUmvfZ7mF8AKKxQsmSXaXDUceRzoWXqXyX6XMnom+RSGra
  A+uTY.aIKAMz5NOR6PzOZZQKh9qc.aGzumV29IJEBaWkgSu2sAvx.ajIlQls
  W6jWVqNjc4F.6zpZSJbzfVY+eIuO4cg4OKHYmcxxOoHUuBUwQSoQV45XJNIp
  LVEEHusWqiMyWLjln6YULjpppuUci3tf3vIcgaXXsi.2MIpLYmc1MGwxMjLr
  EPvbaHRz8Q.NTZ8MxoafI2MXK9NaOOMWYSzzxoYXPm7SMvJvUe+bSBkmrJWi
  BLjqTOtc8rLoVPU3lZYa36IJr2LSNOSg5wsD1RtKY5hCn1PS+1TKTzzfgWAU
  W57SsKCF8gqmx0ULdMjw0wIbb6jnQeX0KU1AYrtk5i.y+scLrrf.gOzW7CDA
  a1PS92RM4pygcgk+stNMe82g+N7c84uji37MvOTkdIpZSVoAEe4w0Cz5Vw4z
  Za6p3ddJ1Tw4RKUqoheaz36RhhSmkEZB7LfiJZsM8DUrAwGK9V67FntiUGOU
  YFxTkQ7qESU3iZdpZZz1SUM.yc8pioetpE3hUG3iZd0g007bTr.UXZ5JliYy
  U7y5dxZ20SVKbxZTXcU7QcOU6ZRXa7D4y7sKHK02V6xRYLMMU8XEHgwOq6IK
  simrVxDKw0n3jU7YMOYo9c8jU1jUwdK8R9UGsSDSc57oprZN8faP.sqkBaZ4
  iEqXLkrsrFRYk8YrMfuTijk9W0szDpvzpfBI3i5dpx5bKd8yUb6VPf63o+UU
  CCM4HCqHyLaaYlI+9WluryH6ZTXYVolVF0VUNPtdRxkASjE6iEg.rtZCxxBH
  R62J3dinWvkuhtUZTI2P0eaYzH2ppblkr9oaIqoQMuHgyxxFNV1oXd6qQ31M
  pZstwtntca1E02.hDC5pxMGc92MK7pnIogS2PE4JC+hwSqy5Y51FZqkoaqdK
  S29Aukoug9Ws5s+hstycyjbTNhPD413VxUqWIjYcXYUWtm4xJM+5m3ZUmzFF
  O3ktTrMLzFUeerMLzoUeeiVuBIxr5dpDbOnaLUhmWegHoEp89RhjNr16625E
  KalY2SifayZioQbr5KzHsPg2WRizgEdeu1WNBqyoQjkX9lSizWT1zFUceYO+
  oCq59NrVmFg18zHNV6DMhceQWSaTx8kzHcXI22t06mgLitmFAONqMmFwouPi
  zB0aeIMRGVu8aVWYnBG5VKWw5rHxndjU7nXixwxrhFf35osUK4cK4cbe9iu9
  42DN5CKCwRiJ7600mJP4Qh4keic0yy9IYs1VJbMqMzgU8BOiOWrs+bw19yEa
  6FlOvgSJVoW2xzwtzP4VovMGjHGMj1TwJWmPI3gqlX0OHUyhYBU.jIASuNbJ
  VVUcLIW9oTEqU46DxcSkEPYCvEyYWJqQoqdgrMUOYj9+j3vFnWbD0CQNJVjN
  x2CU5zhRPs4s+ou8sm+1cHkm2pzV0BqpCxS2REYOegRRhtPD2w4.SIu5je8H
  9r45DNXeDZXVJH8USnGYgEWhknNdaD+nYITm+uzEKE0QFJNzf7FcBn5pSIP8
  583SAhwVq7yrRDk4lpcwnCj31A6v1NkeaTIn1zAd+T3rnNIjywcyWQdQPZvK
  E+82y0B9MlGSbHuLZVp7FfRE9CtlQI2pIEq5M9F6nlQllnQe1yzESrbmTsv8
  S2a2UKR8nZBIkRtbQaXa2k6uACXoxBtnz9AisiGtVDlq1TfBsutMwDGsBS71
  dTb8Zr5D419QQJReucTeB00PqZTzlqTnkGdxynVa4IkislvVeCwTeF8iE6LY
  maASMucR3litDt8umLjyBpKZprS5L5jjzbkMy5UOAkg9PUwbQUlZlAzvZYqE
  ZWwU1s5wLVLG2RrgwtwikUCLy92lJy1qvpXVxhtLmSaGDDBP5xkQzDKSYnuJ
  GEU.GXsM3fN4De5pXdRKf0dxYCTYXt2.wNOBwxJCyVlO9f4dCDCmDGpBPrau
  BhUAGydzcVhK4XQ8PAxrGgBLXOBEXvdzIvPX3jZvriwiQn11uGA0pdVeseDB
  y8IYGPnwTDpcM5YPMUYn1rm.0FagJbZ+.lcT0Poxn+e3fZk4EcMeLB0N8H4d
  NpRUa9HDlQpZ5iMnVHow8wFTWhrlGLn11WYntGI2SUeDcZIP9o6onupT6e4l
  6cS5patGr+7K1UuE0M8Ak0iyZ8iD.lay993YPa01blp41rKqMypaQ1DkO0pZ
  VFcWSJqa64gmmQQje8LpKi0oM6HwTU4TXZXv3O8.UMELsDc4VJ1iArnMuZJX
  JaugXozrjQRkCyayNGIoIWCUTyRJmB3U9MlFJmBlXcnitraMTKlr1iOueNLY
  IiTsaCpZ3Y2tulIHw8TOEJZBqtN8fUzDphG8zoSSlRdKT2SNdPYDBkkxT6ZA
  5.qLMVxdM4Zc4S04MgSeAHbS1o0raFuocKJQOkbTmc1bjEKQTRNit4SmyQkq
  BqY01k17PsY4JEKarqGpsLjB0o6pDC1r1TqnPa+wszgCTxBg+xgtZice64jr
  yetq2VNIKmVjSZ5VYWjwVS3HsKBIb7LZ5YcypUKlKwIwgjiHQwQobKjhlENd
  YtTqpAwafVIKQjnantHQ2LwRVokxaaPdJQH0pbbOm+8yHuO51PNXQ1+1YGnK
  Mc1FlHhRnYRl7EMTSmrGEfiHztlY1MfEk4OnUKXZf+KmLcZvmloObpMJoRPC
  5Xu63Tpii.mZa1PbpcmXbuNpUZNXwHFD6auaJhcvr.EZ8v1poHdqMtm4nOi6
  cU23dpcu059WljbGbjTIWEFNF5qA5lIEaPcsBSJyb2XRYcBSpkFYRYr1hIkY
  1cLol5iI04IAS56ljb+3j6iKXYRqv9IK9PxBGjwtv94gsrdpWy39ZVIOrJL1
  3nfqiSlkFMpcQWN4jVsanKKucRXE0nUs+87W+Cm8isEpxAEcHMYxX2wUBG3o
  x5m+VipL75D45l5StNywskjqa4zYh0MznXc6GUh0my44lWSIOVmTRn7dGq02
  jmzvakokO+RCND1E4r+fk8G1Kuzxqs7h4tZtKm65UQ3ZVCgK5NKpcwgs5gip
  bB2CW4jrnTkxzWaESYS0KlxlMf5UDYn0ofJ6txNSiF41H6G.CbfHuzUauIKK
  n33gFa83VV3zfMnk8s5h3WLeJR1U1NYy5nIMbtKwlTQtt+ZA2qJDLs1dV+O5
  pESJWok6xUy06O7EUI0pkWsSlNddTbB4GVwS4FMG8paNZKaFRXKxj5pV4iOu
  fuU9+BJyK3Kl6Sw5vFV2bv8R10TD6YGOyOWE19bUX6yUgsFVTUtXf0ECzVI6
  w0XQm7sr1mw1VNT751SkaPbbxR4ECxzSfUlTx2O+pqBmR9g4SJzOfy6Gf7Iz
  0I80yxCSqBG0NpurJOsy0LU0TsY6YDGsUmF7vs91BUHaQ28pPCynaoLuIRtP
  81vfw7Iy2OexGpgxsh6pKoEcjY7kukhG6b+pnEyC75pVlElpuZDhrOnR2lhD
  R0Uv.KMQ3I14Wx4ySuadJ4poI2RJQX2ZBPU3Y5RhRarJwYhFfuYhRqpkOV0L
  QOhGgW4rOc6kISZMT2lJkYtLba8LJuAHs0UGo1DSUHltUFvMCMgp.ez8DNy5
  UQu8nAJTzUQQMJMIn0JLgpQSwjoDnQKn2koKUDFDJTCGINPWGa0+6guvCJ4V
  wzzhh9MuKEdPsUjGCti6M6Xx+G5.MipvJojiyt5Sh6Sy5xKDtEb6MsUqv75n
  KJl5RbxNm5wAaex19xBgGaGIer0awLNYbn1Jry91Cc8rcMcNLqZX63tIhH8V
  amOOdTnNQINbiobrXddlPy5ESXCmMVPm0KN44Iwo5DmXCINA0h4awk03lK4Y
  qEmvzJNIK25NVaU8aawl7JYa7M1D9P+nCsgJjmjLkkfnqBl4T9THTfJ9GGuj
  .g774SmxcZ7UQS3ZlNVHv83E7TGuPhywjbQ43XRgCJhpnT+JQo9af5BxIPCC
  CS5hyHfkuyVTWIq6+qz7fVMPEE2yxhqLfA3PRvR1ezMbuBGwcJjPMNfLM71j
  +j+0B22uZRx8WDOMLc9z3B2n4AjqRlNJbFI8lPx07k46HoIjKCIvlIQfiUP.
  2d0KwHjNZRXvzvwsk8+MlKAr32Qj6rxRpp2ZaT7ZKHd5ZAYbPZ.4togfg9b7
  +EC9sKFf3bhO8.9cjE8Y4fbQ7ZOwuu7ILgm.Nf4yHA+YPzDvaetwfxXVWxy9
  WKdVJy7fiwyv.rPd27zMb617a+4vRbgGpxUaqbWn58GrZZgJ1PxVTXprksf6
  DiIdvBcpIfXtTsUA5K5EcSQeqtesaCZcG5LElCMEA.BMpS1181oHa3xdPCsX
  83+QISvr.fO2gopwPOeCpuC7WFx+htxruqH42jCaX0APFucWqcsqg7PEWt91
  Bh4NtfHSfQKmcmUwSuNQOON7i2wmNgi+ANqSI6nwCgrql6Spj+.6rPTG+MYf
  sq6mEL0FBlXl3oR10qUDL4w9rfocSvjbAgg4rRaHXx1PqBljhhd95lh2KMF0
  ix8Yg+im6B26olaLTG150AeeStOJfiCf2.OzB4sqDUauACTMPctH5011ugN6
  WY7m7+rNgVYmjMxaazNqSv+y5D1QcBYKHsmNAeSs0lZvnWoK8.MuY.IqyEdB
  xcayVnA23q4l6H0fq5RWH5F2WHyZjPaUF8nbagjQ+OpdJTsssKp2CSaEZQ15
  gwG8rrnhVSN5Uyc1oArFErpd5hxLpBkt5LPSXXbC.N816R+D4Hhn.XBaT.eJ
  FF8mg0gwU+I6zUfrfVtEqAzJWC1vbROqIv1vQ9G0f5W+F5zcwM29rIaH8pfj
  YUgjkPuFwk4NNmUhNW8d51tgswPelmsm+VgQsqEiJl.5AoVX2gqAwV480sch
  cugl1tTWusB45TExs3jPWsRNeU6Kar9SGdwW01NhO6wGLy75M0wdeUazRt8H
  7rpvLy3wGLS6Q.smpsK.Wm9CLqJh1oG02LUs8H51e51ftJyD573ClEl02WZc
  lVq0pV51d1QKAxdpzc1dDhl6O8fFAHa9npwJI.Y5iplTp.jcTQnge+BlsUAl
  6YjFLkDN673idV3wceBnUSMXOiMzRIf9QHht+zDoUsYr40ePyLUaEa9l8KXV
  ExYO69ELqhvNai9ELqhVEVOid1ToF1c+AlUMFiOBA4djpalxdv1ehT.ipnXC
  eUg4aiFeWRTbpL93dX0X1QVjtMWzIFJ9N0yDk0gSTXtIZFcTr3IhyTqUSLHM
  LSUtQU2i5ouJyt2eXcnVO5jppLH2ijpJRNE0.Z5maLtMqFyNKbBg1z1g5JYH
  rRsH0x6PtrZJlxFXE4GO1GlNatxztVBfU3PgX2lUdXYUH7mmEVasVtq6rMxl
  DAES6SqcnkdAsfMwoyshARgpZtqUmz7Cn5nq1jgH8n6ZOfLCQZpbOfbaa+At
  F5q6GXsEc+.qG7tePa1UPo9dX5+YOz1xiZ6tasaXOSCQk11iZoZ+Fd66MnFc
  SakxUGLf3I8t0P2XxC6gklitn6N6qu9+1Vz92b6sMVpQIwb0e.Bk.USsjqHy
  3no3qI0p.sqZT6JqWzGOtodX8pyq1lafeq1njJGK8fiP7PiCxPHF00UD77a0
  lYsgdQBpHM2xMmVdlxxWxiiZ2NGwKCG+tEo1Yqin7ZnbXeDCICXRC5rwt9OI
  adDnQhltxNkDdHO7L9b6i3ysOhO29HZ5gJbHmis2ejBkmzMrN7X52BmnPsUg
  8B9.Ip3+oKrs2tc74jmqvRzBskmZYsUbG+9nzYry5CUJA0Pvn1da5tVJ1eZb
  PMY1Y+ayOnlNUdVqfvC.GfMYEe6wJByqsQXl53fspUTDk113Hqmd3HqVFG4Z
  7zCG4113H1SNbDynswQO8jGIKROaGNpZDzSOgQLmVEAY+vTpF5zZpNhZn1XK
  oubTTwR52VfC5jisqqsxoo8ZGckGrSXk01.yN8CX1bafY6Gamjs0xy3GLX1X
  afYyGaGUh0x8uGLXdqnm6IGksshdtmjWqpBx8mS2AU4CsaOJ27z8Yi9oap4s
  4TehogLuvj4zlYdgOVsf6vLuvxSe4+D6wTpWrY5IpNnmLraU5Ia2tldxUezS
  zmVzSF5HyvvptYqQOg64dWROouLCa4BvSB5IesjoulsJ4DllCcI4js1Hm7eR
  QM4oCpIKVql1pXgtrKolL0F0j2iIpopNlLhl3ZuI4KyNSLPsoszt3Tw1xoaq
  lZgY8.smur2a15XDisEiX4YJaBvrxaLmEH9sdRlEgBYFnHCpKtoC9ddeNIB+
  bRD94jH7KZVCg7j29he9rWeN44m+hS68solhY2l8ZVMThM.ZpuZ9se8WeQLg
  79ah3JRk8qzv3voQiHyvSP4HNxgSmQtaZx0bc6GRFMMjuTOFZwoDtTDt523q
  E8yzTQXF4+MgyHGONX5XB2TfaRvSIQ1nc6syiiFIjlwecAoDNqQx8yHbVsa.
  PgNDJ82S.FhCI+uyiF8giFcSP70vHKgkCIAyH2GNYB+2vivFR1eFePNfjNMH
  dFTEw4uwIASul+WA2BGYkYv2.8oygWDCOxIoh4JxeCWKEv.y9PHesCPDAj63
  Lxg3UBIu5je8ae06dCG.FGhP80IgbXHJ8l7OoXr+5J+I6hj2c5aO6jWxoce0
  q94We1yO48mc9q2zSBKRgjaChhInPzn+DZeoSCOVbwiHuM4x4yREKK2FNaV.
  mslbYXjHGzRHBq5H7qAe.LF7P3CbizBlDMVNBAWFMIJUTz0mAGtkvoSKLXAi
  9.IgqlUtXl8TjWDEbcbBeweDII9aSt5JxUyiEmjnCECDRTLIIkbaxTnQqxgw
  aQJ.XLkiyubSXLtns5ZFrf.p.OTL4f0CIwHIlOlbncuKAsm6Q3FGCeDHShhm
  GRlNOFnNGJdC7eNKkLFV2lkPt7ShG99fHAMKLvbBQYSgkOFASmxQvGJFxnXN
  oP.mLNdxmD0kdARkCFEZdrvWHoxwN66gYu1TXlIAprGUrPNApc37YZzDDyHa
  IshWMPK8CB7DeAde5AWL3rWe16Axl2d5Iu3Bf6EFmqhlNCnkmFJebwKkl6Zv
  2BXxf3OksVRtEnTtLjrGy1dOwSvjvOGLGyQN7Ggiz2C5yz6Qt+lHjqX9Lfue
  9T9sx48StONadmsd.nw8FELKbFNplhqx4P3xB99rA8E7kzWFFeMmeefPvwrf
  OMibSx8bjBGFuTfDtOhuZbUBHa3XRzUj8L1iDwIfB4L37UtEPAhxvaT7JOAX
  d4lLwMt4vkztb9D9568vKJfvEoGc6bAa+6+o2d5o3fvkue72GxkyP1+hAb7x
  ECNfvNFv.DyiyAyH0Z9YMPDIEFjQc.MSXAjgqt.nFh.Q92to7ljXQjrYn7Mb
  1UqtHFMSPyJVzNDvF7uXbzrQbIsgiODkAyehXtv8EO..PGJgh.7Qgu9JNCVF
  aOG6tb5Q9mbkYH7tGfq3524b82eSn.aCCet6EXlh2KkD9wQPyxFtJzSxOKV1
  KG.kpbZpTfa6UAe7r36lm9N92wInYbB5u+me4+ZI07I709fwii.4BARNwCyH
  bmr.2iGrenmmCuJjNAlj4XeKiYj+kbt5UgPIlQHBkyikachShkikrrQ73hz.
  .brPvST5dvZEm6cRsL4CIimOEEI.Ze457x2T2ALM4YDp.5XGPNg+Jx5anxg4
  FtklxwZLYetZQAKD+2vyhHsCfG+nrwEATwaB5C470dfjBvcvrojA8xvq.nF5
  Hfyf1bPBzwqynO45RDuvr4TEBBKfo3lkL+Vr2nOiSyGBJiHx0FAoalPJTri.
  d4FTLMUpMSrXKeMh6GPvfzRA479lbtVxGhhGKTe.ruyPMjFG46K.wKFbd7nP
  wUtXfXUjKXL3CghQLyG58FmrGWOAX.C7RS3OwgjIQePptE9Y+fCHu.fa9MLS
  bMxLtP.NLCnyWd5K3PPrfqDTItO8Yme0Ub69d1u.dTdHw7YuMb7Ph0y9Qtra
  tlR6m88SlGtbzu7.AzyMa7HeOgwYbsRfZet0WDgSgbgNhKt7YFIeFeh3pvD5
  OC3qDfJJ7QHui6jU3uxUrE.ByS.Q3bixf43kKTeHFqwYikOJvXJYgcXBw.KU
  5yGVPIpT0igwQT9yvoXDLCIy4Dk.tlrO7MvPdvw35BZIkfZCHHOBAoPNIC2E
  QAhKVHVIATUAN9Az8f6+eRrlObovUwa..SAHBBCumS+DDmVfxiOPeHL7NNOT
  3n4oKX6fmcnzzsuE9829su4sm+iu8jWwsP60u+sm+xK3nnXPiIHTURg7xvwu
  gSE9L9L1jP9N9L+a+VxqO+8h23qCSdSzGCmfluvcM7q9Jjdc9sWhVmlmViSp
  .2CHJFjab4jn3O.P2MgStCr7QP1GcUg4B2zlYE.Kx2Oe1mV.TTFGj91uk65K
  vFBCNexyYJ.KuCWpkRHYSLjbdtKh4iYz0buXEFHBDL+YDe4C69F7A036DSRI
  GIHU.71EElDfnZ4YhFgT7wk7mEG6Q4arGkNzvPHuKwaX0QHu.I9..MIGXQ.F
  Egkj4QV7WAJ2Dlz+pzBibF4kJc+ArAQL3bcqBPAzrmIATpokODHMxKN6je70
  m+t2e1yIu+7ye46tHVrNjiufuL7cjU9gCdnPhPT5.+krPX03kO5UbBeg81P2
  pTvCiXRzwoYqX20p3l2wMFRXd1y.1Qi0.BAX7hvIAehHBcLY+amc.n7E41BC
  3xvfUsCQSd4ur8VNs1CY1.zGXzEXcbFpE.IAEelDk07XPfhfVlyYogSQW.vE
  +hvFnNXJ7vBkmQY26LDyuv+IfD.5TTb2DmJ8HUP1.KWYtfDHboQRS9tR7BMi
  KBZ+s4MRAPd1e2BHRXy9GEVtg5UgwLBtYbJKz5g9OFLZT3coyFR9WfzlnzLs
  aYV9s.fAGs.oe2FxsL3SRrSd6S9s7.zuCbzxNTF1Xr3JBQ+i.16Rd7+av4tk
  HX9Se0jfqkBTAkA2CbJWMmSDJ7Cbg2VvPKGu2vYiEp2WXUW9waAS9RIayiE.
  FZRPl+UkYVf7ETwfmiZHSLPidGvx5q4lIf3kwgeT7JVkiAt56.iLxIQJGBsn
  AYFvBgPHS4BR4BxGEvAFvlhYf9JoUfnVTT.CrnNagHCoM737YUT.GCrbli2x
  Bq9I6KPDfLPIIdQqdA2WAC7.O2NfaON2LFwcIDbiVUAVXizRI2I4yFRNSppX
  VNaf3Sx6ml.h+VfWdKGKlgPxaKYd5H.y.AsHNDd+WOIuuuK3rOjrW13smPI8
  Lo..NzsvyPPM.D3mogEXgVX8O7Vk.mvxkL.S7gUCcvQnrs42MVDMKAmPFXKX
  FfudH9rBKv.SGWX33gRyhDgJIsf3QQf9N5eJt+LKdxVoee3s2U.zfHmlLMfa
  QCn0Cr5U3BM52MZdyZ.ERSmyMLjXEjL90M9m0dZj16cjSIum7yj2HoEU8o2x
  28elvE9vQjyua+CH+EXGI2jE.IreAqsNjb9O+927yu+fuK+srzvmhWGE1O7R
  vm98oTalgAdAPCBdM9qTZBw97qxuLbsyuC0abHp9G3W4bRYQHBE1mrHJMKdO
  hFn9j38uXvuabw.bnPNLTGKL.2vIQxn.OlXHsDgyMjIspjQ6mB4pYGBiHbYt
  pBNPueQmX4drmWQA9tOKNBL9KZ1pt7JBTALU92gS4NvOjfpIDpFDQZSnvx.n
  n96cmnZEZlER0dE4DxYjWWKcUqPVABZ1m6D7eIW5IuKSrmPDzMbsWgfDmHHl
  nfSGY1dA2+p1odq3O1GWK9luofkLhuqL0kE9r3tV8NjK5Khmn7E.C+2CdCre
  gWDdwePBluO4m3TU6ufz9TzD7ML0va94fkRKLhLi3SR4.TMxgkKZe+UzCJ4S
  IbcQWGkFL4WlxElWfW7mN6G+I7wIn90EQtAdd4H+2YqJ7eDs9yr6BvLKmTvO
  ugOMgw44PHolk6Zhw3joSC9z6SvGeE9ihLGRB6NQdIeYgKs70DtOAbp6y4+0
  6JiBeWe2RRabxByU.2jgWxQpCb4m+uNDVh451kQu9X4UK0vlJWMe44+RFFuD
  6UjCoT2VlZtuK2aZUaRoxKl+eyYTGFCtz7QKduiwvzHM4UnO7PB2ha4xo.kT
  FgFRolYew27Me2pxY2+G3xY+MofgrPK9RQHFJX53wDtX3C9tRDSW79va49ah
  3F6r+96uN2MDk0CH+W+Wj8KZZ5+ubp0OPbCqcGOqH07B1vBtMT3Y98kBXltj
  klTDjkXk+Niau3a8eV7kR9KbwdZXluObycuBBBdAUYkhjO8su872dLFJTwCO
  S3gOLBn2TRKWAKn4e43IgCIuX9sfLrbn+UF5BxSK6N3KQ3qGotRvPdma3FcS
  vTxX9642JLV+dwwBvehquObqGRJYo8f7h7Vk8ZCRLWviQpvMtZVhd1yxS6fq
  PKcpqZPBtOLxf6gWVLL6ILG3PYWXFLcIN79bijJSgUDFPpYlQkWClVKMkdAs
  cEqku.7o.lLY9gcXl2wbG7uaRXZXdBFAd6u4yVHFUEwLkQlxuoZjDrEBB96U
  DWWP6JNCqS8JGfyzvb5GivcXWLmEa+wnOMhKiY4dDKFp8KZ0xQq5cKH9Ie3y
  PpkrMH34hgDtzXx4Xv9qmUVNJn4x7GJGNekGqvK86pbjEzaKBKsfeaHexia7
  ww0L94X.pYzeK1Pwqaj3KjEYuNPXScZtzjPr0CHsfHL12mLeBraIxcbaVBt4
  zvtDOJcNDA.vc5zLWh4O2P4lKy+SXdJ7UXF2o.9itZ.K9GnXK3YOHCdUgErN
  oP0IigynPjbJbQk0PY.2Fvprfzb5BpRzFSfd7qpg1VRbymw2uHZJ4HpyQUWz
  l4+Igc.g7WXj4j6YmIjRCE20tuOmxHbXJnIDjZxrsO.FnS.6iEa1JeEl+kGt
  rEzitwIAuE6PRIAMpf6A3OUXrD9St3ATTCM5ipXy3E6j6yDRSODiM93EgMHZ
  xmx8lxGQ.fZuvPhi46KrU7KehiyHw3y1rMSe4HC.yOc9aeO4Um9t2cxOd5xq
  HD2j+EymeGHPmXBmXdj3ckskdPHWV9vjRsWL+PWptfJkGc1q+gywck0TjUvQ
  YaiLtGSWFJiX2PxIWw8kBWjg26JRBJ4EjAg0dSBoKK230MNpboLkIwZgxJ4e
  l4TGRbdIRN.InPgUn2e94bN+W+iadMZgsmE9txspbyBQpW2.+KjI.TVREUfJ
  krNJZiBzKGseYlccfNcLscVIX9a3UspOhUtjAutEljh4T15CsXURXN4xovuW
  1ntpEk4mwK1QLgPUQFhf9FwQivcms+mY5e4R+xgcubgeT4137RHqpjJXEI.M
  m3pDeVDa.k7N3OjgXNAAlBhLbzsQo0S9QKfJUJBD3OksS.EeQKBocCkEsv5T
  g8Y2MMTjAMyNlbcBtIEvtujYypXqzVi3oLlewtFHEilc4EWbubIIzyVQ1P3j
  YgH99VbaU49UYlkcMhDlaQRFEIRlG9SOjbFl.K2K1Di376ufz8rYIqkCFGlO
  ExDvhf3kaRTHDATLd5mJxovEauIGY.wDcYRUP9luonFybXlpEyTYRRgh+WLh
  qIRdUwwEd0KuwEVCsWNgvbzk.as2JVLs2+jgu28xrPRFStKFrlUNWLf6ffzF
  p0bOXgiCKiCHD4e3SGH1JP3qVjOBm9BgLVX2Rxsu2Yo.I48hCxyLx4we64Wc
  UtznPFtHfDGFYx+WtkUvWTV38VIZ7K4uDjr.olhO4BaT4O3eKQPh7d3kgiwb
  4QLwWMFm4f00Cz5WsRjVWvlVft4hA+dVrzWa4ujvH.KNmubihVIxsXj8kDwx
  TzfyjcSvLzZi.YBorzV3RjhTBYMZR7o+BGOc9apLrGbHtj3BW5.Vviv5FvB2
  XMC3J19V2Ptp2mUOnY4sPciV18Tyvrh0+0MZaLvQ3PVmXDkDgTBTlYRYcC1p
  lcVxvj2F9Zfp0Ljph0zEgWHKwBx1T6MsBm2+tM9Jf6r8FPPk8lFM3dpYX1zB
  Q9EgwPhyrBc3euXKEHuAdjB6FwwjSxmaOhwTjTgbsphjuH.t6YYoIhbG74N3
  kJECVbuMjYgF+q9se+PQZq7Zorj016v+NaiHe9KO8j29N7EAqpf3T3IfHsHT
  pDgACLh7+i7ZB+2ey2jWIvJRP+KRkhPQ3JhCXu3zmuD2HTB95WTBxAk1ywbe
  c1ecQLlFdPB48s4O0JEa.I9dORN1O3oDlZ44g0bPCVMm7GlxkKumtkcmpZ3J
  Y4KIJstqOyuUVMdLDMhLe6gFLlikEbZv2vAJuliGK00SznRrnxpdwZCkJ8UJ
  Y83daOHwhrJprCnOdkeSspQyCzBfqnHlKOLcrJw80z.CoLKIt2phgpzB7eCZ
  6W9c+Y5e0krZNS+0dq8gVQ04w6JyMiNzkxbrYMojQPcx2yFcF5Y46y01PLoM
  hCmYKnxvpZB1NInaI+sQq1aLoFYm3ISgs.byi39x7MjeUXzm7X.bQL98bWyg
  aldLFVA3uYX96iev7XR1AiB9nE4HzO78WFHzuw7fiE1oVZmDysuT6FbohVLo
  wPWOlknp2RcGRwp4PE8OKWuVs+Yo098JsoMNSIiwtzSsrceh12wfZ0WagirG
  zxcS3limZB8kYo3NSEnuLbkBdUtgBWaiE1tSZ6sZnzEI0CgF5H6DYNMopEsP
  EjSWU0hL80UUKR0RVTOsdEMkj43qdqPO0PcXKZ1RTg8INTErAtztjXqVIifX
  evEYAfXGgkLaZ2Nmhtw6ZtYD0nIgASqTrjo4S8Zczm6WhetTG84Rczt1h4Rw
  .F266vbdn0Jx1yihcXNbpUUzPMoZpM9kRjxp643XJyMOR1TMjLN2pDIanq9R
  4zv6f8Gmq+h+qGIcLQFVBG8810NlntvxvQp7wBtEKD4L2cD25qIT6TNHEJrM
  8wR2q0UffcXsP2q0SyRIzIZt4xiMQ4wHd10eGIjc9OptpZ0MJYmmdc1Qrd.u
  cHnBazzZHI2mdHIjapsnh7d50U97Lw33QsqAEkGG3+zCGHCOrx3.S1CdmIzW
  0NekQuoMR4oHH62afXGEgXy9Su5xj9HDlUsqDx5OjFLUolc5OfrqpjFzGgvb
  +QNGSUwFd8mN7m14.e5lpgy4FCLurM4svEZg83MJM7Vo0DCf57YZRR5MGcl7
  PNezaDd6RFbH7OymcIraK2RsnzAUrau0sO5dd42oJYvB2bRHb3JQ.nPhtz8a
  uadLdM6tak2VenYzfYtc.Aq1TquCk7kpKCFndllv17d5KaWV9L0WjKc2Haw8
  u8+kizmlPxgAZ6cuk0zLfvA27VphoVRYXJVKholjDLFyckdR9KQ8bym+RNF6
  .hpQ4jr7HNtddS.knjdC4jiTBqiH2ITJwgKEC41tnnq3Xoa5MHIAtYmQQdsI
  JxyzxjZLPMU2a8DlYioHhoq54SdoRXZUxBZWMcod9syz0pUmtNLqNaFKOv.6
  5L1oQZauMZZvvq.SZVLoCtbIG9jI+wKO8E+wySljvsYZovxQvWfPiwPetIkt
  NPRN63aP8E+E+WFtdveYBeEcIDecbBexOIZzGVQYx5YaVE4vnuqyPaeee3cR
  sXCsMMr7b4Zf8b3uWeeaJGMZ6Mz2z2f4r3MymXISGioDEaqS3dCtRLSK7kZZ
  ZOjKCxwxmC.lzgl7WpuknOzYQcodlaUd3aZ2NKbvYZ95oPO3nvaZ2w2Ttcft
  FFFTFbvyD1DZ66xW1YdtbTsE2LHnhfa3r8miAKp+hg1lhVPw0C5BmMBO3.Qw
  +J.4R2p7Fs.uu3IKF3UD+tpKxRr7ZtF6VHXHU4VrcgaZMWhW2c30VuUEdrYJ
  .OTNRPW.joqB.TwPJ0oviXsvbCvyJXwNEdL6W3GOUne.zCUKfiuuJnGuM.O2
  FM9tDtynRlbGL0ioFXRih8ITwmV9hZuYfJL.V9ZaA12QA3wUivisJqv5igz2
  RA3wySeLjd8K5GOUz34nOELdFJ.OqPz+fCOVd8L3wsmAO8s0K6dF7X1qLHne
  AMTO8sZ4ph1K6Mwcsh8ILJS30uOldetXDRK9Iv2VHoTLLD2p3icf0KJM+nl5
  S7gqJrG15S8iaASOqf8.3OXZAbbr5WVK3nh+nNF5Cdnp5Og0V3OACCNAdFTb
  odK9TGvQ5XzuVgsUwCMa8YuicOydYaU7+SexOsc5YnG69E5wpMzm2xvylzuX
  qw3Kph7Sa8Y8tsgR1mns3UY4oZzUr0C73pJ7XoG3Q4nOYpG3wVU3QO1uYYpJ
  7nI5YlBxezW3f6YV2ZphsOt1am2eTSagGdNXo7Q5KH7oNvZRklAr90tEQ0n+
  Bll8JySLMUwcS8ANrdF+nRd2owkKUCNf9.mMu2d5C8vTR3i9bcj0ybcjohwR
  LMt24zdF9QIi+0H7zqBUN0WkkKpKUiPjRKXNdZDh5WTzTOU.HJUiXHk1+4Uo
  05VHRIiD8MzHD4nz1RoS5Hk12FWlFgHkjF4oSJ69k3ZWu9kwqTWk3y7zIDoD
  UsmiFgH0zw5sg.HsZH.rcMEElBOrp5X6Zs3icPP.nt8KNAGOkQpzsAoZZU.o
  hUNxNCopzVMREmLYplvqpErbMpJWoMOjpwsymZ2yBwHmLUETjEa6XFrcvZZB
  l6qNXgjR7otfUPsrR2VihXTZernZLsunpnZSi7AJswH5jOvxRIgWZz9Qk1qF
  pcuCh1ZYEXsnzTdBckxJrb5FYEl8rfvs5gkoN835BhTZYVi4LOsusQ.nQLrM
  5ofk9L9R.QzMBQlZFh1rY198MHRXXsYeZUS.QV8IJaADYqIHRE2Jbc11cA2Q
  3slmKsizyzyjXwTwIbGWMCPtaBfr0L.sIJeGysLp.NxRn.d1kgBqaYeDKQnT
  G+R+nq7i1K+XWPxJP.1aBAn4EDmME2HCMCPVaZCSc2RJDYWhxTV1v8sK8iHM
  .yWVvT6TZfMc9kYVa4TTdnRLwPhwrJ+iYsfFe4TzrCmhaTm61dnZnV94mTlT
  yx9nEEKqUddCy9TmME2717oOKuTJUCDtlYpNJ20P3wH0uq3ETxyLMlfDT0NM
  +a4tA3XSw1CpvMbGeKY2ctiPpz9lgQJE0ZMllGzdWVCPUeCS1NIlE1Ep7erK
  n7npHCxWiV.anL2rwVyMi5RcPqF5JMoFpDVFOMZflgRN3anSHpmcb5TJCVaZ
  PhvxczJsyO.RVoM9sRK7a812W0stuUaae727e+k++AGMWGpC
  -----------end_max5_patcher-----------
  </code></pre>


*/


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

    case 12: { //manual every LED
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
  int i = StartLed;
  for ( i = 0; i < NrLeds; i++) {
    strip.setPixelColor((StartLed +(i * (skip+1)x)), strip.Color(r, g, b));
  }
} //end SetBunchOfLeds

/* MAX/MSP control program code

  <pre><code>
  ----------begin_max5_patcher----------
  16593.3oc68s1baajrneN6uhY0duGKsQlAyf2JWuU4Xqj00Zakx16Y2phbkB
  RDRBGSAnCIXr8lJ+2ucO8.P.RPfAfjPfxVYWKQBfA8zS+d5o6e+O8MGbQxmB
  mc.6D1uv9lu42+Sey2H+J7K9F0m+lCtM3SWNIXl71NHN7iIW7+bvwzkRC+Tp
  7qSYWvhx91342FEOILU9Db0WdWP5k2DEe8uNM7xT5MxE1libsOl4Y6Lx3Xlo
  A9uBwHC16UOUzX4vCuxGKL7J7BRlml8FDpucV5mmDJu8r6itozOeWH8BO3hf
  3qO3X1AQw.H7d7l9i+zeB+mi0b5ea3rYAWGtx7myy9p6lFNKLNMHMJIt3bka
  XOxv22y15XlivajgksvyClzB4j1pvj9pj3zYQ+GIPygKTEZUTCZkaaOR.nUe
  OqQvurbV4MTDHKtHU.cy8cqFcy0GcukwwFMgiMb7G4466aH5ObL7aDG60Mbr
  2.CG62HJ1zZjCPFK76OTrvAQwU7FzBE6OvPwdMfhA52QV.F10q2vvd9tcGCK
  LLFXXX2lkS3gDw917iY1d8CJFdka.JlOvPwNMqtyLWTbeghM82DTrXfghs0T
  TbuPECODYQgbr6nnXGqAFJ1pEhh6KLLXR7FfgcFXXXyVImvxuWPw3n1YTr6P
  SamnUZ65GTrmu8lfhMGZddzrnXST0Segccb1Dr6PSFgQiRg8FI.5Wga+ggMc
  2.Lr2fSDgsMiKXNLAiyLXbOXNYvxgxIQwgWlLOVdyVsEmIbk759TLbrkwvwY
  swvw9dB0rl3VcWvGX4XGCI1Q9eUFJK+5nb.hTLTV9FxXtvsZHVVlCK7vrOFA
  yIlnx4sYMyaaO2QfGfFtlvZuhDvptoN2z4gxT2wdjEL0EVvbVXQJ.pcpKtuB
  2ykI2dKH0Zk49OcVSZ1b.FZak.WavgCWWgikUFeNn1qZguh0H7stnAKiPJH6
  vyeQD0J8BzQ1KeMBXLVEE2A73EySSg2cS3LZhXKnIhYNBL+OymRZibLb4ibf
  eDN.Fx2JSuTGzNw2735JCi9tQCkAokZqpgxf6iXJWgeyZn.K.FzZnL5pFJeW
  Zd6HoZZP+D2vefIoZ1GhtqI9NWg4HSvecWGzcc4DzVZGmvnvzTeVNP01hgiu
  lgSGVNCmcoPIXLuHbZ1a32BlFGbaXgq7KVuuMXNGCqErIkHP5DlKSX3pCmVX
  tEwxMXJLsRCm9qgwAWPnK9ltef3dApDlcbNlM5142JG8rABl0YeW9JVvuEN9
  WCRSmFA5DBW7WyTKTpUJb8Xx7vjqx95ruu3DZRR70KulwMxVzJcqytIYZ5R2
  aU2W1bzrhqEE+aQyhxPgz0kjbY+ZKwxFO8km97YMprjblcavuR6e7FxpZbux
  pxeul3q0ykBP8rvqhl.K0KwizJz3Fx2Z8EJeq+dOa6rzfoo.mqtV4twLt9N9
  aLe651H89gsUST0lqYUgp1HdSOuuLYM2W3KWie.eKiePac7wDb3zhSNMZYIo
  YHeRWmc+dadXo5bpUklb80SBqhIitxuvMeutLZ1dqwAYs0BVB0w2.+s8Dsmc
  q0n6cNKzJKA0vFU+81mrRqcWIcZKqjmfSdOKVif2Bq3thgUDD9OSFMILt0yX
  C4FQa6S6Gsecy30ssysTsw1NcAUgJgm+q7+W9W1kq1VLoKE4EKWog6Vdd0gJ
  sLFXgeZZ3cgwioPPwYqgyg23rGiv.F2IdsrNl1CtbNcqRHH0fXwkBQ3Nt0tE
  Q2WRQl8QDxqROb4qnwt0xc8GIDtl1XDz39ibUaTjIHMUH2HgBptRmOcgdzp2
  8fZ2MFSZSwI+GbocOnk5pM2fnMUoDsqljDzK5mKszTit40eeC.SbmENAVOaO
  eEGnwjaauKss81h5XrrEasiQvVUR6sQSCFca5nzj4KXxz+nTjgBLj4itmYsx
  VViXVmVpud4+emPGZtmZJglV9i7ssbAGI2Hy6sL7x2NMSGQ2MuWXNj2MscP9
  dPmlDKtN6k18kJrxjT56yGmLPacjCEs389bSTazSYm2214tRsoIs55aj8uUN
  2se.4Qqi9Nz5LX8mcgwSfkSsV6oEvEaZZJrE4xBcq0zTS9.KCdXuK4sgSiBl
  bPU4Ann8ohhoOoE0kB2ZsJSc0eycK.c4DWOUB26j3k0Zo.BGGoQDVtxouIkD
  u1qgJv3gTfsZQbsFrhAv7WD+Oqh4IRe+eamT1wgKylRaxLCAY725ryXeKglM
  8nyBFWFKd0QpwtkYRWcXOO+hrwTH+K8BzJOwtmvpyuMLddMaQtWS6BfsOgdA
  6Wfe4TQJYlFdq5PuCW5fiAeW3Y+gH6OrWboEWawEKb0BWtv0auSINbtb4xPH
  gawJaTiFqZN71KT1rcBkW3rWesm5dZum5+RW1ygnTPC7pXoptqrXMsX0cmJn
  esrBxKTPAeAJZdSTqs2YYkvXJ.a17ZCes3gD82Rn4ZH+p6NGBI0gf8gn3wyX
  IWwdUx3vYmbdL6uxLdruOKXZH67CNK9xP4UN+fSXo2D9Y1sAeHD+K1SmNddT
  bB6QiSdDaVB.3HABKAdhiYSh9PHNX3O+U1gAGwddBSdCyjWioRu9fISXX5fA
  OFKHdL.HWwNj+jyt5JfX5I+qa.JXfB6IuIb7Hl0S9ooggw.s1S9AXAawnewQ
  RnGHieruGKZFKMgEbYZzuEjFxjRGSmQWbwybo5Y7YxqhSHfSJBIIUOB6sfH0
  v+8wrKBlENFAvnX.OhywK9LPEsXrFmMV93K+pjorKuA7hGuSbbedTv0wIyRi
  tDeWoHNPhlAtNrNDvXOCzqGEOOY9L1sHtlcH9M3PdzIz5BLNAora.JZVZzsg
  Ol.ovqtB3FmIQbwgvW.y6Kf4AP5jhW92Bm9Y1jjj6Fw.CthlHgF7MffoDDCG
  eL6igrOFDmJu3cSStFnAwA5Cgg2wB+T3kySoIRDAciNOtRa47Z6F.Ql4vEzQ
  jiBxBdPqpV.BemlHOqYStXO6rW+iu3mpzeI6ZzX6HMwPXKUbW+94X50V2Dku
  PJI9JJD6pnI3B9rxFA7MGDb2cE95xx2tM3+IQNPt4xffYXxzxp6lFhTTzyaj
  +sASuD4NuLc9TB99jmyBwbHkxz34QkjekARE1UH5H2xocQmR9bKuBAXBV4td
  RxkeHbbw29AI2EFGEurgO4Wdb3UAymj9qU6ue4qeUvkgq8gWxc7ra45oQiSh
  QfnDpF+5rWGZNfjh1t3jQdGwA2UwCCDCHyb0WbFLImO6hfoETKHxtXZRxjxW
  J+4lDdUp5x2EEGuDVLM4t0ewoQWeSMO6EIvEustwVdkY+57X5p+Jvsk9qnR4
  x2GnAPw9Ud3+TPbzsf.XTdmb5ZjeQxxgalc4zjISJMeoq7aUbkw.Q7kgeLZb
  5MxWTQhA31itKiH5f7U4wQWGNKs72kFb8rxeyJLnvWM+BES5uBFfc2DXVT9F
  JUdsJxQVTPUouuVWLWMJ+fVE67ysnUwaqjLayBWnJuHWaXg4Jq8bWDse7D3t
  v52k1MjCJ+hV0uw0fJqy+wB1DkIquKnxRx9aH.h0DDw0p6ac3PBqwI+PWMTh
  qr8jqGEZzHJbMQVb2iAwSoDQDxKSFVkpzFvTFzur8aBUsHKy2yn2JlTMHG7A
  aFoEEmV0wmrNDVdVRODwWxY8ZwDF5fIH6JLy+20JlZ6JmpXnpk2QlWX4loND
  PQd1sAEkGc+ujPQbQqHix2GrunvQ1siNx5KQbje6nibtewQKbaAMZbM1qJml
  30qF4MKY9zKyffLcMrxSYv.7zn3bu39kbCdV59tIZ73v3kwyiilg17Sden0J
  8NCjEXnTGDfLR5nEHKsUhYML.ZqVAzlCCfl2JfVLL.ZiVAzCCZZz5rV.zCCY
  GUAKUC0UHkoa.sx56rvso1GDJ7R0rIHsJLHWOI4hfIpfbj6YYcwDYQfS154G
  blaSyx0Lq+dXYqhRGoM1u1iyB2mOLODFcYhaQmiGMm3FObl3BJly5Mw87GXo
  z1kASBAoDXDBLFstr5yoYBdZd2zr26g1rWQ0q2r28g1rWQ5q2reeK6o3N9pr
  TDOmGbWtGuxi0r94+JkpMVpiMheWx6Ft2Pq1Fy8ZDOZIoLJhFM40WZBzAMpp
  tt7NU2P7t2pKhMUYVbatHJYPm.M5rmsI0OIkraGY56UA+qVXR98WYd.jbcOV
  BVb0OcwD26IrSizcNMWzCTmciMmtSo0bin6bc+Rkty4AEcmP2hswVftSYuxl
  Q2Y+kJcmXehtacIn3qO8re9E+6SeI6su6EO6evZ11EiB0iCNc9.5VYiyzvMu
  RuX54utwSKZPq6ijWK3Crqv+q0EvXQVVYQMHLJ+dWqiBtdCyBipuGyjN3Ks2
  GYJUCb3RIXbrYKA+X4TCVvgOHcVDcSjbYr8HAZc2Brd01S3I6PFN0Ujn.A+C
  s5aukglJq1x9LZpvTcymQ26qfNMaRz3pU9SWQGk+NTYrwTLx2kpvGzlmKqrv
  4ngODmbwkISnLM8WjYiG8Ob0+zYmivDpxt5WpV3dsbORV+N.j9cyS03PMlkq
  XiLZ0Bn5RWbcIDkImaXwOV9WV199ze4YZ6VFusCqoHKQKTWYEoHA08lsEMRV
  qQ0UUUWr6.YMuHscm88ZCIqc79JYsdj078Fx50YxLc3UVb1+Ou4hnBopuXQT
  gq7aqilN6ubUTwuakc40UOM1NVN2jfAMK9LsTpvBABcWpP1lXsYRE3eUpfdR
  E1WDITag+ga29RnEHE.OhmtVZTvKrFDU0m0c15MnJ+lm0lVAXMn5oEgQ5XuV
  wdnVbn3VcmFgjBUOMB2eefHgRk0s.QBgR5XEDyXvRjX1YhDGedyDIB99.Qhv
  Y6PjnPIcjHwYvRjH5NQBYxXCDId6ADIBkYuaNQxFUKB8GrDI7tSjXZlU6ep4
  vUK1GHRb71RDIDJoaDI9CWaRL5NQhgUyRR7c2GHRL82RDIFaTSB79Tey5B3A
  VrLZ.KJbUaCsrhLn1QKitiEscUmsRyL+uaWXM72kg0Xc3IrVtzDdRPUtBEdx
  eShAjrHlY3RGZWitE9mcZW6rVoNFNcVpiO05ibpuouNrcER3RT.aK+kU3jNJ
  1Y35KjQ2CphuMWCxjgsyPYjIaKOlU3jtRlXMXIS5dbU7EBMHSF1V5lQlrs7Y
  VgS5HYh4fs1raz8Hq3QMjmlHS71CHS1ZdMqvIcjLwZ3JMo6wVwSUuYpmLwxd
  efLYa42rBmz0X4yGrjIcO5JdTsioAxDaq8AxjskmyJbRWIS7GfdN+Sm07gH.
  DgZYK77.u37j61L4KGFqoVUXp0HIWbsktSa4sxKPK406zRDolsiFveW6xM3J
  KXNoPf4+YGbm1YQJU3t11XoVV+bepVaMYq5736Bt7CrH5+Z+g3VkpplDxW3U
  uMPqgNwp00a3U+0VWHNkEy00ndDZlgqtxpeN2xs6GkuRDgpSQp6JUodsHBGZ
  GnbpRFvdW3rT1qN8su8o+zousR5PiZZARdFzAFkxMGqZyad+ujK6q1dRbC0F
  UboHb63Y90x95WK6qesruVS0GqrcbKcvtsY+mvoIvKXcU60RE00hZKz6tJrm
  Tztov4pyNwxlq05BVookQVBnt9AbM5UV6tcqekRcKVe3Zpv7ZwvzvSiFkyNb
  4SHTo6qHujqWVEd6qGv9TsbmV+3TPbWYD0bAbKWhI2tUiz53.eUxzP13fz.r
  Z8GytJJcFKJlcw7qtBLsHHNFVitDq39fuDLPPISlBoZUblW+5suFrqdEqwQp
  0HrzM2Y9U0ICjNkSbAw051Q9U2gL+JGXX8Ybl.LC2hYybXtLOlurUOywloI2
  TdOvc5.t8w3aH6qoFruVzgjhNd+YBg82.1WtkWw0SpBK2U1Wg2dJ66+cvjHE
  y4sHmbELwa3hqS2Ts5sEXUofEv4dajpUqAMqpYKYU8Xb+dfccIssaA1USuhq
  othMhc0d.ytV2Rd1p4AZrDX6T.SoL3YkBhdaVBrsnCdCsDX6V4.p6Jf495Jf
  hw5f1qyZqsBvIKOsc1nU.wdpJqqlOYhR0Dni5c+82b5oGnkbK6toIx0ZK3jG
  08IwZowRoNWqVxLFxphrkZYVVSz9nmcXvLsWrnYrQd1I36o7YytMHmQa+M7J
  BS2MO7JbugLimCq.e2AcjgYizKoZQ3BJ5+JFltpXh6u+qXpG3WrsKzff1hgi
  jNd2aH+xfN7FlUpnZUWl5A2jnVJFWTzBwMxMorfRRqhbZKm5ppKty9pQ5Ur3
  dv1ydCpBGoZMWqw5fVsnoZNJpdW0ZDHq6h19pusHqoqjKbmt5wIa6TgGdKt5
  snl0rIqdV6oJ+xCsOaVzs2MQ0xco9sqF1jPQrykjC5xW9fyzwH.V+3o6RxfN
  9eFxB9mNgif6WDmXx2Xq9DzdqPCn6FENHy80c+hVdXiSBmE+Hf1OM.3.9XT5
  Mx5fpr8RCHrDLqT181DpZtzJOoHp9MxlPtJDh0Nf5xFMb4i.kM1LaGYEzrvQ
  3earfTZEYKnrIypirQbi3536qg.7ofiVSCGqgPOkxYuBQUXizqny3oK1+dMZ
  dMgdSt5JcPuTRnZ6rnE+sYnWMFOMQu9CUr6+8Se4KdtN3VUPUj66kk0lia0X
  7zD25MTwsu30RrK6wL3tATy0rWE7oWDe27z2F8eBYOotsNqaZaIzIe0yGPW2
  zZZaN4ajnE6g55yYu9Ym9pyd9oZ6Q.0GZp3nz2Qza8imlnWqgJ5EcBCqjeah
  OXYgNZq4DVCCntaixP1Kr5rYrXJo4uv1A6M2+KcFOssEwXONpQ71f9IaK1dn
  +ZGOc0m5sOi84XX6Va3z6C2dKImQktWahWVKOhazF.6LjCoWoiFzR6+K6cIu
  Mr3YAoNlKGS44EUHVSYrakiLYgynRa0qXK7kuLSxAr07xzUYd6cTXoYzRGbl
  s3pycAwgZg8MoRCAsaRbUxN6rQNhUXDEtjootcDEeutaf5hCUleSr5FT1ciF
  iuo1yyKTSDMsb5nLjMgFEsBb46DLITcxpbMJwGtTwn10yxjagkfZtksgumr9
  cKLANvREiZ0bI4tjo4GPsQl9aSsPQSCFcEVCo0Xs7wnA91NFVVXjtG4K+AiQ
  sXjI7sbSPgMtMqv255r9EXi5WfcfWguqO7NdrP1rbL3Z8Nz0lLS8VzuH3xOb
  8TPe43UFrqiS.vcRzkenvYSbo9lc9wmCz5tlyoUa6+2ddZ19uQYuZ09uuMZ7
  cIQwoyxBkg7b8p19baSOY8aP9wxu0cdqN2wZGOUEFpTqQ9q7oJ9wddphGQos
  6TsGf4c8pioegRGX9pC9wdd0Qrq44vzXFs4xzUNGylqzm66Iq8tdxZQSViRq
  qxO12S0cMIrMcx8E91kjk5a26xRwLRuWlpdhRjvzm66IKeGOYsTIhhqQ4Iq7
  y87jk6uqmrp1uJ00oWvu5z6DwXNysimppR6z8tAA7csTXSKepxESovsk0Htn
  pOaZUZU2zp+W0s5ITgoUIER3G66opXmawqegJcWNAtiW+upZXzSNxHJyLKZK
  yrJlfYkcFUughJyJ0zXnZU4.45IIWDLQUrOxi4Uc0FjEEPjseCe6mkc7sFJM
  YVphdtkpFEsR83t5.RpcQ3Rjk6ahrirb6q221lssnHslXNtsa541uWSrKE+0
  J544vzYV3UQSRCmtTSTqUXWJ1YUL95fcM54Ng9NuCmaqeGN29duCm2P6pdcM
  4By5HJHdMGYnebquwaYNv5My150IBxJ.8aPOYl5DAYEf9t0RlM3CLzmklnOU
  gYeyQepc0sSnOeqAF1yTSrmpdku4XOU8JuSXOugFsmPylHhpLduwXurx3cmv
  dNhAF1iqI1SUcq2brmiU2wd19CLrmglXOUQedywdl1a.1iOrpxoqjFCarQ+d
  bpeLXYpQ+g0+dx590X3E6sf4nwW+raBu7CKr9W2oNsUTzL12sVSu71JUS+Gh
  kAVknor9kDc9s8L9Zcf8q0A1uVGX6XppENgs7wKS+hpjCQzRFbYVcQURyCXs
  n84ymL7AG2mIs4LoJ.1jfoWGNkp3eNlrK9bplkQ2VgbUcdQJcx3hpyjR8xzo
  gbxR92XNhC1LbD2iPNVMQ.tcOrrKpf+8QJkd5adyYu4fNlQcVzANVk30qIsO
  KcX5MuOy7vZPD2AbforW8z+8iAH55DXZ9XxvrTT5aWyYS0oRjvRbGuFwOC0y
  TgRB0Y+itxRwcTwxfLQm2DlvcOMEsCm.SYlQMJ+LaprZR9LR6cwpMV5VbxxL
  6ttu9WG3GmhGSpIg.G2M+Y1yCRCdo7u+APK32ZdBym8xnYopa.q3zacMiJtU
  SNUOF7MZRynX+jF8IOoqLwpf6SMuE6lkl4w2aULlxtHuE.09DMWUJvT1OXzH
  hx0bmXAaehvjsNolXhiVhI1XsXXOsvvBtJlrMSK5ZrOqQo6VxRDhpSOQiXIm
  8AWopAa8sLyMDUYtlMaYUT09qvs+yjQ.KXmMkScH7HmjTlqTKlxXqRT0qDT.
  pJFDUsoTUFVKZ5E0GJC+ANtRNyOniGPGBajUd1x920a+vVuJTU3xY4wz11MJ
  501USGHSrLUg9RCTj0lbPzZBGrSNLRtZlBex41.4XqnMLOXfXm8Prr1vrk49
  GLOXfXLIw4Z.wtCJHVGbrXu6XtUQF6eeAxh8PAFh8PAFh8NAFR6vzClcL1Gg
  Za+ADTq6wPydODlGRxNjAcSOn10XfA0bsgZyABTazBU37gAL6nqgRUQ+e+A0
  ZyK5ZtOB0NCH4dN5RUatGByDUMeeCpkRZb22f5Jj0buA019ZC0CH4d55inyV
  Bje3d.OWWRuuXy8tIc4M26Dlyhc0KOe7prSRqctuS41ruLl49hkaDOk2bOwv
  JE2orIpXpUoeFc64QGYJYDf8LpMmts8zNY12hmz2ogAi+bCm.BSKA0K1oM9f
  usOnulp1rEUS2pX7047zY6b+bV.RStFK1aUbReoq7KhlNoulT8MhunpguzzW
  +izqeA7XECjV3Q21eldaAqIs0n67iyqB2y8z377t75z8144cc7nmNcZxT1av
  ij+IURKUUJSs1yUNUMDrTsCsU5DcZSsgmpBT3lpY.Y2MpM6coPu0HQOk83VK
  IWUetHI4Bd8Rx27SF8i6lJLf2IMItoJKfqRNNIfvJ6eaMEPFRQIuY0ARKJfM
  WcupFArM43Ps8mzzACTwBQ+xgubyGt0bRDwkcmJzEbKm6ANoo0XWjQy1EQDN
  MXWj0PqNCDmDGxdLKJNJErYHZV33E4RcSzJYYjzpBPz0R5Lhkr5ghWWJbGbq
  6KirWGG2yfueF6cQ2FBuH1g2N6nMVSmsgIgojplTYgQ2zzoJ+1z.hMTTgcGX
  QE92GV3+yR+Wd5zoAed1V.mZShpjDgN1aLNk63Hwo1lcDmtSMfnQi6arL93P
  E4RT1t8For0Q0PocrVy.oE1x4gjw8t5abO2dvZc+KSRtCORprqBCGika5sFS
  J0Ck1FLoByMiIUbuxjZoKSpPrkXRElaBSp4CIlTmGDLoucRxGGm7w30aYxJr
  ephUipRyXrAredTSUl60MtOt48gYGiiBtNNYVZzkZhtbJHsZiPWVdajvJtw8
  A55Ym85e7E+TinJGRthxlIiMFW4RsqapjM2ZTkg28pbcSMkqKbb2Nx0sb1.w
  5FOnDqauWIVeNvyMulpwo1TRjL8U6B8GDkFdqJm5gKcvw3V.m8Ghr+vdwkVb
  sEWrvUKb4BWuCDtjKqjFDGQmJ0d9smt0rczsGme3V5q57oo904SyNP8JiLzp
  Xoptqry.oQgMx9dv.GLxKsMn3hrfhSGdrUiaYoCN18gVUz2pyie97oDAdU6j
  snkSZ77WR0E8BMnvRdWUJNa2Wy5+RaWL4fBM2Eqlq1Biqr6Z2mAa9N1SmNdd
  TbB6GWxSY8mi1p9uA0D230Wk0DtUOK8ZWILtnDtk9+kTZ+PpNrQ0YGZ+ncMk
  wn1wy7qUgsuVE19ZUXqiEUkyOv37C574710HuISVU0dekxgxPtm4t5oxMHNN
  Yg7hCxzSPUlT1OL+pqBmx9w4SJ0pJK5Gf5I55I80yxixlBG8NpuBys0I8slo
  ZOUa1dByoymndOZKusHEwVMVIoDFC65zvpTl2DoVndSXvXXx+Cym7gZnbWyc
  0FZQGUVf4ao4wN2eaQKVD36qZYV3FTNUTsbOtlEIDq8JQh4Ddxc9kc17z6lm
  xtZZxsrJD1sh.TMdlV0ToopDmIY3cyDkVaO4iqalzOhGwW4rOe6EIS5bgGzU
  P6lmQ08jiUSf5ALcZoX519ZwE36smzIUOQ0ob1JJLJUwSqJ.VF6qE9mnzjft
  VXBUzTBUl.1r4fhgsd25TQXv3x+yBaHN2W+2ZKvgNcs.GpjJPoAFm721yulk
  P6ArXg5V+BtC7ZdL6+SmKA0YnJphM4zXInVruVuR2r5+KFlFZ6Ss0q..6LTK
  vs0kflsl5wg5Hn19pBt2lTDWGxTOphlbx3Nqc1x2djqmsqoywYUcaG2MnbaO
  .pgzmEeY3lfRb.i1brDddlX+mjxGjlKA9CabxyRhS2DbhMlXFbKguEHqwsPt
  4VKNQLnwIY4v2Ic1HeapWnSrM9MaP1fGczYTg5TrosDD9.EULE.jPIp3ubxB
  BD1ylOcJ3b5qhl.ZlNQJv8jbdpSxk3bBqPzTNgU5.orJJ0uApKLeDMLLL44G
  AAqlkC4TiS39cr7UV2+eqpoq7diVdkAM.GS1V1gWdC3Q3kfCgLtwQrog2l7a
  vWKCSvUSR934wSCSmOMtzMZdD6pjoWFNikdSH6ZXY9NVZB6hPFtoUL73KD.1
  qdAEI1KmDFLMbb6Kv4M4JLXwuiL2bUktUuU1H5UVO8tO4TpaAYbPZ.6togng
  9.9+7C9kyOfv4Le9QvcjEka0fbd7JOw6W7Dl3SfGj8YrfeKHZB5oOXLnJ13U
  7r+d9yxElGcBcVIvEx6lm1vsaC29yvk3ROzZWssJbg0rIlsVbop4uP6oiIcz
  DcpIzZt7MI1Zq4PSu6Em9jmT1K5p1i21fVarGWXNxj5B7tY8NfF5xE6hXjLD
  v7UGBysD4qCUaATQg2sw1yiqlsvnKSlPo6.rThiowHOeCtuC9WFp+huzqoSk
  d36skkki+mtKHlZsfnxzQqFsRvy9deAY3vpPl+NON7S2AHzvw+Hv5TwNmrox
  tzjmh5QQbmFqT4tt6kpD1oBlDlzYT10SOASdhuJXZmJXRsfHnLYoYAS1FeUv
  zRBlThhd1plhukXZ73fWIvOdt4Nvyatk4XueZQJ4fuuI3iB53.iWdKV1Tg71
  MXfpAomlPu11MJjWTSddXuEbm+K.iUyv4jsQMqSv+q5D1s5DxVPzUmfu4W0I
  Tnc3PQu5fcf2ypJkgmjEwtwv666tK5TNCAjrpIRxMXk2VmsDCPVKIRybCRv2
  0cXxdCm2m8tHt2tMzN8W6KJOq.o3i9hrnhVSt.Vyc1Acm5mVp0kqusKG.WdF
  zSXXZC.N816R+L6wLYg1D2n..kDF8ag0gw0+Ia0JPVHMawZ.eqsFzvbpeVSv
  sgi8WpA0u5Mzpcwsv9roZ785fjEaKjrB56QbYgiM5ZQmKeOsqqaaLxW3Y642
  JLp8VEiJm.8CRsztCWChcs2W6536diLsc4tdsB45rsPtkmD8UKqyW29+lX3z
  IY70s8l3K1+fY7rlMPpW9951PmbGP3YcgYgw9GLyGP.smtsk.WmgCLqKh1Y.
  0eN0sML5Nb5pgtZyD5r+AyR2.FJsnSqUZIL61dCxVBj8zoKvsGhlGN85FIHa
  tW0.mjfLeupYnJAYGcDZ3OrfYacf4AFogPKgyN6ezyROzGR.sdpAGXrgVZAz
  6gH5gSypV2l9l2vAMKzsku4aNrfYcHm8rGVvrNB6rMFVvrNZUDCL5YSsZL3C
  GXV2XLtGBxCHU2Bs8fc3Do.AWSwF95By2FM9tjn3TU7w8nhCsipngal20HJ+
  N6mIpXGNQw4lru4woh4HMSsVNub5gYp1MD6ATuCVa18gCqC2ZuSpp1f7.Rpp
  LcWzCn4esA71sZY6rvILdwFQ5RYJrVsT0Z5RXXqpBOcnzQ.wzot5bq6ZZgRh
  V1A9JUAu2NMW001NNnpc3+bVXs0z4N2AcTMiBNkQoVcu0ggc5M4ozcMiiF0e
  cWq60lr.uwtmSFxxiug8SxLjk4FzOIcMdH0kErZQWVv5duKKnU2Gk66Q4An8
  HaKOts6F0xi8LMjUzaOt0FzyiMtO6AoMxA51HCHcnu2VnTJMh8nxvQ2vn9Oj
  5ybsnMy4NXafUWlDCp+vkLFVM0RthMCPSwWypUA5F1X28oidpmp5zUaqRv23
  9o+suNDV0XoMDg3Ql.jgPLpqGK34e+fPVaSy1XagDxjYa4VPO+ZLN0y3fspz
  hcJN5N1KCG+17T6TKDkWiRh8ILjJbGq0Jd+sQG8c8XoGhMoBxPSSWUSchNTH
  dFesMU701TwWaSEc8PENB3X2AGoP0IcipCOl9aPAxa+9.EdWvGXQk+utfs8Z
  w4JrBMOqdp.11U8zg.t9GhRmIdwNoRInPvjFd6Fa7Eb9dK9cqcPME1Y+6Zqv
  ch96bZhN7iGfMUEeanfv7ZGByb+7fstQnHNuc3HquDwQVsBG4Z7kHNxsc3Hw
  Wf3HgQ6vQeIJORUBezFGYsCMEXXhfbZGBxdODAs5YBt8VSJ.u8ks79lQQtaq
  yvaucrccs0NMsW4nqbucBqrZCL6LLfYy1.y16amjsUxy36MX1nMvr491QkXk
  b+6dClaE87.4nr0J54ARdspKHObNcGbsOztCnbyquOazObSMulS8IQSYdgov
  YKl4E9TsFdix7BKuGR4+jXeJ0KZldh2H8jg81jdx1cyombeHQOweXQOYzXlg
  QEfysE8Dse4aF8zCoLCawBvCB5I+lyzWysI4DkhBaF4j8CHxI+GTTSdMRMYI
  1losJUTK2LpIyGPTSd6STSq6XxHahqao7NL6LwfUh1J6nSkaBmtCpzu7t7df
  1yVz6M0BiXrdLhkmopk+JptMbVh4vZKfPdHlEgR4NjXGtKsIE9ddeMIB+ZRD
  90jH7a5VCg7ou44+yW75yXO6rme5tK4qTY2l8J1JrV6B5z1ksaSup5Pie2e8
  uddLi8tah.Eop9UZXb3znKYynSP4k.rAzYr6llbMna+X1kSCgk5wXKNkARQ.
  0uwWK6mooxvLB+MCXjiGGLcLCLE3lD5TRjMZ2d673nKkRyfWWPJCXMR93LFv
  pcCBJ7QXo+dBxPbL6+cdzke3wWdSP703HqfkiYAyXeLbxD323iHFwNbFLHGw
  RmFDOCqh3vabRvzqg+J3V7HqLC+FrOcN57X7QdZpbtR7230RQLvrODBTBHhH
  fcGvHGRWIj8pm9u+tW81eF.fwgDTecRH.CQo2T7Iki8ecs+jcQ1aO8Mu3ouD
  nce0q9mu9EO6ou6Em85ldRbQJjcaPTLiDhF8aX6KcZ3IxK9X1aRtX9rT4xxs
  gylE.r0rKBij4fVBSZUGCtF9AzXviwO.FoELIZrZDBtHZRTprnqOCObKgSmV
  ZvBt7CrDPMqZwL6oXOOJ353DXw+RVR72kb0UrqlGKOIQGKGHhnXRRJ61joXi
  VEfwaIJ.bLUiy+5lvXZQa40LbAAUAdrbxgqGJhQVLLl.z9nKPsmOhAFGieDI
  ShhmGxlNOFoNGIeCvOuHkMFW2lkvt3yxG9iAQRZVbfABQUSgEFifoSAD7wxg
  LJFHEB.x33IeVVW5kHU.LJ07XwuPQkSc12iydso3LSATYOpbgbBV6vgYZzDB
  ynZIsxWMRK8iR7Dr.eH+nyO3Eu9EuCIadyoO84mibu33bUzzYHs7zP0iKeo7
  BWC+VDSFD+4r0R1sHkxEgrGIrsej7IDJ3G.yw.xAdD.o+HrOS+H1GuIh3JlO
  C46mOEtUf2O4iwYy6r0CDM9nKClENiFUS4UANDPVvOjMnOGVReYX70.+9ARA
  GyB97L1MIeDPJ.LdgDI7wHX03pDT1vIrnqXOx3QrHf.JDXvgUtbnfPYzMJek
  OEYdASl.iaNdAsKvm.queDeQALPMPzsykr8u6u+lSOkFDPmvI+PHHmgc34G.
  3kyO3Hl3DDCvLOo.LSTqEm0HQjRXPF0A1LgkPFs5hfZHADEe6lpaRgEIxlQp
  2vKtZ4EwnYRZV4h1wH1.9hwQytDjzFN9XRFL7Dwfv87G.AniUPQ.8n3WeEvf
  kw1CX2ESO1eCzWQv6iPbEneG35+3MgRrMN7EtWjYJ9QorvOcI1rrwqh8j7WD
  q5kCnhXflJE41dUvmdQ7cySeK7c.As.Hn+g+4K+GKnleJr1GLdbDJWHPwIdb
  Fg6jbbOcv9wddN9pH5DbRVf8sJlQ3KAt5kgPElQJBE3wJrNAjXEXIqZDOoLM
  .BG4BdhReDtVAbuSpkIeDa77ojHAT6KnyqXScGwzrmv3RnSbD6ovqHqugpFl
  a.KMUi0X1gfZQIKD7a7YIj1Q3i+3rwk.T4aB6C4vZORRg3Nb1TwfdQ3UHTic
  DvYXaNHA630YzmftD4KLaNsFAgkvTfYIyuk5M5y.Z9PTYDSs1HIcyDRQhcjv
  KXPwzTk1L4hs50HueDAiRKkjyGZBbsrODEOVp9.YemQZHMdruuDDO+fyhuLT
  dkyOPtJBBFC9PnbDy7g9QiSdDnm.MfAeoIvSbLaRzGTpaweNL3H1yQ3FtgYx
  qwlABA.XFQmu7zmCPPrjqDUIdH+Imc0UfUjO4egdTdLy7IuIb7Hl0S9IP1Mn
  oz9I+vj4gKF8KNRB8fQnO12SZbFnUBU6CVewjNEBBcjWbwybo5Y7YxqhSneK
  .VIPUTzivdK3jU3+FTrEfBySPQ3fQY3b7hb0GxwZb1X4SBLlxxsCSJFXgReX
  XQknJUOFFOlCOCPwHYFRlCDkHtlcH9M3PdzIz5BYIkjZCIHeLARg.IC3hnDw
  EKEqjfppPG+P5dz8+OKWyGsP3p7MffoDDQggeDneBhSKQ4ACzGBCuC3gBubd
  ZNaG9riTlt8c3u+tu6meyY+zad5q.Kzd86dyYu7b.EEiZLQgpJJjWFN9mApv
  m.yXSF66gY928crWe16juwWGl7yQeJbBY9B3Z3e9OSzqyu8Bx5zhzZ.oBdOn
  nXTtwEShh+.Bc2DN4NzxGIYezUklKfoMyJAVreX9rOmCTbA.Re22At9hrg3f
  CSdfo.s7NbgVJojM4PB7bmGCiYz0fWrRCDQBleKBV9ntuALnFeubRp3HQoBn
  2tjvj.BUqNSzDjROth+r7XeYwF6QkCMNDp6R9FVdDJJPBF.ro5fKB3nHsjrH
  xBdEjbSbR+uUVXTvHuTk6OnMHxAGzsJAETydlDPklVXHHZjm+hm9Su9r29tW
  7L16N6rW91yikqCE3KfkgumszO.3QBIBIoCvKIWX03EO5U.guzdaraUJ4gIL
  I43zrkr6ZYbyaAigjlm8DjczXEfPBFOObRvmYxPGyN71YGgJeItsv.PFFtpc
  LYxK7xdzho0iHlMD8gFcgVGmgZQPRRwmIQYEOFjnHrk47hzvojK.zheYXCUG
  LEeXoxynr6cFg4y8eBIAvNEE3l3TkGoRxFb4JyEj.oKMJZx2VgWnYbQX6usn
  QJHxy96ygHoM6eRZ4FoWEGyH7loorTqG4+XvkWFdW5rQr+AJsIJMS6Vlke4.
  L5nEJ861PvxfOqvNEsO4WJBPuG4nUcnLpwXAJBI+iP16Jd7+az4tEHX3ouZR
  v0JApnxfOhbJWMGHBk9Al6sENzpw6mA1Xo58bq5JNd4L4KjrMOVBXjIAY9WU
  kYApWvZF7BTCYhA5z6.WVeMXl.gWFG9I4qXYNF7puEMxnfDoBHzxFjYfKDRg
  LUKHEDjeY..LnMEyP8UJq.Isnj.FbQcVtHCkM7z7YYT.fAVLyoaI2pe1gRDA
  JCTQhW1pWz8Uz.Ozysi.6wAyXj2kTvMYUEZgMQKkbmhOaD6EJUEyJXCDLI+3
  zDT7WNd4M.VLCgTzVxhzQHlACZQbH99udRQeey4rOl8nrw6QRkzyTB..nK2y
  PTM.F3mogkXgxs9GeqJfSZ4RFfI+vxgN3wjrs42MVFMKImPFXKYFvudD8rRK
  vPSGyMb7XkYQxPkjVR7nLrgO9uIu+LKdxVoeW3s2UBzvHmlLM.rnA05gV8Jc
  gl76lLuYEfhnoK3FFQrhRF+qc9mUdZh16srSYui8OY+rhVT2mtku6eKAD9.H
  x42c3QreGsiDLYAQBGVxZqiYm8Oe2O+Oe2QeewaYggOkuNIrezEnO8Gx41BC
  C5BnFD5ZvqTYBwgvUgKiW6r6H8FGSp+Q9UfSJKBQjv9j7nzj+djMP8IwGd9A
  u237Cngh3vHcr3.bCPhjQAdByPYIBvMjIsphQ6uGBpYGgiHdYPUA.zGV1IVv
  i8hJJn28KhiPi+hlsrKux.UfSk+S3TvA9QLRMgT0fLRaREVFHE0er4DUKQyj
  KU6UrmxdA600RWsUHqPAMGBNA+6pkd1ayD6IEAcCn8JDk3DgwDEc5Hy1K79W
  1N0ak+wgzZw29skrjQ9cUotrzmk20x2gZQOOdhpW.N7+.5MvgkdQzE+QEX9t
  j+NPUcXNo8ojI3ML0na9YnkR4FQlQ7onbPpF0vBh1ObI8fJ9TFnK55nzfI+q
  ofv7R7h+8W7S+c5wYj907H2fOuZj+irUE3GYq+L6tPLyhIE9yOCSSbbdFFRp
  YEtlbLd5zoAe9cIziuD+QYlCEg8NQdIrr.RKeMC7I.ntOC9q2VEE9l9tUj1z
  jEmqHtICuTfTG4xO6ebLtDC51UQu9D0Uqzvl0tZ9xy9WYX7JrWQMjJcaYp49
  9BuoksIkqtXw+sfQcTL3RKFs3GcBElFkIuR8gGy.KtUKmRTRUDZDkZl8Ee62
  98KKm8veDjy9KJACYgV7kxPLTxzwSXfX3i99JDSW99na4i2DAF6b3gGtJ2MF
  k0iX+W+WrCKaZ5+uBp0ORdCqbGOoL0bNaXI2FJ8Lueg.loKXoYkAYEV4Ox31
  K+V+akeoremVrmFl46CXt6UXPvKoJqRj7ou4Mm8lSnPgJe3YRO7wQf7lRY4J
  ZAM7kimDNh8742hxvJf9WZnKIOsp6.VhnWOQckPg7tvvc4MASYig2yuTZrde
  4wBwexqeHdqGypXo8nhh7Vl8pAIl47Xr03FWMKQO4IEocnUnEN0sdPBuOJxf
  OhtrbXdjzbfiUcgYzzk3vOVXjzYJrjv.VMyLt5Z3zZgoz4z1qYs74nOE3jIy
  Oriy7NFbv+tIgogEIXj3s+.lsXLpJiYphLEtoZjDzBAA+wRhqKocklg0odE.
  3LMLm9oHZG1kyY41eb4muDjwrXOhkC0gksZ4wK6cKJ9oX3yHpkrMH3YxgDuz
  X1YTv9qmUVMJj4xvCU.muziU5k98qcjkza4gkVxuMBl7zFebRMieAFfZF82P
  MT75FIXgrL60QRapSKjlDxsdfnEjgw9iIymf6VhZG2lkPaNMtKwWlNGi..5N
  cZlKwvyMRs4xvehySouBy.mBfGc4.V7WHwV3ydTF7pCKXcRgpSFCvnvTbJfn
  xZnLvaCYUxIMmlSUR1Xhzi+4ZnsUD2vL9i4QSo.QcAp5x1L+2XhiXremhLmZ
  O6LwTZn7t18CETFQCSIMgnTSgs8Q3.8Tz9X4lsBqvvWd7hVPO4FmB7x2gjJB
  ZTI2CneViwRzOEhGPYMzjOpxMiWtStOQJM8XJ13iyCaPzjOW3MULh.H0dogj
  Fy2UZq3W7DmjQhAy1rMSewHi.ye+r27N1qN8su8o+zoKthTbSwWLL+NRhNoD
  Nw7wx2U1V5ggbYwCypzdwhCck5BVq7nW75e7LZWYMkYEbT11HS6wzEgpH1Mh
  8zq.eonEY78tjjfJdAYPXs2jT5xhMdswQEjxTkDqbkUp+LyoNh37Bhb.SPgR
  qPu6ry.N+W+SMuFka6YouqZqJaVHR85FfuPk.PYIUTIpT1pnnFEnWMZ+hL65
  Pc5TZ6rTv7a3UsrOhqcICec4ljR4T1pCsbURZN4hov6qZTW1hxhy37cDSJTU
  lgHjuQ.ZDu6r8+LS+KH8q.18hb+nJrw4UPVsVpfkj.zchqJ7YQtATp6.dHC4
  bBCLEFY3naiRqm7iWBUpUDHnepZm.J+hxCocGkEkacpz9r6lFJyflYmvtNg1
  jBb2WxrYUtUZqP7TEyubWCThQytb9EeTgjD5IKIaHbxrPBeeKsspfeUlYYWi
  Lg4xSxnHYx7.O8H1KnDX4ixMwHt39KnbOaVxJ4fwwESgLIrHIdAShBwHfRwS
  +TYNElu8l.x.iI5hjpf8seaYMlEvLqWLyZSRJR7e9HthH4kEGW5Uu3FysF5Q
  EDBCnKI15QKYwzi9aB589nLKjTwj67CVwJmyO.bPPYC0JtGj63vh3.hQ9G+z
  QxsBD+p77Q3zmKkwh6VRg88NKEHYuSdPdlwNK96N6pqJjFEpvEgj33Hy9+BV
  VgeQUg2aonwuf+RRxhjZZ9j41nBO3enPPx7d3kgiob4QNwWNFmEf0UCz5edo
  HslylVht47CdeVrzWY4uhvHfKNmsXihVJxsTj8UDwpTz.XxtIXFYsQfJgTVX
  KbERQpfrlLI9z+EfmN6mWaXO.Hth3BW4.Vxiv5FvR2XMC3R19V2Ptr2mqePy
  xag5Fsr6olgYIq+qazZLvQzPVmXDsDgTATlYRYcC1xlcVwvTzF9ZfpULjZMq
  o4gWHKwBx1T6lVgK5eWiuB7N2dCHpxtoQCumZFllVHJtHLFSblknC+i7sTf8
  y3iTZ2HNg8zh41ibLkIUHnUUl7EA3cOKKMQT6fO3fWpRLX481PkEZvW8Ku+X
  YZq7ZkrjU16v+HaiHe1KO8ou4szKBWUQwo3SfQZQpTIhBFXD6+G60L32e62V
  TIvRRP+c1ZEgRvUD.XO+zms.2HUB95mWAxgj1CXt+Z1ecdLkFdXB48cEO0Jk
  a.I9d6ri8Cc1f4VddTUEznl5trkX2exeprb48vsr6rtFtRV9RRRqarv6XH64
  X91iLDBGKK7jeuxQ9dCOvvbWOYiJwhqp6Ecp4RkW5tGHM1lyZpCcI3NEa9cN
  i7r78A9QlIeM3VAejKW3XKZKBVXKQvTUkfpr+7116trGVnWtQ1ANwTJJFzNA
  lR9sr+sTmqJKrOOl9dvyH7l4mPd0g+sfReZ5ClmvxNWJ3GsXOlbC5vEwg5aM
  O5DoYBU1Hmb2NbBtbYK9yXjqmvRVhQ4ti3zgoeMsuHWuAV6KZia2l5zlGsbZ
  tkFY6tG0Ri1ds8IrVooONxdOBGIalqkwScg9xTC5KCWkL45vcqoPMX19945t
  put1TwiQSMTt7Ec2V7eacQiIWAjS2KZLl9ObJZL5VwXFnkKlorL+N1REHEaY
  eug5r7N7Z6Sc61tu11rPxftdBhrvI4VR5TVg0g7nxs1xHio4F2soubRXvzFE
  K8.uTy701U2WqzLesRyroc3qTJdc6fZLiGYhhp+mzXC9xb61funY0vnKpkxT
  Br253XtvsHR1rQj71sMEQSqARaAbZ3c31SB5ufesypYRBpp4460XCqy3gXCA
  DOQa6NbKU7mEtMga8eHhZmB.Vnz1zcWyC0UhfcDMgf8dv17PURI5JZtI4wlj
  7XBO61XSZ04qM0xl6USBmuz52WpBzp1HH2sJBRFyq8.jDwqoMRx6KtlhlmIE
  aOtslnH+cYSQ69AGnBXr13.SwdWigyW2FOjwfoK93oIH6OXfXGMgXygSqRxj
  uGBy51T3DCGRCgtTyNCGP1UWRC9dHLObjyIzUrg2voAq06bfObyzq4fw.yqZ
  SdKcgnzvaUlBb.ViDSSRRu4wuPc.Qe7OKcUkcvw3+Le1E3VkbK2hyOn061qv
  yq39ToBa351npNz3qLa215d7RwFXGt8tEw30r6tq81FB8BDJwYCXTw94fNtU
  +pdXjun1MoT28nDdKWDz3hLwhucPC+u.ReZBq.FPmcuUzLhwg171Z2kawFuK
  2k7td2tS2SRBFSIvx1IaP8bKl+RN0J7fOHQTpSX1p4MAVgH1RjSNJYrNxbmv
  tdrj6fKqIVOJ5J.KcyVBIIwM5gh71aPQdlVl7pkKWCpPXSIOhoqL4FpGaH1J
  DLaw4b6mubO+VLesFZyWmRfjlJXrn.woyL1w79ZFGMMXzUnMMMj7g9tNir88
  geg5BrGY.JFb7.Mmt9i3v2ayg4HPT6CnJm791NXP8EISGSIhjnkogF2.z8XZ
  QuSSS6QfnCGKe38axGYBuSeKY26xh6x8LaU5ya1P5yCf8BIFSl7qu7zm+qOK
  YRBX43BUF3g375oXSGn3q4R71nIfwHevhcD5MF43av8k+E7KCWO7uLwuhufd
  353DX1OI5xOjaS4tZ4jCVB5ZXXvE3QCRZUnsuK.uBOWXkzBrNBqxxFNU6tQc
  cWMte9HayI6p.sit3govCO6JvWgqc7VksoMzE1pdonFDp7kWNdrD9cYOmUX4
  Ucx2nj26qycY7bZT3tVwW4U8SdkU71AQ7FgHPTCyrWfHtuqV3HudCGw88FbP
  j+.Chj9XMrvQtlCNHZvgibbFbPj0fixlO3nibFb3HwPCG4M330jaPhPKMsd8
  DDoGcDBQt8CDI7zFhb5IJaSssXi2SPjsFPzRhHt+Ii3F9CLxHBhb5Ql+lLyl
  f65HitMZ7cIQwoyxNIHd7QdllltVneSpdzJ3klE3KkH66V7V2xymFwvRJiVM
  ern37olJNBYdQt7GsEzG4VivlUfqo561ISTsHtcPzgUaln9NRGdkvtpgXux2
  c+svIIDEavBmg+H+cMMndb4xkFy1LUbsV.6Vpklk+tcv7wyTaVJigjPKIDI1
  PgVNl7Ez96HBF8PvhdzjPtNg5fys5QHxZfYbggu1nH9.xNdBt6I63Mb01bG6
  dBhrzFhr5EHxSeKB6ovbZnmET8Fcsu0vBd7FX3GGwvBdr82Fvyx5lMk4.uGV
  +t1AZfsGXqoZJKWZBocKvhNbuQbGGGgSAKbrMK4KC8QGCSkuLdn8yxOtKL8Q
  Gh2krOZWh3MGXwvvRmnEZ1eaKmVwk2UzaLJZgeb5u0Ks1+KyVFMEGp30.VAH
  4L8D4eZGvQp0tbZ42eXTsn35Q3QGIDkONJ6VyQzgCv2YXAOVdCL3wcfAOCs0
  K6AF7zeVDni+YCKng6YOrfm9yb.WcDNa2Dy9RpeEbgL2G8oZgfKkB4k+D2Rp
  T1zvPdqxOtCTNq07C7Up2V+0JyQr6u3D5pQfgwpWWswEdKZtLWWyAawdz3Hn
  7PjJHUtbu7OsK7LVGyA6O5Ma6gE3XsMj2rkgmln+s6O0C15EaJq1IPlKLcjh
  YcoS5D2zVdbdV9iTjL.wg94A8P9c6B1DigU.jr7z0qo9Iv6Vt5BO8SX2sz1q
  x9Ip6V15BO8ihKKScgmdhdVnS1f2efyvJFIl5D0IW6VJk0zVZZqCUj2UFAie
  ZGH+TqYP+ETEScDWsTFqbuCObGi9CfLGT1gYZpic+8G3HFXxGzxMjdb4RWuz
  5Ovo4fXzenGgVBC6wrn2aXQNKzJ8v6OoyB6R1ItFut5OsWBygk1KAefQ+nky
  Z8H7LnBwL2Wqjgvk2iPjVKXN844bbXQQy8zKaD6yy4jVaMfeORE4okQz9F8H
  DoUN760mzQZsACthdDhzRZjWeRYOrDW65MrLtm6pEelWeBQZQU60iYOtqd5X
  a8ocQXgYqmogOsMj1TKVk97t3nt3Nr3Eb7zFs1ljO0hKJgVs3pc6cWgVczKe
  TM5uDR0QuM3nGUmqWdF2i68L2dfEVXtsVQ1vRzN1AaGpdjS4gnCcHAkeZWvJ
  n2VHZ2iBYzKYw6wTlhqi5sdjOPqMypO4Crz6Hjzi1Pp09qwsGbPTqkUPcYJS
  U81TIqvxYGcFhGXApb4LanN838EDo2g4sGcvansYIZd1a756S1Yim2Yu997I
  0rg198HDYrMN4NqrG3NRGp7b46HIVCLZegNNz431y.jaS.jcOCPMIcvosGuQ
  GUAzkJdkXyWqpOZoNhd9U9Q2rCv2hOtSptEtkxZLM5SH8vBhSSwfvnmAHql1
  dR2VRg3yUjDTqkz2txORz.BeUayZmRCzzNdJrZ4TTkK8lTKxVXU8GyZM49po
  n4NbJ1nN21dVB3V9EmTlbyp9nJ9RXutH6S6roXyaZT+oCWqM1WZjeKpaNtFR
  eO396JdAsrwuGSGAtd6+eKisriMWVv6sjNz43KIXkeZmfT4CMCizJ9m8XRCv
  Gb6AMW+fu2JSvMLKE78RedWP6w0QJjeOZCrg17yFslelzl5P1MrqzkpUUX0q
  GMQSG7YW8llJF6A2c2uENclZLkPxA2F7+PESe2ikeLJl9nrBuevzPrELkr3c
  bPvzKuIJM7xz4Soh2+m7n9H7AXCzZZ77HUgfGdy+we5+ewrtcT
  -----------end_max5_patcher-----------
  </code></pre>


*/


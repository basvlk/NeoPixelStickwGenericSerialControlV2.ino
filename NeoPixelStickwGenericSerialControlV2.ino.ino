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
const  int nLEDs = 8; // Number of RGB LEDs:
Adafruit_NeoPixel strip = Adafruit_NeoPixel(nLEDs, PIN, NEO_GRB + NEO_KHZ800);

//PROGRAM CONTROL
const byte ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
const byte  BusyLedPin = 12 ; //optional pin to indicate reading is in progress
unsigned long LoopStartMillis = 0;  // start time current main loop
unsigned long CommsTimeout = 1000;    // How long to wait for expected bytes to arrive

//DIAGNOSTIC TOOLS
byte Diagnostic = 1;                // switches on all kinds of diagnostic feedback from various locations in the program
unsigned long Slowdown = 1000;      // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
byte LoopIteration = 0;             // to track loop iterations
unsigned long PrevLoopMillis = 0;  // start time previous main loop, allows calculating how long loops take to finish
int LooptimeDiag = 0;              // minimal feedback for checking efficiency: only feeds back looptime
int ArrayDiag = 1;                 // if switched on, prints all arrays every cycle                 // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
unsigned long msTable[10] = {0, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000}; //Delay values in ms to 'Slow Down' program for diagnostic purposes

// SERIAL Comms- required for the core functionality of the Serial communication
const int MaxInputSize = 15;     // the maximum length of input data the sketch accepts. Keep it to the minimum required to save memory
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
  FeedbackToHost();
  // End of op housekeeping, and feedback

  //Serial handling
  SerialReadInitial();
  if (ReadingBulkData) {
    SerialReadBulkData();
  }
  //Action!
  switch (Mode) {
    case 99: {
        SetDiagnostic();
        Mode = 0;
      }
    case 98: {// colorByte calculatro - feedback to Host
        int pix = 0 ;
        byte colorByte = 0;
        int r = ReadInBuffer[0];
        int g = ReadInBuffer[1];
        int b = ReadInBuffer[2];
        colorByte = strip.Color(r, g, b);
        strip.setPixelColor(pix, strip.Color(r, g, b)); // Set new pixel 'on'
        strip.show();              // Refresh LED states
        Mode = 0;
        break;

      }
    case 0: { //do nothing
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
  Serial.print(F("[ Start BulkReadLoop. NextReadIndex: "));
  Serial.println( NextReadIndex);
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
  Serial.print(F("[ End BulkReadLoop. NextReadIndex "));
  Serial.println( NextReadIndex);
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
    delay(Slowdown);
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

/* MAX/MSP control program code


  <pre><code>
  ----------begin_max5_patcher----------
  15428.3oc68s1baajrneN4WwDs26QRIxLXF7V45sJGakDUqskKamS1phRkBh
  DRBGSAnCIXr8lJ+2ucO8.R.R7XHIHHjiUpXIR.Lnmd52SOc+me4WbvUIeHb5
  ArSY+J6K9h+7K+huP9U3W7EpO+EGbWvGFNNXp71NHM4laFGdvIzk9ifIwA2E
  l6J+J+2xt3zzONltT12DO6tn3wgoxghu3KSlkt72ReU5GuOjftChhSOf8apq
  deP5vaihu42mDNLktAgiy.g8ILKW6AvuLsGXP+67GJZjDXRt5+4QlFGLejl.
  SfzvI+dXbvUD7lACSC9ivQ+dPZ5jnqlkFt3ulpPPJLDhGFOKL45ruN66yO7i
  ShuoRbUg6b5sISR06VyvOlkbsn3+HZZT1Tht9e8ke4heI+2+5KOQyU96BmNM
  3l4K8ogePh4wElv3QvhAKMgME9S1yBRCddX7Mo2xdLi6yt5i.JikbMaDbgSX
  .Fj8hfObd78yReSz+IDuI6ArmM6t6wQAuoAYukwQwgCSlEmlaV1DgkX8Hrpk
  ph6vG3BjS1V9H8jfKvegenLxJg3fVDwxYBlIyhYybXtLih+2A6QbAOOtPHJG
  W3zlnhZltWmDmNEnhjyVK.hpAKb+jPf.MMHMJIN2DyzySJx.lWvu7DqH4nMw
  dJ4Sk7VxCc4eO4Ppb9lfUmcWX7rxDYCyqqBm7qdqqH6Jvi19Ddz2.+ki0pjG
  og2oz1.W5fSN3DFO6ODY+g8hKs3ZKtXtql6x4tdUKal0oY4DbF2vJnCmiyKg
  gPN8j+agomFKfN78qhmkWwqQwi5VM2.EOQoQAiWcdU1coXNrWrrsSUhUIqf7
  BquwK4nn4MQstqnKyjrX5fLb175jJaI1uzeKglqg7qt6rKM6YXxc.jjthJIA
  6cQwijl07hjQgSO8xX1WyLdjuOKXRH6xCtHdXn7JWdvorzaC+H6tf2Eh+E6I
  SFMKJNgc3njCYSS..GWRYIvSbBabz6BwAC+4qYGEbL6YIL4MLUdM1z2GAz.r
  fwiYO+rmAPPLK.r5J45qYGwe7EWeMr7+3e4VfzDnId7qCGMfY83ebRXXLPc7
  3uGVvVL5WcrD5A5yG46whlh1vELLM5OBRCYR4YoSoKt3YFpdFel7p3DB3jhP
  hH0ivdCHDL7eCl5ELMbDBfQw.dDmiRaAWLVixFKe7kecxD1vaChuQZMILtOK
  J3l3jooQCw2UJhCjnYfc5Qb3YXOEz9GEOKY1T1cHtlcD9M3Pd7oz5BLNAora
  AJZVZzcgOh.ovquF3elJQbwgvW.y6qf4AP5jhW9OBm7Q13jj6Gv.iOiFKgF7
  MffoDDCGcB68gr2GDmJu38SRtAnAwA5cgg2yB+P3vYozDIhftAWFWpcsdaiC
  SFUKXvkLngKLweYaJkLv4VUHZXirsHN78vCuB2w8rmdwK+gy+QMEmZutSMGG
  oMFBaoJYG25j4Y5cP9AJbxxBwtNZLtfOsnZ6u3ff6uO2WWT91cA+OIxAxctL
  HX9Pe0B0cSBQJJ54Ml+sASFhbmCSmMgvIevyYgXNjRYR7rnBxux.oEX.CbJK
  +GGK4ur7LVL+gksaFmL7cgix+pOH49v3n3ksSY9kGEdcvrwo+ddapkV4Wx0u
  NXXXkO7bo3OYBpiO6VtYRznjXDHJfmwuN60g1BHISsyOYj2Qbv8k7vvJLxIW
  9EmBSxYSuJXRNcBhrKlljLt3kl+biCuNUcYvmz3kvhoI2W8EmDcys07rWk.W
  7t5Fa4Ul96yhoq96.qV5uiZjKdef3eEuWwg+CAwQ2AReQgcxoqw7KRJ5uc5v
  IIiGWX9RW4OJ4Ji.J3gguOZT5sxWTdhA31itOiH5f4qxihtIbZZwuKM3loE+
  lUjH.e0rqTbn+NXV08igYQwanPzpxyNlWJUguuNoUEkXMk81j2DlmbsbnbYe
  NyweTkD6pEsUk3MaIGMmbgxwbYoaKKgydYfaI9urKprCJS9dKiAuO3cLNyD9
  eNypYjXYJ.p1a6F73tADog7WdFMgH4FVcExZR38XrxD11nwO1qI9huawWTzS
  4F1Mgv78ZS7kb9slXBiMCSjObx0iLHsqly+2pPEBQdXLysAIr+oKJxydcPQy
  C69emPQbwZQFYx+6HNxd8nir96HNxe8nibVGbzBSsQOSqvFKIPgWubj2zjYS
  FlMiTJG.WTJ.ffQifOwy873Wma+xR22sQiFEFuLFcTzTzNUxhYsVoWWPFwZZ
  AxnkBvhQ+.nsVKf1re.z70BnE8Cf1Xs.Zdu.nQCSVCftevHVFrTNTWhTlMCn
  Y+1WlOXKpnfSwWnlPfuV9Aey3jqBFq7xctWZ04T7BOm27XXWUP5T9fLctZtV
  NmMpeaKrIk9BCo+stV0ECOteqFcx86D2xmmy89ll3Fe5LwM8Uw0vPiItmeKN
  wmNLXbHHk.c21X.iOPyouSqRv6Joz88W7uUO689Ta1mQ0SaCQSSe2O0l9Yz9
  bOcl9sZ1yvWyjEphz6f63qRwNa.t4tbONLE7WIMHZEzEn+WJifXYJ4snS1x3
  Y0QIgz5gFobIKOVzb0MsuUvhFDOmqkIkfSq7VzBKZtQ7hxzGolrNxskx5Htq
  gLwGLckbV1FMfJyVd.nM5tY2I+J6rMfDdsYeoQkn7SXGbUP7MMHr2jHfEU.T
  Zg46GYKjq9YKjXumuFMR24zVzcN1t8P5NKJjraEcmqaufty4SJ5NQqQ2QaKS
  OitSYey1Q2Y2Kn6DOjn6pJ+z9gKdM6kmcwqN+ee1yYu4sm+z+E60+H6M+xSd
  0qN6Ye0WUZhHUpkKkdGaKIrLSsY9lTpdPAy1rRR3ZRDHOJ2syR0oRFJsH7rZ
  SmrCdG6Z7+zje2rUcyPsSJ7pDNTXZ60xSazCKGUJ.v2GdYI7EjAu9zFXaV67
  2g29dYh9WR9ZtWl+Nx4uCQEzfWlNFso6Q5OiEsKAOQoaIknXxq2u5V8.5HZK
  GBIE5cga0FtRekThK2P2pc2nHyMcbznxMPhtRsFHc0MCSFSYWIl5ilX7A3mH
  +KKaee5u7LscAl9h3rUUJ8t3jqJLZYIRIW8OkhLJrf4Ho4DlhA9ttl1NXTrk
  OrkQiqXWONIHE996mktMwRzQQuSPRIucsVIWHjNKc1FXrWMAaYZgZLAq.A0d
  y9qFIq46QxZddZ6lIqobKXuRVKraAxZGudJYM+ACYcUtUPmuioAoylHWBVbj
  F1VWBniuDGrby21xkKlaMuvXSbL.LERRCYATy1dBOLv09tqLdZQNsQaFAP2j
  lzZXGCOB63NG6PtPIrVeSDzvYdAg6T6P6puIsPaV6ByD1KBSWHGUegopiG8d
  TXpkWaXi.umJL8ghjzZ7Whw416kixOHXEOwnJ2jcp0KYq8svOSR1momUEBiZ
  MwdFxs2TgU1Pod1Gz5zHV6UZDRtQ8zHb+9BQRkZrZahDBsrYDIBi1mHwbeRj
  3P6yb8DIBdegHQ3zMDIJzxFRj3z9DIh8JQhJ1j0Sj30SHRDJmE18DIDZYCIR
  7aehD9dkHwTRd3TasExTzWHRTABb2SjPnkMiHweGXShwdkHwvpYII9t8EhDS
  +NhHwvZKLbkuQ5apJNQXY3ncvhBW0lXJK0CTTh1vPBQnIaW6rMGzdShFjeah
  mvpDSKgmDTIwPgm72B7jobj7Lbys87qOdp8k5X3rOk536oJ2F05Iz92UHgKQ
  AzU9KqvKanXmcfuPF60fp3SUfqFHS1+NCkQlzUdLqvKaJYhU6SlrWiqhuPnA
  Yx92R2LxjtxmYEdYCISLMaexj8ZjU7nbjqIxDudBYRm40rBurgjIV6.oI60X
  q3oJUP0SlXY2WHS5J+lU3kMMV971mLYuFcEOpv9z.YhsUegLoq7bVgW1TxjV
  0ive7hsqXW27wVAjbZYK77.e6.eDryRS3MKAr41zS6PUYRKuMJArs38fTrvU
  lB4t9CDBZC6wiMmBSM+OaeRPaA4vfP992TCeZyLXeV78ACeGKh9us5jha0P0
  Md0eoS9ciarBtIb0uojlst360M21E5kjHV9xhIN2vc2bLPU934Yvq50nEI1F
  4o9z2i.To4ySgqrHkxJlhIkdTIp.e5Z.p4788QF1EoQlug8.rDwi+sMu4CJU
  oG.JYF4z.V1wLmfPtPsWiqGV1bOez6JrnTWt0T480oG5tpNJJx5qB6sgSSYu
  3r27lm7im8FMYYMVWEfXc2Udn3sn7qxo1Mk3SwRQr5jNKsUylN.fNdletXD+
  4hQ7mKFw0TeAK5.vRkrCa1+IbRB7Bxc0xNIl5cdM2jxJaUapoqpNlnNyRKat
  +5WriMsLxx76pGwJTcVQ9PzRU.xhlBtp4ffwf1K2XlJsMM0tqRhlWkDBU1zK
  lWy+K1sgZ8JGroOcBbUIO4p1XtNKiJSMaokw5XzdQxjPYKGC6SDwrqiRmxhh
  YWM65qACHBhig0ngXud.6ZYf7PlzLnJ4JM0Z81u84Jop9.mpKnpS+J3P4VvV
  xICY7cHqYIlS2MjszsKYK4VXWmqX6SyiAeiAvI.SE.MIuG3Nc.+8Y7sTHqY6
  y9ZQmEQpBinXey204Zc1WtkW9kaa+REXn6xsWWw99eGLNRwbdGxIWBS7Vt35
  zIZP8ZCVUJzQpJL1FqA0pSYUMWSVUO396erqkosc2xtZ5keI2Y6XWs6pk7rU
  yCZ+k.aZIvM+RvJlezpsJAKKJeqrH+d2F6cDlc1Jfhw5ftQmUWrBvIKOsc1p
  U.QWox55YiGqTMA5nd6O85yNSOuDr6DMQtVsgubTSNkyEKmhkq0ZhQWpJhzx
  rrln+93.GF.S6EKcVam.MdWwNM8tf47S+8IXIBS2sOXIbutMXI4XuNnaXX1o
  peL8soMrkdwF7sggg6uGz+z+3Wrsy0utZyfKp1n0siewsyCtnFdF0+7FhymG
  9otwanrXORKxbOmswaH04foKrEujE28m8FTs.yvHmvrca.iUcxHUulqBA15t
  nY2krlNRtv9zpGmrcSEE3Nb0ipvhbmsa0ypqT9MOB9roQ2c+XUOclZnyGz5p
  yn3x4RxAc4KeLp1z.8U+.pKNuSCymgLwo1AQcf6mGkXx24V8InsXgdgta2tb
  0YaxEs7vFkDNM9Pf1OM.3.deT5sxRAsr+kCyvDLES5c1DppruJOoHh9sylPt
  JTg0Nh5xF0c7QfxFalsirH6lqRNz4JaTwaKC+0AJaxrJI6MtUbc7NKReOAbz
  ZR3n1WuhR4qWtnJrc5UzY.0E8Zzkn2jqudGfdoCYNEPYKiV.8pw.pI5syBSv
  +8Sd94OaGfaUAMgpufVs.tUiATSbamsq0m+RI1k8HFb2.96F1KB9v4w2OK8M
  Q+mP1iWxq9tWaKgMKoZ5uw6MMsck7sRzhcWs9bwKe5Yu3hmc1txi.aQEEVgM
  E8V+.pI5sScBCS57NzGrrPC0dNg0vHp69jzkdgsjMisi0fld9KLcvd26+kNu
  OsMUoS2fQ9ND8SlVzcn+Zee5ptsS29INm4VS3z6V2d0OvFYhY5fM.d42Heah
  htSqpMov47Yo8+k81j2Dl+jcT8ZPEGkjMxNVGS4IGVHpnnFt95UrE9xgzjb.
  qhgTWk49KOwW5XvzhqN2GDGNdW3FFUIPncShqxoYmsyQrbCof5EJB2MDE69.
  .Gpr9lXkMnb3FsEeqsmmmqFYZZ4rgBIxO0Pq.W98ClDpNmTtFEXWWpVu65YY
  xsvpyN2x1v2SVm1El.GUgh0tB1RtOYx7ia1.S+1TKTzjfAWiG2w7SsnahS.L
  33nguaYrQYm8v5VPeDZjusigkEFt6A9xev.QKFXBeK2DaERv5L7stNa9prC7
  N7c8gWxi.tC7GtVujqBF9tal.ZEGUGAZc1jUZPwWb36PstUbpqjSC75kuNMM
  Y1jgYJCUGxDfNo.3OJbZZT7bX7WWHKcoa71nQiBiWFQNJZJdz0nCQmVTUqKL
  ilr1x.8cQitOIJNcZV7T7nzZUFVEaSOYgCQ9whu0c+bUrimpBC+bEil4SUmk
  kO1ASUqc8ppkbpZZZkeUU9wtdpZ2QSUGJC4rrFvEk8Y5TKmGUz4q5N6XTgkg
  p94TjW1ZOvK6uympTpcQ8B74qpFNc9ppfuqmppjsvkpoSyWVketqmr6ZYz1T
  YUP3IJNYketqmrlczj02tfBIe6tmHdWKkFyubICJufBI7ic8T0aWOUsoopoa
  ARX5yc7jcWSAiUrsEEg5EVO18KqNV6bqGUo+8dW4pmWG4S.ERv4SUNeMkKw9
  suLeMjQUFinhlRM0vn0p3dby3jqBFqJcGyi4UcU5iEkCj1ucI9JY+RTuB52h
  HRtVEtNKUIw2RUFh1759tHKo1DYGI40uruaalepURLGW2J7zr6tp71AHcke0
  92Zm5+WFZjB6ZIMN80nXg0XM+KCSSwOqjWlNX540QT.tlFdcz3zvIEln6kBH
  1xKK0TCwp8V6EkQLpi1qeuNwrEqZeBEaniTnaSk5v1tRGJ1O0Pe01oqZo.qd
  npKa6a2m0oVpmazAsZApkbrcsZAiVudXJr1qTIz9OWOUhmWegHY22nETDIaS
  iVvu0qL5By8JMBs+r0Si3X0WnQ18cYAEMx1zkE7Ze4Hh8IMhpWBz.MReQYSG
  zhET8+osoEK3HZcZD9dkFwwpYZD69htlNn+JnnQ1l9qfcq27JEF6UZD5nr1.
  MhSegFY22bETzHaSyUXyZAGU3P2J4J1VFQF8irhGmZtQVlZzv08a4PnTg2sr
  2.97GeySuMb36VDhkMpJ+ugMkDRTkDY3Wuqdd1eJV3rsUROyZrfTUuvy3ykN
  6OW5r+boydCyG3vwEKnqqY5XWZXdqQA3I5Tn5bHdAxfZyxKTc6kpYwToJ.13
  fI2DNgpdpNlrq9XplUd7sB4tkU+OU2NlRoWtn7zUuP1l1MYj9+j4HNnSvQ4a
  WNMfl3dD9wpIZOyNKu8O60u9hWuEo77ZkPqVTYaPc7UpH25KTRR5JDw8.GXJ
  6EO4e+HX1bSB.1OhLLKEk91QnGU8CWgk3J2GqC+zwRnt3ecP+PrC2QEoNxRd
  dSHJ2theJbL.iLi0V4mYkHJScT9osRPtxeSZekcDMg35LcfueBdVTGGBbb29
  UrmEjF7b4e+8fVvuw7TL3jOOZZp5NPRKdeW2nhc1jSkEGeilzMJ5HpzG+39i
  lQ0VpRE5T6lk34w6HjTJ6p4ccusWxuVlvpC9hqJNiJKILZDg41YpPwtUXSrw
  QKwEutGFWuckBE09BxIxQ+lIDcM5TcJ8MmoHST7TGlsFwVN1cD15aXl8Iy9o
  BhlpItPIkYcXotR31+Y7.fEranozytD04cl7SRYjRsnJitCUEChpL6QHKEYk
  g0h1LT8AxnUqWQx43ZhML1YgeXQwbxb9+VswCEVEyRWzEYcZ6ffnoQW4zHY.
  koJpVZfCrVGbvN4Le5pYlRKg0dxY9TaXt2.wNO.wxZCyVlO7f4dCDiGqHtFP
  rauBh0AGK5MPrtmwmRNIt6KPV7.Tfg3An.CwCNAFRCmzClcLdHB018nhKgmt
  GcY6GfvbeR1AFZLMgZWidFTy0FpM6IPswZnBm2OfYGcMTpL5+8GTqMunq4CQ
  n1oGI2yQWpZyGfvLQUyenA0RIMtOzf5Rj0r2fZaesg5djbOc8Qzok.4OcOG8
  Ukb+K1buaSWdy8Nk4rXW8lW4zOnrtbVWdn.nra12mNEZK2CzJtgdh1Lutk4S
  T9jqZyxo6ZRmcaOO5DMJi7qmQsot8lcnXppfJLILXzG2o0SASKA0.To83fu4
  0SASUaLjJYlkLR5bncscZ2SCPZxMXM0rjBp.ckeUzRETASpFvvWzSFpEOVIS
  V9c6r7yIueNrbIuGsvxtGrWqZBJbO2SixlvxqS6sxlPU7nmMYRxD1qwJexoG
  TFQRYoL01V9Nn5UokpaStRe9TetV77WfB2T8ZM6MidxtEknmxdzt4z47n5Uf
  opaUjTdAudo7l66i0lkqRfswN9XskgW3NaQsXvVzlZEkZ6OskNdfJVH5WN7k
  6f6qOmjc9iW85xIY4zhbRSVK6hLZSqAUlLQDMMXxjUqVNW.EngwivjnNMgA3
  6Q4reF60P9z48fkbMaDbgSX.W4pMjnArmM6t6wQAuoA0Zk8VKrWnGIZV1Msx
  Q+nUVxxHeyJDVdqXCuVzusJi9SgueJ6sQ2EByE1Q2M83tRAqsgIgMjJDU47w
  FpfU05DnQD6SzB6MPxfv+fVsRsgtM8jISB93ztCmZSBHkDZN1aONk63Hwo1l
  aHN0dm3SQaUj1bnxEJpQwdazw2jOENTdkhs0X6MVEuvoe3Sgq99Tvs6sNU77
  jj6wyBK65vvQXaFnqYRoNiWqvjJL2NlTwNgI0pkYREhtgIUXtMLol8ClTmOI
  XRey3j2OJ48wErLoUX+TU8HUIIxXaX+7jB04buMi6ayp0hUgwFEEbSbxzzng
  sK5xImzpsCcY4sUBq3Fsp8uW7xe37ersPUNjfCkISFaOtRFX.tpV0u1nJCuc
  hbcy1UttvwsSjqa4rEh0M5Ih0sePIVeFvyMqlZsbaSIQxzcrVMXHog2ox3e3
  RGbBtA0Y+gH6OrWboEWawEyc0bWN20qhv0rNB2SV5fxTJAL4XKomwQr7YuRq
  xvo+A8gp3ro9UwYyMf5MJMJX7pyqxtqryJoQt8OeOXfCF4kc0tpJxh2NcPzV
  MboENDZGzx9VcY7ylMgHIKKzdhczjFOGmT6XJWeksf6UEBlVaOq+G6pESNnx
  xcwp4pMl9hpqZ0551SlLZVTbB6GVxS4MZN5UWgtOuDwk++ZqLfBaUqsg5pm7
  5q9aB2OEq9aTs3wzQc7SMo57ts0mq9aet5u84p+1ZcJWuMRcPRecX.tSWe+r
  wu6fB8Q13jEbJUcW6xyJqiJKU7sz7vx5W0YkMOvuSO5vyQpOGncWsZcsDNs7
  aZ2d7ik1KgsMaMO+wUgRyA6cUUsJL8f9RYzS0cx3ZVoHr5HpN4VwwtXV58yR
  YWOI4tL6ZpgHTqmYWRSZSEDLSeMoIq7DwW8Loap8D3qb5Gu6pjwsFpaKKoUt
  BZaXLJuc3rZR01lXpBAYaGiJzpVLAtM4I8uvqhF4vJU3qBMa8xhpfQ2PYEkl
  DzZkntsFQpHqDpjGqwhZkPzUpH.eWvb04+CumnnvkZqLbpoY6zXQr0ryJQjb
  lfYxrX1LGlK1stx+e8DrWl3KJYhvhHYSDZ1+ckPKCUQkWnlIzDcUI1reI4x0
  1d.2wwwyETGpozK+OMKtutldpspzVup6qSWwasKy8xJy+x8MCrC014s8UEmu
  UN7S6zR8ZykZ4jQgcVYm12dfqmsqoyIYUhaG29RI3lPGWDOLrKQINfE7NVBO
  OSrUBSY0QyUD+tEm7zj3ztDmXiYWA2R3aALJt4xv1ZwIhNEmjk.dm1Y0jbap
  IhSrM9MawTmiN5LTg5LtosDjtpXdNAlBgRTw+3zEDHrmNaxjPfCJZ73nomJE
  3d5bdpSmKw4TVtXYdJqvgXQWTpeknT+FK1k08+MZjJl0gFFFl74mCAqlEi4z
  pAOq3FaVbkAsK.yTV1QCuE70eH3pO351wrIg2k7GvWKCoz0iSd+kwSBSmMIt
  vMZdL65jICACSRuMjcCrLeOZ.yUgnMLoL7rGD.1xb0rquFt8giCClDNpsJJ5
  aLWB2zZfiLAaU0xUuU1M4UVP75pEDooar6mDhNaAK.Wdvud4ADRm4yOFtkrs
  sUMJWFu5i7aKdDS7Q9dowiA+QPzXLJNrnX12KWStLdkm8Om+rbg4wmRG0Abo
  Drzrga2Ft8mhKxEdnJWusZb8thslrEEXZxykTelzgWzol.w5xWF9J4nVu6En
  93GqSDKJamaWGz5NrtSyAmuLkAljLsS06+pqAYH5EX90If2aO46V5fEUhBT6
  Yia48zmgIiojY.VTvavXfmuA22A+KC0ewqYYwoWrrnWLWzcAYGFNF0ZhJmFs
  b1EqIy6Bh620Dx.3Ywge3dX5GN5G.VGs2mssU10l62ohog5sQbmFKd4tteVv
  zZJXRXRmxYWucmfIOwmELsNBlTqIBWicmfIaidjfIknnmtpw36IiQ83feIpH
  0m0N9Zt84X2OrHkbw22DbRAcb.8FX2Kj2tRTs8Z4UeiVqZPJsoUCa6F0IH7+
  rNg0s+FYj2vnciNA+OqSX87RSslrK0I3a1KVS9Oio3W0dqK6zlQjplX3IYSr
  abO.76G1npZ9jbCPwVWv.ztclxrlZjlYVlnCUL2gc7H86pXZ2zipNia6nbHk
  hO54YQEslLGsl6bmFxZRPr9oEtvnJT5xyfNBCSaAvY2ce5GYOhIKOm3VE.Sw
  vn+HrNLt9O4NcEHKfkqwZ.ux0fFlScT95mjbO6eTWt5uxMrS2G2baUlsp9on
  ARVTY15SPeGhKycpOqDct78ra6V2FC7Ed1d9qEF0tVLpbBzMH0B6ObMH1Juu
  camh2afosK20asPtNUgbKNI5pFcmC1rS7zoUtziZabVO75zcZCxBQ+Bn0p4h
  Hr6W.sPKf1oeAz15.zlO7nN5ODzd51Wybr5WnYyGT8uEIHaoEKX+Blc6vdyQ
  GBx8m91h1fLFQf9DZ1QGXtG0g9z0bCqGdfbOxVCbWdzR1LuG0NRcevQZHzsY
  O42ezA5qcKTUzeZbi95RaH5Oh570sCY5Kd3Qb31ifYcaif8Gmt8zseq41eLp
  yU2d0ta+o0m6pK4bOxiaW8kO2eHns0sWM5I5WJu0wYPudlcc5DZCai9ELqSb
  u5QAXTaaQ6S17qsfi9i.ZAWS5Yecg46hFceRTbpZWH7nRisiphoaNuoXT7c1
  MSTwNbhhyMYyJjpwIpYp0xo2TWzsx0k0o+PFxKIpNUr77.DlaM6D9zsq8VQk
  ncZ3XFu9d05RIIrV8u0xaRuhZ6meMUPZwdQEdxQoCGhoSckiVW61sirVYyzH
  DGQ1OOMr1Jx7tt+2nZkDbJKQs1hF+E1n1jGO2JFHMpK5tV6jVj.us58MYHKO
  910DJaro2pPklaQOnz0neziDrVidjf0duGIzlsrTtuGk8e1Crs731taSeRtw
  9Vpmogrna6ws1hNkrwto4S41VLfzQ8tcPoMwE5QIKrGUcN1LDpe+nKwsFMIN
  2da6mZXRLn9CQ1LrTakbMaJflhugslszf0oKxqs1Oe5Dm5Qk8cuZazA9FciI
  Fkik5HDhGYBPFBwntNjfme6hPprSaarKPBat.lLQ1Vt4zyWgcodFcBN5d1yC
  G8l4Y14Fhn7ZwF0qRLrOgdTtxWo060QI8oXWlfLQzzU0qknSvgmwm6xDetKS
  74tLwldlBG.brOLNQgpC2FUGdLa9.N2ONH+AuiEU7+ZOrs2NqSJj6XDVhVnU
  Oy.8Bb82GkNUbd2TnDZEDLosmpF40he+74zThhry92JKBdUdTqvXJfmeMUAe
  6SWDl25gvL6hy0ZOCEw4qGNx5ui3Hq0BG4Z72Qbj65giD+MDGILVObzmZxiz
  nnVsdHHq+1gfbVODj89oRMrSKp5zbmaSMt9lwAtqCNXmbpccs0N6EW4.2r2x
  3Rq0Alc5Gvr45.y1Ozxr0UxLy8FLarNvr4Csiz8JIk1dClWK54dxYFbsnm6I
  Ibotfb+I2x4ZmD+8nbyqqOqDe5lZdMm5ShVJyKLENcWlW3SUX3sJyKr75G4+
  j3gTpWzL8DusnmLr6P5Ia2smdxsePOw+zhdxnsxLLplZ1QzSzttuczS8iLCa
  wBvmDzS9sVl9Z1gjSThNrcjS18BxI+Oonl7ZKpIKQGl1pTktb6nlL6ETSdOj
  nlp5XxvhhiR63juL6.ufEf1RaSSE69ltcTpEl0Czd5hlu4FhQLVWLhkmopIB
  KJu+aVf725uYYQnThAIvf6R6pfum2mShvOmDgeNIB+hMqiP9jW+re97WdA6o
  W7ry5Acolhoul8JVEToE.cP5UUGZ7a+5u9xXF6s2FAJRUMrzv3vIQCYSoSP4
  P.1.5L18SRtAzseBa3jPXodD1iSYfTDP8a7MxFZZpLLivey.F43QASFw.SAt
  MgNkDYi1c2MKNZnTZF75BRY.qQx6mx.VsaQPgO.q72iQFhSX+uyhF9tGM71f
  3avQVAKmvBlxde33wvuwGQLfczTXPNlkNIHdJVDwg233fI2.+Uvc3QVYdShe
  vkw3i7jT4bk3uwqkhXfouKDVYQDQ.6dfQNjtRH6EO4e+su3MuB.fQgDTeSRH
  .CQo2l+Iki8WW4OYWj8lyd84O44.s6KdwO+xye5Sd64W7xldRbQJjcWPTLiD
  hF8GX2KcR3oxK9H1qStZ1zT4xxcgSmF.r0rqBij4fVBSZUGCtF9AzXvSvO.F
  oELNZjZDBtJZbTprlqOEObKgSlTXvBF9NVBnlUsXl8TrmEEbSbBr3OjkD+sI
  WeM65YwxSRzIxAhHJFmjxtKYB1mUAX7NhB.GS037K2FFSKZKulgKHnJvSjSN
  b8PQLxhgwDf1CuB0ddHCLNF+HRlDEOKjMYVLRcNP9FfeNOkMBW2llvt5ixG9
  8AQRZVbfABQUOgEFifIS.D7IxgLJFHEB.x33weTVV5kHU.LJz6XwuPQkSs12
  Sxdso3LSATYOpbgbLV5vgYZzXByn5HsxWMRK8CR7Dr.eD+3KO37Wd9aQxlWe
  1Sd1kH2KNNWGMYJRKOIT83xWJO20vuEwjAweLasjcGRobUH6Pgs8gxmPnfe.
  LGAHG3Q.j9gXil9P16uMh3JlME46mMAtUf2O48wYy6r0CDMd3vfogSoQ0TdU
  fCAjE78YC5yfkzmGFeCvuefTvwzfONkcax6AjB.iWIQBuOBVMtNAkMbJK5Z1
  gFGxh.BnPfAGV4lCEDJitQ4q7IHyKXxDXbyIKncA9DX8883KJfAB7italjs+
  s+zqO6LZP.o+m98gfbF1QWd.fWt7fiYhSQL.y7zbvLQsleViDQJgAYTGXuDV
  BYzpKBpgDPj+saptIEVjHaFndCme8xKhQSkzrxEsSPrA7EihlNDjzFN5DRFL
  7Dwfv84O.BPmnfh.5Qwu9ZfAKisGvtKldr+Inpif2CQbEneG35e+sgRrMN74
  tWjYJ9vTV3GFhcKa7pXSI+7XUqb.U4BzToH21KB9v4w2OK8Mv2ADzBff96+4
  m+uVPM+DXsOXznHTtPfhS7jLB2wyw8zA6Ga543qhnSvIYN12xXFguD3pWFBU
  XFoHTfGK25DPhkikrrQ7zhz.HbLWvST5g3ZEv8NtVl7ArQylPhDPsufNu7c0
  cDSydLiKgNwwrm.uhr1FpZXtErzTMViXGApEkrPvuwmkPZGiO9ixFWBPkuIr
  QjCq8HIEh6vYSIC5UgWiPM1P.mhc4fDrgWmQeB5Rjuvr4TEBBKfo.yRlcG0b
  zmBz7gnxHlZsQR5lIjhD6HgWvfhIoJsYxEa0qQd+HBFkVJImOxD3ZYuKJdjT
  8Ax9NkzPZ7HeeIHd4AWDOLTdkKOPtJBBFCdWnbDy7g9vQIGB5IPCXvWZB7Dm
  vFG8Nk5V7miBNl8LDtgaXp7ZrofP..lQz4yO6Y.DDK4JQUhGwe7EWeMXU3i+
  EzixSXlO90giFvrd7OBxtAMk1O96GOKbwne0wRnGLp7Q9dRiy.sRnZev5Klz
  oPPni7hKdlgpmwmIuJNg9i.Xk.UQQOB6MfSVg+aPwV.JLOAEgCFkgywqlq9P
  NVixFKeRfwD1b6vjhAVnzGFVTIpR0igwi3vy.TLRlgjY.QIhqYGgeCNjGeJs
  tPVRIo1PBxGQfTHPx.tHJQbwRwJInpJzwOjtGc++ix07AKDtJeCHXJAQTX36
  A5mf3zBTdv.8tvv6AdnvgyRmy1gO6.koaeK96u8ae0qu3Ge8SdAXg1Ke6qu3
  4WBnnXTiIJTUQg77vQuBnBeLLiMYruCl4e62xd4EuU9FeYXxqh9P3XUSnOJ9
  q9Jhdc1cWQVmlmVCHUv6AEEixMtZbT76Pn61vw2iV9HI6ittvbALsYZAvh88
  yl9w4.EW.fz29sfquHaHN3vjGXJPKuCWnkRJYSNj.O2kwvXFcC3Eqz.Qjf4O
  hfkOp4a.Cpw2ImjJNRTp.5sKILIfP0pyDMAoziq3OKN1Cy2WOJcnwgPcWx2v
  xiPdARv.f8HGbQ.GEokj4QVvqfjahS5+sxBibF4kpb+AsAQN3ftUInfZ1yj.
  pzzBCAQi7ryexO9xKdyaO+or2dwEO+MWFKWGxwW.KCeGaoe.viDRDRRGfWxb
  gUiV7nWCD9R6swlUojGlvjjiSSWxtqkwMuALFRZd1iQ1QiU.BIX7rvwAejIC
  cL6n6ldLp7k31BC.YX3p1IjIuvK6vESqCIlMD8gFcgVGmgZQPRRwmIQYEOFj
  nHri4bdZ3DxE.ZwuHrgpClfOrT4YT18Nkv7y8eBIAvFEE3l3DkGoRxFb4JyE
  j.oKMJZx2ThWnYbQX2uMuQJHxy96lCQRa1+fzxMRuJNlQ3MSSYoVOx+wfgCC
  uOc5.1+Bk1DklocKyxu4.L5nEJ86tPvxfOpvN4sO4WyCP+FxQqZPYTewBTDR
  9Ggr2k73+2nycKPvvSe83faTBTQkAuG4TtdFPDJ8Cbt2V3PqFuWArwR06ysp
  K+3MmIegjsYwR.iLIHy+pxLKP8BpXvyQMjIFXidG3x5KAyDH7xnvOHeEKywf
  W8MnQF4jHkCgVzfLCbgPJjobAoff7gA.vf1TLE0WorBjzhRBXvE0oyEYnrgm
  lOKiB.LvhYNcKys5mcjDQfx.Uj3Es5EceEMvC8b6XvdbvLF4cIEbSVUgVXSz
  RI2q3yFvNWopXZNaffI46mjfh+liWdMfEyPH4skLOcDhYvfVDGhu+aFm2224
  b1mvNLa7NTpjdpR...cy8LDUCfA9YRXAVn4V+iuUEvIsbICvjeX4PG7HR11r
  6GIilkjSHCrkLC3WOfdVoEXnoiyMb7DkYQxPkjVP7nLLfO5eJu+LKdxVoea3
  c2W.zvHmlLI.rnA05gV8Jcgl76lLuYEfhnoy4FFQrhRF+5M9mUdZh16MryXu
  k8yrWonE08oWy28ej.Be.D4r6O5X1eh1QBlrfHgiJXs0IrK9429pe9sG+c4u
  kEF9T75jv9AWg9zeDmaKLLnKfZPnqAuRkIDGAWEtLdsKtmzabBo9G4WANorH
  DQB6SlGkl4uGY+Seb7QWdvuYb4AzPQbXjNVb.tEHQxn.OkYnrDA3FxjVUxn8
  SgfZ1A3HhWFTU..8QEchE7XOuhB5cedbDZ7Wzzkc4UFnBbp7eBm.NvOfQpIj
  pFjQZSpvx.on9qsmnZIZl4R0dA6IryYurV5pVgrBEzbD3D7epV5YuISrmTDz
  sf1qPThSDFSTzoiLauv6eY6TuS9GGQqEey2TvRF42Ul5xBeVdWKeGpE84wST
  8Bvg+6QuANpvKht3On.y2l7S.U0QyIsOiLAugoFcyOEsTZtQjYDeJJGjpQMr
  fn8iVROnhOkA5htIJMX7uLADlWfW7mN+G+I5wYj904QtAed0H+WYqJvOxN+Y
  1cgXlESJ7mWASSbbdJFRpo4tlbLdxjIAe7sIziuD+QQlCEg8NQdIrr.RKeIC
  7I.ntu.9q2TFE919tUj1zjEmqHtICujiTG4xu3ecBtDC51UQu9T0UK0vlJWM
  e9E+RFFuD6UTCoR2VlZtuK2aZYaR4pKl+eyYTGECtz7QK9vSovznL4UpO7DF
  XwsZ4ThRJiPinTyru3a9luaY4rG8Cfb1eUIXHKzhOWFhgBlNdJCDCe72UhX5
  h2GcKu+1HvXmiN5nU4twnrdL6+5+hcTQSS++kSs9wxaXk63wEolmyFVvsgBO
  yusP.yjErzrhfrBq7WYb6Eeq+yhuT1eRK1SBy78AL28ZLH3ETkUJR9rW+5Kd
  8oTnPkO7ToG93HPdSorbEsfF9xQiCGvd1r6PYX4P+KMzEjmV1c.KQzqmntRn
  Pdma3FdavD1H387qEFqeq3Xg3O40OBu0SXkrzdbdQdKyd0fDy47XrJbiqlkn
  G+37zNzJzBm5pFjv6ihL3gzkkCygRyANQ0DlQSWhCeetQRmovRBCX0Ly3pqg
  SqElROm1th0xmg9TfSlL+vNIy6XvA+6GGlFlmfQh29KX1hwnpHloLxT3lpQR
  vZHH3uVRbcAsqzLrN0q..mog4rODQ6vtbNK29igebHHiYwdDKGpiJZ0xiV16
  VT7S9vmQTKYaPvSkCIdoQrKnf8WOqrZTHykgGJGNeoGqvK86pbjkzayCKsje
  a.L4oM93zZF+bL.0L5ul5m30MRvBYQ1qik1TmlKMIja8.QKHCi86SlMF2sD0
  NtMMg1bZbWhGlNCi..5NcZlKwvyMPs4xvehySouBSAmBfGc4.V7OHwV3ydbF
  7pCKXcRgpSFCvnvTbJfnxZnLvaCYUlSZNYNUIYiIRO9U0PaqHtgY76mGMkbD
  04npKZy7+jINlw9SJxbp8ryDSogh6Z22mSYDMLEzDhRME11GiCzSP6ika1Jr
  BCe4IK5.8jabJva9NjTRPiJ3d.8SEFKQ+jKd.E0PS9nJ2Ld4N49XozzSnXiO
  ZdXChF+wbuo7QD.o1KLjzX91BaE+hm3zLRLX1lsY5KFYDX9oKd8aYu3r27lm
  7ims3JRwM4ewv76XI5jR3DyGIeWYaoGFxkEOLqT6EyOzkpKnR4Qm+xe3BZWY
  MkYEbT11HS6wzUgpH1Mf8jqAeonEY78tjjfRdAYPXs2jT5xhMdswQEjxTlDq
  4JqT+YlScDw4UD4.lfBEVgd6EW.b9u7GadMZtsmE9txsprYgH0qa.9BUB.kk
  TQEnRYqhhZTfd4n8qxrqC0oSosyRAyugW0x9HV4RF95laRJkSYqNzxUIo4jK
  lB+VYi5xVTleFOeGwjBUkYHB4aDfFw6Na+Oyz+BR+xgcuZteT4137RHqpjJX
  II.aNwUI9rH2.J0c.OjgbNgAlBiLbzcQo0S9wKfJ0JBDzOksS.EeQyCo8FJK
  Zt0oR6yteRnLCZldJ6lDZSJvceIylU4VosBwSYL+xcMPIFM6xyu3g4RRnGuj
  rgvwSCI78cz1pB9UYlkcMxDladRFEISlG3oGvNmRfk2K2Di376ufx8roIqjC
  FmjOExjvhj3ELIJDi.JEO8yj4T37s2DPFXLQWjTEru4aJpwLGloZwLUljTj3
  +4i3JhjWVbbgW8habt0PGlSHLftjXqCWxhoC+mB58dXlERpXxc4AqXkykG.N
  HnrgZE2Cl63vh3.hQ9G+zwxsBD+p44ivYOSJiE2sjb66cVJPxdq7f7LkcQ72
  dw0WmKMJTgKBIwwQl8+ErrB+hxBu2RQieA+kjjEI0z7ImaiJ7f+kBAIy6gmG
  NhxkG4De4XblCVWMPqe0RQZcNaZA5lKO32xhk9JK+kDFAbw4hEaTzRQtkhru
  hHVkhF.S1sASIqMBTIjxBagKQJRIj0jIwm8K.d5hWUYXO.Htj3BW5.Vviv5F
  vB2XMC3R19V2Ptr2mUOnY4sPciV18Tyvrj0+0MZMF3HZHqSLhVhPJAJyLort
  AaYyNKYXxaCeMP0JFRUwZ57vKjkXAYapcSqv48uqwWAdms2.hpraZzv6olgo
  oEh7KBivDmYI5v+Z9VJvdE9HE1MhSYOIet8HGSYREBZUkIeQ.d2SyRSD0N3C
  N3kpDCVbuMTYgF7U+5uchLsUdoRVxJ6c3eksQjO84m8jW+F5Egqpn3T7IvHs
  HUpDQACLh8+i8RF76u4axqDXIIn+IqRQnDbEA.1yN6oKvMRkfu7YkfbHo8.l
  6qy9qKiozvCSHuuM+oVoXCHw2q2brenSAL2xyiJafF0TXks1fBWcokKuOcK6
  NU0vUxxWRRZc6bleqrd7XHaEY91CLDBGKK7De2vQJuliGK20S1JRr3pJawF0
  8nlWStaqCRrLqhJ6.5SW4WqqZzzhHYWY0HWcn4DUheqrIEhkx+FafWbgkZIv
  ph2lVMvKsZ.J6iyz+xKY0bl9q8V6CshpKh0itSvG3xEN1hMonQvcx2YFcF3Y
  46C5TXl7MhCWXKIun5ZB0uH3qKwUG0aL4FYm3ISos.f4QfuLeC6eKM5ScL.t
  Ll9dv0b7l4mRgU.+aAk+9zGLOkkcvnvOZwdD4G9QKBD52Xd7oR6TKsSh41s0
  tAWtrISZLv0SXIqssb2AbpZNTQ+yx0qi5eV6f98Je2z5LUrM02Ssrc+aYeGC
  qke5iir6DbjraB2DdZSnuLKE2Y1T+DVu9JbFwlgqRFccHRq1EQ1T0KpkJcQJ
  sPjIPptMlS6W0hlqdxYyqZQl98gpVjtkrndZ8JZBKyw2cQE5YyZAf1xlsDWZ
  5hCu1tjXGUIivXe.hrPHbqvRlqmiDCGGFLQ61cNmb22s1Zcjo4eeq0QeteI9
  4RczmK0QaaKlKkBX7duCyQPQi8BLxdFUK3ow1LmIuWzF+RYJQ+6YbLAEM1Dn
  Dt4QxlMhjM5G8kxIg2i6ONn+B90CkNlnfpti9dM1wD6GXY7HU9fA2RUobgaS
  3V+dApcBLOCk1l9fp605JQxNhlPxd8JoDsKZdWQGyMIQwDN1swNDrymjcU0V
  tkpJb9DqgyUXyopjThJlvZijb+6VW4SwqoMBx6SutxmmIEEOtsl3.+O8vApn
  AqMNvTr26Lg9514qL5MsQJOMAY+dCD6nIDa1e5UWl7GfvrtckPQ+gzPnK0rS
  +Ajc0kzf+.Dl6Ox4D5J1vq+zg+5bNvOcS0vYfw.yJaSdKbAM2i2nzv6TVLb.
  VKOSSRRu8QmqNHyO5UROZYGbB9OyldEtiJ2ws37CpX2dMqa2dOYof.T5NU44
  kempTALrpspZO2A1xiwqY2cq715CMiFJysCXT0lZUxGXA5pfCzOGTDMQAnyd
  9qZ1V9hZ2mxVb+a+eAj9jDVoXf0Y2aEq2dbqQJfRHDGZeaqcmtEcxNcONIXD
  kJLcX9K0LdxyMeBL4TqTiVNmjUGwwUyaBrDkzojS0hhbTBVcj4Ngc8XH2tBE
  cMfkts+fjj3F8PQdcCJxyzxjanoZ81DUHrobFwzsprOuXn8Zw4r87.fzkSXt
  m+ZLgsZyIL2QXsOlwpCXfVyXmMRe6cQSBFbMZTS1rI5l3D.DFGM7cKoIX0TE
  qhTPjKb3Cr888ccPQ9Ted0wZ.GSGdALeLgYB2k6Muy2CVde0Bdzwi+8me1y9
  8mlLNArUagPZ8Scd.BF3YXXX4iuM2AfHCG7u4l7Al.T3aI6nbVHPXNGHvyE7
  MSv9XQU4YY9uG.4jIinjrJaUeHBxDLXLvGLDGw.FCb7M39x+B9kgqG9Wl3Ww
  Ku89YuIKkncyyhVSGLvGhLzLHcFcleOnUof8jJ7scsy2FqpTlsQ+gFFLt0En
  f3B7z3IMz012EVSEdtbfbFL7CKD5FNq+A6vh6Oens4jMif1eWj6vCOgXvWgz
  n7MjvLuM3EDCJGuhQglvuKGu.EVd03DHiGyhorNwhY0.DrZvAVYAe8.H2F.H
  AFqSdGBP1M.PNcLBxnA3AOFfcDBx2WC3wzqA34tnQ2m.drpnicnLMlaPIIJ0
  wPkeZwKp8lA5fQs76rkXeGMfG2NDdr0YEVzcvikFvimWmAOK8p16zOdbMfGG
  qtCdLz.d7c5WvikWOCdb6YvSea8xtmAOlcG7ni34dEzv85tUKWczdY2D20R1
  mH3Boi79T174RACs3m3VTme2vPdqxOtCrdQq4mrAy2UHbcXOr6N0OtEL8rB1
  Cj+PzIfiiU+xZAGgNVKzcd.5v00eBq0veBA4UNcZSb4dy+zNfizwnesBaqiG
  Z1cm8N18L6ks0w+utS9osSOC8X2uPOVsg97VFdZR+hc2YuisNxOs6Nq2sMzx
  9jNKdUVd5FcE6tAdb0Edr5F3Q6nOY1MvistvS2X+1RN1UC7zQzyBMj+zcwKu
  mYcqoN193Zudd+wMskd34PksGkuf3m1AVSp0Ln6B9poVa+PG5ufoYux7DSSc
  b2r6.GQOieTWmw6NvoQhYQ2Q8HzhYu6bUSzybUSniwIBd2AO7dF9QKis6P3o
  WEZZtuNKWbWdGBQZsf430gPT+hhl64qU9NzgXHs1u2ko01sPjVFk42gowimi
  VaCTWRGo09j3J5PHRnENxXccNwQdF3scboBOuu87OtCbOgqU1Fv85R9SsfH4
  9LX2QPjNR46vsni2u1xXtqWOC+nE.w6vbng6pkHdO29FDIYyL6HHRKQ7dcXx
  ZpGDIwQVcDDomIvdcV7T4t8KYQNdZif5nLPVqLFXYqI1sPjVDQxzFuqvQ5sK
  ScnM4Zsq67NLOX318rXyys0J7aVh0K2wUF9ZRIMtCUv0jeZWX9qsVFwa2gBv
  zZCf4cX9Rx0QIXGxGn0NJ1k7AVZIf2rCcgRqM4ja26fn0VVAUyVMUmhckrBK
  mcirBydVzz4lVZqGuqfHsVl6vCaBuusCZjQLhFMGzp6L9xjqwt5w8L6XHpYi
  386aPTW5IndqZco+65QY2kAtyrmw6K2J5lvPNlqmRPPVAUGFnSEMVnhK6iTQ
  Uk63W5GcUezdwG2EpPkH.ql1.X20CAHLUUdCUUF22tzORSQgup.qtSmhlMME
  sVyonA0cQMoluhvhNiJK+wrFUiuZJZtCmhMJ5YcOTNfK24milby7ywrOZwoh
  bkm2frOsSlhN5dLFL0eF5ZHsSk6uqH8zxdvNL+J3BScQih03zfXyo95oz3eG
  eKUqYdGgT48MkHZEqrNLKQ38tjNfqePfWGATVftCTnCUc4.uU8oJoSwOREuZ
  toiEUKy2QabKWGIT9c3FJYnMutwZyqSJ1bHU36J0ZF53pnWG5MsgVNcXzkPT
  O6rwo0gMbScbkJOOK0K9PHYodv2R8euU68dU228Vtm6Au4+5K++mZJlqA
  -----------end_max5_patcher-----------
  </code></pre>



*/


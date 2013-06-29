boolean useCompositePPM;
boolean doubleSweep;
int numChannels;

int countTo;
unsigned long minMicros;
unsigned long minMicrosCorr;
unsigned long maxMicrosCorr;
int shiftSweepAmount;

void setup() {
  Serial.begin(9600);     // opens serial port
  
  char lastByte = 0x0;
  char incomingByte;
  while (true)
    if (Serial.available() > 0) {
      if ((incomingByte = Serial.read()) == '\n') {
        Serial.println(lastByte);
        break;
      }
      lastByte = incomingByte;
    }
  
   useCompositePPM = lastByte & 1;
   doubleSweep = lastByte & 2;
   numChannels = (lastByte >> 2) + 1;
   
   if (useCompositePPM)
     useCompositePPM = false; // useCompositePPM = true: Not implemented yet
   // if overboost: 890 lowest to 2110 highest => use doubleSweep = true
   unsigned long halfSweep;
   if (doubleSweep) {
     halfSweep = 1024;
     shiftSweepAmount = 3;
   } else {
     halfSweep = 512;
     shiftSweepAmount = 2;
   }
   unsigned long maxMicros, shCorrection;
   minMicros = 1500 - halfSweep;
   maxMicros = 1500 + halfSweep;
   shCorrection = 1 << shiftSweepAmount;
   minMicrosCorr = minMicros + (shCorrection-1);
   maxMicrosCorr = maxMicros - shCorrection;
   if (numChannels > 8)
     numChannels = 8;
   countTo = 2 + numChannels;
  
  int pinId;
  for (pinId = 3; pinId < countTo; pinId++)
    pinMode(pinId, INPUT);
  
  while (digitalRead(2) == HIGH); // Begin capturing after falling edge of ch0
}

unsigned long times[10]; // 2 + 8: 2 is first pin on channel 0, max 8 channels with PPM with maxpulse of 2ms: 4ms sync time

void loop() {
  // Capture channels
  int pinId;
  //times[2] = pulseIn(2, HIGH);
  for (pinId = 2; pinId < countTo; pinId++) {
    // don't wait to first become LOW like in pulseIn
    unsigned long startTime;
    while (digitalRead(pinId) == LOW);
    startTime = micros();
    while (digitalRead(pinId) == HIGH);
    times[pinId] = micros() - startTime;
  }
  
  // unsigned long strTime = micros(); // TODO: remove (debug)
  
  // Processing and writing to serial
  String toSend = "";
  for (pinId = 2; pinId < countTo; pinId++) {
    char valueFormatted;
    unsigned long value = times[pinId];
    if (value <= minMicrosCorr)
      valueFormatted = 1;
    else if (value >= maxMicrosCorr)
      valueFormatted = 254;
    else
      valueFormatted = (value - minMicros) >> shiftSweepAmount; // Move to minimum, divide by 2^shiftSweepAmount
    toSend += valueFormatted;
  }
  toSend += "\xff"; // A max byte as message separator
  Serial.print(toSend);

  // Serial.print(micros()-strTime); // Should be less than 4ms * (numChannels/8) TODO: remove (debug)

// Show pulse HIGH times TODO: remove (debug)
//   for (pinId = 2; pinId < countTo; pinId++)
//     Serial.print(times[pinId]);
}


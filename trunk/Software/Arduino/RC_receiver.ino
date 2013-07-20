boolean useCompositePPM;
boolean doubleSweep;
int numChannels;


// Setup //

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
        useCompositePPM = lastByte & 1;
        doubleSweep = lastByte & 2;
        numChannels = (lastByte >> 2);
        if (numChannels == 0)
          numChannels = numChannels_autodetect();
        
        lastByte = (numChannels << 2) + (doubleSweep << 1) + useCompositePPM;
        Serial.println(lastByte);
        break;
      }
      lastByte = incomingByte;
    }
  
   //useCompositePPM = lastByte & 1;
   //doubleSweep = lastByte & 2;
   //numChannels = (lastByte >> 2);
   
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
   else if (numChannels == 0)
     numChannels = 1;
   countTo = 2 + numChannels;
  
  int pinId;
  for (pinId = 2; pinId < 2 + 8; pinId++) // For autodetection and to be safe if the user enters a too low numChannels value
    pinMode(pinId, INPUT);
  
  while (digitalRead(2) == HIGH); // Begin capturing after falling edge of ch0
}


// Autodetection at setup (non-PPM) //

int numIterations = 6;
int votes[9]; // prediction of numChannels results, max 8 channels including zero channel amount
unsigned long times[10]; // 2 + 8: 2 is first pin on channel 0, max 8 channels with PPM with maxpulse of 2ms: 4ms sync time

int numChannels_autodetect() {
  int chId;
  for (chId = 0; chId < 8; chId++) { // init
    votes[chId] = 0;
    times[2+chId] = 0;
  }
  
  int i;
  for (i = 0; i < numIterations; i++) {
    while (digitalRead(2) == HIGH); // ensure same starting conditions
    int prediction;
    prediction = numChannels_autodetect_iteration();
    if ((0 < prediction) && (prediction <= 8))
      votes[prediction]++;
  }
  
  int numChs;
  for (numChs = 0; numChs <= 8; numChs++)
    if (votes[numChs] > numIterations/2)
      return numChs;
  
  return 0;
}

int numChannels_autodetect_iteration() {
  int maxPrediction = 8;
  
  // Capture channels
  int pinId;
  for (pinId = 2; pinId < 10; pinId++) {
    unsigned long startTime, waitTime;
    waitTime = micros();
    while (digitalRead(pinId) == LOW) {
      if (micros() - waitTime > 22000) { // 22ms, exceeding total PPM period, so can't be a valid channel
        maxPrediction = pinId - 2;
        goto verifyMicros;
      }
    }
    startTime = micros();
    while (digitalRead(pinId) == HIGH) {
      if (micros() - startTime > 2524) { // exceeding maxMicros, even in case of doubleSweep: 2524us, or exceeding total PPM period, so can't be a valid channel
        maxPrediction = pinId - 2;
        goto verifyMicros;
      }
    }
    times[pinId] = micros() - startTime;
  }
  
  // Verify all channels past the test for now
  verifyMicros:
    int chId;
    for (chId = 0; chId < maxPrediction; chId++)
      if (times[2+chId] < 476) // lower than minMicros, even in case of doubleSweep: 476us
        return chId;
  
  return maxPrediction;
}


// Processing (non-PPM) //

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


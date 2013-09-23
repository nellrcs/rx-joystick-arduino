boolean useCompositePPM;
boolean doubleSweep;
int numChannels;


// Setup //

int countTo;
unsigned long minMicros;
unsigned long minMicrosCorr;
unsigned long maxMicrosCorr;
int shiftSweepAmount;

int numChannelsDetected;
unsigned long periodDetected;
void (*capture)();

union u_tag {
  unsigned long ulval;
  char c[4];
} u;

void setup() {
  Serial.begin(9600);     // opens serial port
  while (!Serial); // wait for serial port to connect. Needed for Leonardo only
  
  char lastByte = 0x0;
  char incomingByte;
  while (true)
    if (Serial.available() > 0) {
      if ((incomingByte = Serial.read()) == '\xff') {
        numChannels = (lastByte >> 3);
        useCompositePPM = (lastByte & (1 << 2)) >> 2;
        boolean modeAutodetect = (lastByte & (1 << 1)) >> 1;
        doubleSweep = lastByte & 1;
        u.ulval = 0;
        if (modeAutodetect || numChannels == 0) {
          int useCompositePPMDetected = mode_autodetection();
          if (modeAutodetect && useCompositePPMDetected > -1) {
            useCompositePPM = useCompositePPMDetected;
            modeAutodetect = false;
          }
          if (numChannels == 0)
            numChannels = numChannelsDetected;
          u.ulval = periodDetected;
        }
        lastByte = (numChannels << 3) + (useCompositePPM << 2) + (modeAutodetect << 1) + doubleSweep;
        
        String toSend = "";
        toSend += lastByte;
        int i;
        for (i = 0; i < 4; i++)
          toSend += u.c[i];
        Serial.println(toSend);
        break;
      }
      lastByte = incomingByte;
    }
  
  //useCompositePPM = lastByte & 1;
  //doubleSweep = lastByte & 2;
  //numChannels = (lastByte >> 2);
   
  if (useCompositePPM)
    capture = capture_ppm;
  else 
    capture = capture_pwm;
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
}


// Autodetection at setup //

int (*numChannels_autodetect_iteration)();
int numIterations = 6;
int votes[9]; // prediction of numChannels results, max 8 channels including zero channel amount
unsigned long times[10]; // 2 + 8: 2 is first pin on channel 0, max 8 channels with PPM with maxpulse of 2ms: 4ms sync time
unsigned long accumulatedPeriods[8];

// Autodetection of period //

unsigned long period_pwm_autodetection() {
  unsigned long waitTime = micros();
  while (digitalRead(2) == LOW)
    if (micros() - waitTime > 30000) // 30ms: a value exceeding any PPM period
      return 0;
  unsigned long startTime = micros();
  while (digitalRead(2) == HIGH)
    if (micros() - startTime > 2524) // exceeding maxMicros
      return 0;
  while (digitalRead(2) == LOW)
    if (micros() - startTime > 30000) // 30ms: a value exceeding any PPM period
      return 0;
  return micros() - startTime;
}

unsigned long period_ppm_waitForStart() {
  unsigned long beginTime, waitTime, startTime;
  beginTime = micros();
  while (true) {
    waitTime = micros();
    while (digitalRead(8) == HIGH)
      if (micros() - waitTime > 30000) // 30ms: a value exceeding any PPM period
        return 0;
    while (digitalRead(8) == LOW)
      if (micros() - waitTime > 30000) // 30ms: a value exceeding any PPM period
        return 0;
    startTime = micros();
    if (startTime - waitTime >= 4000) // 4ms: minimal length of sync pulse
      return startTime;
    else if (startTime - beginTime > 30000) // 30ms: a value exceeding any PPM period
      return 0;
  }
}

// Autodetection of useCompositePPM //

int mode_autodetection() {
   // On success, save found parameters in "numChannelsDetected" and "periodDetected"
  
  numChannels_autodetect_iteration = numChannels_pwm_autodetect_iteration;
  numChannelsDetected = numChannels_autodetect();
  if (numChannelsDetected > 0) {
    periodDetected = period_pwm_autodetection();
    return 0;
  }
  
  numChannels_autodetect_iteration = numChannels_ppm_autodetect_iteration;
  numChannelsDetected = numChannels_autodetect();
  periodDetected = 0;
  if (numChannelsDetected > 0) {
    periodDetected = accumulatedPeriods[numChannelsDetected-1] / votes[numChannelsDetected];
    return 1;
  }
  
  return -1;
}

// Autodetection of numChannels //

int numChannels_autodetect() {
  int chId;
  for (chId = 0; chId < 8; chId++) { // init
    votes[chId] = 0;
    times[2+chId] = 0;
    accumulatedPeriods[chId] = 0;
  }
  
  int i;
  for (i = 0; i < numIterations; i++) {
    int prediction = numChannels_autodetect_iteration();
    if ((0 < prediction) && (prediction <= 8))
      votes[prediction]++;
  }
  
  int numChs;
  for (numChs = 0; numChs <= 8; numChs++)
    if (votes[numChs] > numIterations/2)
      return numChs;
  
  return 0;
}

int numChannels_pwm_autodetect_iteration() {
  int maxPrediction = 8;
  
  // Capture channels
  while (digitalRead(2) == HIGH); // ensure same starting conditions
  int pinId;
  for (pinId = 2; pinId < 10; pinId++) {
    unsigned long startTime, waitTime;
    waitTime = micros();
    while (digitalRead(pinId) == LOW) {
      if (micros() - waitTime > 30000) { // 30ms, exceeding total PPM period, so can't be a valid channel
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

int numChannels_ppm_autodetect_iteration() {
  int maxPrediction = 8;
  
  // Calculate period
  unsigned long startPulse = period_ppm_waitForStart();
  unsigned long stopPulse = period_ppm_waitForStart();
  if ((startPulse == 0) || (stopPulse == 0))
    return 0;
  unsigned long period = stopPulse - startPulse;
  
  // Capture channels
  unsigned long startTime, stopTime, waitTime;
  startTime = stopPulse;
  int pinId;
  for (pinId = 2; pinId < 10; pinId++) {
    while (digitalRead(8) == HIGH) {
      if (micros() - startTime > 2524) { // exceeding maxMicros, even in case of doubleSweep: 2524us, or exceeding PPM sync pulse period, so can't be a valid channel
        maxPrediction = pinId - 2;
        goto verifyMicros;
      }
    }
    while (digitalRead(8) == LOW) {
      if (micros() - startTime > 2524) { // exceeding maxMicros, even in case of doubleSweep: 2524us, or exceeding PPM sync pulse period, so can't be a valid channel
        maxPrediction = pinId - 2;
        goto verifyMicros;
      }
    }
    stopTime = micros();
    times[pinId] = stopTime - startTime;
    startTime = stopTime;
  }
  
  // Verify all channels pass the test for now
  verifyMicros:
    int chId;
    for (chId = 0; chId < maxPrediction; chId++)
      if (times[2+chId] < 476) // lower than minMicros, even in case of doubleSweep: 476us
        return chId;
  
  // Save PPM period
  if ((0 < maxPrediction) && (maxPrediction <= 8))
    accumulatedPeriods[maxPrediction-1] += period;

  return maxPrediction;
}


// Processing //

void sendToComputer() {
  // Processing and writing to serial
  String toSend = "";
  int pinId;
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
}

void capture_pwm() {
  // Ensure starting conditions
  while (digitalRead(2) == HIGH); // Begin capturing after falling edge of ch0
  
  // Capture channels
  while (true) {
    int pinId;
    for (pinId = 2; pinId < countTo; pinId++) {
      unsigned long startTime;
      while (digitalRead(pinId) == LOW);
      startTime = micros();
      while (digitalRead(pinId) == HIGH);
      times[pinId] = micros() - startTime;
    }
    sendToComputer();
  }
}

void capture_ppm() {
  unsigned long lastTime, currentTime;
  lastTime = micros();
  
  while (true) {
    // Ensure starting conditions
    while (true) {
      while (digitalRead(8) == HIGH);
      while (digitalRead(8) == LOW);
      currentTime = micros();
      if (currentTime - lastTime >= 4000) { // 4ms: minimal length of sync pulse
        lastTime = currentTime;
        break;
      }
      lastTime = currentTime;
    }
    
    // Capture channels
    int pinId;
    for (pinId = 2; pinId < countTo; pinId++) {
      while (digitalRead(8) == HIGH);
      while (digitalRead(8) == LOW);
      currentTime = micros();
      times[pinId] = currentTime - lastTime;
      lastTime = currentTime;
    }
    sendToComputer();
  }
}

void loop() {
  capture(); // Apparently you can't overwrite the 'loop' function, so for minimal amount of function calls, do it this way
}


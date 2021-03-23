void setup() {
    Serial.begin(9600);
  kf.getABC(A, B, C);
  kf.getQR(Q, R);
  ctrlr.getGains(kp, ki, kd);
  ctrlr.getControlConstraints(minControl, maxControl);
  mpu.setupSensor();
}

void loop() {
  fromKeyboard();
  if (started == 1) {
      Serial.println("   started");
    float *p;
    p = mpu.readSensor(sensorReturn);
    float rolldGyro = *(p);
    float pitchdGyro = *(p + 1);
    float yawdGyro = *(p + 2);
    float rollAcc = *(p + 3);
    float pitchAcc = *(p + 4);
    float xyaw = *(p + 5);
    kf.kalman(xroll, Proll, rolldGyro, rollAcc);
    kf.kalman(xpitch, Ppitch, pitchdGyro, pitchAcc);

    // error calculations
    float states[3] = {xroll[0], xpitch[0], xyaw};
    ctrlr.errosCalc(references, states);
    ctrlr.pidAll();

    // motor mix
    ctrlr.motorMix(throttle);
    ctrlr.toMotors(m1, m2, m3, m4);
  }
  else {
      Serial.println("  not started");
    // reset
    ctrlr.reset(throttle);
    throttle = 1011;
  }
}

void fromKeyboard() {

  if (Serial.available()) {
    int data = Serial.read();
    switch (data) {
      case 48 : throttle += 5;           // 0
        break;
      case 49 : throttle -= 5; // 1
        break;
      case 50 : references[ROLL] += 1;   // 2
        break;
      case 51 : references[ROLL] -= 1;  // 3
        break;
      case 52 : started = started * (-1);      // 4
        break;
    }
  }

}

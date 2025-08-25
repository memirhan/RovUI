int sensor1, sensor2, sensor3;

void setup() {
  Serial.begin(9600);
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);

}

int roundToNearestStep(int value) {
  const int levels[5] = {0, 256, 512, 768, 1023};  
  int closest = levels[0];
  for (int i = 1; i < 5; i++) {
    if (abs(value - levels[i]) < abs(value - closest)) {
      closest = levels[i];
    }
  }
  return closest;
}

void readSensors(int &sensor1, int &sensor2, int &sensor3) {
  long sensor1Sum = 0, sensor2Sum = 0, sensor3Sum = 0;
  int sampleCount = 10;

  for (int i = 0; i < sampleCount; i++) {
    sensor1Sum += analogRead(A0);
    sensor2Sum += analogRead(A1);
    sensor3Sum += analogRead(A2);
    delay(10);
  }

  sensor1 = sensor1Sum / sampleCount;
  sensor2 = sensor2Sum / sampleCount;
  sensor3 = sensor3Sum / sampleCount;
}

void scaleAndRoundSensors(int &sensor1, int &sensor2, int &sensor3) {
  int minVal = 433;
  int maxVal = 600;
  sensor1 = map(sensor1, minVal, maxVal, 0, 1023);
  sensor2 = map(sensor2, minVal, maxVal, 0, 1023);
  sensor3 = map(sensor3, minVal, maxVal, 0, 1023);

  sensor1 = roundToNearestStep(sensor1);
  sensor2 = roundToNearestStep(sensor2);
  sensor3 = roundToNearestStep(sensor3);
}

void loop() {
  readSensors(sensor1, sensor2, sensor3);
  scaleAndRoundSensors(sensor1, sensor2, sensor3);
  int button_1=digitalRead(8);
  int button_2=digitalRead(9);
  int button_3=digitalRead(10);
  int button_4=digitalRead(11);



 Serial.print(sensor1); //ileri geri
 Serial.print(",");
 Serial.print(sensor2);// sağ sol
 Serial.print(",");
 Serial.print(sensor3); // yukarı asagi
 Serial.print(",");
 Serial.print(button_1); //kıskaç kol
 Serial.print(",");
 Serial.print(button_2);//
 Serial.print(",");
 Serial.print(button_3);//
 Serial.print(",");
 Serial.println(button_4);//lümen fener

  delay(100);
}

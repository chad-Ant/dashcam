void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Serial started.");

  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  Serial1.begin(115200);
  while(!Serial1);
  Serial.println("GPS module started.");
}

void loop() {
  // put your main code here, to run repeatedly:
  int len = Serial1.available();
  while(len > 0) // Do blocks of bytes for efficiency
  {
    uint8_t buffer[128];
    int xferlen = len;
    if (len > sizeof(buffer)) xferlen = sizeof(buffer);
    Serial1.readBytes(buffer, xferlen);
    Serial.write(buffer, xferlen);
    len -=xferlen;
  }
  
  int lenWrite = Serial.available();
  while(lenWrite > 0) // Do blocks of bytes for efficiency
  {
    uint8_t bufferWrite[256];
    int sendlen = lenWrite;
    if (lenWrite > sizeof(bufferWrite)) sendlen = sizeof(bufferWrite);
    Serial.readBytes(bufferWrite, sendlen);
    Serial1.write(bufferWrite, sendlen);
    lenWrite -=sendlen;
  }
}
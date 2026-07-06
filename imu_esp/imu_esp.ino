#include <SPI.h>
#include <Wire.h>
#include <JY901.h>
#include <ESP32-TWAI-CAN.hpp>


#define WT_TX 22
#define WT_RX 23 
#define CAN_TX 18
#define CAN_RX 19
#define axis17 0x011       
#define sendIMUData 0x01F 

const int samplingTime = 25;
unsigned char stmp1[8] = { 0 };
unsigned char stmp2[8] = { 0 };
unsigned char stmp[8] = { 0 };
float xAngle = 0.0;
float yAngle = 0.0;
float zAngle = 0.0;
uint32_t last_time = 0;

HardwareSerial WitMotionSerial(2);

void setup() {
  Serial.begin(115200);

  // WitMotionSerial.begin(9600, SERIAL_8N1, WT_RX, WT_TX);
  WitMotionSerial.begin(115200, SERIAL_8N1, WT_RX, WT_TX);
  
  ESP32Can.setPins(CAN_TX, CAN_RX);
  Serial.println("ESP32 CAN IMU SENDER - Starting...");
  if (ESP32Can.begin(ESP32Can.convertSpeed(1000))) {
    Serial.println("CAN bus started successfully at 1000kbps!");
  } else {
    Serial.println("CAN bus failed to start!");
  }
}

void sendFrame(uint8_t axisId, uint8_t commandId, unsigned char message[8]) {
    int msgId = (axisId << 5) + commandId;

    CanFrame frame = {0};
    frame.identifier       = msgId; 
    frame.extd             = 0;
    frame.data_length_code = 8;

    for(size_t i=0;i<8;++i){
      frame.data[i]=message[i];
    }
    
    
    ESP32Can.writeFrame(frame);
}

void loop() {

  if (millis() - last_time > samplingTime) {
    last_time = millis();

    while (WitMotionSerial.available()) {
      JY901.CopeSerialData(WitMotionSerial.read());  
    }

    xAngle = (float)JY901.stcAngle.Angle[0] / 32768 * 180;
    yAngle = (float)JY901.stcAngle.Angle[1] / 32768 * 180;
    zAngle = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
    Serial.print(xAngle);
    Serial.print(" " );
    Serial.print(yAngle);
    Serial.print(" " );
    Serial.println(zAngle);

    memcpy(stmp1, &xAngle, sizeof(xAngle));
    memcpy(stmp2, &yAngle, sizeof(yAngle));

    for (size_t i = 0; i < 4; ++i) {  // print the data 
      stmp[i] = stmp1[i];
      stmp[i + 4] = stmp2[i];
    }
    sendFrame(axis17, sendIMUData, stmp);
  }
}

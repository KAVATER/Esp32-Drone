#include <Wire.h>

#define MPU6050_ADDR 0x68
#define SDA_PIN 21  // Change as needed
#define SCL_PIN 22  // Change as needed

float RateRoll, RatePitch, RateYaw;
float RateCalibRoll, RateCalibPitch, RateCalibYaw;
int RateCalibNum;

void gyro_signals() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1A);  // Configure DLPF
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // Configure Gyroscope sensitivity
    Wire.write(0x08);  // ±500 °/s
    Wire.endTransmission();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);  // Request gyroscope data
    Wire.endTransmission();

    Wire.requestFrom(MPU6050_ADDR, 6);
    if (Wire.available() < 6) return; // Ensure all bytes are received

    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN, 400000);

    delay(250);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // Wake up MPU6050
    Wire.write(0x00);
    Wire.endTransmission();

    // Verify MPU6050 connection
    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU6050 not found!");
        while (1);
    }

    // Gyro Calibration
    for (RateCalibNum = 0; RateCalibNum < 2000; RateCalibNum++) {
        gyro_signals();
        RateCalibRoll += RateRoll;
        RateCalibPitch += RatePitch;
        RateCalibYaw += RateYaw;
        delay(1);
    }

    RateCalibRoll /= 2000;
    RateCalibPitch /= 2000;
    RateCalibYaw /= 2000;
}

void loop() {
    gyro_signals();
    RateRoll -= RateCalibRoll;
    RatePitch -= RateCalibPitch;
    RateYaw -= RateCalibYaw;

    Serial.print("Roll rate [°/s]= ");
    Serial.print(RateRoll);
    Serial.print(" Pitch Rate [°/s]= ");
    Serial.print(RatePitch);
    Serial.print(" Yaw Rate [°/s]= ");
    Serial.println(RateYaw);

    delay(50);
}

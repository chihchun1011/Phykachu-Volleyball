/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "Keyboard.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define BODY 1
#define HAND 0
#define ARM_LO 2
#define ARM_UP 3

#define EMG_DATA_LENGTH 200
#define EMG_LEVEL_1 15
#define EMG_LEVEL_2 30

enum Action{
    IDLE, FW, BW, JUMP, HIT_UP, HIT_FW, HIT_DN, DIVE_FW, DIVE_BW
};

int FLEX_PIN = A1;
int EMG_PIN = A0;

int EMGData[EMG_DATA_LENGTH];


MPU6050 mpu0(0x68);  // AD0 low

// MPU control/status vars
bool dmpReady[2] = {false, false};  // set true if DMP init was successful
uint8_t mpuIntStatus[2];   // holds actual interrupt status byte from MPU
uint8_t devStatus[2];      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize[2];    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount[2];     // count of all bytes currently in FIFO
uint8_t fifoBuffer[2][64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// {XAccel, YAccel, ZAccel, XGyro, YGyro, ZGyro}
int16_t MPUOffset[2][6] = {-82, 1479, 1288, 77, 0, 53, // #5
                           -1154, 51, 536, 111, 81, 19}; //#6

unsigned long startTime = 0, endTime = 0;
Action action;





void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // wait for ready
    Serial.println(F("\nSend any character to begin"));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again


    Serial.println(F("Initializing I2C devices..."));

    mpu0.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu0.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    devStatus[0] = mpu0.dmpInitialize();
    setOffset(&mpu0, MPUOffset[BODY]);
    mpu0.PrintActiveOffsets();

    // make sure it worked (returns 0 if so)
    if (devStatus[0] == 0) {

        mpu0.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        // dmpReady[0] = true;

        // get expected DMP packet size for later comparison
        packetSize[0] = mpu0.dmpGetFIFOPacketSize();
    }
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus[0]);
        Serial.println(F(")"));
    }

    Keyboard.begin();
}

void loop() {

    // startTime = micros();
    readMPUData(&mpu0, 0);
    action = getAction();
    sendAction(action);

    // endTime = micros();

    // Serial.print("time: ");
    // Serial.println(endTime-startTime);
    // printAll(BODY);
    printRealWorldAccel();
    // printAll(HAND);

    delay(100);

}


void setOffset(MPU6050* mpu, int16_t* offsets) {

    mpu->setXAccelOffset(offsets[0]);
    mpu->setYAccelOffset(offsets[1]);
    mpu->setZAccelOffset(offsets[2]);
    mpu->setXGyroOffset(offsets[3]);
    mpu->setYGyroOffset(offsets[4]);
    mpu->setZGyroOffset(offsets[5]);
}


void printRealWorldAccel(){

    Serial.print("aworld\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
}

void printYPR(){
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
}

void printAll(){
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.print(aaWorld.z);
    Serial.print("\t");
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);
}


void readMPUData(MPU6050* mpu, int mpuNum){
    // wait for MPU interrupt or extra packet(s) available
    mpu->resetFIFO();
    fifoCount[mpuNum] = mpu->getFIFOCount();
    while (fifoCount[mpuNum] < packetSize[mpuNum]) {
        fifoCount[mpuNum] = mpu->getFIFOCount();
    }

    // check for overflow (this should never happen unless our code is too inefficient)
    if (fifoCount[mpuNum] >= 1024) {
        // reset so we can continue cleanly
        mpu->resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return;
    }

    // read a packet from FIFO
    while(fifoCount[mpuNum] >= packetSize[mpuNum]){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu->getFIFOBytes(fifoBuffer[mpuNum], packetSize[mpuNum]);
        fifoCount[mpuNum] -= packetSize[mpuNum];
    }

    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu->dmpGetQuaternion(&q, fifoBuffer[mpuNum]);
    mpu->dmpGetAccel(&aa, fifoBuffer[mpuNum]);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
    
}

int calMPULevel(int* mpuData){

    return 0;
}


void readEMGData(int dt){

    for(int i=0;i<EMG_DATA_LENGTH;++i){
        EMGData[i] = analogRead(EMG_PIN);
        delay(dt);
    }
}

int calEMGLevel(int* emgData){

    double var = 0.;
    double mean = 0.;
    for (int i = 0; i < EMG_DATA_LENGTH; ++i){
        mean += emgData[i];
    }
    mean /= EMG_DATA_LENGTH;
    for(int i=0;i<EMG_DATA_LENGTH;++i){
        var += sq(emgData[i]-mean);
    }
    var /= EMG_DATA_LENGTH;

    if(var > EMG_LEVEL_2) {
        return 2;
    }
    else if(var > EMG_LEVEL_1){
        return 1;
    }
    else {
        return 0;
    }
}


Action getAction(){

    if(aaReal.x > 200){
        return Action::FW;
    }

    return Action::IDLE;
}

void sendAction(Action action){

    switch(action){
        case FW:
            Keyboard.write(KEY_LEFT_ARROW);
            break;
        case BW:
            Keyboard.write(KEY_RIGHT_ARROW);
            break;
        case JUMP:
            Keyboard.write(KEY_UP_ARROW);
            break;
        case HIT_UP:
            Keyboard.press(KEY_UP_ARROW);
            Keyboard.press(KEY_RETURN);
            Keyboard.releaseAll();
            break;
        case HIT_FW:
            Keyboard.press(KEY_LEFT_ARROW);
            Keyboard.press(KEY_RETURN);
            Keyboard.releaseAll();
            break;
        case HIT_DN:
            Keyboard.press(KEY_DOWN_ARROW);
            Keyboard.press(KEY_RETURN);
            Keyboard.releaseAll();
            break;
        case DIVE_FW:
            Keyboard.press(KEY_LEFT_ARROW);
            Keyboard.press(KEY_RETURN);
            Keyboard.releaseAll();
            break;
        case DIVE_BW:
            Keyboard.press(KEY_RIGHT_ARROW);
            Keyboard.press(KEY_RETURN);
            Keyboard.releaseAll();
            break;
        default:
            break;
    }

}


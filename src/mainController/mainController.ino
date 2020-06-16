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


#define DATA_LENGTH 100
#define MPU_DATA_LENGTH 10

#define THREDSHOLD_FWAC 150
#define THREDSHOLD_BWAC -100
#define THREDSHOLD_JPAC 600
#define THREDSHOLD_FWAC_ADD 60
#define THREDSHOLD_BWAC_ADD -60
#define THREDSHOLD_FWV 60
#define THREDSHOLD_BWV -60
#define THREDSHOLD_EMG 30
#define THREDSHOLD_FLEX_POS 50
#define THREDSHOLD_FLEX_NEG -200

#define AIRTIME 1200 // (ms)

#define CONTROL_LEFT_PLAYER

#ifdef CONTROL_LEFT_PLAYER
#define KEY_RETURN 122       // z
#define KEY_LEFT_ARROW 103   // g (forward)
#define KEY_RIGHT_ARROW 100  // d (backward)
#define KEY_UP_ARROW 114     // r
#define KEY_DOWN_ARROW 102   // f
#endif

enum Action{
    FW, BW, JUMP, HIT_UP, HIT_FW, HIT_DN, DIVE_FW, DIVE_BW
};

enum LR_STATE{
    IDLE, FWD, BWD
};

const int EMG_PIN = A0;
const int FLEX_PIN = A1;

const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 10000.0; // Measured resistance of 3.3k resistor
const float STRAIGHT_RESISTANCE = 21480.0; // resistance when straight
const float BEND_RESISTANCE = 18680.0; // resistance at 90 deg

int FlexData[DATA_LENGTH];
int EMGData[DATA_LENGTH];


MPU6050 mpu0(0x68);  // AD0 low

// MPU control/status vars
// bool dmpReady[2] = {false, false};  // set true if DMP init was successful
// uint8_t mpuIntStatus[2];   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

// {XAccel, YAccel, ZAccel, XGyro, YGyro, ZGyro}
int16_t MPUOffset[2][6] = {-82, 1479, 1288, 77, 0, 53, // #5
                           -1154, 51, 536, 111, 81, 19}; //#6

Action action;
LR_STATE lr_state = LR_STATE::IDLE;


float worldZ = 0;
float localZ = 0;
double velocity = 0;
bool inAir = false;
unsigned long airStartTime = 0;
int consec_noacc = 0;

// Debug
unsigned long startTime = 0, endTime = 0;

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
    devStatus = mpu0.dmpInitialize();
    setOffset(&mpu0, MPUOffset[0]);
    mpu0.PrintActiveOffsets();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        mpu0.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        // dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu0.dmpGetFIFOPacketSize();
    }
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    delay(4000);
    Serial.println("============================ End init ============================");
    Keyboard.begin();

}

void loop() {

    readAnalogDatas(1);
    readMPUDatas();

    // printAccel();

    switch(lr_state){
        case IDLE:
            velocity = 0;
            if(localZ > THREDSHOLD_FWAC){
                velocity += localZ > THREDSHOLD_FWAC_ADD ? localZ : 0;
                lr_state = LR_STATE::FWD;
            }
            if(localZ < THREDSHOLD_BWAC){
                velocity += localZ < THREDSHOLD_BWAC_ADD ? localZ : 0;
                lr_state = LR_STATE::BWD;
            }
            break;

        case FWD:
            sendAction(Action::FW);
            velocity += localZ > THREDSHOLD_FWAC_ADD ? localZ : 0;

            if(velocity < THREDSHOLD_FWV){
                lr_state = LR_STATE::IDLE;
            }
            break;

        case BWD:
            sendAction(Action::BW);
            velocity += localZ < THREDSHOLD_BWAC_ADD ? localZ : 0;

            if(velocity > THREDSHOLD_BWV){
                lr_state = LR_STATE::IDLE;
            }
            break;
    }

    
    if(localZ < THREDSHOLD_FWAC && localZ > THREDSHOLD_BWAC){
        consec_noacc += 1;
    }
    if(consec_noacc == 3){
        // Serial.println("BAD");
        velocity = 0;
        lr_state = LR_STATE::IDLE;
        consec_noacc = 0;
    }

    // Serial.print("State: ");
    // switch(lr_state){
    //     case FWD:
    //         Serial.println("FWD");
    //         break;
    //     case BWD:
    //         Serial.println("BWD");
    //         break;
    //     case IDLE:
    //         Serial.println("IDLE");
    //         break;

    // }
    Serial.print("Velo: ");
    Serial.println(velocity);


    if (inAir) {
        // Serial.println("inAir");
        if (isTriggerEMG()){
            sendAction(getFlexLevel());
        }
        if (millis()-airStartTime > AIRTIME){
            inAir = false;
        }
    }

    else {

        if (worldZ > THREDSHOLD_JPAC){
            inAir = true;
            sendAction(Action::JUMP);
            airStartTime = millis();
        }
        if (isTriggerEMG()){
            if (lr_state == LR_STATE::FWD){
                sendAction(Action::DIVE_FW);
            }
            else if (lr_state == LR_STATE::BWD){
                sendAction(Action::DIVE_BW);
            }
        }

    }

    // delay(1000);

}


void setOffset(MPU6050* mpu, int16_t* offsets) {

    mpu->setXAccelOffset(offsets[0]);
    mpu->setYAccelOffset(offsets[1]);
    mpu->setZAccelOffset(offsets[2]);
    mpu->setXGyroOffset(offsets[3]);
    mpu->setYGyroOffset(offsets[4]);
    mpu->setZGyroOffset(offsets[5]);
}

void readMPUData(MPU6050* mpu){
    // wait for MPU interrupt or extra packet(s) available
    mpu->resetFIFO();
    fifoCount = mpu->getFIFOCount();
    while (fifoCount < packetSize) {
        fifoCount = mpu->getFIFOCount();
    }

    // check for overflow (this should never happen unless our code is too inefficient)
    if (fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu->resetFIFO();
        Serial.println(F("FIFO overflow!"));
        return;
    }

    // read a packet from FIFO
    while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu->getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
    }

    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu->dmpGetQuaternion(&q, fifoBuffer);
    mpu->dmpGetAccel(&aa, fifoBuffer);
    mpu->dmpGetGravity(&gravity, &q);
    mpu->dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu->dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    // mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
    
}


void readMPUDatas(){
    // Serial.println("readMPUDatas");
    float localZs[MPU_DATA_LENGTH];
    float worldZs[MPU_DATA_LENGTH];
    localZ = 0.;
    worldZ = 0.;

    for (int i=0;i<MPU_DATA_LENGTH;++i){
        readMPUData(&mpu0);
        localZs[i] = aaReal.z;
        worldZs[i] = aaWorld.z;
        // printAccel();
    }
    for (int i=0;i<MPU_DATA_LENGTH;++i){

        localZ += localZs[i];
        worldZ += worldZs[i];
    }
    localZ /= MPU_DATA_LENGTH;
    worldZ /= MPU_DATA_LENGTH;
    Serial.print("localz worldz: ");
    Serial.print(localZ);
    Serial.print("    ");
    Serial.println(worldZ);

}


void readAnalogDatas(int dt){
    // Serial.println("readAnalogDatas");
    for(int i=0;i<DATA_LENGTH;++i){
        EMGData[i] = analogRead(EMG_PIN);
        // EMGData[i] = 0;
        FlexData[i] = analogRead(FLEX_PIN);
        delay(dt);
        // Serial.print(EMGData[i]);
        // Serial.print(" ");
    }
    // Serial.println(" ");
}

bool isTriggerEMG(){
    // Serial.print("emg: ");

    double var = 0.;
    double mean = 0.;
    for (int i = 0; i < DATA_LENGTH; ++i){
        mean += EMGData[i];
        // Serial.print(EMGData[i]);
        // Serial.print(" ");
    }
    // Serial.println(" ");
    // Serial.print("emg2: ");
    mean /= DATA_LENGTH;
    for(int i=0;i<DATA_LENGTH;++i){
        var += sq(EMGData[i]-mean);
        // Serial.print(EMGData[i]-mean);
        // Serial.print(" ");
    }
    // Serial.println(" ");
    var /= DATA_LENGTH;
    var = sqrt(var);
    Serial.print("EMG: ");
    Serial.print(mean);
    Serial.print("     ");
    Serial.println(var);
    return var > THREDSHOLD_EMG;
}

double getAngle(int flexADC){
    float flexV = flexADC * VCC / 1023.0;
    float flexR = R_DIV * (VCC / flexV - 1.0);
    float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);
    return angle;
}


Action getFlexLevel(){
    double angle = 0;
    for(int i=0;i<DATA_LENGTH;++i){
        angle += getAngle(FlexData[i]);
    }
    angle /= DATA_LENGTH;
    Serial.print("Angle: ");
    Serial.println(angle);


    if (angle > THREDSHOLD_FLEX_POS){
        return Action::HIT_UP;
    }
    else if (angle < THREDSHOLD_FLEX_NEG){
        return Action::HIT_DN;
    }
    else {
        return Action::HIT_FW;
    }

}

void sendAction(Action action){

    // Serial.print("Send: ");
    // switch(action){
    //     case FW:
    //         Serial.println("FW");
    //         break;
    //     case BW:
    //         Serial.println("BW");
    //         break;
    //     case JUMP:
    //         Serial.println("JUMP");
    //         break;
    //     case HIT_UP:
    //         Serial.println("HIT_UP");
    //         break;
    //     case HIT_FW:
    //         Serial.println("HIT_FW");
    //         break;
    //     case HIT_DN:
    //         Serial.println("HIT_DN");
    //         break;
    //     case DIVE_FW:
    //         Serial.println("DIVE_FW");
    //         break;
    //     case DIVE_BW:
    //         Serial.println("DIVE_BW");
    //         break;
    //     default:
    //         break;
    // }
    // return;

    switch(action){
        case FW:
            Keyboard.press(KEY_LEFT_ARROW);
            delay(100);
            Keyboard.releaseAll();
            break;
        case BW:
            Keyboard.press(KEY_RIGHT_ARROW);
            delay(100);
            Keyboard.releaseAll();
            break;
        case JUMP:
            Keyboard.press(KEY_UP_ARROW);
            delay(30);
            Keyboard.releaseAll();
            break;
        case HIT_UP:
            Keyboard.press(KEY_UP_ARROW);
            Keyboard.press(KEY_RETURN);
            delay(200);
            Keyboard.releaseAll();
            break;
        case HIT_FW:
            Keyboard.press(KEY_LEFT_ARROW);
            Keyboard.press(KEY_RETURN);
            delay(200);
            Keyboard.releaseAll();
            break;
        case HIT_DN:
            Keyboard.press(KEY_DOWN_ARROW);
            Keyboard.press(KEY_RETURN);
            delay(200);
            Keyboard.releaseAll();
            break;
        case DIVE_FW:
            Keyboard.press(KEY_LEFT_ARROW);
            Keyboard.press(KEY_RETURN);
            delay(30);
            Keyboard.releaseAll();
            break;
        case DIVE_BW:
            Keyboard.press(KEY_RIGHT_ARROW);
            Keyboard.press(KEY_RETURN);
            delay(30);
            Keyboard.releaseAll();
            break;
        default:
            break;
    }

}


void printAccel(){

    Serial.print("alocal world\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.print(aaReal.z);
    Serial.print("\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.println(aaWorld.z);
}


#include <Arduino.h>
#include <FlexCAN.h> //https://github.com/teachop/FlexCAN_Library 
#include <Smoothed.h> //https://github.com/MattFryer/Smoothed



/////////// Pin Assignments /////////
const int led = 13;


/////////// Variables //////////////
int rpm;
int mtemp;
int hstemp;
int amps;
int potnom;
int pot;
int run;
int dir = 0;
int brake;
int boost;
int maxBoost;
int throtRamp;
int pot2;
int fweak;
int minBoost;
int brkNomPedal;
int brkMax;
int baseRegen;
int maxRegen;
int idleThrot;
int ampMin;
int slipstart;
int idleRPM;
int neg = 4294967295;
float maxSlip;
float minSlip;
float fslip;
float fslipmin;
bool startup;


CAN_message_t msg;
CAN_message_t inMsg;
CAN_filter_t filter;

//////////////Smoothing///////////////

Smoothed <int> idleRamp;
Smoothed <float> fslipRamp;


void setup() {

    pinMode(led, OUTPUT);
    Can0.begin(500000);

    //set filters for standard
    for (int i = 0; i < 8; i++)
    {
        Can0.setFilter(filter, i);
    }
    //set filters for extended
    for (int i = 9; i < 13; i++)
    {
        Can0.setFilter(filter, i);
    }

    digitalWrite(led, HIGH);
    Serial.begin(1152000);

    idleRamp.begin(SMOOTHED_AVERAGE, 60);
    fslipRamp.begin(SMOOTHED_AVERAGE, 60);

}

void loop() {

    while (Can0.available())
    {
        Can0.read(inMsg);
        decodeCAN();
    }

    parameterMap();
    boostMap();
    idleThrottle();
    regenStuff();
   
}




void decodeCAN() {

    if (inMsg.id == 0x135) {
        if ((((inMsg.buf[3] << 8) + inMsg.buf[2])) <= 2000) {

            amps = (((inMsg.buf[3] << 8) + inMsg.buf[2]) * 1.83);
        }
        else if ((((inMsg.buf[3] << 8) + inMsg.buf[2])) >= 3000) {

            amps = (((((inMsg.buf[3] << 8) + inMsg.buf[2]) - 65535) * 1.83) * -1);

        }
        rpm = (((inMsg.buf[1] << 8) + inMsg.buf[0]));

        if ((inMsg.buf[4]) > 0) {
            mtemp = (inMsg.buf[4]);
        }

        if ((inMsg.buf[5]) > 0) {
            hstemp = (inMsg.buf[5]);
        }
        if ((((inMsg.buf[7] << 8)) + inMsg.buf[6]) <= 2000) {
            potnom = (((inMsg.buf[7] << 8)) + inMsg.buf[6]);
        }
        else if ((((inMsg.buf[7] << 8)) + inMsg.buf[6]) >= 2000) {
            potnom = ((((inMsg.buf[7] << 8)) + inMsg.buf[6]) - 65535);
        }
    }

    else if (inMsg.id == 79) {

        dir = (inMsg.buf[0]);
        brake = (inMsg.buf[1]);

    }

    else if (inMsg.id == 0x136) {

        run = (inMsg.buf[0]);
    }


    else if (inMsg.id == 0x113) {
        pot = ((inMsg.buf[1] << 8) + inMsg.buf[0]);
        pot2 = ((inMsg.buf[3] << 8) + inMsg.buf[2]);
    }
   
}



void parameterMap() {

    //boost
    maxBoost = 1720;
    minBoost = 1360;

    //fweak
    fweak = 258;
    canSet(1, fweak);


    //fslipmin
    fslipmin = .84;
    canSet(4, fslipmin);

    //fslipmax
    maxSlip = (3.08 * 32);
    if (pot >= 2800) {
        minSlip = map(pot, 2800, 4095, fslipmin * 32, maxSlip);

    }
    else { minSlip = fslipmin *32 ; }

    if (rpm <= 4500) {
        fslip = map(rpm, 0, 4200, minSlip, maxSlip);
    }
    else { fslip = maxSlip; }

    fslipRamp.add(fslip);
    canSet(5, fslipRamp.get()/32);

    // throtramp
    throtRamp = 25;
    canSet(49, throtRamp);

    //ampmin
    ampMin = 1;
    canSet(51, ampMin);


    //slipstart
    slipstart = 29;
    canSet(52, slipstart);

}
void boostMap()
{
    if (pot > 3700) {
        boost = map(pot, 3700, 4095, minBoost, maxBoost);
        canSet(0, boost);
    }
    else {
        boost = minBoost;
        canSet(0, boost);
    }

}

void regenStuff() {

    baseRegen = 45;

    //brakenompedal
    if (pot2 > 3700) {
        brkNomPedal = ((neg - (maxRegen * 32)) / 32);
    }
    else {
        brkNomPedal = map(pot2, 600, 3700, ((neg - (baseRegen * 32)) / 32), ((neg - (maxRegen * 32))) / 32);
    }
    canSet(53, brkNomPedal);

    //brakemax
    brkMax = ((neg - (baseRegen * 32)) / 32);
    canSet(56, brkMax);
}


void idleThrottle() {

    if (run == 0) {
        canSet(64, 1);
    }
    else {
        canSet(64, 0);
    }

    idleRamp.add(pot2);
    idleThrot = map(idleRamp.get(), 600, 1150, 27, 0);
    canSet(63, idleThrot);

    if (pot2 > 1550) {
        idleRPM = map(pot2, 1550, 3900, 1750, 0);
    }
    else {
        idleRPM = 1750;
    }
    canSet(62, idleRPM);

}

void canSet(int index, float value) {
    int val = (value * 32);
    int byte1;
    int byte2;
    int byte3;
    int byte4;
    byte1 = val & 0xFF;
    byte2 = (val >> 8) & 0xFF;
    byte3 = (val >> 16) & 0xFF;
    byte4 = (val >> 24) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40;
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = index;
    msg.buf[4] = byte1;
    msg.buf[5] = byte2;
    msg.buf[6] = byte3;
    msg.buf[7] = byte4;
    Can0.write(msg);
}






/*Also this version has CANOpen implemented. You can send so called SDO messages:

Id    Len cmd  index  subindex data
0x601 8   0x40 0x2000 paramidx value

cmd is 1 byte, index is 2 bytes(little endian), subindex 1 byte, data 4 bytes.
The subindex is the parameter index, which is a bit hard to find right now.Basically if you type
"list" you count at which position the parameter shows up.For example boost has subindex 0 because
its the very first.
The value is the desired value * 32. So if you want to set boost to 2000 you would send
Id    Len cmd  index     subindex data
0x601 8   0x40 0x00 0x20 0x00     0x00 0xFA 0x00 0x00
*/


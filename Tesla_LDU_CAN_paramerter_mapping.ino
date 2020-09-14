#include <Arduino.h>
#include <FlexCAN.h> //https://github.com/teachop/FlexCAN_Library 



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
int throtByte1;
int throtByte2;
int throtRamp;
int pot2;
int brkNomPedal;
int brkMax;
int brakeByte1;
int brakeByte2;
int regenByte1;
int regenByte2;
int baseRegen;
int maxRegen;
int idleThrot;
int iThrotByte1;
int iThrotByte2;
int neg = 4294967295;
float maxSlip;
float minSlip;
float fslip;
bool startup;


CAN_message_t msg;
CAN_message_t inMsg;
CAN_filter_t filter;


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

}

void loop() {
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
    //boost max
    maxBoost = 215;
    //fweak
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20; //
    msg.buf[3] = 0x01;//index:boost=0
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x20;//258=8102=0x2000
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);

    //slip max
    maxSlip = (3.08 * 32);
    if (pot >= 2800) {
        minSlip = map(pot, 2800, 4095, (.84 * 32), maxSlip);

    }
    else { minSlip = (.84 * 32); }

    if (rpm <= 4200) {
        fslip = map(rpm, 0, 4200, minSlip, maxSlip);
    }
    else { fslip = maxSlip; }
    //fslipmax
    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20; //
    msg.buf[3] = 0x05;//index:boost=0
    msg.buf[4] = fslip;
    msg.buf[5] = 0x00;//258=8102=0x2000
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);

    // throtramp

    if (rpm < 2000 && pot < 3000) {
        throtRamp = (8 * 32);
    }
    else if (rpm >= 2000 && rpm <= 4500) {
        throtRamp = map(rpm, 2000, 4500, (8 * 32), (25 * 32));
    }
    else {
        throtRamp = (25 * 32);
    }

    
    throtByte1 = throtRamp & 0xFF;
    throtByte2 = (throtRamp >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 49;//index
    msg.buf[4] = throtByte1;
    msg.buf[5] = throtByte2;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
    Can0.write(msg);



}
void boostMap()
{
    if (pot > 3700) {

        boost = map(pot, 2700, 4095, 100, maxBoost);

        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00;
        msg.buf[2] = 0x20; //
        msg.buf[3] = 0x00;//index:boost=0, count down for index number
        msg.buf[4] = 0x00;
        msg.buf[5] = boost;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

    else {
        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; //
        msg.buf[2] = 0x20; //
        msg.buf[3] = 0x00;
        msg.buf[4] = 0x00;
        msg.buf[5] = 175;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

}

void regenStuff() {

    //baseRegen
   
    baseRegen = 45;
    maxRegen = 90;

    //brakenompedal
    if (pot2 > 3700) {
        brkNomPedal = (neg - (maxRegen * 32));
    }

    else {
        brkNomPedal = map(pot2, 600, 3700, (neg - (baseRegen * 32)), (neg - (maxRegen * 32)));
    }

    //brkNomPedal = (neg -  (11 * 32));

    brakeByte1 = brkNomPedal & 0xFF;
    brakeByte2 = (brkNomPedal >> 8) & 0xFF;


    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD SET
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 53;//index
    msg.buf[4] = brakeByte1;
    msg.buf[5] = brakeByte2;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFF;
    Can0.write(msg);

    //brakemax
    brkMax = (neg - (baseRegen * 32));
    regenByte1 = brkMax & 0xFF;
    regenByte2 = (brkMax >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD SET
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20;
    msg.buf[3] = 56;//index
    msg.buf[4] = regenByte1;
    msg.buf[5] = regenByte2;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFF;
    Can0.write(msg);

}

void idleThrottle() {
    if (rpm < 35) {
        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; //
        msg.buf[2] = 0x20; //
        msg.buf[3] = 64;
        msg.buf[4] = 0x20;
        msg.buf[5] = 0x00;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }

    else {
        msg.id = 0x601; //set parameter ID
        msg.len = 8;
        msg.buf[0] = 0x40; //CMD
        msg.buf[1] = 0x00; //
        msg.buf[2] = 0x20; //
        msg.buf[3] = 64;
        msg.buf[4] = 0x00;
        msg.buf[5] = 0x00;//value x 32
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        Can0.write(msg);
    }


    if (rpm == 0) {
        startup = true;
    }
    if (rpm > 200) {
        startup = false;
    }

    if (startup == true && brake == 0) {

        if (idleThrot <= (27 * 32)) {
            idleThrot += 4;
        }

    }
    if (startup == true && brake == 1) {
        idleThrot = (8 * 32);
        
    }

    if (startup == false) {
        idleThrot = map(pot2, 600, 1150, (27 * 32), 0);
    }


    iThrotByte1 = idleThrot & 0xFF;
    iThrotByte2 = (idleThrot >> 8) & 0xFF;

    msg.id = 0x601; //set parameter ID
    msg.len = 8;
    msg.buf[0] = 0x40; //CMD
    msg.buf[1] = 0x00;
    msg.buf[2] = 0x20; //
    msg.buf[3] = 63;//index
    msg.buf[4] = iThrotByte1;
    msg.buf[5] = iThrotByte2;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;
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


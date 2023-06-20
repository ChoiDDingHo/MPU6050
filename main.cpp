#include "mbed.h"
#include "MPU6050.h"
#include <math.h>

#define pi 3.141592654

float dt = 0;
float ARoll, APitch, AYaw = 0;//가속도 센서 롤, 피치, 요
float GRoll, GPitch, GYaw = 0;//자이로 센서 롤, 피치, 요
float gyroX, gyroY, gyroZ = 0;
float accelX, accelY, accelZ = 0;
float lgyroX, lgyroY, lgyroZ = 0;
float FPitch = 0;//필터 적용된 피치값


MPU6050 mpu6050;
Timer t;
Serial pc(USBTX, USBRX, 9600); // tx, rx

int main(){
    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C
    t.start();

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68){
        pc.printf("MPU6050 is online...");
        wait(1);

        mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values

        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu6050.initMPU6050();
            pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

            pc.printf("\nMPU6050 passed self test... Initializing");
            wait(2);
        } else pc.printf("\nDevice did not the pass self-test!\n\r");
    } else {
        pc.printf("Could not connect to MPU6050: \n\r");
        pc.printf("%#x \n",  whoami);
        while(1) ; // Loop forever if communication doesn't happen
    }

    while(1) {
        Now = t.read_us();
        // If data ready bit set, all data registers have new data
        if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
            mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu6050.readAccelData(accelCount);
            mpu6050.getGres();
            mpu6050.getAres();

            gyroY = (float)gyroCount[0]*gRes;
            gyroX = (float)gyroCount[1]*gRes;
            gyroZ = (float)gyroCount[2]*gRes; // - gyroBias[2];
            
            accelX = (float)accelCount[0]*aRes;
            accelY = (float)accelCount[1]*aRes;
            accelZ = (float)accelCount[2]*aRes;
        }

        ARoll = (180/pi)*(atan(accelX/(sqrt((accelY*accelY)+(accelZ*accelZ))))) - 4.36;
        APitch = (180/pi)*(atan(accelY/(sqrt((accelX*accelX)+(accelZ*accelZ))))) - 0.063;
        AYaw = (180/pi)*(atan((sqrt((accelX*accelX)+(accelY*accelY)))/accelZ)) - 3.93;

        GYaw += ((lgyroZ+gyroZ)/2)*0.02*2;
        GPitch += ((lgyroY+gyroY)/2)*0.02*2;
        FPitch = (0.98*(FPitch+(((lgyroY+gyroY)/2)*2*0.02)))+0.02*APitch;//complementary fillter
        lgyroZ = gyroZ;
        lgyroY = gyroY;
        pc.printf("%f,%f,%f\n",FPitch, APitch, GPitch);

        dt = (float)((t.read_us() - Now)/1000000.0f); // set integration time by time elapsed since last filter update
        wait(0.02-dt);
    }
}

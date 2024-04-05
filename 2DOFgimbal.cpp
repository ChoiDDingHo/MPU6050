#include "mbed.h"
#include "MPU6050.h"
#include <math.h>
#define pi 3.141592654
//제어주기 20ms

float dt = 0;
float Roll, Pitch, Yaw = 0;
float ARoll, APitch, AYaw = 0;//가속도 센서 롤, 피치, 요
float GRoll, GPitch, GYaw = 0;//자이로 센서 롤, 피치, 요
float gyroX, gyroY, gyroZ = 0;
float accelX, accelY, accelZ = 0;
float lgyroX, lgyroY, lgyroZ = 0;
float FPitch, FRoll = 0;//필터 적용된 피치0값

MPU6050 mpu6050;
Timer t;
Serial pc(USBTX, USBRX, 115200); // tx, rx
PwmOut servo1(PC_8);//roll
PwmOut servo2(PC_6);//pitch

float Roll_target_angle = 0;
float Pitch_target_angle = 0;
float Roll_output, Pitch_output;
float Roll_angle_iterm, Pitch_angle_iterm;
float Roll_rate_iterm, Pitch_rate_iterm;
float Roll_rate_dterm, Pitch_rate_dterm;
float Roll_angle_input, Pitch_angle_input;
float Roll_rate_input, Pitch_rate_input;

float Roll_angle_kp = 2.3;
float Roll_angle_ki = 0;
float Roll_rate_kp = 0.2;
float Roll_rate_ki = 0;
float Roll_rate_kd = 0.0001;

float Pitch_angle_kp = 2.3;
float Pitch_angle_ki = 0;
float Pitch_rate_kp = 0.2;
float Pitch_rate_ki = 0;
float Pitch_rate_kd = 0.0001;

float Roll_angle_error;
float Roll_target_rate;
float Roll_rate_error;
float Roll_prev_error;
float Roll_angle_pterm;
float Roll_rate_pterm;
float Roll_rate_prev_dterm;
float Roll_prev_rate;

float Pitch_angle_error;
float Pitch_target_rate;
float Pitch_rate_error;
float Pitch_prev_error;
float Pitch_angle_pterm;
float Pitch_rate_pterm;
float Pitch_rate_prev_dterm;
float Pitch_prev_rate;

float servo_Pitch = 1500;
float servo_Roll = 1500;

void dualPID(float target_angle,
             float angle_input,
             float rate_input,
             float angle_kp,
             float angle_ki,
             float rate_kp,
             float rate_ki,
             float rate_kd,
             float &angle_iterm,
             float &rate_iterm,
             float &rate_dterm,
             float &output)
{
    float angle_error;
    float target_rate;
    float rate_error;
    float prev_error;
    float angle_pterm, rate_pterm;
    float rate_prev_dterm;
    float prev_rate;
    
    angle_error = target_angle - angle_input;

    angle_pterm = angle_kp * angle_error;

    angle_iterm += angle_ki * angle_error * dt;
    
    target_rate = angle_pterm; //외부 루프 각도 P제어, 내부 루프 각속도 제어의 target rate가 된다.

    rate_error = target_rate - rate_input;
    prev_error = target_rate - prev_rate; // 이전 오차 = 목표값 - 이전 값
    
    rate_pterm = rate_kp * rate_error; 
    rate_iterm += rate_ki * rate_error * dt; 
    rate_dterm = ((rate_error - prev_error)/dt) * rate_kd;

    rate_dterm = 0.9 * rate_prev_dterm + 0.1 * rate_dterm;

    if(angle_iterm > 50)
    {
        angle_iterm = 50;
    }
    else if(angle_iterm < -50)
    {
        angle_iterm = -50;
    }

    if(rate_iterm > 30)
    {
        rate_iterm = 30;
    }
    else if(rate_iterm < -30)
    {
        rate_iterm = -30;
    }

    output = rate_pterm + rate_iterm + angle_iterm + rate_dterm;

    prev_rate = rate_input;
    rate_prev_dterm = rate_dterm;
}


int main(){
    servo1.period_ms(20);
    servo2.period_ms(20);
    servo1.pulsewidth_us(1500);
    servo2.pulsewidth_us(1500);
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

            gyroX = (float)gyroCount[0]*gRes;
            gyroY = (float)gyroCount[1]*gRes;

            accelX = (float)accelCount[0]*aRes;
            accelY = (float)accelCount[1]*aRes;
            accelZ = (float)accelCount[2]*aRes;
        }

        ARoll = (180/pi)*(atan(accelX/(sqrt((accelY*accelY)+(accelZ*accelZ)))))-7.3;
        APitch = (180/pi)*(atan(accelY/(sqrt((accelX*accelX)+(accelZ*accelZ)))))-0.9;

        gyroX = gyroX / 131.0;
        gyroY = gyroY / 131.0;

        FPitch = (0.95*(FPitch+(((lgyroY+gyroY))*0.02)))+0.05*APitch;//complementary fillter
        FRoll = (0.95*(FRoll+(((lgyroX+gyroX))*0.02)))+0.05*ARoll;//complementary fillter
        
        Roll_angle_input = FRoll;
        Pitch_angle_input = FPitch;

        Roll_rate_input = gyroX;
        Pitch_rate_input = gyroY;
        //-----------------------------------------------------------------------------------------------------------------
        Roll_angle_error = Roll_target_angle - Roll_angle_input;
        Roll_angle_pterm = Roll_angle_kp * Roll_angle_error;

        Roll_angle_iterm += Roll_angle_ki * Roll_angle_error * 0.02;
    
        Roll_target_rate = Roll_angle_pterm; //외부 루프 각도 P제어, 내부 루프 각속도 제어의 target rate가 된다.

        Roll_rate_error = Roll_target_rate - Roll_rate_input;
        Roll_prev_error = Roll_target_rate - Roll_prev_rate; // 이전 오차 = 목표값 - 이전 값
    
        Roll_rate_pterm = Roll_rate_kp * Roll_rate_error; 
        Roll_rate_iterm += Roll_rate_ki * Roll_rate_error * 0.02; 
        Roll_rate_dterm = ((Roll_rate_error - Roll_prev_error)/0.02) * Roll_rate_kd;

        Roll_rate_dterm = 0.9 * Roll_rate_prev_dterm + 0.1 * Roll_rate_dterm;

        if(Roll_angle_iterm > 50)
        {
           Roll_angle_iterm = 50;
        }
        else if(Roll_angle_iterm < -50)
        {
            Roll_angle_iterm = -50;
        }

        if(Roll_rate_iterm > 30)
        {
            Roll_rate_iterm = 30;
        }
        else if(Roll_rate_iterm < -30)
        {
            Roll_rate_iterm = -30;
        }

        Roll_output = Roll_rate_pterm + Roll_rate_iterm + Roll_angle_iterm + Roll_rate_dterm;

        Roll_prev_rate = Roll_rate_input;
        Roll_rate_prev_dterm = Roll_rate_dterm;
        //-----------------------------------------------------------------------------------------------------------------
        Pitch_angle_error = Pitch_target_angle - Pitch_angle_input;

        Pitch_angle_pterm = Pitch_angle_kp * Pitch_angle_error;

        Pitch_angle_iterm += Pitch_angle_ki * Pitch_angle_error * 0.02;
    
        Pitch_target_rate = Pitch_angle_pterm; //외부 루프 각도 P제어, 내부 루프 각속도 제어의 target rate가 된다.

        Pitch_rate_error = Pitch_target_rate - Pitch_rate_input;
        Pitch_prev_error = Pitch_target_rate - Pitch_prev_rate; // 이전 오차 = 목표값 - 이전 값
    
        Pitch_rate_pterm = Pitch_rate_kp * Pitch_rate_error; 
        Pitch_rate_iterm += Pitch_rate_ki * Pitch_rate_error * 0.02; 
        Pitch_rate_dterm = ((Pitch_rate_error - Pitch_prev_error)/0.02) * Pitch_rate_kd;

        Pitch_rate_dterm = 0.9 * Pitch_rate_prev_dterm + 0.1 * Pitch_rate_dterm;

        if(Pitch_angle_iterm > 50)
        {
        Pitch_angle_iterm = 50;
        }
        else if(Pitch_angle_iterm < -50)
        {
            Pitch_angle_iterm = -50;
        }

        if(Pitch_rate_iterm > 30)
        {
            Pitch_rate_iterm = 30;
        }
        else if(Pitch_rate_iterm < -30)
        {
            Pitch_rate_iterm = -30;
        }

        Pitch_output = Pitch_rate_pterm + Pitch_rate_iterm + Pitch_angle_iterm + Pitch_rate_dterm;

        Pitch_prev_rate = Pitch_rate_input;
        Pitch_rate_prev_dterm = Pitch_rate_dterm;
        //-----------------------------------------------------------------------------------------------------------------
        if(Pitch_output < 0.2 && Pitch_output>-0.2){
            Pitch_output = 0;
        }
        if(Roll_output < 0.2 && Roll_output>-0.2){
            Roll_output = 0;
        }
        if(Pitch_output>50){
            Pitch_output=50;
        }
        else if(Pitch_output<-50){
            Pitch_output = -50;
        }
        if(Roll_output>50){
            Roll_output=50;
        }
        else if(Roll_output<-50){
            Roll_output = -50;
        }

        servo_Pitch -= Pitch_output;
        servo_Roll += Roll_output;

        servo1.pulsewidth_us(servo_Roll);//roll
        servo2.pulsewidth_us(servo_Pitch);//pitch
        
        lgyroX = gyroX;
        lgyroY = gyroY;

        pc.printf("%f,%f,%f,%f\n",Roll_output, Pitch_output,FRoll,FPitch);
        dt = (float)((t.read_us() - Now)/1000000.0f); // set integration time by time elapsed since last filter update
        wait(0.02-dt);
    }
}

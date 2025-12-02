#include "Gyro.h"

int32_t GyroOn=1;//锟角凤拷锟斤拷锟�
uint8 GyroINT=1;//锟斤拷时没锟斤拷
float ang_z=0;//Z锟斤拷锟斤拷俣锟�
float offset_=0.0;//1m偏锟斤拷值
float current_yaw;
///**************************去锟斤拷漂*********************************************************************
float CALIBRATION_SAMPLES = 50 * 2;
float gyroXOffset = 0;
float gyroXOffset_rcc = 0;

float gyroYOffset = 0;
float gyroYOffset_rcc = 0;

float gyroZOffset = 0;
float gyroZOffset_rcc = 0;

// 锟斤拷锟劫讹拷Z锟斤拷锟斤拷偏校准

float calibratedGyro_Z1 = 0;
static float zGyroBias = 0.0f;
static uint32_t zGyroCalibCount = 0;
static float sum = 0.0f;
uint8_t calibrationComplete = 0;
#define CALIBRATION_SAMPLES 80

float calibrateZ_Gyro (float rawZRate) {
   
  float sum = 0.0f;
    
    // 锟斤拷示锟矫伙拷锟斤拷锟街达拷锟斤拷锟斤拷锟斤拷止
    // 锟斤拷锟界：通锟斤拷LED锟斤拷烁锟津串匡拷锟斤拷锟斤拷锟绞�
    
    // 锟缴硷拷指锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
    for (uint32_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += rawZRate;
       // delay_ms(10); // 锟饺达拷10ms锟斤拷锟斤拷锟狡诧拷锟斤拷频锟斤拷
    }
    
    // 锟斤拷锟斤拷平锟斤拷锟斤拷偏值
    return sum / CALIBRATION_SAMPLES;
}    
void calibrateGyro() {          //锟斤拷锟斤拷锟角筹拷始锟斤拷校准

   imu660ra_get_acc();

    float sumX = 0,sunX_rcc = 0;
    float sumY = 0,sunY_rcc = 0;
    float sumZ = 0,sunZ_rcc = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)                     //2s
    {
        imu660ra_get_gyro();
        sumX += imu660ra_gyro_transition(imu660ra_gyro_x);
        sunX_rcc += imu660ra_acc_transition(imu660ra_acc_x);

        sumY += imu660ra_gyro_transition(imu660ra_gyro_y);
        sunY_rcc += imu660ra_acc_transition(imu660ra_acc_y);

        sumZ += imu660ra_gyro_transition(imu660ra_gyro_z);
        sunZ_rcc += imu660ra_acc_transition(imu660ra_acc_z);

        system_delay_ms(20);
    }
    gyroXOffset = sumX / CALIBRATION_SAMPLES ;
    gyroXOffset_rcc = sunX_rcc / CALIBRATION_SAMPLES;

    gyroYOffset = sumY / CALIBRATION_SAMPLES ;
    gyroYOffset_rcc = sunY_rcc / CALIBRATION_SAMPLES;

    gyroZOffset = sumZ / CALIBRATION_SAMPLES ;
    gyroZOffset_rcc = sunZ_rcc / CALIBRATION_SAMPLES;
}

///**************************锟斤拷锟斤拷锟斤拷锟剿诧拷*********************************************************************

typedef struct
{
    float LastP;//锟较次癸拷锟斤拷协锟斤拷锟斤拷
    float Now_P;//锟斤拷前锟斤拷锟斤拷协锟斤拷锟斤拷
    float out;//锟斤拷锟斤拷锟斤拷锟剿诧拷锟斤拷锟斤拷锟�
    float Kg;//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    float Q;//锟斤拷锟斤拷锟斤拷锟斤拷协锟斤拷锟斤拷
    float R;//锟桔诧拷锟斤拷锟斤拷协锟斤拷锟斤拷
}KFP;//Kalman Filter parameter

//KFP KFP_height_1={0.02,0,0,0,0.1,0.1};
#define GRAVITY 9.81  // 锟斤拷锟斤拷锟斤拷锟劫讹拷 (m/s2)
KFP KFP_height_1={0.02,0,0,0,0.001,0.543};
KFP KFP_height_2={0.02,0,0,0,0.001,0.543};
KFP KFP_height_3={0.02,0,0,0,0.001,0.543};

float kalmanFilter_1(KFP *kfp,float input)           //锟斤拷锟斤拷锟斤拷锟剿诧拷
{
     float really_jiaodu = input ;
     //预锟斤拷协锟斤拷锟筋方锟教ｏ拷k时锟斤拷系统锟斤拷锟斤拷协锟斤拷锟斤拷 = k-1时锟教碉拷系统协锟斤拷锟斤拷 + 锟斤拷锟斤拷锟斤拷锟斤拷协锟斤拷锟斤拷
     kfp->Now_P = kfp->LastP + kfp->Q;
     //锟斤拷锟斤拷锟斤拷锟斤拷锟芥方锟教ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 = k时锟斤拷系统锟斤拷锟斤拷协锟斤拷锟斤拷 / 锟斤拷k时锟斤拷系统锟斤拷锟斤拷协锟斤拷锟斤拷 + 锟桔诧拷锟斤拷锟斤拷协锟斤拷锟筋）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //锟斤拷锟斤拷锟斤拷锟斤拷值锟斤拷锟教ｏ拷k时锟斤拷状态锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷值 = 状态锟斤拷锟斤拷锟斤拷预锟斤拷值 + 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 * 锟斤拷锟斤拷锟斤拷值 - 状态锟斤拷锟斤拷锟斤拷预锟斤拷值锟斤拷
     kfp->out = kfp->out + kfp->Kg * (really_jiaodu -kfp->out);//锟斤拷为锟斤拷一锟轿碉拷预锟斤拷值锟斤拷锟斤拷锟斤拷一锟轿碉拷锟斤拷锟街�
     //锟斤拷锟斤拷协锟斤拷锟筋方锟斤拷: 锟斤拷锟轿碉拷系统协锟斤拷锟筋付锟斤拷 kfp->LastP 锟斤拷锟斤拷一锟斤拷锟斤拷锟斤拷准锟斤拷锟斤拷
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
}

///**************************锟斤拷元锟截伙拷锟斤拷劢锟�*********************************************************************
//锟斤拷锟斤拷锟斤拷: 锟斤拷锟劫硷拷锟斤拷 1/Sqrt(x)
//锟斤拷  锟斤拷锟斤拷要锟斤拷锟斤拷锟街�
//锟斤拷锟斤拷值锟斤拷锟斤拷锟�
//锟斤拷  注锟斤拷锟斤拷锟斤拷通Sqrt()锟斤拷锟斤拷要锟斤拷锟侥憋拷
//*********************************************************************************************************/
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//=============== 锟斤拷锟酵讹拷锟藉（锟斤拷锟斤拷锟斤拷锟斤拷锟借补锟戒） ==============
typedef struct {
    float X;
    float Y;
    float Z;
} FLOAT_XYZ;

typedef struct {
    float yaw;   // Z锟斤拷偏锟斤拷锟角ｏ拷直锟斤拷锟斤拷锟斤拷锟斤拷元锟斤拷锟斤拷
    float pit;
    float rol;
} FLOAT_ANGLE;

//=============== 锟姐法锟斤拷锟斤拷锟睫革拷 ================

//锟斤拷锟斤拷锟斤拷锟斤拷元锟斤拷锟斤拷始锟斤拷为 q0=1, q1=q2=q3=0锟斤拷锟斤拷锟斤拷转状态锟斤拷

#define Kp 3.0f    // 锟斤拷锟斤拷锟斤拷锟斤拷俣锟�
#define Ki 0.001f  // 锟斤拷锟酵伙拷锟斤拷影锟斤拷
#define halfT 0.001f//2ms
#define RadtoDeg 57.295779513f

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

float lv_gyro_z;

void IMUupdate(FLOAT_XYZ *Gyr_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle)
{
    // 锟斤拷锟斤拷锟斤拷锟斤拷预锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆硷拷锟斤拷锟斤拷锟�
    float ax = Acc_filt->X;
    float ay = Acc_filt->Y;
    float az = Acc_filt->Z;
    float gx = Gyr_rad->X;  // 确锟斤拷锟窖撅拷锟角伙拷锟饺碉拷位
    float gy = Gyr_rad->Y;
    float gz = Gyr_rad->Z;

    // 锟斤拷锟劫度计癸拷一锟斤拷
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // 锟斤拷元锟斤拷预锟斤拷
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 锟斤拷畈癸拷锟�
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    exInt += ex * Ki;
    eyInt += ey * Ki;
    ezInt += ez * Ki;

    // 锟斤拷锟劫度诧拷锟斤拷
    gx += Kp*ex + exInt;
    gy += Kp*ey + eyInt;
    gz += Kp*ez + ezInt;

    // 锟斤拷元锟斤拷锟斤拷锟斤拷
    float q0_pred = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    float q1_pred = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    float q2_pred = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    float q3_pred = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // 锟斤拷元锟斤拷锟斤拷一锟斤拷
    norm = invSqrt(q0_pred*q0_pred + q1_pred*q1_pred + q2_pred*q2_pred + q3_pred*q3_pred);
    q0 = q0_pred * norm;
    q1 = q1_pred * norm;
    q2 = q2_pred * norm;
    q3 = q3_pred * norm;

    // 直锟接硷拷锟斤拷Z锟斤拷嵌龋锟斤拷丶锟斤拷薷牡悖�
    Att_Angle->yaw = atan2f(2.0f*(q1*q2 + q0*q3),
                          q0*q0 + q1*q1 - q2*q2 - q3*q3) * RadtoDeg;
}

float gyro_x_pretreatment = 0;
float gyro_y_pretreatment = 0;
float gyro_z_pretreatment = 0;

float roll,pitch;

unsigned char imu660ra_in_once_flag = 0;

float GetFusedZAngle()
{


    gyro_x_pretreatment = imu660ra_gyro_transition(imu660ra_gyro_x)  - gyroXOffset;
    gyro_x_pretreatment = kalmanFilter_1(&KFP_height_1,imu660ra_gyro_transition(imu660ra_gyro_x));                   //锟斤拷锟斤拷锟斤拷锟剿诧拷
    gyro_x_pretreatment = (float)((int)gyro_x_pretreatment);

    gyro_y_pretreatment = imu660ra_gyro_transition(imu660ra_gyro_y)  - gyroYOffset;
    gyro_y_pretreatment = kalmanFilter_1(&KFP_height_2,imu660ra_gyro_transition(imu660ra_gyro_y));                   //锟斤拷锟斤拷锟斤拷锟剿诧拷
    gyro_y_pretreatment = (float)((int)gyro_y_pretreatment);

    gyro_z_pretreatment = imu660ra_gyro_transition(imu660ra_gyro_z)  - gyroZOffset;
    gyro_z_pretreatment = kalmanFilter_1(&KFP_height_3,imu660ra_gyro_transition(imu660ra_gyro_z));                   //锟斤拷锟斤拷锟斤拷锟剿诧拷
    gyro_z_pretreatment = (float)((int)gyro_z_pretreatment);

    // 锟斤拷锟斤拷转锟斤拷锟斤拷使锟斤拷锟斤拷锟皆硷拷锟斤拷锟斤拷锟�
    FLOAT_XYZ acc = {
        .X = imu660ra_acc_transition(imu660ra_acc_x) - gyroXOffset_rcc,
        .Y = imu660ra_acc_transition(imu660ra_acc_y) - gyroYOffset_rcc,
        .Z = imu660ra_acc_transition(imu660ra_acc_z) - gyroZOffset_rcc
    };

    FLOAT_XYZ gyro_rad = {
        .X = gyro_x_pretreatment * 0.0174532925f, // 转锟斤拷锟斤拷
        .Y = gyro_y_pretreatment * 0.0174532925f,
        .Z = gyro_z_pretreatment * 0.0174532925f
    };

    if(imu660ra_in_once_flag == 0)
    {
       roll = atan2f(acc.Y, acc.Z);
       pitch = atan2f(-acc.X, sqrtf(acc.Y*acc.Y + acc.Z*acc.Z));
       q0 = cos((double)(roll/2)) * cos((double)(pitch/2));
       q1 = sin((double)(roll/2)) * cos((double)(pitch/2));
       q2 = cos((double)(roll/2)) * sin((double)(pitch/2));
       q3 = sin((double)(roll/2)) * sin((double)(pitch/2));
      imu660ra_in_once_flag = 1;
    }


    FLOAT_ANGLE angle;
    IMUupdate(&gyro_rad, &acc, &angle);


    lv_gyro_z = gyro_rad.Z;

    return angle.yaw; // 直锟接凤拷锟斤拷锟斤拷元锟斤拷锟斤拷锟斤拷锟絑锟斤拷嵌锟�
}




//=================== 锟角讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 ====================
typedef struct {
    float total_angle;   // 锟桔计角度ｏ拷锟斤拷锟斤拷锟斤拷
    float prev_raw;      // 前一锟斤拷原始锟角讹拷
    uint8_t is_first;    // 锟阶次碉拷锟矫憋拷志
} AngleUnwrapper;

float UnwrapAngle(AngleUnwrapper* handler, float current_angle)
{
    if(handler->is_first)
    {
        handler->total_angle = current_angle;
        handler->prev_raw = current_angle;
        handler->is_first = 0;
        return handler->total_angle;
    }

    // 锟斤拷锟斤拷原始锟角度变化锟斤拷
    float delta = current_angle - handler->prev_raw;

    // 锟角讹拷锟斤拷锟戒补锟斤拷锟斤拷锟斤拷锟斤拷锟姐法锟斤拷
    if(delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    // 锟桔硷拷锟斤拷效锟戒化锟斤拷
    handler->total_angle += delta;
    handler->prev_raw = current_angle;

    return handler->total_angle;
}



AngleUnwrapper z_angle_handler = {.is_first=1};

float GetContinuousZAngle(void)
{
    // 1. 锟斤拷取原始Z锟斤拷嵌龋锟斤拷锟斤拷锟斤拷锟皆拷锟斤拷锟�
    float raw_angle = GetFusedZAngle();

    // 2. 锟斤拷锟叫角讹拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    return UnwrapAngle(&z_angle_handler, raw_angle);
}

static float yawSoftwareOffset = 0.0f;

// 锟斤拷锟姐函锟斤拷
void ResetYawZero(void) {
    yawSoftwareOffset = GetContinuousZAngle();  // 锟斤拷取锟斤拷前锟桔伙拷锟角讹拷
}

// 锟斤拷偏锟狡的角度伙拷取
float GetCurrentYaw(void) {
    return GetContinuousZAngle() - yawSoftwareOffset;
}




//void get_gyro(void)
//{
//    imu660ra_get_gyro();
//    static int16_t imu660ra_gyro_z_last;
//    static int16_t imu660ra_gyro_z_low;
//    imu660ra_gyro_z_low = (int)(0.09*(imu660ra_gyro_z) + 0.91* (imu660ra_gyro_z_last));
//    imu660ra_gyro_z_last=imu660ra_gyro_z_low;
//    
////
////    if(imu660ra_gyro_z>2500)imu660ra_gyro_z=2500;
////    else if(imu660ra_gyro_z<-2500)imu660ra_gyro_z=-2500;
//
//    ang_z=(float)(imu660ra_gyro_transition((float)imu660ra_gyro_z_low- offset_)*(float)PI/180);
//}

void GyroResolve(void)
{
    
       

        if(GyroOn)
        {
            current_yaw = GetCurrentYaw();
        }
  
    

}




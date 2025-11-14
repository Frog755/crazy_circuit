#include "Gyro.h"

int32_t GyroOn=1;//�Ƿ����
uint8 GyroINT=1;//��ʱû��
float ang_z=0;//Z����ٶ�
float offset_=0.0;//1mƫ��ֵ
float current_yaw;
///**************************ȥ��Ư*********************************************************************
float CALIBRATION_SAMPLES = 50 * 2;
float gyroXOffset = 0;
float gyroXOffset_rcc = 0;

float gyroYOffset = 0;
float gyroYOffset_rcc = 0;

float gyroZOffset = 0;
float gyroZOffset_rcc = 0;

// ���ٶ�Z����ƫУ׼

float calibratedGyro_Z1=0;
 static float zGyroBias = 0.0f;
    static uint32_t zGyroCalibCount = 0;
    static float sum = 0.0f;
    uint8_t calibrationComplete = 0;
#define CALIBRATION_SAMPLES 80

float calibrateZ_Gyro(float rawZRate) {
   
  float sum = 0.0f;
    
    // ��ʾ�û����ִ�������ֹ
    // ���磺ͨ��LED��˸�򴮿������ʾ
    
    // �ɼ�ָ�����������������
    for (uint32_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += rawZRate;
       // delay_ms(10); // �ȴ�10ms�����Ʋ���Ƶ��
    }
    
    // ����ƽ����ƫֵ
    return sum / CALIBRATION_SAMPLES;
}    
void calibrateGyro() {          //�����ǳ�ʼ��У׼

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

///**************************�������˲�*********************************************************************

typedef struct
{
    float LastP;//�ϴι���Э����
    float Now_P;//��ǰ����Э����
    float out;//�������˲������
    float Kg;//����������
    float Q;//��������Э����
    float R;//�۲�����Э����
}KFP;//Kalman Filter parameter

//KFP KFP_height_1={0.02,0,0,0,0.1,0.1};
#define GRAVITY 9.81  // �������ٶ� (m/s2)
KFP KFP_height_1={0.02,0,0,0,0.001,0.543};
KFP KFP_height_2={0.02,0,0,0,0.001,0.543};
KFP KFP_height_3={0.02,0,0,0,0.001,0.543};

float kalmanFilter_1(KFP *kfp,float input)           //�������˲�
{
     float really_jiaodu = input ;
     //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
     kfp->Now_P = kfp->LastP + kfp->Q;
     //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
     kfp->out = kfp->out + kfp->Kg * (really_jiaodu -kfp->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
     //����Э�����: ���ε�ϵͳЭ����� kfp->LastP ����һ������׼����
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
}

///**************************��Ԫ�ػ���۽�*********************************************************************
//������: ���ټ��� 1/Sqrt(x)
//��  ����Ҫ�����ֵ
//����ֵ�����
//��  ע������ͨSqrt()����Ҫ���ı�
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

//=============== ���Ͷ��壨���������貹�䣩 ==============
typedef struct {
    float X;
    float Y;
    float Z;
} FLOAT_XYZ;

typedef struct {
    float yaw;   // Z��ƫ���ǣ�ֱ��������Ԫ����
    float pit;
    float rol;
} FLOAT_ANGLE;

//=============== �㷨�����޸� ================

//��������Ԫ����ʼ��Ϊ q0=1, q1=q2=q3=0������ת״̬��

#define Kp 3.0f    // ��������ٶ�
#define Ki 0.001f  // ���ͻ���Ӱ��
#define halfT 0.001f//2ms
#define RadtoDeg 57.295779513f

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

float lv_gyro_z;

void IMUupdate(FLOAT_XYZ *Gyr_rad, FLOAT_XYZ *Acc_filt, FLOAT_ANGLE *Att_Angle)
{
    // ��������Ԥ�������������ԭʼ������
    float ax = Acc_filt->X;
    float ay = Acc_filt->Y;
    float az = Acc_filt->Z;
    float gx = Gyr_rad->X;  // ȷ���Ѿ��ǻ��ȵ�λ
    float gy = Gyr_rad->Y;
    float gz = Gyr_rad->Z;

    // ���ٶȼƹ�һ��
    float norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // ��Ԫ��Ԥ��
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // ����
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    exInt += ex * Ki;
    eyInt += ey * Ki;
    ezInt += ez * Ki;

    // ���ٶȲ���
    gx += Kp*ex + exInt;
    gy += Kp*ey + eyInt;
    gz += Kp*ez + ezInt;

    // ��Ԫ������
    float q0_pred = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    float q1_pred = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    float q2_pred = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    float q3_pred = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // ��Ԫ����һ��
    norm = invSqrt(q0_pred*q0_pred + q1_pred*q1_pred + q2_pred*q2_pred + q3_pred*q3_pred);
    q0 = q0_pred * norm;
    q1 = q1_pred * norm;
    q2 = q2_pred * norm;
    q3 = q3_pred * norm;

    // ֱ�Ӽ���Z��Ƕȣ��ؼ��޸ĵ㣩
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
    gyro_x_pretreatment = kalmanFilter_1(&KFP_height_1,imu660ra_gyro_transition(imu660ra_gyro_x));                   //�������˲�
    gyro_x_pretreatment = (float)((int)gyro_x_pretreatment);

    gyro_y_pretreatment = imu660ra_gyro_transition(imu660ra_gyro_y)  - gyroYOffset;
    gyro_y_pretreatment = kalmanFilter_1(&KFP_height_2,imu660ra_gyro_transition(imu660ra_gyro_y));                   //�������˲�
    gyro_y_pretreatment = (float)((int)gyro_y_pretreatment);

    gyro_z_pretreatment = imu660ra_gyro_transition(imu660ra_gyro_z)  - gyroZOffset;
    gyro_z_pretreatment = kalmanFilter_1(&KFP_height_3,imu660ra_gyro_transition(imu660ra_gyro_z));                   //�������˲�
    gyro_z_pretreatment = (float)((int)gyro_z_pretreatment);

    // ����ת����ʹ�����ԭʼ������
    FLOAT_XYZ acc = {
        .X = imu660ra_acc_transition(imu660ra_acc_x) - gyroXOffset_rcc,
        .Y = imu660ra_acc_transition(imu660ra_acc_y) - gyroYOffset_rcc,
        .Z = imu660ra_acc_transition(imu660ra_acc_z) - gyroZOffset_rcc
    };

    FLOAT_XYZ gyro_rad = {
        .X = gyro_x_pretreatment * 0.0174532925f, // ת����
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

    return angle.yaw; // ֱ�ӷ�����Ԫ�������Z��Ƕ�
}




//=================== �Ƕ����������� ====================
typedef struct {
    float total_angle;   // �ۼƽǶȣ�������
    float prev_raw;      // ǰһ��ԭʼ�Ƕ�
    uint8_t is_first;    // �״ε��ñ�־
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

    // ����ԭʼ�Ƕȱ仯��
    float delta = current_angle - handler->prev_raw;

    // �Ƕ����䲹���������㷨��
    if(delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    // �ۼ���Ч�仯��
    handler->total_angle += delta;
    handler->prev_raw = current_angle;

    return handler->total_angle;
}



AngleUnwrapper z_angle_handler = {.is_first=1};

float GetContinuousZAngle(void)
{
    // 1. ��ȡԭʼZ��Ƕȣ�������Ԫ����
    float raw_angle = GetFusedZAngle();

    // 2. ���нǶ�����������
    return UnwrapAngle(&z_angle_handler, raw_angle);
}

static float yawSoftwareOffset = 0.0f;

// ���㺯��
void ResetYawZero(void) {
    yawSoftwareOffset = GetContinuousZAngle();  // ��ȡ��ǰ�ۻ��Ƕ�
}

// ��ƫ�ƵĽǶȻ�ȡ
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




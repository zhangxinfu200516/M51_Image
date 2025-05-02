#ifndef CRT_IMAGE_H
#define CRT_IMAGE_H

#include "dvc_djimotor.h"
#include "dvc_lkmotor.h"
#include "alg_fsm.h"

class Class_Image;

enum Enum_Image_Control_Type :uint8_t
{
    Image_Control_Type_DISABLE = 0,
    Image_Control_Type_CALIBRATE,//У׼ģʽ
    Image_Control_Type_NORMAL,//����ģʽ
};
class Class_FSM_Image_Control : public Class_FSM
{
public:
    Class_Image *Image;
    void Reload_TIM_Status_PeriodElapsedCallback();
};


class Class_Image
{
public:
    //ͼ��roll����
    Class_DJI_Motor_C610 Motor_Image_Roll;
    //ͼ��pitch����
    Class_DJI_Motor_C610 Motor_Image_Pitch;
    //ͼ������״̬��
    Class_FSM_Image_Control FSM_Image_Control;
    friend class Class_FSM_Image_Control;
    //��������
    void Init();
    inline float Get_Target_Image_Roll_Angle();
    inline float Get_Target_Image_Pitch_Angle();
    inline float get_Image_Roll_Calibrate_Speed();
    inline float get_Image_Pitch_Calibrate_Offset();
    inline float get_Image_Roll_Calibrate_Offset();
    inline void Set_Target_Image_Roll_Angle(float __Target_Image_Roll_Angle);
    inline void Set_Target_Image_Pitch_Angle(float __Target_Image_Pitch_Angle);
    inline void Set_Image_Control_Type(Enum_Image_Control_Type __Image_Control_Type);
    inline void Set_Image_Roll_Calibrate_Offset(float __Image_Roll_Calibrate_Offset);
    inline void Set_Image_Pitch_Calibrate_Offset(float __Image_Pitch_Calibrate_Offset);
    inline void Set_Set_Image_Roll_Calibrate_Speed(float __Image_Roll_Calibrate_Speed);
    bool Motor_Calibration(Class_DJI_Motor_C610 *motor,float *Cali_Offset,float Cali_Omega,float Cali_Max_Out);
    void TIM_Calculate_PeriodElapsedCallback();
protected:

    //ͼ������ģʽ
    Enum_Image_Control_Type Image_Control_Type = Image_Control_Type_DISABLE;
     //ͼ��roll��Ƕ�
    float Target_Image_Roll_Angle = 0.0f;
    //ͼ��pitch��Ƕ�
    float Target_Image_Pitch_Angle = 0.0f;//����
    //ͼ��roll��У׼�ٶ�rad/s
    float Image_Roll_Calibrate_Speed = 3.5f;
    //ͼ��pitch��У׼�ٶ�rad/s 
    float Image_Pitch_Calibrate_Speed = -5.0f;
    //ͼ��roll��У׼������λ��
    float Image_Roll_Calibrate_Offset = 0.0f;
    //ͼ��pitch��У׼������λ��
    float Image_Pitch_Calibrate_Offset = 0.0f;
    //ͼ��roll��У׼�ٽ�����
    float Image_Roll_Calibrate_Stiffness = 810.0f;//510.0f;
    //ͼ��pitch��У׼�ٽ�����
    float Image_Pitch_Calibrate_Stiffness = 1900.0f;//4500.0f;
    //�ڲ�����
    void Output();
};

/**
 * @brief ��ȡͼ��roll��Ƕ�
 *
 */
float Class_Image::Get_Target_Image_Roll_Angle()
{
    return (Target_Image_Roll_Angle);
}
/**
 *   @brief ��ȡͼ��pitch��Ƕ�
 *
 */
float Class_Image::Get_Target_Image_Pitch_Angle()
{
    return (Target_Image_Pitch_Angle);
}
/**
 * @brief ��ȡͼ��roll��У׼�ٶ�
 */
float Class_Image::get_Image_Roll_Calibrate_Speed()
{
    return (Image_Roll_Calibrate_Speed);
}
/**
 * @brief ��ȡͼ��roll��У׼������λ��
 * 
 */
float Class_Image::get_Image_Pitch_Calibrate_Offset()
{
    return (Image_Pitch_Calibrate_Offset);
}
/**
 * @brief ��ȡͼ��roll��У׼������λ��
 * 
 */
float Class_Image::get_Image_Roll_Calibrate_Offset()
{
    return (Image_Roll_Calibrate_Offset);
}
/**
 * @brief �趨ͼ��roll��Ƕ�
 */
void Class_Image::Set_Target_Image_Roll_Angle(float __Target_Image_Roll_Angle)
{
    Target_Image_Roll_Angle = __Target_Image_Roll_Angle;
}
/**
 * @brief �趨ͼ��pitch��Ƕ�
 * 
 */
void Class_Image::Set_Target_Image_Pitch_Angle(float __Target_Image_Pitch_Angle)
{
    Target_Image_Pitch_Angle = __Target_Image_Pitch_Angle;
}
/**
 * @brief �趨ͼ������ģʽ
 * 
 */
void Class_Image::Set_Image_Control_Type(Enum_Image_Control_Type __Image_Control_Type)
{
    Image_Control_Type = __Image_Control_Type;
}
/**
 * @brief �趨ͼ��roll��У׼������λ��
 * 
 */
void Class_Image::Set_Image_Roll_Calibrate_Offset(float __Image_Roll_Calibrate_Offset)
{
    Image_Roll_Calibrate_Offset = __Image_Roll_Calibrate_Offset;
}

/**
 * @brief �趨ͼ��roll��У׼������λ��
 * 
 */
void Class_Image::Set_Image_Pitch_Calibrate_Offset(float __Image_Pitch_Calibrate_Offset)
{
    Image_Pitch_Calibrate_Offset = __Image_Pitch_Calibrate_Offset;
}

/**
 * @brief �趨ͼ��roll��У׼�ٶ�
 */
void Class_Image::Set_Set_Image_Roll_Calibrate_Speed(float __Image_Roll_Calibrate_Speed)
{
    Image_Roll_Calibrate_Speed = __Image_Roll_Calibrate_Speed;
}
#endif

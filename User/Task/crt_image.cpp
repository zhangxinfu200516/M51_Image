#include "crt_image.h"

float target_roll = 0.0f;
float target_pitch = 0.0f;
uint8_t Cali_Flag[2] = {0, 0};
void Class_FSM_Image_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    // static uint8_t Cali_Flag[2] = {0, 0};
    Status[Now_Status_Serial].Time++;
    switch (Now_Status_Serial)
    {
    case (0): // У׼��ʼ״̬
    {

        if (Image->Motor_Image_Pitch.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && Cali_Flag[0] == 0)
        {
            Cali_Flag[0] = Image->Motor_Calibration(&Image->Motor_Image_Pitch,
                                                    &Image->Image_Pitch_Calibrate_Offset,
                                                    Image->Image_Pitch_Calibrate_Speed,
                                                    Image->Image_Pitch_Calibrate_Stiffness);
        }
        if (Image->Motor_Image_Roll.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && Cali_Flag[1] == 0)
        {

            Cali_Flag[1] = Image->Motor_Calibration(&Image->Motor_Image_Roll,
                                                    &Image->Image_Roll_Calibrate_Offset,
                                                    Image->Image_Roll_Calibrate_Speed,
                                                    Image->Image_Roll_Calibrate_Stiffness);
        }

        if (Cali_Flag[0] && Cali_Flag[1])
            Set_Status(1);
    }
    break;
    case (1)://����״̬
    {
        Image->Set_Image_Control_Type(Image_Control_Type_NORMAL);
        Image->Output();
        if (Image->Motor_Image_Roll.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE ||
            Image->Motor_Image_Pitch.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
        {
            Set_Status(0);//���ʧ�ܺ����У׼
            Cali_Flag[0] = 0;
            Cali_Flag[1] = 0;
        }
        
    }
    break;
    }
}

bool  Class_Image::Motor_Calibration(Class_DJI_Motor_C610 *motor,float *Cali_Offset,float Cali_Omega,float Cali_Max_Out)
{
    //��¼�����תʱ��
	static uint16_t count = 0;
	//����Ϊ�ٶȻ�У׼
	motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
	motor->Set_Target_Omega_Radian(Cali_Omega);
	//������ֵ������ֵ��ͬʱ�ٶ�С��һ����ֵ���ж�Ϊ��ת����
	if( (fabs(motor->Get_Now_Torque()) >= Cali_Max_Out) && (fabs(motor->Get_Now_Omega_Radian()) < 0.01f*PI) )
	{
		count++;
		//������һ��ʱ�䣬�ж�Ϊ��ת
		if(count >= 50)
		{
			count = 0;
			//��Ϊ�ǶȻ������ùؽڽǶ�
			motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
			//�ֱ��¼2006����λ�Ƕ�
			*Cali_Offset = motor->Get_Now_Radian();
            //���õ����Ŀ��Ƕ�
            motor->Set_Target_Radian(*Cali_Offset);
			return true;
		}	
	}
	else
	{
		count=0;
	}
	return false;
}

void Class_Image::Init()
{
    FSM_Image_Control.Init(2,0);
    FSM_Image_Control.Image = this;
     //ͼ��roll����
    Motor_Image_Roll.PID_Angle.Init(125.0f, 1.5f, 5.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Roll.PID_Omega.Init(250.0f, 0.5f, 0.0f, 0.0f, 500.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Roll.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
    //ͼ��pitch����
    Motor_Image_Pitch.PID_Angle.Init(100.0f, 1.5f, 5.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Pitch.PID_Omega.Init(300.0f, 50.0f, 0.0f, 0.0f, 500.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Image_Pitch.Init(&hcan1, DJI_Motor_ID_0x206,DJI_Motor_Control_Method_ANGLE);
	
//	 Motor_Image_Roll.PID_Angle.Init(250.0f, 0.0f, 5.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
//    Motor_Image_Roll.PID_Omega.Init(250.0f, 0.0f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
//    Motor_Image_Roll.Init(&hcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
//    //ͼ��pitch����
//    Motor_Image_Pitch.PID_Angle.Init(50.0f, 1.5f, 2.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
//    Motor_Image_Pitch.PID_Omega.Init(500.0f, 200.0f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
//    Motor_Image_Pitch.Init(&hcan1, DJI_Motor_ID_0x206,DJI_Motor_Control_Method_ANGLE);
}

void Class_Image::Output()
{
    switch (Image_Control_Type)
    {
    case Image_Control_Type_DISABLE:
    {
         //ͼ�����ʧ��
        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Image_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Image_Roll.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Image_Roll.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Image_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Image_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Image_Pitch.Set_Target_Torque(0.0f);
        Motor_Image_Roll.Set_Target_Torque(0.0f);
    }
    break;
    case Image_Control_Type_CALIBRATE:
    {
        Motor_Calibration(&Motor_Image_Roll,&Image_Roll_Calibrate_Offset,Image_Roll_Calibrate_Speed,Image_Roll_Calibrate_Stiffness);
        Motor_Calibration(&Motor_Image_Pitch,&Image_Pitch_Calibrate_Offset,Image_Pitch_Calibrate_Speed,Image_Pitch_Calibrate_Stiffness);
    }
    break;
    case Image_Control_Type_NORMAL:
    {
        Motor_Image_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Image_Pitch.Set_Target_Radian(Target_Image_Pitch_Angle/180.0f*PI + Image_Pitch_Calibrate_Offset);
        Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Image_Roll.Set_Target_Radian(Target_Image_Roll_Angle/180.0f*PI + Image_Roll_Calibrate_Offset);
    }
    break;
    }
}

void Class_Image::TIM_Calculate_PeriodElapsedCallback()
{
    FSM_Image_Control.Reload_TIM_Status_PeriodElapsedCallback();
    

    Motor_Image_Pitch.TIM_PID_PeriodElapsedCallback();

    // Motor_Image_Roll.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
    // Motor_Image_Roll.Set_Target_Torque(0.0f);
	Motor_Image_Roll.TIM_PID_PeriodElapsedCallback();
}
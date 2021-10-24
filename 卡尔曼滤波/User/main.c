#include "stm32f10x.h"
#include <stdio.h>
#include "delay.h"
#include "sys.h"
#include "Usart.h"
#include "MPU6050.h"
//******�ǶȲ���************
float Gyro_y;        //Y�������������ݴ�
float Angle_gy;      //�ɽ��ٶȼ������б�Ƕ�
float Accel_x;	     //X����ٶ�ֵ�ݴ�
float Angle_ax;      //�ɼ��ٶȼ������б�Ƕ�
float Angle;         //���ղ����Ƕ�
void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//����һ����ʱ���ṹ�����

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��2
  
  TIM_TimeBaseStructure.TIM_Period = (10000 - 1);	//����100�Σ���Ϊ��0��ʼ�����Լ�1
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);	//ʱ��72��Ƶ����Ϊ0����Ƶ�����Լ�1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	// ʹ�õĲ���Ƶ��֮��ķ�Ƶ����
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//���ϼ���
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//��ʼ����ʱ��2

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		//�����ʱ��2�жϱ�־λ
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		//�򿪶�ʱ��2�ж�

  TIM_Cmd(TIM2, ENABLE);  //������ʹ�ܣ���ʼ����
}

void NVIC_Configuration(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

/********************ע�ᶨʱ��2�жϴ�����***********************/

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

/******************ע�ᴮ��1�жϷ�����************************/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//���ô����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//ռ��ʽ���ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //�����ȼ�����Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//�жϽ�ֹ
	NVIC_Init(&NVIC_InitStructure);//�жϳ�ʼ��


}

/*************�������˲�*********************************/
void Kalman_Filter(float Accel,float Gyro)		
{
		
	static const float Q_angle=0.001;  
	static const float Q_gyro=0.003;
	static const float R_angle=0.5;
	static const float dt=0.01;	                  //dtΪkalman�˲�������ʱ��;
	static const char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	
	Angle+=(Gyro - Q_bias) * dt; //�������

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	Gyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}
void Angle_Calculate(void)
{
	static	uint8_t DataBuffer[2];	//���ݻ���
/****************************���ٶ�****************************************/
	I2C_ReadBuffer(DataBuffer, ACCEL_XOUT_H, 2);
	Accel_x  = (short)((DataBuffer[0]<<8)+DataBuffer[1]);	  //��ȡX����ٶ�
	Angle_ax = (Accel_x +220) /16384;   //ȥ�����ƫ��,����õ��Ƕȣ����ȣ�
	Angle_ax = Angle_ax*1.2*180/3.14;     //����ת��Ϊ��,

/****************************���ٶ�****************************************/
	I2C_ReadBuffer(DataBuffer, GYRO_YOUT_H, 2);
	Gyro_y = (short)((DataBuffer[0]<<8)+DataBuffer[1]);	      //��ֹʱ���ٶ�Y�����Ϊ-18����
	Gyro_y = (Gyro_y + 18)/16.4;         //ȥ�����ƫ�ƣ�������ٶ�ֵ 
//		Angle_gy = Angle_gy + Gyro_y*0.01;  //���ٶȻ��ֵõ���б�Ƕ�.	

/***************************�������˲�+�Ƕ��ں�*************************************/
	Kalman_Filter(Angle_ax,Gyro_y);       //�������˲��������

/*******************************�����˲�******************************************/

/*����ԭ����ȡ��ǰ��Ǻͼ��ٶȻ�
	����ǲ�ֵ���зŴ�Ȼ��������
	�ǽ��ٶȵ��Ӻ��ٻ��֣��Ӷ�ʹ��
	�������Ϊ���ٶȻ�õĽǶ�0.5
	Ϊ�Ŵ������ɵ��ڲ�����;
	0.01Ϊϵͳ����10ms	
*/	
//	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
															  		
}
int main(void)
{
	Stm32_Clock_Init(6);//ϵͳʱ������Ϊ�ⲿ����9��Ƶ
	delay_init(72);//ϵͳSysTick��ʼ��
	Usart_Configuration();
	TIM2_Configuration();
	NVIC_Configuration();
	I2C_Congiguration();
	MPU6050_Init();
	while (1)
	{
		printf("%f	%f	",Angle,Gyro_y);
		USART1_Send_Enter();	
		delay_ms(100);
	}
}
/****************��ʱ��2�жϷ�����***************************************/
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		Angle_Calculate();
	}
}

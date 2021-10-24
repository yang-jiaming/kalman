#include "stm32f10x.h"
#include <stdio.h>
#include "delay.h"
#include "sys.h"
#include "Usart.h"
#include "MPU6050.h"
//******角度参数************
float Gyro_y;        //Y轴陀螺仪数据暂存
float Angle_gy;      //由角速度计算的倾斜角度
float Accel_x;	     //X轴加速度值暂存
float Angle_ax;      //由加速度计算的倾斜角度
float Angle;         //最终测量角度
void TIM2_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//定义一个定时器结构体变量

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器2
  
  TIM_TimeBaseStructure.TIM_Period = (10000 - 1);	//计数100次，因为从0开始，所以减1
  TIM_TimeBaseStructure.TIM_Prescaler = (72 - 1);	//时钟72分频，因为0不分频，所以减1
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	// 使用的采样频率之间的分频比例
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//向上计数
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//初始化定时器2

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		//清除定时器2中断标志位
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);		//打开定时器2中断

  TIM_Cmd(TIM2, ENABLE);  //计数器使能，开始计数
}

void NVIC_Configuration(void)
{ 
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

/********************注册定时器2中断处理函数***********************/

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

/******************注册串口1中断服务函数************************/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//配置串口中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//占先式优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //副优先级设置为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//中断禁止
	NVIC_Init(&NVIC_InitStructure);//中断初始化


}

/*************卡尔曼滤波*********************************/
void Kalman_Filter(float Accel,float Gyro)		
{
		
	static const float Q_angle=0.001;  
	static const float Q_gyro=0.003;
	static const float R_angle=0.5;
	static const float dt=0.01;	                  //dt为kalman滤波器采样时间;
	static const char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	
	Angle+=(Gyro - Q_bias) * dt; //先验估计

	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}
void Angle_Calculate(void)
{
	static	uint8_t DataBuffer[2];	//数据缓存
/****************************加速度****************************************/
	I2C_ReadBuffer(DataBuffer, ACCEL_XOUT_H, 2);
	Accel_x  = (short)((DataBuffer[0]<<8)+DataBuffer[1]);	  //读取X轴加速度
	Angle_ax = (Accel_x +220) /16384;   //去除零点偏移,计算得到角度（弧度）
	Angle_ax = Angle_ax*1.2*180/3.14;     //弧度转换为度,

/****************************角速度****************************************/
	I2C_ReadBuffer(DataBuffer, GYRO_YOUT_H, 2);
	Gyro_y = (short)((DataBuffer[0]<<8)+DataBuffer[1]);	      //静止时角速度Y轴输出为-18左右
	Gyro_y = (Gyro_y + 18)/16.4;         //去除零点偏移，计算角速度值 
//		Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度积分得到倾斜角度.	

/***************************卡尔曼滤波+角度融合*************************************/
	Kalman_Filter(Angle_ax,Gyro_y);       //卡尔曼滤波计算倾角

/*******************************互补滤波******************************************/

/*补偿原理是取当前倾角和加速度获
	得倾角差值进行放大，然后与陀螺
	仪角速度叠加后再积分，从而使倾
	角最跟踪为加速度获得的角度0.5
	为放大倍数，可调节补偿度;
	0.01为系统周期10ms	
*/	
//	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
															  		
}
int main(void)
{
	Stm32_Clock_Init(6);//系统时钟设置为外部晶振，9倍频
	delay_init(72);//系统SysTick初始化
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
/****************定时器2中断服务函数***************************************/
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		Angle_Calculate();
	}
}

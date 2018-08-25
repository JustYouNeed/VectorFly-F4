/**
  *******************************************************************************************************
  * File Name: filter.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ������˸����˲�����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	

# ifndef __FILTER_H
# define __FILTER_H

# define IIR_SHIFT	8

typedef struct 
{
	float a1, a2;
	float b0, b1, b2;
	float delay_e1, delay_e2;
} lpf2pData_t;

/*  һά�������˲��ṹ��  */
typedef struct 
{
	double Q;
	double R;
	double P;
	double Kg;
	double Output;
}Kalman1Dim_TypeDef;


typedef struct
{
	double X[2];			/*  ��������״̬X,������̬������˵,X[0]������̬��  */
	int16_t Gyro;			/*  ���Ž��ٶ�  */
	double A[2][2];		/*  AΪϵͳΪk-1��kʱ�̵�״̬ת�ƾ���  */
	double B[2];			/*  BΪϵͳ����  */
	double P[2][2];		/*  PΪϵͳЭ����  */
	double Q[2][2];		/*  QΪϵͳ����  */
	double R;					/*  ��������  */
	double Kg[2];			/*  KgΪϵͳ����  */	
	double dt;				/*  �˲�������ʱ��  */
}Kalman_TypeDef;

void filter_SildingAverage(uint16_t Array[], uint16_t *Average, uint16_t Length);
void filter_Kalman1Dim_Init(Kalman1Dim_TypeDef *Kalman_struct, double Q, double R);
void filter_Kalman1Dim(Kalman1Dim_TypeDef *Kalam_Struct, double input);
void filter_KanlmanInit(Kalman_TypeDef *Kalman);
void filter_KalmanFilter(Kalman_TypeDef *Kalman, double Gyro, double AccAngle);

void filter_LPF2P_Init(lpf2pData_t *lpfData, float sample_freq, float cutoff_freq);
void filter_LPF2P_SetCutoffFreq(lpf2pData_t *lpfData, float sample_freq, float cutoff_freq);
float filter_LPF2P_Filter(lpf2pData_t *lpfData, float sample);
float filter_LPF2P_Reset(lpf2pData_t *lpfData, float sample);
# endif

/********************************************  END OF FILE  *******************************************/


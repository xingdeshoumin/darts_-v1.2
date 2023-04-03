/******************************************************************************/
/** @file pid.c
 *  @version 1.0
 *  @date 2018.12.8
 *
 *  @brief the declerations of abs/inc pid functions 
 *
 *  @author 
 *
 */
 /*****************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "pid.h"
#include "string.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



/* Private functions ---------------------------------------------------------*/
/*********************����ʽPID����***********************/
void PID_IncrementMode(s_pid_increase_t *pid)
{
	 if(pid->kp<0) pid->kp=-pid->kp;
	 if(pid->ki<0) pid->ki=-pid->ki;
	 if(pid->kd<0) pid->kd=-pid->kd;
	
	 if(pid->errNow >5 && pid->errNow<-5)pid->errNow=0;

	 pid->dErrP=pid->errNow-pid->errOld1;
	 pid->dErrI=pid->errNow;
	 pid->dErrD=pid->errNow-2*pid->errOld1+pid->errOld2;
	
	 pid->errOld2=pid->errOld1;
	 pid->errOld1=pid->errNow;
	
	 pid->dCtrOut=pid->kp*pid->dErrP+pid->ki*pid->dErrI+pid->kd*pid->dErrD;
	
	 if(pid->dCtrOut>pid->dOutMAX) pid->dCtrOut=pid->dOutMAX;
     else if(pid->dCtrOut<-pid->dOutMAX) pid->dCtrOut=-pid->dOutMAX;

	
	 if(pid->kp==0 && pid->ki==0 && pid->kd==0) pid->ctrOut=0;
	 else pid->ctrOut+=pid->dCtrOut;
	 
	 if(pid->ctrOut>pid->OutMAX) pid->ctrOut=pid->OutMAX;
     else if(pid->ctrOut<-pid->OutMAX)
   
	 pid->ctrOut=-pid->OutMAX; 
}
/********************����ʽPID����**************************/
void PID_AbsoluteMode(s_pid_absolute_t *pid)
{
	//����ȡ��
	if( pid->Kp < 0) pid->Kp = -pid->Kp;
	if( pid->Ki < 0) pid->Ki = -pid->Ki;
	if( pid->Kd < 0) pid->Kd = -pid->Kd;
	if( pid->IerrorLim < 0) pid->IerrorLim = -pid->IerrorLim;
    //PID������ƫ��
	pid->Perror = pid->NowError;                  //P����ƫ���ǵ�ǰƫ��  
	pid->Ierror += pid->NowError;                 //I����ƫ�����ϵ��һֱ���������ڵ�ƫ�� 
	pid->Derror = pid->NowError - pid->LastError; //D����ƫ���ǵ�ǰƫ�����ϴ�ƫ��Ĳ�ֵ����ƫ������
	pid->LastError = pid->NowError;               //����ƫ��	
	//���ƻ�����ʷƫ��
	if( pid->Ierror > pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror < -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID�����������
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID�������
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//����PID�������
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief   init the parameters of the absolutely mode pid
 * @param 	PID_AbsoluteType *pid, float kp, float ki, float kd, float errILim, float MaxOutCur		
 * @return None
 * @attention None
 */
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	memset(pid,0,sizeof(s_pid_absolute_t));
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}


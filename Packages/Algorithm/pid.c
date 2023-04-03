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
/*********************增量式PID控制***********************/
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
/********************绝对式PID控制**************************/
void PID_AbsoluteMode(s_pid_absolute_t *pid)
{
	//参数取正
	if( pid->Kp < 0) pid->Kp = -pid->Kp;
	if( pid->Ki < 0) pid->Ki = -pid->Ki;
	if( pid->Kd < 0) pid->Kd = -pid->Kd;
	if( pid->IerrorLim < 0) pid->IerrorLim = -pid->IerrorLim;
    //PID各环节偏差
	pid->Perror = pid->NowError;                  //P环节偏差是当前偏差  
	pid->Ierror += pid->NowError;                 //I环节偏差是上电后一直持续到现在的偏差 
	pid->Derror = pid->NowError - pid->LastError; //D环节偏差是当前偏差与上次偏差的差值，即偏差增量
	pid->LastError = pid->NowError;               //更新偏差	
	//限制积分历史偏差
	if( pid->Ierror > pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror < -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID各环节输出量
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID总输出量
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//限制PID总输出量
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


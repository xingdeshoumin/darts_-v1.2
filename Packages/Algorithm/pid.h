#ifndef __PID_H__
#define __PID_H__

/********increase mode*******/
typedef struct 
{
    float kp;
	float ki;
	float kd;
	
	float dErrP;
	float dErrI;
	float dErrD;
	
	float errNow;
	float errOld1;
	float errOld2;

	float dCtrOut;
	float dOutMAX;
	float ctrOut;
	float OutMAX;
	
}s_pid_increase_t;//增量式
/********absolute mode*******/
typedef struct
{
    float Kp;//比例环节参数
	float Ki;//积分环节参数
	float Kd;//微分环节参数

	float Perror;//比例偏差值
	float Ierror;//积分偏差值
	float Derror;//微分偏差值
	
	float Pout;//比例环节输出量
	float Iout;//积分环节输出量
	float Dout;//微分环节输出量
	
	float NowError;   //当前偏差
	float LastError;  //上次偏差
	float IerrorLim;  //积分偏差上限
	float PIDout;     //PID运算后输出量
	float PIDoutMAX;  //PID运算后输出量上限
}s_pid_absolute_t; //绝对式

void PID_IncrementMode(s_pid_increase_t *pid);
void PID_AbsoluteMode(s_pid_absolute_t *pid);
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur);

#endif

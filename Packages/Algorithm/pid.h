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
	
}s_pid_increase_t;//����ʽ
/********absolute mode*******/
typedef struct
{
    float Kp;//�������ڲ���
	float Ki;//���ֻ��ڲ���
	float Kd;//΢�ֻ��ڲ���

	float Perror;//����ƫ��ֵ
	float Ierror;//����ƫ��ֵ
	float Derror;//΢��ƫ��ֵ
	
	float Pout;//�������������
	float Iout;//���ֻ��������
	float Dout;//΢�ֻ��������
	
	float NowError;   //��ǰƫ��
	float LastError;  //�ϴ�ƫ��
	float IerrorLim;  //����ƫ������
	float PIDout;     //PID����������
	float PIDoutMAX;  //PID��������������
}s_pid_absolute_t; //����ʽ

void PID_IncrementMode(s_pid_increase_t *pid);
void PID_AbsoluteMode(s_pid_absolute_t *pid);
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur);

#endif

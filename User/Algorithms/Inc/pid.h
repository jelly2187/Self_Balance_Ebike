//
// Created by Jelly on 2025/8/21.
//

#ifndef BALANCE_EBIKE_PID_H
#define BALANCE_EBIKE_PID_H

#include <stdint.h>
#include "config.h"
#include "lpf.h"


/**
 * @brief macro definition of the VAL_LIMIT that restricts the value of the specified variable.
 * @param x: the specified variable
 * @param min: the minimum value of the specified variable
 * @param max: the maximum value of the specified variable
 * @retval none
 */
#define VAL_LIMIT(x,min,max)  do{ \
if ((x) > (max)) {(x) = (max);} \
else if ((x) < (min)) {(x) = (min);} \
}while(0U)

/**
 * @brief macro definition of the number of pid parameters
*/
#ifndef PID_PARAMETER_NUM
#define PID_PARAMETER_NUM 7
#endif


/**
 * @brief typedef enum that contains the Error status for the pid controller.
 */
typedef enum
{
    PID_ERROR_NONE = 0x00U,        /*!< No error */
    PID_FAILED_INIT = 0x01U,        /*!< Initialization failed */
        PID_CALC_NANINF = 0x02U,      /*!< Not-a-number (NaN) or infinity is generated */
    PID_Status_NUM,
}PID_Status_e;

/**
 * @brief typedef enum that contains the type for the pid controller.
 */
typedef enum
{
    PID_Type_None = 0x00U,         /*!< No Type */
    PID_POSITION = 0x01U,          /*!< position pid */
    PID_VELOCITY = 0x02U,          /*!< velocity pid */
PID_TYPE_NUM,
}PID_Type_e;

/**
 * @brief typedef structure that contains the information for the pid Error handler.
 */
typedef struct
{
    uint16_t ErrorCount;    /*!< Error status judgment count */
    PID_Status_e Status;    /*!< Error Status */
}PID_ErrorHandler_Typedef;

/**
 * @brief typedef structure that contains the parameters for the pid controller.
 */
typedef struct
{
    float KP;             //����ϵ��
    float KI;             //����ϵ��
    float KD;             //΢��ϵ��
    float Alpha;           //΢��һ���˲���ϵ��
    float Deadband;       //���� ��������ֵС������ PIDֹͣ���㡣
    float LimitIntegral;  //�����޷�
    float LimitOutput;    //������޷�
}PID_Parameter_Typedef;

/**
 * @brief typedef structure that contains the information for the pid controller.
 */
typedef struct _PID_TypeDef
{
    PID_Type_e Type;    //PID���� λ��ʽor����ʽ ͨ��ʹ��λ��ʽ

    float Target;       //Ŀ��ֵ
    float Measure;      //ʵ��ֵ

    float Err[3];       //��� Ŀ��ֵ-����ֵ = ��� ��ǰ�Լ���ȥ���ε����
    float Integral;     //������ֵ ����ۼ�
    float Pout;         // KP * ���ֵ �������
    float Iout;         // KI * ������ �������
    float Dout;         // KD * ���΢�֣���֣�΢�����
    float Output;       //����� Pout + Iout + Dout = Output

    LowPassFilter1p_Info_TypeDef Dout_LPF; //΢�������һ���˲���


    PID_Parameter_Typedef Param;            //PID�����ṹ��
    PID_ErrorHandler_Typedef ERRORHandler;  //PID�������ṹ��

    /**
     * @brief ��ʼ��PID�����ĺ���ָ�룬��PID����װ����PID�����ṹ���С�
     * @param PID: ָ��_pid_TypeDef�ṹ��ָ�룬����PID����������Ϣ��
     * @param Param: ָ��PID�����ĸ�����ָ�룬����PID������
     * @retval PID����״̬ ����PID�Ƿ��ʼ���ɹ���
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *PID,float *Param);

    /**
     * @brief ���pid���㺯���ļ��ָ�롣
     * @param PID:ָ��_pid_TypeDef�ṹ��ָ�룬����PID����������Ϣ��
     * @retval ��.
     */
    void (*PID_Calc_Clear)(struct _PID_TypeDef *PID);

}PID_Info_TypeDef;


/**
 * @brief Initializes the PID Controller.
 */
extern void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM]);
/**
  * @brief  Caculate the PID Controller
  */
extern float PID_Calculate(PID_Info_TypeDef *PID, float Target,float Measure);



// PID控制器结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float target; // 目标值
    float feedback; // 反馈值

    float error; // 当前误差
    float last_error; // 上一次误差
    float integral; // 积分项

    float output; // PID计算输出
    float output_max; // 输出限幅
    float integral_max; // 积分限幅
} PID_Controller_t;

void Motor_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float output_max, float integral_max);

float Motor_PID_Calculate(PID_Controller_t *pid, float target, float feedback);

#endif //BALANCE_EBIKE_PID_H

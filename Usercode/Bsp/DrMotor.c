/**************************************************
��Ȼ������-��ת����ʽ��ˢ�ŷ���������

����ƽ̨��STM32ƽ̨
��汾�ţ�v2.1
�������ذ汾��STM32f103c8t6
*************************************************/
#include "DrMotor.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
//#include "stm32F1xx.h"
//#include "usart.h"
#include <stdio.h>
#include "can.h"

#define INPUT_MODE_PASSTHROUGH 1
#define INPUT_MODE_VEL_RAMP 2
#define AXIS_STATE_IDLE 1
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8
#define INPUT_MODE_TORQUE_RAMP  6

//#define SERVO_USART huart1              /* �������ʹ�õĴ��� */
#define SERVO_RECEIVE_TIMEOUT 1000        /* ���ڽ��ճ�ʱ(ms) */
#define ENABLE_INPUT_VALIDITY_CHECK 1   /* ����Ϸ��Լ�顣Ϊ 0 ʱ�������ָ���жϵȼ�飬�ɼ�С����������ӿ촦���ٶȡ� */

#define SERVO_MALLOC(size) malloc(size) /* ʹ�ñ�׼�� malloc����ʹ�� RTOS ���滻Ϊ��Ӧ���亯�� */
#define SERVO_FREE(ptr) free(ptr)       /* ʹ�ñ�׼�� free����ʹ�� RTOS ���滻Ϊ��Ӧ�ͷź��� */
#define SERVO_DELAY(n) HAL_Delay(n)     /* ��ʱ����������Ϊ HAL_Delay�������滻���޸Ĵ˴� */
#define SERVO_SPRINTF sprintf           /* ʹ�����������Ҫ�� Keil �ﹴѡ Use MicroLIB ���� STM32CubeIDE �ﹴѡ Use float with printf from newlib-nano */
//#define PRE_READ() __HAL_UART_FLUSH_DRREGISTER(&SERVO_USART)

char command[64];       // ����ͻ�����
char DaTemp[4];
uint8_t i;
uint8_t rx_buffer[8];
uint16_t can_id = 0x00;
volatile int8_t READ_FLAG = 0;  // ��ȡ�����־λ

int8_t TRAJ_MODE = 1;  //�ٶȹ켣ģʽѡ����ƣ�1��ʾ���ι켣ģʽ��2��ʾS�ι켣ģʽ��ָ�����ٶ�������״��������λ�ÿ���-���ι켣ģʽ��

int8_t enable_replay_state = 1;  //����Ҫ���˶�����ָ��ʵʱ״̬���ع��ܣ��뽫�ñ�����Ϊ1�����������MOTOR_NUM����Ϊ�����ϵ������ID��

float motor_state[MOTOR_NUM][5];    //���״̬��ά���飬ͨ��motor_state[id_num-1]��ȡ���id_num��ʵʱ����״̬[angle,speed,torque,traj_done,axis_error]����λ�ֱ�Ϊdegree��r/min,Nm����������ֵ��ָ���ǵ�������
// # ����motor_state[id_num-1][0]��ʾid_num�ŵ���ĽǶȣ�motor_state[id_num-1][1]��ʾid_num�ŵ�����ٶȣ�motor_state[id_num-1][2]��ʾid_num�ŵ�������Ť��
uint32_t reply_state_error = 0;   // reply_state��������ۻ���־

//cur_angle_list = [];  //��ǰ�Ƕ��б�

//"""
//�ڲ������������û�����ʹ��
//"""

// CAN���ͺ���
void send_command(uint8_t id_num, char cmd, unsigned char *data,uint8_t rt )
{
    short id_list = (id_num << 5) + cmd;
    Can_Send_Msg(id_list, 8, data);
}

void SERVO_DELAY_US(uint32_t us)
{
    /* DWT 周期计数器，72MHz 下 1 周期 = 1/72 μs ≈ 13.9ns */
    static uint8_t dwt_ready = 0;
    if (!dwt_ready)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        dwt_ready = 1;
    }
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks);
}

//CAN���պ���
void receive_data(void)
{
    uint8_t OutTime_mark=0;
    uint32_t  OutTime=0;
    while(OutTime_mark==0)
    {
        if( READ_FLAG ==1) 
        OutTime_mark=1;
        SERVO_DELAY_US(1);
        OutTime++;
        if( OutTime ==0xfffe) 
        {
            OutTime_mark=1;
        }
    }
}


// ���ݸ�ʽת��������decode�ǽ�������(bytes)ת�����˿��Ķ������ݣ�encode��֮
/*
float��short��unsigned short��int��unsigned int��������������byte���͵�ת��
�������ݶ�Ӧ����0,1,2,3,4;
ʹ�÷������ں����е���ʱ��
    ����Ҫ�ڵ���ǰ�Խṹ���ڵ����ݽ��и�ֵ
    value_data[3]�������������͵�����ת��Ϊbyte����ֵ
    byte_data[8]������byteת��Ϊ�������ݵ����ͣ���ֵ
    type_data[3]��������ֵ����Ҫ��
    length�����ݸ�����ֵ����Ҫ��

�����������͵�����ת��Ϊbyte������format_data(data_struct data_list , char * str)��
    ����Ϊ���ṹ��ָ�룬Ҫ���Ĳ��������롰encode����
����byteת��Ϊ�������ݵ����ͣ�����format_data(data_struct data_list , char * str)��
    ����Ϊ���ṹ��ָ�룬Ҫ���Ĳ��������롰decode����

�����������������������������Զ����ã�����Ҫʹ������������
    byte2value()
    value2byte()

type���ͣ�
type_data=0, ��������float��,���ݳ���32λ,���š�f��;
type_data=1, �޷��Ŷ�������unsigned short int��,���ݳ���16λ,���š�u16��;
type_data=2, �з��Ŷ�������short int��,���ݳ���16λ,���š�s16��;
type_data=3, �޷��Ŷ�������unsigned int��,���ݳ���32λ,���š�u32��;
type_data=4, �з��Ŷ�������int��,���ݳ���32λ,���š�s32��;

*/
struct format_data_struct
{
    float value_data[3];//�����������͵�����ת��Ϊbyte����ֵ
    unsigned char byte_data[8];//����byteת��Ϊ�������ݵ����ͣ���ֵ
    int type_data[3];//��������ֵ����Ҫ��
    int length;//���ݸ�����ֵ����Ҫ��
}*data_struct,data_list;  //����ȫ�ֱ��� data_list�����ڽ���CAN����encode��decode�����д洢�������뼰������

// note: the bit order is in accordance with intel little endian
static inline void uint16_to_data(uint16_t val, uint8_t *data)
{
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint16_t data_to_uint16(uint8_t *data)
{
    uint16_t tmp_uint16;
    tmp_uint16 = (((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint16;
}
static inline void uint_to_data(uint32_t val, uint8_t *data)
{
    data[3] = (uint8_t)(val >> 24);
    data[2] = (uint8_t)(val >> 16);
    data[1] = (uint8_t)(val >> 8);
    data[0] = (uint8_t)(val);
}
static inline uint32_t data_to_uint(uint8_t *data)
{
    uint32_t tmp_uint;
    tmp_uint = (((uint32_t)data[3] << 24) + ((uint32_t)data[2] << 16) + ((uint32_t)data[1] << 8)  + ((uint32_t)data[0]));
    return tmp_uint;
}
static inline void int16_to_data(int16_t val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
}
static inline int16_t data_to_int16(uint8_t *data)
{
    int16_t tmp_int16;
    *(((uint8_t*)(&tmp_int16)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int16)) + 1) = data[1];
    return tmp_int16;
}
static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t*)(&tmp_int)) + 0) = data[0];
    *(((uint8_t*)(&tmp_int)) + 1) = data[1];
    *(((uint8_t*)(&tmp_int)) + 2) = data[2];
    *(((uint8_t*)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}
static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t*)(&val)) + 0);
    data[1] = *(((uint8_t*)(&val)) + 1);
    data[2] = *(((uint8_t*)(&val)) + 2);
    data[3] = *(((uint8_t*)(&val)) + 3);
}
static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t*)(&tmp_float)) + 0) = data[0];
    *(((uint8_t*)(&tmp_float)) + 1) = data[1];
    *(((uint8_t*)(&tmp_float)) + 2) = data[2];
    *(((uint8_t*)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

//������������
void byte2value()
{
    int value_index = 0;
    int byte_index = 0;
    while (1)
    {
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            data_list.value_data[value_index] = (float)data_to_float(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 1:
        {
            data_list.value_data[value_index] = (float)data_to_uint16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 2:
        {
            data_list.value_data[value_index] = (float)data_to_int16(&data_list.byte_data[byte_index]);
            byte_index = 2+byte_index;
            value_index++;
        }
        break;
        case 3:
        {
            data_list.value_data[value_index] = (float)data_to_uint(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        case 4:
        {
            data_list.value_data[value_index] = (float)data_to_int(&data_list.byte_data[byte_index]);
            byte_index = 4+byte_index;
            value_index++;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index>=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}
//������������
void value2byte()
{
    int byte_index=0;
    int value_index=0;
    while(1)
    {
        if (data_list.type_data[value_index]==0)
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        switch(data_list.type_data[value_index])
        {
        case 0:
        {
            float_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 1:
        {
            uint16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 2:
        {
            int16_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 2;
            value_index += 1;
        }
        break;
        case 3:
        {
            uint_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        case 4:
        {
            int_to_data(data_list.value_data[value_index],&data_list.byte_data[byte_index]);
            byte_index += 4;
            value_index += 1;
        }
        break;
        default:
            value_index++;
            break;
        }
        if((byte_index >=8)||(value_index>=data_list.length))
        {
            return;
        }
    }
}

//�����ṹ��ָ�룬Ҫ���Ĳ���������typeתbyte���롰encode����byteת����type���롰decode����
void format_data( float *value_data, int *type_data,int length, char * str)
{
    data_list.length=length;
    for (int i = 0; i < length; i++)
    {
        data_list.value_data[i]= value_data[i];
        data_list.type_data[i] = type_data[i];
    }
    if (strcmp(str,"encode")==0)
    {
        value2byte();
    }
    if (strcmp(str,"decode")==0)
    {
        for (int i = 0; i < 8; i++)
        {
            data_list.byte_data[i]=rx_buffer[i];
        }
        byte2value();
    }
}

/**
 * @brief ����˶�����ָ��״̬ʵʱ���ز���
 * ͨ���ú�����ȡ����˶�����ָ��ʵʱ���صĵ��״̬����[angle,speed,torque]����λ�ֱ�Ϊdegree��r/min,Nm����������ֵ��ָ���ǵ�������
 * ����motor_state[id_num-1][0]��ʾid_num�ŵ���ĽǶȣ�motor_state[id_num-1][1]��ʾid_num�ŵ�����ٶȣ�motor_state[id_num-1][2]��ʾid_num�ŵ�������Ť��
 * @param id_num ��Ҫ��ȡ�ĵ�����  ע���ָ��id_num����Ϊ0

 */
void reply_state(uint8_t id_num)
{
    if(enable_replay_state&&id_num<=MOTOR_NUM)   //id_num不能为0
    {
        READ_FLAG=0;
        receive_data();
        if (READ_FLAG == 1)
        {
            uint8_t rx_id = (uint8_t)((can_id & 0x07E0) >> 5) & 0xFF;
            if(id_num==0)  //如果ID号为0，则通过解析CAN帧的ID信息确定ID号
            {
                id_num = rx_id;
            }
            else if(rx_id != 0 && rx_id != id_num)
            {
                /* 收到其他电机的回复，不是请求的电机，丢弃并报错 */
                READ_FLAG = -1;
                reply_state_error += 1;
                return;
            }
            float factor = 0.01f;
            float value_data[3]= {0,0,0};
            int type_data[3]= {0,2,2};
            format_data(value_data,type_data,3,"decode");
            motor_state[id_num-1][0]=data_list.value_data[0];
            motor_state[id_num-1][1]=data_list.value_data[1]*factor;
            motor_state[id_num-1][2]=data_list.value_data[2]*factor;
            motor_state[id_num -1][3] =(int)((can_id & 0x02) >> 1);
            motor_state[id_num -1][4] = (int)((can_id & 0x04) >> 2);
        }
        else
        {
            READ_FLAG=-1;
            reply_state_error += 1;
        }
    }
}

/**
 * @brief ��������Ƕȿ��ƺ�����
 * ����ָ�������ŵĵ������ָ�����ٶ�ת����ָ���ĽǶȣ����ԽǶȣ�����ڵ�����λ�ã���
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param angle ����Ƕȣ�-360~360��*n��֧�ִ�Ƕ�ת��
 * @param t mode=0,�����ã�ֱ�Ӹ�0����; mode=1, �˶�ʱ�䣨s��; mode =2, ǰ���ٶȣ�r/min)
 * @param param mode=0,�Ƕ������˲�������<300����mode=1,������ֹͣ�׶μ��ٶȣ�(r/min)/s��; mode =2, ǰ��Ť�أ�Nm)
 * @param mode �Ƕȿ���ģʽѡ�񣬵��֧�����ֽǶȿ���ģʽ��
 *             mode = 0: �������켣����ģʽ������һ�������˶����ر��ʺ϶���켣�����룬�Ƕ������������������Ϊָ���Ƶ�ʵ�һ�롣
 *             mode = 1: ���������ι켣ģʽ����ʱspeed���˶�ʱ��t��s����ʾ��paramΪĿ����ٶȣ�(r/min)/s����
 *             mode = 2: ǰ������ģʽ������ģʽ�µ�tΪǰ���ٶȣ�paramΪǰ��Ť�ء�ǰ��������ԭ��PID���ƻ����ϼ����ٶȺ�Ť��ǰ�������ϵͳ����Ӧ���Ժͼ��پ�̬��
 * @note ��mode=1,���ι켣ģʽ�У�speed��accel����Ҫ����0.���speed=0�ᵼ�µ����motor error ���˳��ջ�����ģʽ������������ģʽ�����speed=0,�ᱻ��0.01���档
         �����������ģʽ��accel=0�����������ٶ��˶���angle,speed�������������á�
 */
void preset_angle(uint8_t id_num, float angle, float t, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_time = (int)(abs(t) / factor);
        if (param > 300)
            //print("input_filter_width = " + str(param) + ", which is too big and resized to 300")
            param = 300;
        int  s16_width = (int)(abs(param / factor));

        float value_data[3]= {f_angle, s16_time, s16_width};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 1)
    {
        float f_angle = angle;
        int s16_time = (int)(abs(t) / factor);
        int s16_accel = (int)((abs(param)) / factor);

        float value_data[3]= {f_angle,s16_time,s16_accel};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    else if(mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((t) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
        send_command(id_num,0x0C,data_list.byte_data,0);
    }
    reply_state(id_num);
}
/**
 * @brief ��������ٶ�Ԥ�躯����
 * Ԥ��ָ�������ŵĵ����Ŀ���ٶȣ�֮����Ҫ��mvָ������ת����
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param speed Ŀ���ٶȣ�r/min��
 * @param param mode=1, ǰ��Ť�أ�Nm); mode!=1,��Ŀ����ٶȣ�(r/min)/s��
 * @param mode ����ģʽѡ��
 *             mode=1, �ٶ�ǰ������ģʽ�������Ŀ���ٶ�ֱ����Ϊspeed
 *             mode!=1,�ٶ���������ģʽ�����������Ŀ����ٶ�axis0.controller.config_.vel_ramp_rate�仯��speed��
 */
void preset_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01;
    float f_speed = speed;
    if (mode == 1)
    {
        int  s16_torque = (int)((param) / factor);
        if (f_speed == 0)
            s16_torque = 0;
        int s16_input_mode = (int)(INPUT_MODE_PASSTHROUGH / factor);

        float value_data[3]= {f_speed, s16_torque, s16_input_mode};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
    }
    else
    {
        int s16_ramp_rate = (int)((param) / factor);
        int s16_input_mode = (int)(INPUT_MODE_VEL_RAMP / factor);

        float value_data[3]= {f_speed, s16_ramp_rate, s16_input_mode};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");
    }
    send_command(id_num, 0x0C,data_list.byte_data, 0);
    reply_state(id_num);
}

/**
 * @brief �����������Ԥ�躯����
 * Ԥ��ָ�������ŵĵ��Ŀ��Ť�أ�Nm��
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param torque ��������Nm)
 * @param param mode=1,�Ĳ��������壻mode!=1,Ť�������ٶ�axis0.controller.config.torque_ramp_rate��Nm/s��
 * @param mode ����ģʽѡ��
 *             mode=1, Ť��ֱ�ӿ���ģʽ�������Ŀ��Ť��ֱ����Ϊtorque
 *             mode!=1,Ť����������ģʽ�����������Ť����������axis0.controller.config.torque_ramp_rate��Nm/s���仯��torque��
 */
void preset_torque(uint8_t id_num, float torque, float param, int mode)
{

    float factor = 0.01;
    float f_torque = torque;
    int s16_ramp_rate, s16_input_mode;
    if (mode == 1)
    {
        s16_input_mode = (int)(INPUT_MODE_PASSTHROUGH / factor);
        s16_ramp_rate = 0;
    }
    else
    {
        s16_input_mode = (int)(INPUT_MODE_TORQUE_RAMP / factor);
        s16_ramp_rate =  (int)((param) / factor);
    }

    float value_data[3]= {f_torque, s16_ramp_rate, s16_input_mode};
    int type_data[3]= {0,2,2};

    format_data(value_data,type_data,3,"encode");

    send_command(id_num,0x0C,data_list.byte_data,0);
    reply_state(id_num);
}
// ���ܺ������û�ʹ��

// �˶����ƹ���

/**
 * @brief ��������Ƕȿ��ƺ�����
 * ����ָ�������ŵĵ������ָ�����ٶ�ת����ָ���ĽǶȣ����ԽǶȣ�����ڵ�����λ�ã���
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param angle ����Ƕȣ�-360~360��*n��֧�ִ�Ƕ�ת��
 * @param speed ����ٶ����ƻ�ǰ���ٶȣ�r/min��
 * @param param mode=0,�Ƕ������˲�������<300����mode=1,������ֹͣ�׶μ��ٶȣ�(r/min)/s��; mode =2, ǰ��Ť�أ�Nm)
 * @param mode �Ƕȿ���ģʽѡ�񣬵��֧�����ֽǶȿ���ģʽ��
 *             mode = 0: �켣����ģʽ������һ�������˶����ر��ʺ϶���켣�����룬�Ƕ������������������Ϊָ���Ƶ�ʵ�һ�롣
 *             mode = 1: ���ι켣ģʽ������ģʽ�¿���ָ���˶������е��ٶȣ�speed������ͣ���ٶȣ�accel����
 *             mode = 2: ǰ������ģʽ������ģʽ�µ�speed��torque�ֱ�Ϊǰ��������.ǰ��������ԭ��PID���ƻ����ϼ����ٶȺ�Ť��ǰ�������ϵͳ����Ӧ���Ժͼ��پ�̬��
 * @note ��mode=1,���ι켣ģʽ�У�speed��accel����Ҫ����0.���speed=0�ᵼ�µ����motor error ���˳��ջ�����ģʽ������������ģʽ�����speed=0,�ᱻ��0.01���档
 *       �����������ģʽ��accel=0�����������ٶ��˶���angle,speed�������������á�
 */
void set_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    float factor = 0.01;
    if (mode == 0)
    {
        float f_angle = angle;
        int s16_speed = (int)((abs(speed)) / factor);
        if (param > 300)
            param = 300;
        int s16_width = (int)(abs(param / factor));

        float value_data[3]= {f_angle,s16_speed,s16_width};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num,0x19,data_list.byte_data,0);
    }
    else if (mode == 1)
    {
        if ((speed > 0)&&(param > 0))
        {
            float f_angle = angle;
            int s16_speed = (int)((abs(speed)) / factor);
            int s16_accel = (int)((abs(param)) / factor);

            float value_data[3]= {f_angle,s16_speed,s16_accel};
            int type_data[3]= {0,2,2};

            format_data(value_data,type_data,3,"encode");

            send_command(id_num,0x1A,data_list.byte_data,0);
        }
    }
    else if (mode == 2)
    {
        float f_angle = angle;
        int s16_speed_ff = (int)((speed) / factor);
        int s16_torque_ff = (int)((param) / factor);

        float value_data[3]= {f_angle,s16_speed_ff,s16_torque_ff};
        int type_data[3]= {0,2,2};

        format_data(value_data,type_data,3,"encode");

        send_command(id_num, 0x1B, data_list.byte_data, 0);
    }
    reply_state(id_num);
}

/**
 * @brief ���������ƺ�����
 * ����ָ�������ŵĵ������ָ�����ٶ�ת����ָ���ĽǶȣ���֤������ͬʱ����Ŀ��Ƕȡ�
 *
 * @param id_list ��������ɵ�����
 * @param angle_list ����Ƕ���ɵ�����
 * @param speed ���ĵ��ת�����ٶȣ�r/min����ǰ���ٶ�
 * @param param mode=0,�Ƕ������˲�������<300����mode=1,������ֹͣ�׶μ��ٶȣ�(r/min)/s��; mode =2, ǰ���ٶȣ�r/min)
 * @param mode �Ƕȿ���ģʽѡ�񣬵��֧�����ֽǶȿ���ģʽ��
 *             mode = 0: �������켣����ģʽ������һ�������˶����ر��ʺ϶���켣�����룬�Ƕ������������������Ϊָ���Ƶ�ʵ�һ�롣
 *             mode = 1: ���������ι켣ģʽ����ʱspeedΪ�������е�����ٶȣ�r/min����paramΪĿ����ٶȣ�(r/min)/s����
 *             mode = 2: ǰ������ģʽ������ģʽ�µ�speed��torque�ֱ�Ϊǰ��������.ǰ��������ԭ��PID���ƻ����ϼ����ٶȺ�Ť��ǰ�������ϵͳ����Ӧ���Ժͼ��پ�̬��
 * @param n ���鳤��
 */
void set_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{

#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    static int last_count = 0;
    static uint8_t *last_id_list = NULL;
    static float *current_angle_list = NULL;

    uint8_t state = 0;
    for (size_t i = 0; i < n; i++)
    {
        if (last_id_list[i] != id_list[i])
        {
            state = 1;
            break;
        }
    }

    if (last_count != n || state)
    {
        last_count = n;
        SERVO_FREE(last_id_list);
        SERVO_FREE(current_angle_list);
        last_id_list = SERVO_MALLOC(n * sizeof(uint8_t));
        current_angle_list = SERVO_MALLOC(n * sizeof(float));
        memcpy(last_id_list, id_list, n * sizeof(uint8_t));
        for (size_t i = 0; i < n; i++)
            current_angle_list[i] = get_state(id_list[i]).angle;
    }
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i],angle_list[i],speed,param,mode);
        unsigned char order_num = 0x10;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};

        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0);  // ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
    }
    else if (mode == 1)
    {
        if ((speed < 0) ||( param < 0)) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if (delta_angle < (fabs_temp = fabs(angle_list[i] - current_angle_list[i])))
                delta_angle = fabs_temp;
        }
        if (TRAJ_MODE == 2)
        {
            if(delta_angle <= (12 * speed * speed / fabs(param)))
            {
                speed = sqrt((fabs(param) * delta_angle) / 12);
                t = 4 * speed / fabs(param);
            }
            else
            {
                t = delta_angle / (6 * speed) + 2 * speed / fabs(param);
            }
        }
        else
        {
            if (delta_angle <= (6 * speed * speed / fabs(param)))
                t = 2 * sqrt(delta_angle / (6 * fabs(param)));
            else
                t = speed / fabs(param) + delta_angle / (6 * speed);
        }
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i],angle_list[i],t,param,mode);
        unsigned char order_num = 0x11;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0); // ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
    }
    else if( mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i],speed,param,mode);
        unsigned char order_num = 0x12;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0);  // ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
    }
    memcpy(current_angle_list, angle_list, n * sizeof(float));
}

/**
 * @brief ���������ԽǶȿ��ƺ�����
 * ����ָ�������ŵĵ������ָ�����ٶ����ת��ָ���ĽǶȣ���ԽǶȣ�����ڵ����ǰλ�ã���
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param angle �����ԽǶȣ�-360~360��*n��֧�ִ�Ƕ�ת��
 * @param speed ����ٶ����ƻ�ǰ���ٶȣ�r/min��
 * @param param mode=0,�Ƕ������˲�������<300����mode=1,������ֹͣ�׶μ��ٶȣ�(r/min)/s��; mode =2, ǰ��Ť�أ�Nm)
 * @param mode �Ƕȿ���ģʽѡ�񣬵��֧�����ֽǶȿ���ģʽ��
 *             mode = 0: �켣����ģʽ������һ�������˶����ر��ʺ϶���켣�����룬�Ƕ������������������Ϊָ���Ƶ�ʵ�һ�롣
 *             mode = 1: ���ι켣ģʽ������ģʽ�¿���ָ���˶������е��ٶȣ�speed������ͣ���ٶȣ�accel����
 *             mode = 2: ǰ������ģʽ������ģʽ�µ�speed��torque�ֱ�Ϊǰ��������.ǰ��������ԭ��PID���ƻ����ϼ����ٶȺ�Ť��ǰ�������ϵͳ����Ӧ���Ժͼ��پ�̬��
 * @note ��mode=1,���ι켣ģʽ�У�speed��accel����Ҫ����0.���speed=0�ᵼ�µ����motor error ���˳��ջ�����ģʽ������������ģʽ�����speed=0,�ᱻ��0.01���档
 *       �����������ģʽ��accel=0�����������ٶ��˶���angle,speed�������������á�
 */
void step_angle(uint8_t id_num, float angle, float speed, float param, int mode)
{
    step_angles(&id_num,&angle,speed,param,mode,1);
}
/**
 * @brief ��������ԽǶȿ��ƺ�����
 * ����ָ�������ŵĵ������ָ����ʱ�����ת�������Ƕȡ�
 *
 * @param id_list ��������ɵ��б�
 * @param angle_list ����Ƕ���ɵ��б�
 * @param speed ���ĵ��ת�����ٶȣ�r/min����ǰ���ٶ�
 * @param param mode=0,�Ƕ������˲�������<300����mode=1,������ֹͣ�׶μ��ٶȣ�(r/min)/s��; mode =2, ǰ���ٶȣ�r/min)
 * @param mode �Ƕȿ���ģʽѡ�񣬵��֧�����ֽǶȿ���ģʽ��
 *             mode = 0: �������켣����ģʽ������һ�������˶����ر��ʺ϶���켣�����룬�Ƕ������������������Ϊָ���Ƶ�ʵ�һ�롣
 *             mode = 1: ���������ι켣ģʽ����ʱspeedΪ�������е�����ٶȣ�r/min����paramΪĿ����ٶȣ�(r/min)/s����
 *             mode = 2: ǰ������ģʽ������ģʽ�µ�speed��torque�ֱ�Ϊǰ��������.ǰ��������ԭ��PID���ƻ����ϼ����ٶȺ�Ť��ǰ�������ϵͳ����Ӧ���Ժͼ��پ�̬��
 * @param n ���鳤��
 */
void step_angles(uint8_t *id_list, float *angle_list, float speed, float param, int mode, size_t n)
{
#if ENABLE_INPUT_VALIDITY_CHECK
    if (id_list == NULL || angle_list == NULL) return;
#endif
    if (mode == 0)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        unsigned char order_num = 0x10;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0); //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
    }
    else if( mode == 1)
    {
        if (speed <= 0 || param <= 0) return;
        float delta_angle = 0;
        float t = 0;
        float fabs_temp;
        for (size_t i = 0; i < n; i++)
        {
            if(delta_angle < (fabs_temp = fabs(angle_list[i])))
                delta_angle = fabs_temp;
        }
        if(TRAJ_MODE == 2)
        {
            if(delta_angle <= (12 * speed * speed / fabs(param)))
            {
                speed = sqrt((fabs(param) * delta_angle) / 12);
                t = 4 * speed / fabs(param);
            }
            else
            {
                t = delta_angle / (6 * speed) + 2 * speed / fabs(param);
            }
        }
        else
        {
            if(delta_angle <= (6 * speed * speed / fabs(param)))
                t = 2 * sqrt(delta_angle / (6 * fabs(param)));
            else
                t = speed / fabs(param) + delta_angle / (6 * speed);
        }
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], t, param, mode);
        unsigned char order_num = 0x11;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0, 0x08, data_list.byte_data,0);  // ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
    }
    else if (mode == 2)
    {
        for (size_t i = 0; i < n; i++)
            preset_angle(id_list[i], angle_list[i], speed, param, mode);
        unsigned char order_num = 0x12;

        float value_data[3]= {order_num,0,0};
        int type_data[3]= {3,1,1};
        format_data(value_data,type_data,3,"encode");

        send_command(0,0x08,data_list.byte_data,0);  //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
    }
}
/**
 * @brief ��������ٶȿ��ƺ�����
 * ����ָ�������ŵĵ������ָ�����ٶ���������ת����
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param speed Ŀ���ٶȣ�r/min��
 * @param param mode=1, ǰ��Ť�أ�Nm); mode!=1,��Ŀ����ٶȣ�(r/min)/s��
 * @param mode ����ģʽѡ��
 *             mode=1, �ٶ�ǰ������ģʽ�������Ŀ���ٶ�ֱ����Ϊspeed
 *             mode!=1,�ٶ���������ģʽ�����������Ŀ����ٶ�axis0.controller.config_.vel_ramp_rate�仯��speed��
 * @note ���ٶ�����ģʽ�£����Ŀ����ٶ�axis0.controller.config_.vel_ramp_rate����Ϊ0�������ٶȽ����ֵ�ǰֵ���䡣
 */
void set_speed(uint8_t id_num, float speed, float param, int mode)
{
    float factor = 0.01;
    float f_speed = speed;
    if (mode == 1)
    {
        int s16_torque = (int)((param) / factor);
        if( f_speed == 0)
            s16_torque = 0;
        unsigned short u16_input_mode = INPUT_MODE_PASSTHROUGH;

        float value_data[3]= {f_speed,s16_torque,u16_input_mode};
        int type_data[3]= {0,2,1};
        format_data(value_data,type_data,3,"encode");
    }
    else
    {
        int s16_ramp_rate = (int)((param) / factor);
        unsigned short u16_input_mode = INPUT_MODE_VEL_RAMP;

        float value_data[3]= {f_speed,s16_ramp_rate,u16_input_mode};
        int type_data[3]= {0,2,1};
        format_data(value_data,type_data,3,"encode");
    }
    send_command(id_num,0x1c,data_list.byte_data,0);
    reply_state(id_num);
}
/**
 * @brief �������ٶȿ��ƺ�����
 * ����ָ����������ŵĵ������ָ�����ٶ���������ת����
 *
 * @param id_list ��������ɵ��б�
 * @param speed_list ���Ŀ���ٶȣ�r/min����ɵ��б�
 * @param param mode=1, ǰ��Ť�أ�Nm); mode!=1,��Ŀ����ٶȣ�(r/min)/s��
 * @param mode ����ģʽѡ��
 *             mode=1, �ٶ�ǰ������ģʽ�������Ŀ���ٶ�ֱ����Ϊspeed
 *             mode!=1,�ٶ���������ģʽ�����������Ŀ����ٶ�axis0.controller.config_.vel_ramp_rate�仯��speed��
 * @param n ���鳤��
 */
void set_speeds(uint8_t *id_list, float *speed_list, float param, float mode, size_t n)
{
    if (id_list == NULL || speed_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_speed(id_list[i], speed_list[i], param, mode);
    unsigned char order_num = 0x13;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08, data_list.byte_data, 0); //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
}

/**
 * @brief ����������أ��������ջ����ƺ�����
 * ����ָ�������ŵĵ�����ָ����Ť�أ�Nm��
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param torque ��������Nm)
 * @param param mode=1,�Ĳ��������壻mode!=1,Ť�������ٶ�axis0.controller.config.torque_ramp_rate��Nm/s��
 * @param mode ����ģʽѡ��
 *             mode=1, Ť��ֱ�ӿ���ģʽ�������Ŀ��Ť��ֱ����Ϊtorque
 *             mode!=1,Ť����������ģʽ�����������Ť����������axis0.controller.config.torque_ramp_rate��Nm/s���仯��torque��
 * @note ������ת�ٳ��������õ� vel_limit �������������ؽ����С��
 *       �������� axis0.controller.config.enable_current_mode_vel_limit = False ����ֹ���ؼ�С��
 *       ������Ť����������ģʽ�£�������Ť����������axis0.controller.config.torque_ramp_rateΪ0������Ť�ؽ��ڵ�ǰֵ���ֲ��䡣
 */
void set_torque(uint8_t id_num, float torque, float param, int mode)
{
    float factor = 0.01;
    int u16_input_mode,s16_ramp_rate;
    float f_torque = torque;
    if (mode == 1)
    {
        u16_input_mode = INPUT_MODE_PASSTHROUGH;
        s16_ramp_rate = 0;
    }
    else
    {
        u16_input_mode = INPUT_MODE_TORQUE_RAMP;
        s16_ramp_rate = (int)((param) / factor);
    }
    float value_data[3]= {f_torque, s16_ramp_rate,u16_input_mode};
    int type_data[3]= {0,2,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x1d,data_list.byte_data,0);
    reply_state(id_num);
}
/**
 * @brief �����������ؿ��ƺ�����
 * ͬʱ���ƶ�������ŵĵ��Ŀ��Ť�أ�Nm��
 *
 * @param id_list ��������ɵ��б�
 * @param torque_list ���Ŀ��Ť�أ�Nm)��ɵ��б�
 * @param param mode=1,�Ĳ��������壻mode!=1,Ť�������ٶ�axis0.controller.config.torque_ramp_rate��Nm/s��
 * @param mode ����ģʽѡ��
 *             mode=1, Ť��ֱ�ӿ���ģʽ�������Ŀ��Ť��ֱ����Ϊtorque
 *             mode!=1,Ť����������ģʽ�����������Ť����������axis0.controller.config.torque_ramp_rate��Nm/s���仯��torque��
 * @param n ���鳤��
 */
void set_torques(uint8_t *id_list, float *torque_list, float param, int mode, size_t n)
{
    if (id_list == NULL || torque_list == NULL) return;
    for (size_t i = 0; i < n; i++)
        preset_torque(id_list[i], torque_list[i], param, mode);
    unsigned char order_num = 0x14;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08, data_list.byte_data, 0);  //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
		
}
/**
* @brief ��������迹���ƺ�����
* ��ָ�������ŵĵ�������迹���ơ�
*
* @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
* @param pos ���Ŀ��Ƕȣ��ȣ�
* @param vel ���Ŀ���ٶȣ�r/min��
* @param tff ǰ��Ť�أ�Nm)
* @param kp �ն�ϵ��(rad/Nm)
* @param kd ����ϵ��(rad/s/Nm)
* @note �迹����ΪMIT��Դ�����еĿ���ģʽ����Ŀ�����Ť�ؼ��㹫ʽ���£�
        torque = kp*( pos �C pos_) + t_ff + kd*(vel �C vel_)
        ����pos_��vel_�ֱ�Ϊ����ᵱǰʵ��λ�ã�degree���͵�ǰʵ���ٶȣ�r/min��, kp��kdΪ�ն�ϵ��������ϵ����ϵ��������MIT��Ч
*/
void impedance_control(uint8_t id_num, float pos, float vel, float tff, float kp, float kd)
{
    float factor = 0.01;
    preset_angle(id_num,pos,vel, tff, 2);
    unsigned char order_num = 0x15;
    float value_data[3]= {order_num,(int)(kp / factor),(int)(kd / factor)};
    int type_data[3]= {3,2,2};
    format_data(value_data,type_data,3,"encode");
    send_command(0, 0x08,data_list.byte_data,0);//��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
		
}
/**
 * @brief ��ͣ����
 * ���Ƶ������ֹͣ�������ͣ���л���IDLE����ģʽ�����ж�ز�����ERROR_ESTOP_REQUESTED�����־��������Ӧset_angle/speed/torqueָ�
 * ���Ҫ�ָ���������ģʽ����Ҫ������clear_error��������־��,Ȼ����set_mode������ģʽ����Ϊ2���ջ�����ģʽ����
 *
 * @param id_num ��Ҫ��ͣ�ĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 */
void estop(uint8_t id_num)
{
    unsigned char order_num = 0x06;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08,data_list.byte_data,0);  // ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
}
// �������ù���

/**
 * @brief ���õ��ID�š�
 * �ı���ID�ţ����籣�棩
 *
 * @param id_num ��Ҫ�������ñ�ŵĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ�����������������ᱻ���ó���ͬ���
 * @param new_id �µ����ţ����ID�ŷ�ΧΪ1~63
 */
void set_id(uint8_t id_num, int new_id)
{
    write_property(id_num,31001,3,new_id);
    save_config(new_id);
}
/**
 * @brief ���õ�����ڲ����ʡ�
 * ����UART���ڲ����ʣ����籣�棩
 *
 * @param id_num ��Ҫ�������ò����ʵĵ�����,�����֪����ǰ�����ţ�������0�㲥��
 * @param baud_rate uart���ڲ����ʣ�֧��9600,19200,57600,115200������һ�֣��޸ĳɹ������ֶ�������UART������Ҳ�޸�Ϊ��ֵͬ
 * @note ������ڲ�����ֻ��UART���ߣ�TX/RX���ӿ���Ч��USB�ӿ��еĴ���Ϊ���⴮�ڣ������ʲ������ã������Զ���Ӧ��λ���Ĳ����ʡ�
 */
void set_uart_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num,10001,3,baud_rate);
    save_config(id_num);
}
/**
 * @brief ���õ��CAN�����ʡ�
 * ����CAN�����ʣ����籣�棩
 *
 * @param id_num ��Ҫ�������ò����ʵĵ�����,�����֪����ǰ�����ţ�������0�㲥��
 * @param baud_rate CAN�����ʣ�֧��125k,250k,500k,1M������һ��,�޸ĳɹ������ֶ�������CAN������Ҳ�޸�Ϊ��ֵͬ��
 */
void set_can_baud_rate(uint8_t id_num, int baud_rate)
{
    write_property(id_num,21001,3,baud_rate);
    save_config(id_num);
}
/* @brief ���õ��ģʽ��
* ���õ�����벻ͬ�Ŀ���ģʽ��
*
* @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
* @param mode ���ģʽ���
*             mode = 1: IDLE����ģʽ��������ص�PWM��������ж��
*             mode = 2: �ջ�����ģʽ��set_angle�� set_speed, set_torque���������ڱջ�����ģʽ�²��ܽ��п��ơ�������ϵ���Ĭ��ģʽ��
* @note ģʽ3��ģʽ4������У׼����ͱ���������������ǰ�����У׼����������²�Ҫʹ�á�
*/
void set_mode(uint8_t id_num, int mode)
{
    if (mode == 1)
        write_property(id_num,30003,3,AXIS_STATE_IDLE);
    else if (mode == 2)
        write_property(id_num,30003,3, AXIS_STATE_CLOSED_LOOP_CONTROL);
}
/**
 * @brief ���õ�����λ�ú���
 * ���õ�ǰλ��Ϊ����������㣬�������ǰλ��Ϊ0��
 *
 * @param id_num ��Ҫ���õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 */
void  set_zero_position(uint8_t id_num)
{

    unsigned char order_num = 0x05;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x08, data_list.byte_data, 0); //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
}
/**
 * @brief ����GPIO���ƽӿ�ģʽ
 * ���õ��Ԥ������������ģʽ��֧�ֵ�ģʽ��UART���ں�Step/Dir�ӿڣ�ͨ�����øú��������л������û�������
 *
 * @param id_num ��Ҫ���õĵ�����,�����֪����ǰ�����ţ�������0�㲥������������ж������������������ִ�иò�����
 * @param mode ��Ҫѡ���ģʽ��mode == 0 ��ʾѡ��uart����ģʽ��mode == 1��ʾѡ��Step/Dir�ӿڿ���ģʽ
 * @param param ��mode='tx_rx'ʱ��param��ʾ���ڵĲ����ʣ�֧��9600��19200��57600��115200����һ�֣���mode='step_dir'ʱ��param��ʾ���תһȦ��Ӧ����������֧��1-1024(����������)
 */
void set_GPIO_mode(uint8_t id_num, uint8_t mode, uint32_t param)
{

#if ENABLE_INPUT_VALIDITY_CHECK
    if (mode != 0 && mode != 1) return;
#endif
    const uint32_t sleep_time = 100;
    uint8_t enable_uart, enable_step_dir;
    get_GPIO_mode(id_num, &enable_uart, &enable_step_dir, NULL);
    if (enable_uart != !mode)
    {
        write_property(id_num, 10008, 1, !mode);
        SERVO_DELAY(sleep_time);
    }
    if (enable_step_dir != mode)
    {
        write_property(id_num, 31006, 1, mode);
        SERVO_DELAY(sleep_time);
    }
    if (mode == 0) write_property(id_num, 10001, 3, param);
    else if (mode == 1) write_property(id_num, 38019, 0, param);
    SERVO_DELAY(sleep_time);
    save_config(id_num);
    SERVO_DELAY(sleep_time);
    reboot(id_num);
}
/**
 * @brief ���õ��������λ����λ��
 * ���õ��Ԥ�����������λ����λ��ֵ�����óɹ�������λ�á��ٶȼ�Ť�ؿ���ģʽ�������Ὣ��������[angle_min, angle_max]��Χ��
*  ��ע�⣺��ǰ�����λ�ñ�����[angle_min, angle_max]��Χ�ڣ���������ʧ�ܣ�
 *
 * @param id_num ��Ҫ���õĵ�����,�����֪����ǰ�����ţ�������0�㲥������������ж������������������ִ�иò�����
 * @param angle_min ������λ��С�Ƕȣ��ò�����axis0.output_shaft.circular_setpoint_min��Ӧ��
 * @param angle_max ������λ���Ƕȣ��ò�����axis0.output_shaft.circular_setpoint_max��Ӧ��
 * @return �Ƿ����óɹ�
 */
int8_t set_angle_range(uint8_t id_num, float angle_min, float angle_max)
{

    float pos_vel = get_state(id_num).angle;
    if (READ_FLAG == 1)
    {
        if (pos_vel < angle_min || pos_vel > angle_max || READ_FLAG != 1) return -1;
        write_property(id_num, 38010, 0, angle_min);
        write_property(id_num, 38011, 0, angle_max);
        write_property(id_num, 31202, 1, 1);
        save_config(id_num);
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief�����ٶȹ켣ģʽ���ٶ���������
 * �����ٶȹ켣ģʽ��1��ʾ���ι켣ģʽ��2��ʾS�ι켣ģʽ��ָ�����ٶ�������״��������λ�ÿ���-���ι켣ģʽ��
 *
 * @param id_num: ��Ҫ���õĵ�����,�����֪����ǰ�����ţ�������0�㲥������������ж������������������ִ�иò�����
 * @param   mode: 1��ʾ���ι켣��2��ʾS�ι켣��ָ�����ٶ�������״��������λ�ÿ���-���ι켣ģʽ��
 * @return ��
*/
void set_traj_mode(uint8_t id_num,int mode)
{
    write_property(id_num,35104,3, mode);
    TRAJ_MODE = mode;
}
/**
 * @brief �޸ĵ�����Բ���
 * �޸ĵ�����Բ�������������Բ���Ϊ������Ʋ���
 *
 * @param id_num ��Ҫ�޸ĵĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 * @param param_address  ��Ҫ��ȡ�����Բ����ĵ�ַ������"vbus_voltage"��"axis0.config.can_node_id"�ȣ�����������Ƽ������ò��������ַ�������������ơ�
 * @param param_type ��Ҫ��ȡ�����Բ�����������:0-float,1-unsigned short int,2-short int,3-unsigned int,4-int��
 * @param value ��Ӧ������Ŀ��ֵ��
 */
void write_property(uint8_t id_num,unsigned short param_address,int8_t param_type,float value)
{
    float value_data[3]= {param_address,param_type,value};
    int type_data[3]= {1,1,param_type};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num, 0x1F,data_list.byte_data,0);  //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
}
/**
 * @brief ��ȡ���ID��
 *
 * @param id_num ��Ҫ��ȡ�ĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ����������򽫱�����
 * @return ����� ID ��
 */
uint8_t get_id(uint8_t id_num)
{
    return read_property(id_num,31001,3);
}
/**
 * @brief ��ȡ����ĵ�ǰλ�ú��ٶ�
 * ��ȡ�������ᵱǰλ�ú��ٶ��б�����λ�ֱ�Ϊ�ȣ��㣩��תÿ����(r/min)
 *
 *ͬʱ��Ϊʵʱ״̬��ʵʱλ�á�ʵʱ�ٶȡ�ʵʱŤ�ء��Ƿ񵽴�Ŀ��λ�á��Ƿ񱨴������ٶ�ȡ�ӿڣ�����ʵʱ����ʱ�ɲ��øú������п��ٶ�ȡ���״̬
 *ע��1. ������Ҫ��MOTOR_NUM�����������ĵ��ID�Ž��е�������֤MOTOR_NUM���ڻ�������ĵ��ID�ţ�
      2. enable_reply_stateֵ��Ӱ����ٶ�ȡ�ӿڣ�ֻӰ�췢�Ϳ���ָ��ʱ�Ƿ�ʵʱ���ص��״̬��

 * @return �����λ�ú��ٶ�
 * @param id_num ��Ҫ��ȡ�ĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ����������򽫱�����
 * @return struct servo_state ������λ�ú��ٶȵĽṹ��
 */
struct servo_state get_state(uint8_t id_num)
{
    struct servo_state state = {0, 0};
    float value_data[3]= {0x00,0x00,0};
    int type_data[3]= {1,1,3};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x1E,data_list.byte_data,0);// ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
		READ_FLAG=0;
    receive_data();
    if(id_num<=MOTOR_NUM)//id_num����Ϊ0
    {
			if(READ_FLAG==1)
			{
					if(id_num==0)  //���ID��Ϊ0����ͨ����������֡��ID��Ϣ����ID��
					{
						id_num = (uint8_t)((can_id & 0x07E0) >> 5)&0xFF;
					}
					float factor = 0.01;
					float value_data[3]= {0,0,0};
					int type_data[3]= {0,2,2};
					format_data(value_data,type_data,3,"decode");
					motor_state[id_num-1][0]=data_list.value_data[0];
					motor_state[id_num-1][1]=data_list.value_data[1]*factor;
					motor_state[id_num-1][2]=data_list.value_data[2]*factor;
					motor_state[id_num-1][3] =(int)((can_id & 0x02) >> 1);
					motor_state[id_num-1][4] = (int)((can_id & 0x04) >> 2);

					state.angle = data_list.value_data[0];
					state.speed = data_list.value_data[1]*factor;
			}
			else
			{
					READ_FLAG=-1;
					reply_state_error++;
			}
    }
    return state;
}

/**
 * @brief ��ȡ����ĵ�ǰ��ѹ�͵���
 * ��ȡ�����ǰ��ѹ��q������б�����λ�ֱ�Ϊ����V���Ͱ�(A)
 *
 * @param id_num ��Ҫ��ȡ�ĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ����������򽫱�����
 * @return struct servo_volcur ��������ѹ�͵����Ľṹ��
 */
struct servo_volcur get_volcur(uint8_t id_num)
{
    struct servo_volcur volcur = {0, 0};
    volcur.vol  = read_property(id_num, 1, 0);
    if(READ_FLAG==1) {
        volcur.cur  = read_property(id_num, 33206,0);
    }
    else READ_FLAG=-1;
    return volcur;
}
/**
 * @brief ��ȡGPIO���ƽӿ�ģʽ
 * ��ȡ���Ԥ�����������ŵ�ǰģʽ��������֧�ֵ�ģʽ��UART���ں�Step/Dir�ӿ�
 *
 * @param id_num ��Ҫ��ȡ�ĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ����������򽫱�����
 * @param enable_uart �Ƿ�Ϊ uart ģʽ
 * @param enable_step_dir �Ƿ�Ϊ step dir ģʽ
 * @param n ���ڲ����ʻ�������/ÿȦ
 * @return �Ƿ��ȡ�ɹ�
 */
int8_t get_GPIO_mode(uint8_t id_num, uint8_t *enable_uart, uint8_t *enable_step_dir, uint32_t *n)
{

    if (enable_uart != NULL) *enable_uart = read_property(id_num,10008,1) == 0 ? 0 : 1;
    if (enable_step_dir != NULL) *enable_step_dir = read_property(id_num,31006,1) == 0 ? 0 : 1;
    if (READ_FLAG == 1)
    {
        if (*enable_uart && !*enable_step_dir)
        {
            if (n != NULL)
                *n = read_property(id_num,10001,3);
            return 1;
        }
        else if (!*enable_uart && *enable_step_dir)
        {
            if (n != NULL)
                *n = read_property(id_num,38019,0);
            return 1;
        }
        return -1;
    }
    return 0;
}
/**
 * @brief ��ȡ������Բ���
 * ��ȡ������Բ�������������Բ����������״̬����������Ʋ���
 *
 * @param id_num ��Ҫ��ȡ�ĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ����������򽫱�����
 * @param param_address  ��Ҫ��ȡ�����Բ����ĵ�ַ������"vbus_voltage"��"axis0.config.can_node_id"�ȣ�����������Ƽ������ò��������ַ�������������ơ�
 * @param param_type ��Ҫ��ȡ�����Բ�����������:0-float,1-unsigned short int,2-short int,3-unsigned int,4-int��
 * @return ��Ӧ���Բ�����ֵ
 */
float read_property(uint8_t id_num,int param_address,int param_type)
{
    float value_data[3]= {param_address,param_type,0};
    int type_data[3]= {1,1,3};
    format_data(value_data,type_data,3,"encode");
    READ_FLAG=0;
    send_command(id_num,0x1E,data_list.byte_data,0);// ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡

    receive_data();

    if (READ_FLAG == 1)
    {
        float value_data[3]= {0,0,0};
        int type_data[3]= {1,1,param_type};
        format_data(value_data,type_data,3,"decode");

        float value=data_list.value_data[2];
        return value;
    }
    else
    {
        READ_FLAG=-1;
        return 0;
    }
}

/*
����ϵͳ����������һ�����������ʹ��
*/


/**
 * @brief ��������־����
 * һ��������й����г����κδ��󣬵��������IDLEģʽ�����Ҫ�ָ���������ģʽ����Ҫ������clear_error��������־��,Ȼ����set_mode������ģʽ����Ϊ2���ջ�����ģʽ����
 *
 * @param id_num ��Ҫ��������־�ĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 */
void clear_error(uint8_t id_num)
{
    unsigned char order_num = 0x04;
    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");
    send_command(id_num,0x08, data_list.byte_data, 0);  //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡
}


/**
 * @brief ��ӡ���������
 * ��ȡ���������Ϣ���룬����������Ϊ0����ʾ���쳣�����������벻Ϊ0�����ʾ���ڹ��ϡ�
 *
 * @param id_num ��Ҫ��ȡ�ĵ�����,�����֪����ǰ�����ţ�������0�㲥��������ʱ������ֻ����һ����������򽫱�����
 * @return ����׳��Ĵ�����Ϣ��ţ����Ϊ������ʾ�ڶ�ȡ��Ӧ������Ϣʱͨ��ʧ�ܣ�δ�ɹ���ȡ������
 *         0 ���κδ���
 *         1 axis_error
 *         2 motor_error
 *         3 controller_error
 *         4 encoder_error
 *         5 can_error
 *         6 fet_thermistor_error
 *         7 motor_thermistor_error
 */
int8_t dump_error(uint8_t id_num)
{

    unsigned int axis_error = read_property(id_num, 30001,3);
    if(READ_FLAG==1&&axis_error!=0)
        return 1;
    else if(READ_FLAG!=1)
        return -1;
    unsigned int motor_error = read_property(id_num, 33001,3);
    if(READ_FLAG==1&&motor_error!=0)
        return 2;
    else if(READ_FLAG!=1)
        return -2;
    unsigned int controller_error = read_property(id_num, 32001,3);
    if(READ_FLAG==1&&controller_error!=0)
        return 3;
    else if(READ_FLAG!=1)
        return -3;
    unsigned int encoder_error = read_property(id_num, 34001,3);
    if(READ_FLAG==1&&encoder_error!=0)
        return 4;
    else if(READ_FLAG!=1)
        return -4;
    unsigned int can_error = read_property(id_num, 20001,3);
    if(READ_FLAG==1&&can_error!=0)
        return 5;
    else if(READ_FLAG!=1)
        return -5;
    unsigned int fet_thermistor_error = read_property(id_num, 36001,3);
    if(READ_FLAG==1&&fet_thermistor_error!=0)
        return 6;
    else if(READ_FLAG!=1)
        return -6;
    unsigned int motor_thermistor_error = read_property(id_num, 37001,3);
    if(READ_FLAG==1&&motor_thermistor_error!=0)
        return 7;
    else if(READ_FLAG!=1)
        return -7;
    else
        return 0;
}

/**
 * @brief �������ú���
 * ��������£�ͨ��write_property�޸ĵ����Ե���ϵ�����֮�󣬻�ָ�Ϊ�޸�ǰ��ֱ����������ñ��棬����Ҫ��save_config��������ز������浽flash�У����粻��ʧ��
 *
 * @param id_num ��Ҫ�������õĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 */
void save_config(uint8_t id_num)
{

    unsigned char order_num = 0x01;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");

    send_command(id_num,0x08, data_list.byte_data,0);  //��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡


}
/**
 * @brief �����������
 * �������������Ч���������ϵ����ơ�
 *
 * @param id_num ��Ҫ�����ĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 */
void reboot(uint8_t id_num)
{

    unsigned char order_num = 0x03;

    float value_data[3]= {order_num,0,0};
    int type_data[3]= {3,1,1};
    format_data(value_data,type_data,3,"encode");

    send_command(id_num,0x08, data_list.byte_data,0);  // ��Ҫ�ñ�׼֡������֡�����з��ͣ�������Զ��֡

}
/**
 * @brief ��������ȴ�����
 * ��ʱ�ȴ�ֱ�������������Ŀ��λ��(ֻ�ԽǶȿ���ָ����Ч)
 *
 * @param id_num ��Ҫ�����ĵ��ID���,�����֪����ǰ���ID��������0�㲥������������ж������������������ִ�иò�����
 ** @return ��
 */
void position_done(uint8_t id_num)
{
    int traj_done = 0;
    while(traj_done == 0 || READ_FLAG == -1)
    {
        traj_done = read_property(id_num, 32008,3);
    }
}
/**
 * @brief �������ȴ�����
 * ����ȴ���������ֱ�����е��������Ŀ��λ��(ֻ�ԽǶȿ���ָ����Ч)
 *
 * @param id_list: ��������ɵ��б�
 ** @return ��
 */
void positions_done(uint8_t *id_list,size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        position_done(id_list[i]);
    }
}

/*
 * MPU6050.c
 *
 *  Created on: 2022年7月17日
 *      Author: S10
 */
#include "driverlib.h"
#include "mpu6050.h"

void ByteWrite6050(unsigned char REG_Address,unsigned char REG_data);
unsigned char ByteRead6050(unsigned char REG_Address);
int Get6050Data(unsigned char REG_Address);
void InitMPU6050();
float Mpu6050AccelAngle(char dir);
float Mpu6050GyroAngle(char dir);

//开启信号
void IIC_start()
{
    SDA_OUT;
    SCL_OUT;
    SCL_HIGH;
    SDA_HIGH;
    __delay_cycles(16);
    SDA_LOW;
    __delay_cycles(16);
    SCL_LOW;
}
//停止信号
void IIC_stop()
{
    SDA_OUT;
    SCL_OUT;
    SDA_LOW;
    SCL_HIGH;
    __delay_cycles(16);
    SDA_HIGH;
    __delay_cycles(16);
}


//发送应答信号(MCU=>||)
void SendACK(unsigned char ack)
{
    SDA_OUT;
    SCL_OUT;
    if(ack==1)
    {
        SDA_HIGH;
    }
    else if(ack==0)
    {
        SDA_LOW;
    }
    else
        return;
    SCL_HIGH;
    __delay_cycles(16);
    SCL_LOW;
    __delay_cycles(16);
}

//接收应答信号（||=>MCU）
unsigned char IIC_testACK()
{
    unsigned char a;
    SCL_OUT;
    SDA_IN;
//GPIO_setAsInputPinWithPullUpResistor (GPIO_PORT_P8, GPIO_PIN2);
    SCL_HIGH;
    __delay_cycles(16);
    if(GPIO_getInputPinValue (GPIO_PORT_P8, GPIO_PIN2)==1)
    {
        a=1;
    }
    else
    {
        a=0;
    }

    SCL_LOW;
    __delay_cycles(16);
    SDA_OUT;
    return a;
}

//向IIC总线发送数据（MCU=>||）
void IIC_writebyte(unsigned char IIC_byte)
{
    unsigned char i;
    SDA_OUT;
    SCL_OUT;
//    SCL_LOW;
        for (i=0; i<8; i++)         //8位计数器
            {
                if((IIC_byte<<i)&0x80)
                {
                    SDA_HIGH;
                }
                else
                {
                    SDA_LOW;
                }

        __delay_cycles(16);
        SCL_HIGH;
        __delay_cycles(16);
        SCL_LOW;
//        __delay_cycles(16);
//        IIC_byte<<=1;
    }

    IIC_testACK();
}


//IIC接收一个字节(||——>MCU)
unsigned char IIC_readebyte(void)
{
    unsigned char i,k=0;
    SDA_IN;
    GPIO_setAsInputPinWithPullUpResistor (GPIO_PORT_P8, GPIO_PIN2);
    SCL_OUT;
    SCL_LOW;
    __delay_cycles(160);
    for(i=0;i<8;i++)
    {
        SCL_HIGH;
        k=k<<1;
        if(SDA)
            k|=1;
        SCL_LOW;
        __delay_cycles(160);
    }
    SDA_OUT;
    __delay_cycles(160);
    return k;
}


//**************************************
//向I2C设备写入一个字节数据
//**************************************
void ByteWrite6050(unsigned char REG_Address,unsigned char REG_data)
{
    IIC_start();                  //起始信号
    IIC_writebyte(SlaveAddress);   //发送设备地址+写信号
    IIC_writebyte(REG_Address);    //内部寄存器地址，
    IIC_writebyte(REG_data);        //内部寄存器数据，
    IIC_stop();                   //发送停止信号
}

//**************************************
//从I2C设备读取一个字节数据
//**************************************
unsigned char ByteRead6050(unsigned char REG_Address)
{
    unsigned char REG_data;
    IIC_start();                   //起始信号
    IIC_writebyte(SlaveAddress);     //发送设备地址+写信号
    IIC_writebyte(REG_Address);      //发送存储单元地址，从0开始
    IIC_start();                   //起始信号
    IIC_writebyte(SlaveAddress+1);  //发送设备地址+读信号
    REG_data=IIC_readebyte();       //读出寄存器数据
    SendACK(1);                //接收应答信号
    IIC_stop();                    //停止信号
    return REG_data;
}

//**************************************
//合成数据
//**************************************
int Get6050Data(unsigned char REG_Address)
{
    char H,L;
    H=ByteRead6050(REG_Address);
    L=ByteRead6050(REG_Address+1);
    return (H<<8)+L;   //合成数据
}

/**
 * @brief  初始化MPU6050。
 *       P1.3 - SDA | P1.2 - SCL
 * @param  无
 * @retval 无
 */
void InitMPU6050()
{
    ByteWrite6050(PWR_MGMT_1, 0x00);  // 解除休眠状态
    ByteWrite6050(SMPLRT_DIV, 0x07);  // 陀螺仪采样率设置（125HZ）
    ByteWrite6050(CONFIG, 0x06);      // 低通滤波器设置（5HZ频率）
    ByteWrite6050(GYRO_CONFIG, 0x18); // 陀螺仪自检及检测范围设置(不自检,16.4LSB/DBS/S)
    ByteWrite6050(ACCEL_CONFIG, 0x01); // 不自检，量程2g
}

/*
**********************************************
**函数名  ：float Mpu6050AccelAngle(int8 dir)
**函数功能：输出加速度传感器测量的倾角值
**            范围为2g时，换算关系：16384 LSB/g
**            角度较小时，x=sinx得到角度（弧度）, deg = rad*180/3.14
**            因为x>=sinx,故乘以1.2适当放大
**返回参数：测量的倾角值
**传入参数：dir - 需要测量的方向
**           ACCEL_XOUT - X方向
**           ACCEL_YOUT - Y方向
**           ACCEL_ZOUT - Z方向
**********************************************
*/
float Mpu6050AccelAngle(char dir)
{
    float accel_agle; // 测量的倾角值
    float result; // 测量值缓存变量
    result = (float)Get6050Data(dir); // 测量当前方向的加速度值,转换为浮点数
    accel_agle = (result + MPU6050_ZERO_ACCELL)/16384; // 去除零点偏移,计算得到角度（弧度）
    accel_agle = accel_agle*1*180/3.14;     //弧度转换为度

    return accel_agle; // 返回测量值
}

/*
**********************************************
**函数名  ：float Mpu6050GyroAngle(int8 dir)
**函数功能：输出陀螺仪测量的倾角加速度
**            范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
**返回参数：测量的倾角加速度值
**传入参数：dir - 需要测量的方向
**           GYRO_XOUT - X轴方向
**           GYRO_YOUT - Y轴方向
**           GYRO_ZOUT - Z轴方向
**********************************************
*/
float Mpu6050GyroAngle(char dir)
{
    float gyro_angle;
    gyro_angle = (float)Get6050Data(dir);   // 检测陀螺仪的当前值
    gyro_angle = -(gyro_angle + MPU6050_ZERO_GYRO)/16.4;    //去除零点偏移，计算角速度值,负号为方向处理

    return gyro_angle; // 返回测量值
}



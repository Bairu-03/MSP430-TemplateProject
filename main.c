#include "driverlib/MSP430F5xx_6xx/driverlib.h"
#include "System/Sys_Clock.h"
#include "Hardware/OLED/OLED.h"
#include "Hardware/UART/MSP430F5529_UART.h"
#include "Hardware/MPU6050/MPU6050.h"

void main(void)
{
    WDT_A_hold(WDT_A_BASE);
    SystemClock_Init();

    __bis_SR_register(GIE);

    OLED_Init();

    UART_Init(USCI_A1_BASE, 115200);

    InitMPU6050();

    while(1)
    {
        OLED_ShowFloat(1, 1, Mpu6050AccelAngle(ACCEL_XOUT), 2, 2, 8);
        OLED_ShowFloat(3, 1, Mpu6050AccelAngle(ACCEL_YOUT), 2, 2, 8);
        OLED_ShowFloat(5, 1, Mpu6050AccelAngle(ACCEL_ZOUT), 2, 2, 8);

        OLED_ShowFloat(1, 65, Mpu6050GyroAngle(GYRO_XOUT), 2, 2, 8);
        OLED_ShowFloat(3, 65, Mpu6050GyroAngle(GYRO_YOUT), 2, 2, 8);
        OLED_ShowFloat(5, 65, Mpu6050GyroAngle(GYRO_ZOUT), 2, 2, 8);

        UART_printf(USCI_A1_BASE, "%f\t%f\t%f\t%f\t%f\t%f\n",
                    Mpu6050AccelAngle(ACCEL_XOUT),
                    Mpu6050AccelAngle(ACCEL_YOUT),
                    Mpu6050AccelAngle(ACCEL_ZOUT),
                    Mpu6050GyroAngle(GYRO_XOUT),
                    Mpu6050GyroAngle(GYRO_YOUT),
                    Mpu6050GyroAngle(GYRO_ZOUT)
                    );
    }
}

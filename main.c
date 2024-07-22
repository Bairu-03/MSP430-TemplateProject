#include "driverlib/MSP430F5xx_6xx/driverlib.h"
#include "System/Sys_Clock.h"
#include "Hardware/OLED/OLED.h"
#include "Hardware/nRF24L01/nRF24L01.h"

void main(void)
{
    WDT_A_hold(WDT_A_BASE);
    SystemClock_Init();

    __bis_SR_register(GIE);

    OLED_Init();

    OLED_ShowString(7, 1, "nRF24L01 Init...", 8);
    NRF24L01_Init();
    OLED_Clear();

    /****接收端****/
    // uint8_t Buf[32] = {0};
    /**************/

    /****发送端****/
    uint8_t Buf[32] = {5, 0x12, 0x22, 0x32, 0x42, 0x52};
    NRF24L01_SendBuf(Buf);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);
    /**************/

    while(1)
    {
        /***************发送端***************/
        if(!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1))
        {
            delay_ms(10);
            while(!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1));
            Buf[2] = 0xAA;
        }
        if(!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1))
        {
            delay_ms(10);
            while(!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1));
            Buf[2] = 0xBB;
        }
        OLED_ShowString(1, 1, "Len:", 8);
        OLED_ShowHexNum(1, 33, Buf[0], 2, 8);
        OLED_ShowHexNum(3, 1, Buf[1], 2, 8);
        OLED_ShowHexNum(3, 25, Buf[2], 2, 8);
        OLED_ShowHexNum(5, 1, Buf[3], 2, 8);
        OLED_ShowHexNum(5, 25, Buf[4], 2, 8);
        OLED_ShowHexNum(7, 1, Buf[5], 2, 8);
        OLED_ShowHexNum(7, 25, Buf[6], 2, 8);
        NRF24L01_SendBuf(Buf);
        /************************************/

        /***************接收端***************/
        // if (NRF24L01_Get_Value_Flag() == 0)
        // {
        //   NRF24L01_GetRxBuf(Buf);
        // }
        // OLED_ShowString(1, 1, "Len:", 8);
        // OLED_ShowHexNum(1, 33, Buf[0], 2, 8);
        // OLED_ShowHexNum(3, 1, Buf[1], 2, 8);
        // OLED_ShowHexNum(3, 25, Buf[2], 2, 8);
        // OLED_ShowHexNum(5, 1, Buf[3], 2, 8);
        // OLED_ShowHexNum(5, 25, Buf[4], 2, 8);
        // OLED_ShowHexNum(7, 1, Buf[5], 2, 8);
        // OLED_ShowHexNum(7, 25, Buf[6], 2, 8);
        /************************************/
    }
}

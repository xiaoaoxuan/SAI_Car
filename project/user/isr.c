/**
 * @file    isr.c
 * @brief   中断服务函数集合：GPIO端口中断(P0~PB)、外部中断(INT0~4)、DMA串口中断(UART1~8)、定时器中断(TM0~11/17/18)、摄像头场中断
 * @author  RRZ (基于 SEEKFREE 库文件修改)
 * @date    2026-05-16
 */

#include "zf_common_headfile.h"

/**
 * @brief  P0端口中断处理
 * @note   读取中断标志寄存器，逐位轮询调用对应回调
 */
void P0_IRQHandler() interrupt P0INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x00);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x01][i]();
        }
    }
}

/**
 * @brief  P1端口中断处理
 */
void P1_IRQHandler() interrupt P1INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x01);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x01][i]();
        }
    }
}

/**
 * @brief  P2端口中断处理
 */
void P2_IRQHandler() interrupt P2INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x02);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x02][i]();
        }
    }
}

/**
 * @brief  P3端口中断处理
 */
void P3_IRQHandler() interrupt P3INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x03);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x03][i]();
        }
    }
}

/**
 * @brief  P4端口中断处理
 */
void P4_IRQHandler() interrupt P4INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x04);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x04][i]();
        }
    }
}

/**
 * @brief  P5端口中断处理
 */
void P5_IRQHandler() interrupt P5INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x05);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x05][i]();
        }
    }
}

/**
 * @brief  P6端口中断处理
 */
void P6_IRQHandler() interrupt P6INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x06);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x06][i]();
        }
    }
}

/**
 * @brief  P7端口中断处理
 */
void P7_IRQHandler() interrupt P7INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x07);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x07][i]();
        }
    }
}

/**
 * @brief  P8端口中断处理
 */
void P8_IRQHandler() interrupt P8INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x08);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x08][i]();
        }
    }
}

/**
 * @brief  P9端口中断处理
 */
void P9_IRQHandler() interrupt P9INT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x09);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x09][i]();
        }
    }
}

/**
 * @brief  PA端口中断处理
 */
void PA_IRQHandler() interrupt PAINT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x0a);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x0a][i]();
        }
    }
}

/**
 * @brief  PB端口中断处理
 */
void PB_IRQHandler() interrupt PBINT_VECTOR
{
    uint8 i;
    volatile uint8 far *intf_reg = (volatile uint8 far *)(0x7efd10 + 0x0b);
    uint8 flags = *intf_reg;
    *intf_reg &= ~flags;

    for(i=0; i<8; i++)
    {
        if(flags & (0x01 << i))
        {
            gpio_int_irq_handlers[0x0b][i]();
        }
    }
}


/**
 * @brief  外部中断0处理
 */
void INT0_IRQHandler() interrupt INT0_VECTOR
{
    if (int_irq_handlers[0] != NULL)
    {
        int_irq_handlers[0]();
    }
}

//void INT1_IRQHandler(void) interrupt INT1_VECTOR  //外部中断1被摄像头场中断占用

/**
 * @brief  外部中断2处理
 */
void INT2_IRQHandler(void) interrupt INT2_VECTOR
{
    if (int_irq_handlers[2] != NULL)
    {
        int_irq_handlers[2]();
    }
}

/**
 * @brief  外部中断3处理
 */
void INT3_IRQHandler(void) interrupt INT3_VECTOR
{
    if (int_irq_handlers[3] != NULL)
    {
        int_irq_handlers[3]();
    }
}

/**
 * @brief  外部中断4处理
 */
void INT4_IRQHandler(void) interrupt INT4_VECTOR
{
    if (int_irq_handlers[4] != NULL)
    {
        int_irq_handlers[4]();
    }
}

/**
 * @brief  DMA UART1接收中断：处理接收完成和半满中断
 * @note   检测0x7F自动下载指令，连续20次触发IAP_CONTR跳转
 */
void DMA_UART1_IRQHandler(void) interrupt DMA_UR1R_VECTOR
{
    static vuint8 dwon_count = 0;

    if (DMA_UR1R_STA & 0x01) // 半满中断
    {
        DMA_UR1R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_1); // 开启下一次接收，关闭当前

        // 自动下载检测：连续收到20个0x7F触发IAP跳转
        if (uart_rx_buff[UART_1][0] == 0x7F)
        {
            if (dwon_count++ > 20)   // 0x7F自动下载计数阈值
            {
                IAP_CONTR = 0x60;
            }
        }
        else
        {
            dwon_count = 0;
        }

        if (uart_rx_handlers[UART_1] != NULL)
        {
            uart_rx_handlers[UART_1](uart_rx_buff[UART_1][0]);
        }
    }

    if (DMA_UR1R_STA & 0x02) // 数据溢出
    {
        DMA_UR1R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_1); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  DMA UART2接收中断
 */
void DMA_UART2_IRQHandler(void) interrupt DMA_UR2R_VECTOR
{
    if (DMA_UR2R_STA & 0x01) // 半满中断
    {
        DMA_UR2R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_2); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_2] != NULL)
        {
            uart_rx_handlers[UART_2](uart_rx_buff[UART_2][0]);
        }
    }

    if (DMA_UR2R_STA & 0x02) // 数据溢出
    {
        DMA_UR2R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_2); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  DMA UART3接收中断
 */
void DMA_UART3_IRQHandler(void) interrupt DMA_UR3R_VECTOR
{
    if (DMA_UR3R_STA & 0x01) // 半满中断
    {
        DMA_UR3R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_3); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_3] != NULL)
        {
            uart_rx_handlers[UART_3](uart_rx_buff[UART_3][0]);
        }
    }

    if (DMA_UR3R_STA & 0x02) // 数据溢出
    {
        DMA_UR3R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_3); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  DMA UART4接收中断
 */
void DMA_UART4_IRQHandler(void) interrupt DMA_UR4R_VECTOR
{
    if (DMA_UR4R_STA & 0x01) // 半满中断
    {
        DMA_UR4R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_4); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_4] != NULL)
        {
            uart_rx_handlers[UART_4](uart_rx_buff[UART_4][0]);
        }
    }

    if (DMA_UR4R_STA & 0x02) // 数据溢出
    {
        DMA_UR4R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_4); // 开启下一次接收，关闭当前
    }
}


/**
 * @brief  DMA UART5接收中断
 */
void DMA_UART5_IRQHandler(void) interrupt DMA_UR5R_VECTOR
{
    if (DMA_UR5R_STA & 0x01) // 半满中断
    {
        DMA_UR5R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_5); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_5] != NULL)
        {
            uart_rx_handlers[UART_5](uart_rx_buff[UART_5][0]);
        }
    }

    if (DMA_UR5R_STA & 0x02) // 数据溢出
    {
        DMA_UR5R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_5); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  DMA UART6接收中断
 */
void DMA_UART6_IRQHandler(void) interrupt DMA_UR6R_VECTOR
{
    if (DMA_UR6R_STA & 0x01) // 半满中断
    {
        DMA_UR6R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_6); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_6] != NULL)
        {
            uart_rx_handlers[UART_6](uart_rx_buff[UART_6][0]);
        }
    }

    if (DMA_UR6R_STA & 0x02) // 数据溢出
    {
        DMA_UR6R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_6); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  DMA UART7接收中断
 */
void DMA_UART7_IRQHandler(void) interrupt DMA_UR7R_VECTOR
{
    if (DMA_UR7R_STA & 0x01) // 半满中断
    {
        DMA_UR7R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_7); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_7] != NULL)
        {
            uart_rx_handlers[UART_7](uart_rx_buff[UART_7][0]);
        }
    }

    if (DMA_UR7R_STA & 0x02) // 数据溢出
    {
        DMA_UR7R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_7); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  DMA UART8接收中断
 */
void DMA_UART8_IRQHandler(void) interrupt DMA_UR8R_VECTOR
{
    if (DMA_UR8R_STA & 0x01) // 半满中断
    {
        DMA_UR8R_STA &= ~0x01;      // 清标志位
        uart_rx_start_buff(UART_8); // 开启下一次接收，关闭当前

        if (uart_rx_handlers[UART_8] != NULL)
        {
            uart_rx_handlers[UART_8](uart_rx_buff[UART_8][0]);
        }
    }

    if (DMA_UR8R_STA & 0x02) // 数据溢出
    {
        DMA_UR8R_STA &= ~0x02;      // 清标志位
        uart_rx_start_buff(UART_8); // 开启下一次接收，关闭当前
    }
}

/**
 * @brief  定时器0中断处理
 */
void TM0_IRQHandler() interrupt TMR0_VECTOR
{
    TIM0_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_0] != NULL)
    {
        tim_irq_handlers[TIM_0]();
    }
}

/**
 * @brief  定时器1中断处理
 */
void TM1_IRQHandler() interrupt TMR1_VECTOR
{
    TIM1_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_1] != NULL)
    {
        tim_irq_handlers[TIM_1]();
    }
}

/**
 * @brief  定时器2中断处理
 */
void TM2_IRQHandler() interrupt TMR2_VECTOR
{
    TIM2_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_2] != NULL)
    {
        tim_irq_handlers[TIM_2]();
    }
}

/**
 * @brief  定时器3中断处理
 */
void TM3_IRQHandler() interrupt TMR3_VECTOR
{
    TIM3_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_3] != NULL)
    {
        tim_irq_handlers[TIM_3]();
    }
}

/**
 * @brief  定时器4中断处理
 */
void TM4_IRQHandler() interrupt TMR4_VECTOR
{
    TIM4_CLEAR_FLAG;
    if (tim_irq_handlers[TIM_4] != NULL)
    {
        tim_irq_handlers[TIM_4]();
    }
}

/**
 * @brief  定时器5/6共用中断处理
 */
void TM5_TM6_IRQHandler() interrupt TMR5_TMR6_VECTOR
{
    if(TIM5_GET_FLAG)
    {
        TIM5_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_5] != NULL)
        {
            tim_irq_handlers[TIM_5]();
        }
    }

    if(TIM6_GET_FLAG)
    {
        TIM6_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_6] != NULL)
        {
            tim_irq_handlers[TIM_6]();
        }
    }
}

/**
 * @brief  定时器7/8共用中断处理
 */
void TM7_IRQHandler() interrupt TMR7_TMR8_VECTOR
{
    if(TIM7_GET_FLAG)
    {
        TIM7_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_7] != NULL)
        {
            tim_irq_handlers[TIM_7]();
        }
    }

    if(TIM8_GET_FLAG)
    {
        TIM8_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_8] != NULL)
        {
            tim_irq_handlers[TIM_8]();
        }
    }
}


/**
 * @brief  定时器9/10共用中断处理
 */
void TM9_TM10_IRQHandler() interrupt TMR9_TMR10_VECTOR
{
    if(TIM9_GET_FLAG)
    {
        TIM9_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_9] != NULL)
        {
            tim_irq_handlers[TIM_9]();
        }
    }

    if(TIM10_GET_FLAG)
    {
        TIM10_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_10] != NULL)
        {
            tim_irq_handlers[TIM_10]();
        }
    }
}


/**
 * @brief  定时器11中断处理
 */
void TM11_IRQHandler() interrupt TMR11_VECTOR
{
    TIM11_CLEAR_FLAG;

    if (tim_irq_handlers[TIM_11] != NULL)
    {
        tim_irq_handlers[TIM_11]();
    }
}

/**
 * @brief  定时器17/18共用中断处理
 */
void TM17_TM18_IRQHandler() interrupt TMR17_TMR18_VECTOR
{
    if(TIM17_GET_FLAG)
    {
        TIM17_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_17] != NULL)
        {
            tim_irq_handlers[TIM_17]();
        }
    }

    if(TIM18_GET_FLAG)
    {
        TIM18_CLEAR_FLAG;
        if (tim_irq_handlers[TIM_18] != NULL)
        {
            tim_irq_handlers[TIM_18]();
        }
    }
}

/**
 * @brief  摄像头场同步中断(INT1) + MT9V03X VSYNC处理
 * @note   每帧图像到来时触发，调用 mt9v03x_vsync_handler()
 */
void INT1_IRQHandler() interrupt INT1_VECTOR
{
//    if(mt9v03x_finish_flag == 1)
//    {
//        return;
//    }
    mt9v03x_vsync_handler();
}

/**
 * @brief  DMA LCM中断：MT9V03X摄像头DMA传输完成
 */
void DMA_LCM_IRQHandler() interrupt DMA_LCM_VECTOR
{
    mt9v03x_dma_handler();
}

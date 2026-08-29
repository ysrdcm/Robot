#include "sccb.h"
#include "sys.h"

/**
 * @brief   SCCB延时函数
 * @note    用于控制SCCB通信速率
 * @param   无
 * @retval  无
 */
static void sccb_delay(void)
{
    sys_delay_us(5);
}

/**
 * @brief   初始化SCCB
 * @param   无
 * @retval  无
 */
void sccb_init(void)
{
    sccb_stop();
}

/**
 * @brief   产生SCCB起始信号
 * @param   无
 * @retval  无
 */
void sccb_start(void)
{
    SCCB_SDA(1);
    SCCB_SCL(1);
    sccb_delay();
    SCCB_SDA(0);
    sccb_delay();
    SCCB_SCL(0);
}

/**
 * @brief   产生SCCB停止信号
 * @param   无
 * @retval  无
 */
void sccb_stop(void)
{
    SCCB_SDA(0);
    sccb_delay();
    SCCB_SCL(1);
    sccb_delay();
    SCCB_SDA(1);
    sccb_delay();
}

/**
 * @brief   产生SCCB NACK信号
 * @param   无
 * @retval  无
 */
void sccb_nack(void)
{
    sccb_delay();
    SCCB_SDA(1);
    SCCB_SCL(1);
    sccb_delay();
    SCCB_SCL(0);
    sccb_delay();
    SCCB_SDA(0);
    sccb_delay();
}

/**
 * @brief   SCCB发送一个字节
 * @param   data: 待发送的一字节数据
 * @retval  发送结果
 * @arg     0: 发送成功
 * @arg     1: 发送失败
 */
uint8_t sccb_send_byte(uint8_t data)
{
    uint8_t i;
    uint8_t res;
    
    for (i=0; i<8; i++)
    {
        SCCB_SDA((data & 0x80) >> 7);
        sccb_delay();
        SCCB_SCL(1);
        sccb_delay();
        SCCB_SCL(0);
        data <<= 1;
    }
    SCCB_SDA(1);
    sccb_delay();
    
    SCCB_SCL(1);
    sccb_delay();
    if (SCCB_SDA_READ)
    {
        res = 1;
    }
    else
    {
        res = 0;
    }
    SCCB_SCL(0);
    
    return res;
}

/**
 * @brief   SCCB读取一个字节
 * @param   无
 * @retval  读取到的一字节数据
 */
uint8_t sccb_read_byte(void)
{
    uint8_t i;
    uint8_t receive = 0;
    
    for (i=0; i<8; i++)
    {
        receive <<= 1;
        SCCB_SCL(1);
        sccb_delay();
        
        if (SCCB_SDA_READ)
        {
            receive++;
        }
        
        SCCB_SCL(0);
        sccb_delay();
    }
    
    return receive;
}

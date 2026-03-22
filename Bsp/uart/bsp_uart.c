#include "bsp_uart.h"

#include "usart.h"
#include "FreeRTOS.h"
#include "Semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"




#define UART_RX_QUEUE_LEN 100
#define UART_RX_BUFFER_SIZE 256

static struct uart_device g_uart1_it;
static struct uart_device g_uart1_dma;
static struct uart_device g_uart3_dma;

struct uart_data {
    UART_HandleTypeDef *handle;
    SemaphoreHandle_t xTxSem;
    QueueHandle_t xRxQueue;
    uint8_t rxdatas[UART_RX_BUFFER_SIZE];
};

static struct uart_data g_uart1_data = {
    .handle = &huart1,
};

static struct uart_data g_uart3_data = {
    .handle = &huart3,
};
/**********************************************************************************************************************/
/*回调函数*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    struct uart_data *pData;
    if (huart == &huart1)
    {
        pData = g_uart1_dma.priv_data;
        xSemaphoreGiveFromISR(pData->xTxSem, NULL);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    struct uart_data *pData;
    int len = huart->RxXferSize - huart->RxXferCount;

    // LED effect is centralized in chassis task; do not toggle LEDs in UART callbacks.

    if (huart == &huart1)
    {
        pData = g_uart1_dma.priv_data;

        for (int i = 0; i < len; i++) {
            xQueueSendFromISR(pData->xRxQueue, &pData->rxdatas[i], NULL);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(pData->handle, &pData->rxdatas, huart->RxXferSize);
    }
    if (huart == &huart3)
    {
        pData = g_uart3_dma.priv_data;

        for (int i = 0; i < len; i++) {
            xQueueSendFromISR(pData->xRxQueue, &pData->rxdatas[i], NULL);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(pData->handle, &pData->rxdatas, huart->RxXferSize);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    struct uart_data *pData;
    int len = Size;

    // LED effect is centralized in chassis task; do not toggle LEDs in UART callbacks.


    if (huart == &huart1)
    {
        pData = g_uart1_dma.priv_data;



        for (int i = 0; i < len; i++) {
            xQueueSendFromISR(pData->xRxQueue, &pData->rxdatas[i], NULL);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(pData->handle, &pData->rxdatas, huart->RxXferSize);
    }

    if (huart == &huart3)
    {
        pData = g_uart3_dma.priv_data;

        for (int i = 0; i < len; i++) {
            xQueueSendFromISR(pData->xRxQueue, &pData->rxdatas[i], NULL);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(pData->handle, &pData->rxdatas, huart->RxXferSize);
    }
}

/**********************************************************************************************************************/
/*使用中断*/

static int uart_it_init(struct uart_device *pDev, int baud, int datas, char  parity, int stop)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;
    pData->xTxSem = xSemaphoreCreateBinary();
    pData->xRxQueue = xQueueCreate(UART_RX_QUEUE_LEN, sizeof(char));

    HAL_UART_Receive_IT(pData->handle, &pData->rxdatas, 1);

    return 0;
}

static int uart_it_send(struct uart_device *pDev, char *data, int len, int timeout_ms)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;

    HAL_UART_Transmit_IT(pData->handle, data, len);

    if(pdTRUE == xSemaphoreTake(pData->xTxSem, timeout_ms)) {
        return 0;
    }
    else {
        return -1;
    }
}

int uart_it_printf(struct uart_device *pDev, const char *fmt,...)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;
    static uint8_t tx_buf[1024] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    HAL_UART_Transmit_IT(&huart1, tx_buf, len);

    if(pdTRUE == xSemaphoreTake(pData->xTxSem, 100)) {
        return 0;
    }
    else {
        return -1;
    }
}

static int uart_it_recv(struct uart_device *pDev, char *data, int max_len, int timeout_ms)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;

    memset(data, 0, 1);
    if(pdPASS == xQueueReceive(pData->xRxQueue, data, timeout_ms)) {
        return 1;
    }
    else {
        return -1;
    }
}
/**********************************************************************************************************************/
/*使用中断实例*/
static struct uart_device g_uart1_it = {
    "uart1_it",
    uart_it_init,
    uart_it_send,
    uart_it_printf,
    uart_it_recv,
    &g_uart1_data,
};

/**********************************************************************************************************************/
/*使用DMA*/


static int uart_dma_init(struct uart_device *pDev, int baud, int datas, char  parity, int stop)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;
    pData->xTxSem = xSemaphoreCreateBinary();
    pData->xRxQueue = xQueueCreate(UART_RX_QUEUE_LEN, sizeof(char));

    HAL_UARTEx_ReceiveToIdle_DMA(pData->handle, &pData->rxdatas, UART_RX_BUFFER_SIZE);

    return 0;
}

static int uart_dma_send(struct uart_device *pDev, char *datas, int len, int timeout_ms)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;

    HAL_UART_Transmit_DMA(pData->handle, datas, len);

    if(pdTRUE == xSemaphoreTake(pData->xTxSem, timeout_ms)) {
        return 0;
    }
    else {
        return -1;
    }
}

int uart_dma_printf(struct uart_device *pDev, const char *fmt,...)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;
    static uint8_t tx_buf[1024];
    va_list ap;
    uint16_t len;

    va_start(ap, fmt);
    len = vsnprintf((char *)tx_buf, sizeof(tx_buf), fmt, ap);
    va_end(ap);

    HAL_UART_Transmit_DMA(&huart1, tx_buf, len);

    if(pdTRUE == xSemaphoreTake(pData->xTxSem, 100)) {
        return 0;
    }
    else {
        return -1;
    }
}

static int uart_dma_recv(struct uart_device *pDev, char *data, int max_len, int timeout_ms)
{
    struct uart_data *pData = (struct uart_data *)pDev->priv_data;
    int count = 0;
    memset(data, 0, max_len);

    // 等待第一个字符
    if(pdPASS == xQueueReceive(pData->xRxQueue, &data[count], timeout_ms)) {
        count++;

        // 继续接收剩余字符(非阻塞)
        while(count < max_len &&
              pdPASS == xQueueReceive(pData->xRxQueue, &data[count], 0)) {
            count++;
              }

        return count; // 返回实际接收到的字符数
    }

    return -1; // 超时
}

/**********************************************************************************************************************/
/*使用DMA实例*/
static struct uart_device g_uart1_dma = {
    "uart1_dma",
    uart_dma_init,
    uart_dma_send,
    uart_dma_printf,
    uart_dma_recv,
    &g_uart1_data,
};

static struct uart_device g_uart3_dma = {
    "uart3_dma",
    uart_dma_init,
    uart_dma_send,
    uart_dma_printf,
    uart_dma_recv,
    &g_uart3_data,
};

/**********************************************************************************************************************/
/*对外接口*/

struct uart_device *g_uart_devs[] = {&g_uart1_it, &g_uart1_dma, &g_uart3_dma};

struct uart_device *uart_get_device(char *name)
{
    int i;
    for (i = 0; i < sizeof(g_uart_devs) / sizeof(g_uart_devs[0]); i++) {
        if (strcmp(g_uart_devs[i]->name, name) == 0) {
            return g_uart_devs[i];
        }
    }
    return NULL;
}


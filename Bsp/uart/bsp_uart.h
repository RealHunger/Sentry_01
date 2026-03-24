#ifndef DVC_UART_H
#define DVC_UART_H


struct uart_device
{
    const char *name;
    int (*Init)(struct uart_device *pDev, int baud, int datas, char  parity, int stop);
    int (*Send)(struct uart_device *pDev, char *data, int len, int timeout_ms);
    int (*Print)(struct uart_device *pDev, const char *fmt,...);
    int (*Recv)(struct uart_device *pDev, char *data, int max_len, int timeout_ms);
    void *priv_data;
};

struct uart_device *uart_get_device(const char *name);

#endif //DVC_UART_H

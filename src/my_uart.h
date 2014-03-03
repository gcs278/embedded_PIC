#ifndef __my_uart_h
#define __my_uart_h

#include "messages.h"

#define MAXUARTBUF 12
#if (MAXUARTBUF > MSGLEN)
#define MAXUARTBUF MSGLEN
#endif
typedef struct __uart_comm {
	unsigned char buffer[MAXUARTBUF];
	unsigned char	buflen;
        unsigned char state;
        unsigned char msgtype;
        unsigned char data_length;
        unsigned char count;
        unsigned char data_read;
} uart_comm;

void init_uart_recv(uart_comm *);
void uart_recv_int_handler(void);
void uart_recv_state(unsigned char byte);

#endif

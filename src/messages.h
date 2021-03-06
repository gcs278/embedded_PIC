#ifndef __messages
#define __messages

// Rover Debugging commands
#define RoverMsgMotorForward 0x01
#define RoverMsgMotorRight 0x02
#define RoverMsgMotorLeft 0x03
#define RoverMsgMotorBack 0x04
#define RoverMsgMotorStop 0x05

// Motor encoder distance requests
#define RoverMsgMotorLeftData 0x07
#define RoverMsgMotorRightData 0x08

// Sensor data requests
#define RoverMsgSensorAllData 0x10
#define RoverMsgSensorRightForward 0x11
#define RoverMsgSensorRightRear 0x12
#define RoverMsgSensorForwardLeft 0x13
#define RoverMsgSensorForwardRight 0x14
#define RoverMsgSensorRightAverage 0x15

#define RoverMsgTurnOnWallTracking 0x0E
#define RoverMsgTurnOffWallTracking 0x0F

// Motor adjustment commands
#define RoverMsgMotorInRange 0x17

#define RoverMsgMotorLeft2 0x1A
#define RoverMsgMotorLeft5 0x1B
#define RoverMsgMotorLeft7 0x1C
#define RoverMsgMotorLeft10 0x1D
#define RoverMsgMotorLeft15 0x1E
#define RoverMsgMotorLeft20 0x1F
#define RoverMsgMotorLeft25 0x20
#define RoverMsgMotorLeft30 0x21
#define RoverMsgMotorLeft45 0x22
#define RoverMsgMotorLeft75 0x23
#define RoverMsgMotorLeft90 0x24

#define RoverMsgMotorRight2 0x25
#define RoverMsgMotorRight5 0x26
#define RoverMsgMotorRight7 0x27
#define RoverMsgMotorRight10 0x28
#define RoverMsgMotorRight15 0x29
#define RoverMsgMotorRight90 0x2F

// Speed Modes
#define RoverMsgMotorSpeedCreepin 0x2A
#define RoverMsgMotorSpeedSlow 0x2B
#define RoverMsgMotorSpeedMedium 0x2C
#define RoverMsgMotorSpeedMediumFast 0x2D
#define RoverMsgMotorSpeedFastBRAH 0x2E

#define RoverMsgMotorForwardCMDelim 0x30

#define RoverMsgRESET 0x29

// The maximum length (in bytes) of a message
#if defined(MAIN_PIC)
    #define MSGLEN 10
#else
    #define MSGLEN 12
#endif

// Length of the I2C response
#define I2CMSGLEN 10
// How many UART data bytes are coming over from rover
#define UARTDATALEN 9
// The maximum number of messages in a single queue
#if defined (ARM_PIC) || defined (MAIN_PIC)
#define MSGQUEUELEN 40
#else
#define MSGQUEUELEN 4
#endif

typedef struct __msg {
	unsigned char	full;
	unsigned char	length;
	unsigned char msgtype;
	unsigned char data[MSGLEN];
} msg;

typedef struct __msg_queue {
	msg	queue[MSGQUEUELEN];
	unsigned char	cur_write_ind;
	unsigned char	cur_read_ind;
} msg_queue;

// Error Codes
// Too many messages in the queue
#define MSGQUEUE_FULL -1
// Message sent okay
#define MSGSEND_OKAY 1
// The length of the message is either too large or negative
#define MSGBAD_LEN -2
// The message buffer is too small to receive the message in the queue
#define MSGBUFFER_TOOSMALL -3
// The message queue is empty
#define MSGQUEUE_EMPTY -4
// This call must be made from a low-priority interrupt handler
#define MSG_NOT_IN_LOW -5
// This call must be made from a high-priority interrupt handler
#define MSG_NOT_IN_HIGH -6
// This call must be made from the "main()" thread
#define MSG_NOT_IN_MAIN -7

// This MUST be called before anything else in messages and should
// be called before interrupts are enabled
void init_queues(void);

// This is called from a high priority interrupt to decide if the
// processor may sleep. It is currently called in interrupts.c
void SleepIfOkay(void);

// This is called in the "main()" thread (if desired) to block
// until a message is received on one of the two incoming queues
void block_on_To_msgqueues(void);

// Queue:
// The "ToMainLow" queue is a message queue from low priority
// interrupt handlers to the "main()" thread.  The send is called
// in the interrupt handlers and the receive from "main()"
signed char	ToMainLow_sendmsg(unsigned char,unsigned char,void *);
signed char	ToMainLow_recvmsg(unsigned char,unsigned char *,void *);

// Queue:
// The "ToMainHigh" queue is a message queue from high priority
// interrupt handlers to the "main()" thread.  The send is called
// in the interrupt handlers and the receive from "main()"
signed char	ToMainHigh_sendmsg(unsigned char,unsigned char,void *);
signed char	ToMainHigh_recvmsg(unsigned char,unsigned char *,void *);

// Queue:
// The "FromMainLow" queue is a message queue from the "main()"
// thread to the low priority interrupt handlers.  The send is called
// in the "main()" thread and the receive from the interrupt handlers.
signed char	FromMainLow_sendmsg(unsigned char,unsigned char,void *);
signed char	FromMainLow_recvmsg(unsigned char,unsigned char *,void *);

// Queue:
// The "FromMainHigh" queue is a message queue from the "main()"
// thread to the high priority interrupt handlers.  The send is called
// in the "main()" thread and the receive from the interrupt handlers.
signed char	FromMainHigh_sendmsg(unsigned char,unsigned char,void *);
signed char	FromMainHigh_recvmsg(unsigned char,unsigned char *,void *);
#endif

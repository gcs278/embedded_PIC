                    #ifndef __maindefs
#define __maindefs

#ifdef __XC8
#include <xc.h>
#ifdef _18F45J10
#define __USE18F45J10 1
#else
#ifdef _18F2680
#define __USE18F2680 1
#else
#ifdef _18F26J50
#define __USE18F26J50 1
#else
#ifdef _18F46J50
#define __USE18F46J50 1
#endif
#endif
#endif
#endif
#else
#ifdef __18F45J10
#define __USE18F45J10 1
#else
#ifdef __18F2680
#define __USE18F2680 1
#else
#ifdef __18F26J50
#define __USE18F26J50 1
#else
#ifdef __18F46J50
#define __USE18F46J50 1
#endif
#endif
#endif
#endif
#include <p18cxxx.h>
#endif

// Message type definitions
#define MSGT_TIMER0 10
#define MSGT_TIMER1 11
#define MSGT_MAIN1 20
#define	MSGT_OVERRUN 30
#define MSGT_UART_DATA 31
#define MSGT_I2C_DBG 41
#define	MSGT_I2C_DATA 40
#define MSGT_I2C_RQST 42
#define MSGT_I2C_MASTER_SEND_COMPLETE 43
#define MSGT_I2C_MASTER_SEND_FAILED 44
#define MSGT_I2C_MASTER_RECV_COMPLETE 45
#define MSGT_I2C_MASTER_RECV_FAILED 46
#define MSGT_UART_SEND 47
#define MSGT_BUF_PUT_DATA 55
#define MSGT_QUEUE_GET_DATA 56
#define MSGT_DISPLAY_LED 57
#define MSGT_MOTOR_ENCODER 58
#define MSGT_GET_SENSOR_DATA 59

#define UART_STATE_HEADER1 48
#define UART_STATE_HEADER2 53
#define UART_STATE_MSGTYPE 49
#define UART_STATE_LENGTH 50
#define UART_STATE_DATA 51
#define UART_STATE_FOOTER 52
#define UART_STATE_COUNT 54

#define UART_HEADER1 0x2B
#define UART_HEADER2 0x9F
#define UART_FOOTER 0x5C


// PIC Type Configuration
#define MAIN_PIC
// #define SENSOR_PIC
//#define MOTOR_PIC
//#define ARM_PIC

//#define MOTOR_SCRIPT_MS4
//#define ADJUST_SCRIPT_MS4

// I2C Configuration
// #define I2C_MASTER
// #define I2C_SLAVE

unsigned char wallCorrection = 1;
unsigned char tempWallCorrection = 1;
unsigned char firstSensorRead = 1;
char sensorOffset = 0;
int timer1_extender = 0;

#ifdef MAIN_PIC
unsigned char finishLine = 0;
#endif
#endif


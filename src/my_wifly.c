#include "maindefs.h"
#include "my_wifly.h"

void initWiFly()
{

    WriteUSART('$');
    for (int i = 1; i != 0; i++);
    WriteUSART('$');
    for (int i = 1; i != 0; i++);
    WriteUSART('$');
    for (int j = 0; j < 5; j++)
        for (int i = 1; i != 0; i++); // integer wraparound
    putsUSART("");
    putsUSART("close");
    for (int j = 0; j < 5; j++)
        for (int i = 1; i != 0; i++);
    putsUSART("open 1.2.3.20 2000");
    for (int j = 0; j < 5; j++)
        for (int i = 1; i != 0; i++);
}

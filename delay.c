
#include "delay.h"

void delay_ms(int n)
{
    int i,j;
    for(i=0;i<n;i++)
        for(j=0;j<3180;j++)
        {}
}

void delay_us(int n)
{
    int i,j;
    for(i=0;i<n;i++)
        for(j=0;j<3;j++)
        {}
}
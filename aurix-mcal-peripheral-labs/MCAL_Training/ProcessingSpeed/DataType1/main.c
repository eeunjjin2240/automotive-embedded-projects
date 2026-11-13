#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>

#define LOOP_COUNT 1000000000

double timeTestInteger(void)
{
    int sumInteger = 0;
    double start = (double)clock() / CLOCKS_PER_SEC;    
    for (uint64_t i = 0; i < LOOP_COUNT; i++) 
    {
        sumInteger = sumInteger * i;
    }
    double end = (double)clock() / CLOCKS_PER_SEC;    
    return end - start;
}

double timeTestFloat(void)
{
    float sumFloat = 0;
    double start = (double)clock() / CLOCKS_PER_SEC;    
    for (uint64_t i = 0; i < LOOP_COUNT; i++) 
    {
        sumFloat = sumFloat * i;
    }
    double end = (double)clock() / CLOCKS_PER_SEC;    
    return end - start;
}

int main(void)
{
    printf("Integer Time :%lf\n", timeTestInteger());
    printf("Float Time :%lf\n", timeTestFloat());
    system("pause");
}
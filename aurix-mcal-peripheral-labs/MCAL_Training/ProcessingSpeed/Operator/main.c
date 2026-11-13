#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#define LOOP_COUNT 100000000
int test = 2;

void test_shift()
{    
    for (int i = 0; i < LOOP_COUNT; i++)
    {
        test = test << 10;
    }
}

void test_multi()
{
    for (int i = 0; i < LOOP_COUNT; i++)
    {
        test = test * 1024;
    }
}

void main()
{
    clock_t start, finish;

    printf("Start test_shift\n");
    start = clock();
    for (int i = 0; i < 50; i++)
    {
        test_shift();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    printf("Start test_multi \n");
    start = clock();
    for (int i = 0; i < 50; i++)
    {
        test_multi();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    system("pause");
}

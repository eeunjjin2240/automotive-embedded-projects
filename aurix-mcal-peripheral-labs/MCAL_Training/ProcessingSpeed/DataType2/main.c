#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

void test_uint32_t()
{
    uint32_t test = 0;
    clock_t start, finish;
    start = clock();
    for (int i = 0; i < 100000000; i++)
    {
        test = 0;
        test = test + 1;
        test = test + 1;
        test = test * 100;
        test = test / 10;
        test = test - 1;
    }
    finish = clock();
    printf("Time : %f sec\n", (float)(finish - start) / CLOCKS_PER_SEC);
}

void test_uint8_t()
{
    uint8_t test = 0;
    clock_t start, finish;
    start = clock();
    for (int i = 0; i < 100000000; i++)
    {
        test = 0;
        test = test + 1;
        test = test + 1;
        test = test * 100;
        test = test / 10;
        test = test - 1;
    }
    finish = clock();
    printf("Time : %f sec\n", (float)(finish - start) / CLOCKS_PER_SEC);
}

void main()
{
    clock_t start, finish;

    printf("\nStart test_uint32_t\n");
    start = clock();
    for (int i = 0; i < 10; i++)
    {
        test_uint32_t();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    printf("\nStart test_uint8_t \n");
    start = clock();
    for (int i = 0; i < 10; i++)
    {
        test_uint8_t();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    system("pause");
}

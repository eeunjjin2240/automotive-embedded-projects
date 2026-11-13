#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#define LOOP_COUNT 100000000
int test = 0;

const static int value = 1;
void test_const()
{    
    for (int i = 0; i < LOOP_COUNT; i++)
    {
        test += value;
    }
}

void test_literal()
{
    for (int i = 0; i < LOOP_COUNT; i++)
    {
        test += 1;
    }
}

#define VALUE 1
void test_define()
{    
    for (int i = 0; i < LOOP_COUNT; i++)
    {
        test += VALUE;
    }
}

void main()
{
    clock_t start, finish;

    printf("Start test_const\n");
    start = clock();
    for (int i = 0; i < 50; i++)
    {
        test_const();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    printf("Start test_literal \n");
    start = clock();
    for (int i = 0; i < 50; i++)
    {
        test_literal();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    printf("Start test_define \n");
    start = clock();
    for (int i = 0; i < 50; i++)
    {
        test_define();
    }
    finish = clock();
    printf("Total Time : %f sec\n\n", (float)(finish - start) / CLOCKS_PER_SEC);

    system("pause");
}

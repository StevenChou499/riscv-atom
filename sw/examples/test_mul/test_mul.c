#include <stdio.h>

int main(void)
{
    int zero = 0, dividend = 87;
    int q = dividend / zero;
    int r = dividend % zero;
    printf("the quotient is %d, remainder is %d\n", q, r);

    int intmin = 1 << 31;
    int neg_one = -1;
    printf("The quotient is %d, remainder is %d\n", intmin / neg_one, intmin % neg_one);

    int a = 3, b = 4;
    int c = a * b;
    printf("The value of 3 * 4 is %d\n", c);
    a = 3; b = -4;
    c = a * b;
    printf("The value of 3 * -4 is %d\n", c);
    a = -3; b = 4;
    c = a * b;
    printf("The value of -3 * 4 is %d\n", c);
    a = -3; b = -4;
    c = a * b;
    printf("The value of -3 * -4 is %d\n", c);
    a = 3; b = 4;
    c = a / b;
    printf("The value of 3 / 4 is %d\n", c);
    a = 3; b = 4;
    c = a % b;
    printf("The value of 3 %% 4 is %d\n", c);
    a = 4; b = 3;
    c = a / b;
    printf("The value of 4 / 3 is %d\n", c);
    a = 4; b = 3;
    c = a % b;
    printf("The value of 4 %% 3 is %d\n", c);
    a = -4; b = 3;
    c = a / b;
    printf("The value of -4 / 3 is %d\n", c);
    a = 4; b = -3;
    c = a % b;
    printf("The value of 4 %% -3 is %d\n", c);
    a = 3; b = -4;
    c = a / b;
    printf("The value of 3 / -4 is %d\n", c);
    a = -3; b = 4;
    c = a % b;
    printf("The value of -3 %% 4 is %d\n", c);

    int what = 17189;
    int hello = -53421;
    printf ("17189 * -53421 = %d\n", what * hello);

    unsigned int un_a = 2147499999;
    unsigned int un_b = 3535416584;
    unsigned int un_c = un_a * un_b;
    printf("The product of un_a and un_b is %d\n", un_c);
    return 0;
}

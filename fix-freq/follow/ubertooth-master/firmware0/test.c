#include <stdio.h>
#include <stdint.h>

const int BITS = 8;

int convert (int8_t binary[])
{
  int power = 128;
  int sum = 0;
  int i;
 
  for (i = 0; i < BITS; ++i)
  {
    if (i == 0 && binary[i] != 0)
    {
      sum = -128;
    }		
    else
    {
	sum += (binary[i] - 0) * power;
    }
    power /= 2;
  }
  return sum;
}

int main ()
{
 
  int8_t k = 0xf5; 
  int8_t bin[8];
 
  int i,j;
  for (i = 0; i < 1; i++)
  {
       for (j = 0; j < 8; j++)
       {
           bin[i * 8 + j] = (k & 0x80) >> 7;
           k <<= 1;
       }
  }
  int8_t dec = convert(bin);
  double offset = (double)dec * 5.2;
  int8_t mod = (int8_t)(offset/5.2);
/*  for (i = 0; i < 8; i++)
  {
    printf("%d \n", (mod & 0x80) >> 7);
    mod <<= 1;
  }
*/
  uint16_t temp = 0x0000;
  uint8_t add1 = 0x29;
  printf("%d \n", (uint16_t)(temp+add1));
  return 0;
}

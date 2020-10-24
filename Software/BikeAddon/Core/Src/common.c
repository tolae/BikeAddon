#include "common.h"

unsigned int bit_position_to_int(unsigned int num)
{
	int i = 1;
	int pos = 1;

	while (i < num)
	{
		i  = i << 1;
		pos++;
	}
	return pos;
}

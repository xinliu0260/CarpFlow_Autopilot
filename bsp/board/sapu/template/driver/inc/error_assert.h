#ifndef ERROR_ASSERT_H
#define ERROR_ASSERT_H

#include "rttypes.h"

void _Error_Handler(char *s, int num);

#ifndef Error_Handler
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif

void assert_hook(const char* ex, const char* func, rt_size_t line);




#endif
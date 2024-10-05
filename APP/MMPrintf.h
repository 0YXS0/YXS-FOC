#ifndef MM_PRINTF_H
#define MM_PRINTF_H

#include <stdarg.h>  //可变参数头文件

void MM_printf(const char* fmt, ...);   //串口1打印信息函数
int vsprintf(char* buf, const char* fmt, va_list args);  //格式化字符串到buf
int sprintf(char* buf, const char* fmt, ...);   //格式化字符串到buf

#endif // !MM_PRINTF_H

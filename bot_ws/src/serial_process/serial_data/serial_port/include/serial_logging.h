/**
 * @file serial_logging.h
 * @author zhaoxi (535394140@qq.com)
 * @brief
 * @version 1.0
 * @date 2022-11-03
 *
 * @copyright Copyright SCUT RobotLab(c) 2022
 *
 */

#include <stdio.h>

//! @addtogroup serial_logging
//! @{

#ifdef NDEBUG
#define DEBUG_SER_WARNING(msg) ((void)0)
#define DEBUG_SER_ERROR(msg) ((void)0)
#define DEBUG_SER_HIGHLIGHT(msg) ((void)0)
#define DEBUG_SER_INFO(msg) ((void)0)
#define DEBUG_SER_PASS(msg) ((void)0)
#else
#define DEBUG_SER_WARNING(msg) SER_WARNING(msg)
#define DEBUG_SER_ERROR(msg) SER_ERROR(msg)
#define DEBUG_SER_HIGHLIGHT(msg) SER_HIGHLIGHT(msg)
#define DEBUG_SER_INFO(msg) SER_INFO(msg)
#define DEBUG_SER_PASS(msg) SER_PASS(msg)
#endif

/*
  do while（false）将后面的多个语句组织在一个块中，以便作为一个整体使用。
  \033是转义字符，用于在C/C++中开始ANSI转义码序列。
  \033[35m是用于将文本颜色更改为品红色/紫色的转义序列。35表示品红色的ANSI颜色代码。
  [ SER-INFO ]是一个字符串文字，将以所选颜色打印。
  \033[0m 用于将文本颜色重置为默认颜色（通常为白色或灰色
*/
// 35m 品红
#define SER_HIGHLIGHT(msg...)                \
    do                                       \
    {                                        \
        printf("\033[35m[ SER-INFO ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

// 33m 黄色
#define SER_WARNING(msg...)                  \
    do                                       \
    {                                        \
        printf("\033[33m[ SER-ERR  ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

// 32m 绿色
#define SER_PASS(msg...)                     \
    do                                       \
    {                                        \
        printf("\033[32m[ SER-PASS ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

// 31m 红色
#define SER_ERROR(msg...)                    \
    do                                       \
    {                                        \
        printf("\033[31m[ SER-ERR  ] " msg); \
        printf("\033[0m\n");                 \
    } while (false)

#define SER_INFO(msg...)             \
    do                               \
    {                                \
        printf("[ SER-INFO ] " msg); \
        printf("\n");                \
    } while (false)

//! @} serial_logging

#ifndef OSQP_CONFIGURE_H
#define OSQP_CONFIGURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 强制使用 64 位整型，匹配官方 Windows 64 位 DLL 的编译标准 */
#define OSQP_USE_LONG 1

/* OSQP 默认使用 double 双精度，不需要额外宏定义，保持留空即可 */

#ifdef __cplusplus
}
#endif

#endif /* OSQP_CONFIGURE_H */
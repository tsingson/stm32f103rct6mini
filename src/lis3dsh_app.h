/* src/lis3dsh_app.h */
#ifndef LIS3DSH_APP_H_
#define LIS3DSH_APP_H_

#include <zephyr/kernel.h>
#include <stdbool.h>

/*
 * 全局控制红线标识：
 * true  -> 确认老人在 5 米范围外持续移动（允许触发 4G/GPS 发射）
 * false -> 老人处于静止或 5 米范围内的安全微幅晃动（4G/GPS 保持彻底断电静默）
 */
extern bool is_elderly_moving;

/* 初始化 LIS3DSH 加速度计并使能硬件 Any-Motion 突发中断线 */
int lis3dsh_motion_init(void);

#endif /* LIS3DSH_APP_H_ */

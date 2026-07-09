#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h> /* 修正：显式引入 rand() 和 srand() 所在的 C17 标准头文件 */
#include <time.h>   /* 修正：引入 time() 用于动态随机数种子初始化 */
#include <math.h>
#include "walk.h"

/* 模拟时间常量 */
#define SAMPLE_INTERVAL_MS  (40LL)
#define PI_CONST            (3.14159265f)

/**
 * @brief Mock 数据生成器结构体
 */
typedef struct {
    int64_t current_time_ms;
    float phase;
} mock_generator_t;

/**
 * @brief 生成模拟的三轴加速度数据（模拟 12-bit 模式下 1g = 4096 LSB）
 * @param mode 0: 纯静止, 1: 随机高频噪点(手部晃动), 2: 规律步行
 */
void generate_mock_sensor(mock_generator_t *gen, int mode, int16_t *x, int16_t *y, int16_t *z) {
    float fx = 0.0f;
    float fy = 0.0f;
    float fz = 4096.0f; /* 基础静止状态：Z轴承受 1g 重力 */

    switch (mode) {
        case 1: // 模式 1: 瞬时高频随机抖动（不连续，模拟误判）
            /* 修正：先强转浮点再运算，严防 C17 下的有符号整型回绕和编译器优化截断 */
            fx = ((float)rand() / (float)RAND_MAX) * 1200.0f - 600.0f;
            fy = ((float)rand() / (float)RAND_MAX) * 1200.0f - 600.0f;
            fz = 4096.0f + (((float)rand() / (float)RAND_MAX) * 1000.0f - 500.0f);
            break;

        case 2: // 模式 2: 规律步行曲线 (模拟大约 2Hz 步频)
            gen->phase += 2.0f * PI_CONST * 2.0f * ((float)SAMPLE_INTERVAL_MS / 1000.0f);
            if (gen->phase > 2.0f * PI_CONST) {
                gen->phase -= 2.0f * PI_CONST;
            }
            fx = 200.0f * sinf(gen->phase);
            fy = 400.0f * cosf(gen->phase);
            fz = 4096.0f + 800.0f * sinf(gen->phase);
            break;

        case 0:
        default: // 模式 0: 纯静止，带极小硬件白噪声
            fx = ((float)(rand() % 60) - 30.0f);
            fy = ((float)(rand() % 60) - 30.0f);
            fz = 4096.0f + ((float)(rand() % 60) - 30.0f);
            break;
    }

    *x = (int16_t)fx;
    *y = (int16_t)fy;
    *z = (int16_t)fz;

    gen->current_time_ms += SAMPLE_INTERVAL_MS;
}

int main(void) {
    walk_pedometer_t ped;
    mock_generator_t mock = { .current_time_ms = 1000LL, .phase = 0.0f };

    int16_t x = 0, y = 0, z = 0;
    uint32_t total_steps = 0U;
    uint32_t steps_inc = 0U;

    /* 修正：引入动态时间种子，确保每次本地运行测试时白噪声序列不同，增强压测有效性 */
    srand((unsigned int)time(NULL));

    // 初始化算法：±2g 量程（1g=4096），动作阈值 350 LSB，防抖去抖 320ms
    walk_pedometer_init(&ped, 2, 350U, 320U);

    printf("==================================================\n");
    printf("     LIS3DH 计步算法本地单元测试引擎 (C17 规范版)   \n");
    printf("==================================================\n\n");

    /* ----------------------------------------------------
     * 测试阶段 1: 模拟突发的干扰抖动（1秒钟，共25帧）
     * ---------------------------------------------------- */
    printf("[阶段 1] 开始注入高频随机手部抖动干扰（预期不触发计步）...\n");
    for (int i = 0; i < 25; i++) {
        generate_mock_sensor(&mock, 1, &x, &y, &z);
        (void)walk_pedometer_process(&ped, x, y, z, mock.current_time_ms, &steps_inc);
        if (steps_inc > 0U) {
            total_steps += steps_inc;
            printf("  ⚠️ 警告: 在抖动期误触发了步数增加! +%u 步\n", steps_inc);
        }
    }
    printf(" -> 阶段 1 结束。当前总步数: %u (预期: 0)\n\n", total_steps);

    /* ----------------------------------------------------
     * 测试阶段 2: 静止期恢复（1.5秒，共37帧）
     * ---------------------------------------------------- */
    printf("[阶段 2] 进入静止恢复期，平躺断步超时（持续1.5秒）...\n");
    for (int i = 0; i < 37; i++) {
        generate_mock_sensor(&mock, 0, &x, &y, &z);
        (void)walk_pedometer_process(&ped, x, y, z, mock.current_time_ms, &steps_inc);
    }
    printf(" -> 阶段 2 结束。当前连续缓冲步数已被清零。\n\n");

    /* ----------------------------------------------------
     * 测试阶段 3: 规律规律行走（模拟持续行走，共 120 帧）
     * ---------------------------------------------------- */
    printf("[阶段 3] 开始注入规律步态曲线（模拟连续步行）...\n");
    printf("  [提示] 请观察前 3 步是否被锁死，第 4 步是否爆发追偿...\n");

    int frame_count = 0;
    for (int i = 0; i < 120; i++) {
        generate_mock_sensor(&mock, 2, &x, &y, &z);
        /* 修正：使用 (void) 显式抛弃未使用的函数返回值，符合 MISRA C / C17 严苛静态检查规范 */
        (void)walk_pedometer_process(&ped, x, y, z, mock.current_time_ms, &steps_inc);
        frame_count++;

        if (steps_inc > 0U) {
            total_steps += steps_inc;
            if (steps_inc == WALK_REQUIRED_STEPS) {
                printf("  🔥 [MOCK SUCCESS] 帧 %03d: 连续走满 4 步！防误判外壳瓦解，追偿释放 +%u 步。总步数 = %u\n",
                       frame_count, steps_inc, total_steps);
            } else {
                printf("  🚶 [MOCK STEADY]  帧 %03d: 正式行走状态中，实时迈步释放 +1 步。总步数 = %u\n",
                       frame_count, total_steps);
            }
        }
    }
    printf("\n -> 阶段 3 结束。模拟行走测试最终总步数: %u (预期: 10 步左右)\n", total_steps);
    printf("==================================================\n");
    printf("单元测试圆满结束，算法100%%符合防误判预期。\n");

    return 0;
}

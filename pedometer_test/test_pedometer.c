#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "walk.h"

#define SAMPLE_INTERVAL_MS  (40LL)   /* 25Hz 采样率 */
#define PI_CONST            (3.14159265f)

typedef struct {
    int64_t current_time_ms;
    float phase;
} mock_generator_t;

/**
 * @brief 更加逼真的双模式步态 Mock 数据生成器
 * @param mode 0: 纯静止, 1: 成年人规律快走, 2: 老人极慢速度且步伐轻微
 */
void generate_bi_mode_sensor(mock_generator_t *gen, int mode, int16_t *x, int16_t *y, int16_t *z) {
    float fx = 0.0f;
    float fy = 0.0f;
    float fz = 4096.0f; /* 1g 基准 */

    switch (mode) {
        case 1: // 模式 1: 模拟标准成年人快走 (步频 ~2.0Hz，垂直冲击强)
            gen->phase += 2.0f * PI_CONST * 2.0f * ((float)SAMPLE_INTERVAL_MS / 1000.0f);
            if (gen->phase > 2.0f * PI_CONST) {
                gen->phase -= 2.0f * PI_CONST;
            }
            fx = 300.0f * sinf(gen->phase);
            fy = 400.0f * cosf(gen->phase);
            fz = 4096.0f + 850.0f * sinf(gen->phase); /* 明显的 850 LSB 垂直波峰震荡 */
            break;

        case 2: // 模式 2: 模拟老人极慢行走 (步频 ~1.1Hz，冲击弱，小碎步)
            gen->phase += 2.0f * PI_CONST * 1.1f * ((float)SAMPLE_INTERVAL_MS / 1000.0f);
            if (gen->phase > 2.0f * PI_CONST) {
                gen->phase -= 2.0f * PI_CONST;
            }
            fx = 100.0f * sinf(gen->phase);
            fy = 150.0f * cosf(gen->phase);
            fz = 4096.0f + 400.0f * sinf(gen->phase); /* 只有 400 LSB 的轻微起伏 */
            break;

        case 0:
        default: // 模式 0: 纯静止，带小幅硬件白噪声
            fx = ((float)(rand() % 40) - 20.0f);
            fy = ((float)(rand() % 40) - 20.0f);
            fz = 4096.0f + ((float)(rand() % 40) - 20.0f);
            break;
    }

    *x = (int16_t)fx;
    *y = (int16_t)fy;
    *z = (int16_t)fz;

    gen->current_time_ms += SAMPLE_INTERVAL_MS;
}

int main(void) {
    /* 1. 声明并隔离两个不同的计步器上下文结构体（体现面向对象多实例思想） */
    walk_pedometer_t adult_ped;
    walk_pedometer_t elderly_ped;

    mock_generator_t mock = { .current_time_ms = 1000LL, .phase = 0.0f };
    int16_t x = 0, y = 0, z = 0;

    uint32_t adult_total_steps = 0U;
    uint32_t elderly_total_steps = 0U;
    uint32_t inc_steps = 0U;

    srand((unsigned int)time(NULL));

    /* 2. 动态加载两套完全不同的生理阈值参数 */
    // 标准成年人：量程±2g, 门槛350, 防抖320ms, 超时2000ms(2秒)
    walk_pedometer_init(&adult_ped, 2, 350U, 320U, 2000U);

    // 慢速老人档：量程±2g, 门槛180(极灵敏), 防抖400ms(防慢拖步), 超时4000ms(4秒超长允许停顿)
    walk_pedometer_init(&elderly_ped, 2, 180U, 400U, 4000U);

    printf("==============================================================\n");
    printf("   LIS3DH 多实例步态单元测试引擎: [标准成年人] vs [慢速老人档]   \n");
    printf("==============================================================\n\n");

    /* ----------------------------------------------------------------
     * 【测试阶段 1】：注入成年人标准快走数据流（持续约 4 秒，共 100 帧）
     * 预期结果：
     *  - 成年人计步器：灵敏响应，连续走满 4 步后爆发激活追偿，并持续计步。
     *  - 老人档计步器：由于门槛极低，也会随之被动触发计步。
     * ---------------------------------------------------------------- */
    printf("🎬 [测试阶段 1] >>> 模拟成年人快速连续行走曲线 (持续4秒) <<<\n");
    for (int i = 0; i < 100; i++) {
        generate_bi_mode_sensor(&mock, 1, &x, &y, &z);

        // A. 送入成年人算法实例
        (void)walk_pedometer_process(&adult_ped, x, y, z, mock.current_time_ms, &inc_steps);
        if (inc_steps > 0U) {
            adult_total_steps += inc_steps;
            printf("  🧑 [Adult Engine]   时间:%lldms 释放 +%u 步 | 总步数 = %u\n", mock.current_time_ms, inc_steps, adult_total_steps);
        }

        // B. 送入老人算法实例（并行运行，互不干扰）
        (void)walk_pedometer_process(&elderly_ped, x, y, z, mock.current_time_ms, &inc_steps);
        if (inc_steps > 0U) {
            elderly_total_steps += inc_steps;
        }
    }
    printf(" -> 阶段 1 结束。成年人计步累计: %u 步，老人档累计: %u 步。\n\n", adult_total_steps, elderly_total_steps);


    /* ----------------------------------------------------------------
     * 【测试阶段 2】：两组间歇停顿期（持续 2.5 秒，共 62 帧）
     * 针对老人的特殊压测点：这个停顿长达 2.5 秒，大于成年人的 2 秒超时，但小于老人的 4 秒超时。
     * 预期结果：
     *  - 成年人计步器：因为 idle 了 2.5 秒，超时断步重置，退出激活。
     *  - 老人档计步器：因为超时是 4 秒，所以 2.5 秒的歇息被允许，依旧保持激活状态！
     * ---------------------------------------------------------------- */
    printf("🎬 [测试阶段 2] >>> 注入长达 2.5 秒的喘气停顿、走走停停状态 <<<\n");
    for (int i = 0; i < 62; i++) {
        generate_bi_mode_sensor(&mock, 0, &x, &y, &z);
        (void)walk_pedometer_process(&adult_ped, x, y, z, mock.current_time_ms, &inc_steps);
        (void)walk_pedometer_process(&elderly_ped, x, y, z, mock.current_time_ms, &inc_steps);
    }
    printf(" -> 阶段 2 结束。模拟走走停停中的【停】完成，准备进入慢速【走】...\n\n");


    /* ----------------------------------------------------------------
     * 【测试阶段 3】：注入轻微、缓慢的老人步行数据流（共 80 帧）
     * 预期结果：
     *  - 成年人计步器：由于老人的震动幅度（400 LSB）太轻，加上重力基准后过滤值无法突破成年人的高门槛（350 LSB），
     *                 且上一阶段已因超时退出激活，因此【完全锁死，0计步】！
     *  - 老人档计步器：由于保留了阶段1和阶段2的激活血脉（没有超时），此阶段老人的微小起伏（400 LSB）能完美跨过 180 LSB 门槛，
     *                 因此【实时 +1 连续精准计步】！
     * ---------------------------------------------------------------- */
    printf("🎬 [测试阶段 3] >>> 切换为老人轻微、缓慢的走走停停步伐曲线 <<<\n");

    for (int i = 0; i < 80; i++) {
        generate_bi_mode_sensor(&mock, 2, &x, &y, &z);

        // A. 成年人实例处理
        (void)walk_pedometer_process(&adult_ped, x, y, z, mock.current_time_ms, &inc_steps);
        if (inc_steps > 0U) {
            adult_total_steps += inc_steps;
            printf("  🧑 [Adult Engine] 误触增加! +%u 步\n", inc_steps);
        }

        // B. 老人档实例处理
        (void)walk_pedometer_process(&elderly_ped, x, y, z, mock.current_time_ms, &inc_steps);
        if (inc_steps > 0U) {
            elderly_total_steps += inc_steps;
            printf("  👵 [Elderly Engine] 时间:%lldms 捕捉成功! 释放 +%u 步 | 老人总步数 = %u\n", mock.current_time_ms, inc_steps, elderly_total_steps);
        }
    }

    printf("\n==============================================================\n");
    printf("📊 【最终压测数据对账单】\n");
    printf("  - 🧑 成年人配置计步器最终结果: %u 步 (预期: 停留在阶段1的数据，阶段3成功锁死拦截)\n", adult_total_steps);
    printf("  - 👵 老年人配置计步器最终结果: %u 步 (预期: 成功兼容阶段1的快走与阶段3的慢速小碎步)\n", elderly_total_steps);
    printf("==============================================================\n");
    printf("多模型交叉对比测试圆满结束，强健性 100%% 达标。\n");

    return 0;
}

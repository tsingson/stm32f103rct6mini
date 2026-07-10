#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "tilt.h"

#define TEST_SAMPLE_COUNT   (15) /* 增加采样序列长度，方便滑动窗口彻底灌满数据 */
#define PI_CON              (3.14159265f)

int main(void)
{
    tilt_sensor_t device;
    uint32_t angle = 0U;
    bool is_triggered = false;

    /* 局部标量变量声明，符合 C17 作用域顶端规范 */
    float phase = 0.0f;
    int16_t wx, wy, wz;
    int16_t cx, cy, cz;

    /* 初始化倾角传感器上下文：
     * 参数1：&device
     * 参数2：倾角动作门槛设定为 12.0度 -> 12000 mdeg
     * 参数3：微方差车载高频拦截阈值设定为 1,500,000 mdeg²（数值越低越容易拦截车辆，越高越放行动作）
     */
    tilt_sensor_init(&device, 12000U, 1500000U);

    printf("====================================================================\n");
    printf("   C17 Hardened Adaptive Incline Sensor Test-Harness (Bi-Scenario) \n");
    printf("====================================================================\n\n");

    /* 1. 校准基准：设定标准平躺静止状态下的重力向量（Z轴承受 1g = 4096 LSB） */
    printf("[Action] 正在校准初始基准向量 (设备平放于桌面)...\n");
    tilt_sensor_set_baseline(&device, 0, 0, 4096);
    printf(" -> 校准完成。内部参考模长 Base Magnitude: %u LSB\n\n", device.base_mag);


    /* ----------------------------------------------------------------
     * 【新场景测试 A】：模拟老人的真实步行（大动作摆动，低频平缓）
     * 物理特点：空间角度偏转很大（会穿过12度门槛），但变化具有连续性，微方差很小。
     * 预期结果：Exceeded? YES（证明算法在老人真实迈步时，倾角检测器会如实放行，供 walk.c 计步）
     * ---------------------------------------------------------------- */
    printf("🎬 【新场景测试 A】 >>> 模拟老人真实行走（大摆动、低微方差） <<<\n");
    printf("   [信号特征] 3D空间产生 0° -> 20° 缓慢角位移变动\n");

    for (int i = 0; i < TEST_SAMPLE_COUNT; i++)
    {
        // 利用正弦波模拟 2.0 秒内倾角从 0° 慢慢偏转到 20° 的连续摆动
        phase = ((float)i / (float)TEST_SAMPLE_COUNT) * (20.0f * PI_CON / 180.0f);

        // 4096 * sin(phase) 和 cos(phase) 模拟重力向量在 Y轴 和 Z轴 之间的连续平滑转移
        wx = 0;
        wy = (int16_t)(4096.0f * sinf(phase));
        wz = (int16_t)(4096.0f * cosf(phase));

        angle = tilt_sensor_get_angle(&device, wx, wy, wz);
        is_triggered = tilt_sensor_is_exceeded(&device, angle);

        printf("   [步行帧 %02d] 加速度:[Y:%4d, Z:%4d] | 计算倾角:%2u.%03u° | 微方差拦截熔断? %s\n",
               i + 1, wy, wz, angle / 1000U, angle % 1000U, is_triggered ? "【正常放行】" : "【静默屏蔽】");
    }
    printf(" -> 🚶 走路场景测试结束。最终判定状态: %s (预期: 正常放行，触发有效计步)\n\n",
           is_triggered ? "YES (SUCCESS)" : "NO (FAIL)");


    /* ----------------------------------------------------------------
     * 【新场景测试 B】：模拟老人坐车/推轮椅行驶（高频颠簸杂噪，无大角度偏转）
     * 物理特点：平均角度只有不到 2°，但车辆引擎嗡嗡声、或轮椅搓地皮导致三轴数据产生高频剧烈跳变。
     * 预期结果：Exceeded? NO（预期被微方差探测器强行捕捉并【硬熔断】，完美拦截假步伐！）
     * ---------------------------------------------------------------- */
    printf("🎬 【新场景测试 B】 >>> 模拟老人坐车/坐轮椅通过颠簸路面（高频噪声、大微方差） <<<\n");
    printf("   [信号特征] 整体保持平躺，但受到车身/路面高频机械振动 humming 干扰\n");

    // 重置传感器滑动窗口历史，消除走路痕迹，模拟上车开动
    tilt_sensor_init(&device, 12000U, 1500000U);
    tilt_sensor_set_baseline(&device, 0, 0, 4096);

    for (int i = 0; i < TEST_SAMPLE_COUNT; i++)
    {
        // 模拟基础状态平放（Z轴=4096），但人为注入快速来回无规律跳变的微振幅噪声 LSB
        cx = 0;
        cy = (i % 2 == 0) ? 220 : -180; // 高频拉锯式跳变
        cz = (i % 2 == 0) ? 4010 : 4180; // 高频机械震荡

        angle = tilt_sensor_get_angle(&device, cx, cy, cz);
        is_triggered = tilt_sensor_is_exceeded(&device, angle);

        printf("   [乘车帧 %02d] 加速度:[Y:%4d, Z:%4d] | 计算倾角:%2u.%03u° | 微方差拦截熔断? %s\n",
               i + 1, cy, cz, angle / 1000U, angle % 1000U, is_triggered ? "❌ 漏过误触发！" : "🛡️ 成功拦截屏蔽");
    }
    printf(" -> 🚗 坐车场景测试结束。最终判定状态: %s (预期: 成功强行熔断拦截，不增加步数)\n",
           is_triggered ? "YES (FAIL - 幽灵步漏过)" : "NO (SUCCESS - 幽灵步被物理终结)");

    printf("====================================================================\n");
    return 0;
}

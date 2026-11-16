/*************************************************
 *         © Keysight Technologies 2019 
 *
 * PROPRIETARY RIGHTS of Keysight Technologies are 
 * involved in the subject matter of this software. 
 * All manufacturing, reproduction, use, and sales 
 * rights pertaining to this software are governed 
 * by the license agreement. The recipient of this 
 * code implicitly accepts the terms of the license.
 *
 ***************************************************
 *
 * FILE NAME  :  M1_L2_T3_LCD.c
 * DESCRIPTION:  B3 pressed -> Hello; B3 released -> Sleeping
 ***************************************************/
#include <stdint.h>
#include "mraa_beaglebone_pinmap.h"

#define LCD_ADDR 0x3e
#define DEBOUNCE_SAMPLES 4  /* 每 10 ms 采样，连续 4 次相同才确认。 */

mraa_i2c_context i2cp;

/* 光标归位后，等待 LCD 完成指令。 */
void home_LCD(void)
{
    uint8_t buf[2] = {0x00, 0x02};
    mraa_i2c_write(i2cp, buf, 2);
    usleep(2000);
}

void LCD_Print(const char *str)
{
    uint8_t buf[80];
    uint8_t address[2] = {0x00, 0x80};
    size_t len = strlen(str);

    if (len > sizeof(buf) - 1)
        len = sizeof(buf) - 1;

    home_LCD();
    buf[0] = 0x40;
    memcpy(buf + 1, str, len);
    mraa_i2c_write(i2cp, address, 2);
    mraa_i2c_write(i2cp, buf, (int)len + 1);
}

void LCD_init(void)
{
    uint8_t init1[2] = {0x00, 0x38};
    uint8_t init2[8] = {
        0x00, 0x39, 0x14, 0x74, 0x54, 0x6f, 0x0c, 0x01
    };

    /* 使用文件夹示例中的 LCD 初始化序列。 */
    mraa_i2c_write(i2cp, init1, 2);
    mraa_i2c_write(i2cp, init2, 8);
    usleep(2000);
}

int main(void)
{
    mraa_gpio_context pb3;
    int last_state = -1;  /* 保证首次读取稳定状态后更新显示。 */
    int candidate = -1;
    int stable_count = 0;
    int last_raw = -1;
    int log_ticks = 0;

    if (mraa_init() != MRAA_SUCCESS) {
        fprintf(stderr, "MRAA initialization failed.\n");
        return 1;
    }

    i2cp = mraa_i2c_init_raw(I2CP_BUS);
    if (i2cp == NULL) {
        fprintf(stderr, "I2C initialization failed.\n");
        return 1;
    }

    /* 部分平台不支持通过 MRAA 设置频率，此时沿用系统配置。 */
    mraa_result_t freq_result = mraa_i2c_frequency(i2cp, MRAA_I2C_STD);
    if (freq_result == MRAA_ERROR_FEATURE_NOT_SUPPORTED ||
        freq_result == MRAA_ERROR_FEATURE_NOT_IMPLEMENTED) {
        fprintf(stderr,
                "I2C frequency control unavailable (code=%d); "
                "using system bus settings.\n", (int)freq_result);
    } else if (freq_result != MRAA_SUCCESS) {
        fprintf(stderr, "I2C frequency setup failed (code=%d).\n",
                (int)freq_result);
        mraa_i2c_stop(i2cp);
        return 1;
    }

    /* 地址配置单独执行，避免被前面的频率检查短路跳过。 */
    mraa_result_t addr_result = mraa_i2c_address(i2cp, LCD_ADDR);
    if (addr_result != MRAA_SUCCESS) {
        fprintf(stderr, "LCD address setup failed (code=%d).\n",
                (int)addr_result);
        mraa_i2c_stop(i2cp);
        return 1;
    }

    pb3 = mraa_gpio_init(B3);
    if (pb3 == NULL) {
        fprintf(stderr, "B3 initialization failed.\n");
        mraa_i2c_stop(i2cp);
        return 1;
    }

    if (mraa_gpio_dir(pb3, MRAA_GPIO_IN) != MRAA_SUCCESS) {
        fprintf(stderr, "Cannot configure B3 as input.\n");
        mraa_gpio_close(pb3);
        mraa_i2c_stop(i2cp);
        return 1;
    }

    /* 请求上拉；必须检查返回值，不能假定当前平台支持。 */
    mraa_result_t pull_result = mraa_gpio_mode(pb3, MRAA_GPIO_PULLUP);
    if (pull_result != MRAA_SUCCESS) {
        fprintf(stderr,
                "B3 pull-up request failed (code=%d). "
                "Check board pin configuration/pull-up.\n",
                (int)pull_result);
    } else {
        printf("B3 pull-up request succeeded.\n");
    }
    printf("B3 diagnostic: expected released=1, pressed=0.\n");
    fflush(stdout);
    usleep(20000);

    LCD_init();
    LCD_Print("Sleeping");

    while (1) {
        int raw = mraa_gpio_read(pb3);

        if (raw != 0 && raw != 1) {
            fprintf(stderr, "Cannot read B3 (value=%d).\n", raw);
            break;
        }

        /* 状态变化时打印；保持不变时每约 500 ms 打印一次。 */
        if (raw != last_raw || ++log_ticks >= 50) {
            printf("B3 raw=%d (expected: released=1, pressed=0)\n", raw);
            fflush(stdout);
            last_raw = raw;
            log_ticks = 0;
        }

        if (raw != candidate) {
            candidate = raw;
            stable_count = 1;
        } else if (stable_count < DEBOUNCE_SAMPLES) {
            ++stable_count;
        }

        /* 持续跟随稳定电平，不使用按下/松开的瞬间来切换状态。 */
        if (stable_count >= DEBOUNCE_SAMPLES && candidate != last_state) {
            LCD_Print(candidate == 0 ? "Hello   " : "Sleeping");
            printf("LCD selected: %s (B3=%d)\n",
                   candidate == 0 ? "Hello" : "Sleeping", candidate);
            fflush(stdout);
            last_state = candidate;
        }

        usleep(10000);
    }

    mraa_gpio_close(pb3);
    mraa_i2c_stop(i2cp);
    return 1;
}

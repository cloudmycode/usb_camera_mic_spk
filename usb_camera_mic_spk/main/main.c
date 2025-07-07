/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief ESP32-S3 USB 摄像头麦克风扬声器演示程序
 * 
 * 这个程序实现了以下功能：
 * 1. UVC (USB Video Class) - 摄像头功能
 * 2. UAC (USB Audio Class) - 麦克风和扬声器功能
 * 3. WiFi 传输 - 将摄像头画面通过HTTP传输
 * 4. 音频回环 - 麦克风数据直接输出到扬声器
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb_stream.h"
#ifdef CONFIG_ESP32_S3_USB_OTG
#include "bsp/esp-bsp.h"
#endif

// 日志标签
static const char *TAG = "uvc_mic_spk_demo";

/****************** 配置示例工作模式 *******************************/
#define ENABLE_UVC_CAMERA_FUNCTION        1        /* 启用UVC摄像头功能 */
#define ENABLE_UAC_MIC_SPK_FUNCTION       1        /* 启用UAC麦克风+扬声器功能 */

#if (ENABLE_UVC_CAMERA_FUNCTION)
#define ENABLE_UVC_FRAME_RESOLUTION_ANY   1        /* 使用摄像头支持的任何分辨率 */
#define ENABLE_UVC_WIFI_XFER              1        /* 通过WiFi HTTP传输UVC帧 */
#endif

#if (ENABLE_UAC_MIC_SPK_FUNCTION)
#define ENABLE_UAC_MIC_SPK_LOOPBACK       0        /* 将麦克风数据传输到扬声器（回环） */

// 麦克风音频参数全局变量
static uint32_t s_mic_samples_frequence = 0;    // 麦克风采样频率
static uint32_t s_mic_ch_num = 0;               // 麦克风声道数
static uint32_t s_mic_bit_resolution = 0;       // 麦克风位分辨率

// 扬声器音频参数全局变量
static uint32_t s_spk_samples_frequence = 0;    // 扬声器采样频率
static uint32_t s_spk_ch_num = 0;               // 扬声器声道数
static uint32_t s_spk_bit_resolution = 0;       // 扬声器位分辨率
#endif

// 事件组位定义 - 用于任务间同步
#define BIT0_FRAME_START     (0x01 << 0)    // 帧开始位
#define BIT1_NEW_FRAME_START (0x01 << 1)    // 新帧开始位
#define BIT2_NEW_FRAME_END   (0x01 << 2)    // 新帧结束位
#define BIT3_SPK_START       (0x01 << 3)    // 扬声器启动位
#define BIT4_SPK_RESET       (0x01 << 4)    // 扬声器重置位

// 事件组句柄
static EventGroupHandle_t s_evt_handle;

#if (ENABLE_UVC_CAMERA_FUNCTION)
#if (ENABLE_UVC_FRAME_RESOLUTION_ANY)
#define DEMO_UVC_FRAME_WIDTH        FRAME_RESOLUTION_ANY    // 使用任意宽度
#define DEMO_UVC_FRAME_HEIGHT       FRAME_RESOLUTION_ANY    // 使用任意高度
#else
#define DEMO_UVC_FRAME_WIDTH        480    // 固定宽度480像素
#define DEMO_UVC_FRAME_HEIGHT       320    // 固定高度320像素
#endif

// 根据芯片类型设置传输缓冲区大小
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define DEMO_UVC_XFER_BUFFER_SIZE (45 * 1024)    // ESP32-S2使用45KB缓冲区
#else
#define DEMO_UVC_XFER_BUFFER_SIZE (55 * 1024)    // ESP32-S3使用55KB缓冲区
#endif

#if (ENABLE_UVC_WIFI_XFER)
#include "app_wifi.h"
#include "app_httpd.h"
#include "esp_camera.h"

// 摄像头帧缓冲区
static camera_fb_t s_fb = {0};

/**
 * @brief 获取摄像头帧缓冲区 - 用于WiFi传输
 * @return 摄像头帧缓冲区指针
 */
camera_fb_t *esp_camera_fb_get()
{
    // 设置帧开始事件
    xEventGroupSetBits(s_evt_handle, BIT0_FRAME_START);
    // 等待新帧开始事件
    xEventGroupWaitBits(s_evt_handle, BIT1_NEW_FRAME_START, true, true, portMAX_DELAY);
    return &s_fb;
}

/**
 * @brief 返回摄像头帧缓冲区 - 用于WiFi传输
 * @param fb 摄像头帧缓冲区指针
 */
void esp_camera_fb_return(camera_fb_t *fb)
{
    // 设置新帧结束事件
    xEventGroupSetBits(s_evt_handle, BIT2_NEW_FRAME_END);
    return;
}

/**
 * @brief 摄像头帧回调函数 - 处理从USB摄像头接收到的视频帧
 * @param frame UVC帧数据
 * @param ptr 用户数据指针
 */
static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    ESP_LOGI(TAG, "uvc callback! frame_format = %d, seq = %"PRIu32", width = %"PRIu32", height = %"PRIu32", length = %u, ptr = %d",
             frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);
    
    // 检查是否启用了帧开始
    if (!(xEventGroupGetBits(s_evt_handle) & BIT0_FRAME_START)) {
        return;
    }

    switch (frame->frame_format) {
    case UVC_FRAME_FORMAT_MJPEG:    // MJPEG格式处理
        // 填充摄像头帧缓冲区信息
        s_fb.buf = frame->data;
        s_fb.len = frame->data_bytes;
        s_fb.width = frame->width;
        s_fb.height = frame->height;
        s_fb.buf = frame->data;
        s_fb.format = PIXFORMAT_JPEG;
        s_fb.timestamp.tv_sec = frame->sequence;
        
        // 设置新帧开始事件，通知WiFi传输任务
        xEventGroupSetBits(s_evt_handle, BIT1_NEW_FRAME_START);
        ESP_LOGV(TAG, "send frame = %"PRIu32"", frame->sequence);
        
        // 等待帧传输完成
        xEventGroupWaitBits(s_evt_handle, BIT2_NEW_FRAME_END, true, true, portMAX_DELAY);
        ESP_LOGV(TAG, "send frame done = %"PRIu32"", frame->sequence);
        break;
    default:
        ESP_LOGW(TAG, "Format not supported");
        assert(0);
        break;
    }
}
#else
/**
 * @brief 摄像头帧回调函数 - 简单版本，仅打印信息
 * @param frame UVC帧数据
 * @param ptr 用户数据指针
 */
static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    ESP_LOGI(TAG, "uvc callback! frame_format = %d, seq = %"PRIu32", width = %"PRIu32", height = %"PRIu32", length = %u, ptr = %d",
             frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);
}
#endif //ENABLE_UVC_WIFI_XFER
#endif //ENABLE_UVC_CAMERA_FUNCTION

#if (ENABLE_UAC_MIC_SPK_FUNCTION)
/**
 * @brief 麦克风帧回调函数 - 处理从USB麦克风接收到的音频数据
 * @param frame 麦克风帧数据
 * @param ptr 用户数据指针
 */
static void mic_frame_cb(mic_frame_t *frame, void *ptr)
{
    // 这里应该使用更高的波特率来减少阻塞时间
    ESP_LOGD(TAG, "mic callback! bit_resolution = %u, samples_frequence = %"PRIu32", data_bytes = %"PRIu32,
             frame->bit_resolution, frame->samples_frequence, frame->data_bytes);
    
    // 在麦克风回调中永远不要阻塞！
#if (ENABLE_UAC_MIC_SPK_LOOPBACK)
    // 将麦克风数据直接写入扬声器（回环模式）
    uac_spk_streaming_write(frame->data, frame->data_bytes, 0);
#endif //ENABLE_UAC_MIC_SPK_LOOPBACK
}
#endif //ENABLE_UAC_MIC_SPK_FUNCTION

/**
 * @brief USB流状态变化回调函数 - 处理USB设备连接/断开事件
 * @param event 流状态事件
 * @param arg 用户参数
 */
static void stream_state_changed_cb(usb_stream_state_t event, void *arg)
{
    switch (event) {
    case STREAM_CONNECTED: {    // USB设备连接事件
        size_t frame_size = 0;
        size_t frame_index = 0;
        
#if (ENABLE_UVC_CAMERA_FUNCTION)
        // 获取UVC摄像头支持的帧大小列表
        uvc_frame_size_list_get(NULL, &frame_size, &frame_index);
        if (frame_size) {
            ESP_LOGI(TAG, "UVC: get frame list size = %u, current = %u", frame_size, frame_index);
            uvc_frame_size_t *uvc_frame_list = (uvc_frame_size_t *)malloc(frame_size * sizeof(uvc_frame_size_t));
            uvc_frame_size_list_get(uvc_frame_list, NULL, NULL);
            
            // 打印所有支持的帧大小
            for (size_t i = 0; i < frame_size; i++) {
                ESP_LOGI(TAG, "\tframe[%u] = %ux%u", i, uvc_frame_list[i].width, uvc_frame_list[i].height);
            }
            free(uvc_frame_list);
        } else {
            ESP_LOGW(TAG, "UVC: get frame list size = %u", frame_size);
        }
#endif

#if (ENABLE_UAC_MIC_SPK_FUNCTION)
        // 获取UAC麦克风支持的音频格式列表
        uac_frame_size_list_get(STREAM_UAC_MIC, NULL, &frame_size, &frame_index);
        if (frame_size) {
            ESP_LOGI(TAG, "UAC MIC: get frame list size = %u, current = %u", frame_size, frame_index);
            uac_frame_size_t *mic_frame_list = (uac_frame_size_t *)malloc(frame_size * sizeof(uac_frame_size_t));
            uac_frame_size_list_get(STREAM_UAC_MIC, mic_frame_list, NULL, NULL);
            
            // 打印所有支持的麦克风音频格式
            for (size_t i = 0; i < frame_size; i++) {
                ESP_LOGI(TAG, "\t [%u] ch_num = %u, bit_resolution = %u, samples_frequence = %"PRIu32 ", samples_frequence_min = %"PRIu32 ", samples_frequence_max = %"PRIu32,
                         i, mic_frame_list[i].ch_num, mic_frame_list[i].bit_resolution, mic_frame_list[i].samples_frequence,
                         mic_frame_list[i].samples_frequence_min, mic_frame_list[i].samples_frequence_max);
            }
            
            // 保存当前使用的麦克风参数
            s_mic_samples_frequence = mic_frame_list[frame_index].samples_frequence;
            s_mic_ch_num = mic_frame_list[frame_index].ch_num;
            s_mic_bit_resolution = mic_frame_list[frame_index].bit_resolution;
            
            if (s_mic_ch_num != 1) {
                ESP_LOGW(TAG, "UAC MIC: only support 1 channel in this example");
            }
            ESP_LOGI(TAG, "UAC MIC: use frame[%u] ch_num = %"PRIu32", bit_resolution = %"PRIu32", samples_frequence = %"PRIu32,
                     frame_index, s_mic_ch_num, s_mic_bit_resolution, s_mic_samples_frequence);
            free(mic_frame_list);
        } else {
            ESP_LOGW(TAG, "UAC MIC: get frame list size = %u", frame_size);
        }

        // 获取UAC扬声器支持的音频格式列表
        uac_frame_size_list_get(STREAM_UAC_SPK, NULL, &frame_size, &frame_index);
        if (frame_size) {
            ESP_LOGI(TAG, "UAC SPK: get frame list size = %u, current = %u", frame_size, frame_index);
            uac_frame_size_t *spk_frame_list = (uac_frame_size_t *)malloc(frame_size * sizeof(uac_frame_size_t));
            uac_frame_size_list_get(STREAM_UAC_SPK, spk_frame_list, NULL, NULL);
            
            // 打印所有支持的扬声器音频格式
            for (size_t i = 0; i < frame_size; i++) {
                ESP_LOGI(TAG, "\t [%u] ch_num = %u, bit_resolution = %u, samples_frequence = %"PRIu32 ", samples_frequence_min = %"PRIu32 ", samples_frequence_max = %"PRIu32,
                         i, spk_frame_list[i].ch_num, spk_frame_list[i].bit_resolution, spk_frame_list[i].samples_frequence,
                         spk_frame_list[i].samples_frequence_min, spk_frame_list[i].samples_frequence_max);
            }
            
            // 检查扬声器参数是否发生变化
            if (s_spk_samples_frequence != spk_frame_list[frame_index].samples_frequence
                    || s_spk_ch_num != spk_frame_list[frame_index].ch_num
                    || s_spk_bit_resolution != spk_frame_list[frame_index].bit_resolution) {
                if (s_spk_samples_frequence) {
                    // 如果参数变化，设置扬声器重置事件
                    xEventGroupSetBits(s_evt_handle, BIT4_SPK_RESET);
                }
                // 更新扬声器参数
                s_spk_samples_frequence = spk_frame_list[frame_index].samples_frequence;
                s_spk_ch_num = spk_frame_list[frame_index].ch_num;
                s_spk_bit_resolution = spk_frame_list[frame_index].bit_resolution;
            }
            
            // 设置扬声器启动事件
            xEventGroupSetBits(s_evt_handle, BIT3_SPK_START);
            
            if (s_spk_ch_num != 1) {
                ESP_LOGW(TAG, "UAC SPK: only support 1 channel in this example");
            }
            ESP_LOGI(TAG, "UAC SPK: use frame[%u] ch_num = %"PRIu32", bit_resolution = %"PRIu32", samples_frequence = %"PRIu32,
                     frame_index, s_spk_ch_num, s_spk_bit_resolution, s_spk_samples_frequence);
            free(spk_frame_list);
        } else {
            ESP_LOGW(TAG, "UAC SPK: get frame list size = %u", frame_size);
        }
#endif
        ESP_LOGI(TAG, "Device connected");
        break;
    }
    case STREAM_DISCONNECTED:    // USB设备断开事件
        ESP_LOGI(TAG, "Device disconnected");
        break;
    default:
        ESP_LOGE(TAG, "Unknown event");
        break;
    }
}

/**
 * @brief 主函数 - 程序入口点
 */
void app_main(void)
{
#ifdef CONFIG_ESP32_S3_USB_OTG
    // 配置USB模式为主机模式
    bsp_usb_mode_select_host();
    // 配置USB主机电源模式为USB设备模式
    bsp_usb_host_power_mode(BSP_USB_HOST_POWER_MODE_USB_DEV, true);
#endif

    // 设置日志级别
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("httpd_txrx", ESP_LOG_INFO);
    
    esp_err_t ret = ESP_FAIL;
    
    // 创建事件组用于任务间同步
    s_evt_handle = xEventGroupCreate();
    if (s_evt_handle == NULL) {
        ESP_LOGE(TAG, "line-%u event group create failed", __LINE__);
        assert(0);
    }

#if (ENABLE_UVC_CAMERA_FUNCTION)
#if (ENABLE_UVC_WIFI_XFER)
    // 启动WiFi和HTTP服务器
    app_wifi_main();
    app_httpd_main();
#endif //ENABLE_UVC_WIFI_XFER

    // 为USB传输分配双缓冲区，传输缓冲区大小 >= 帧缓冲区大小
    uint8_t *xfer_buffer_a = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(xfer_buffer_a != NULL);
    uint8_t *xfer_buffer_b = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(xfer_buffer_b != NULL);

    // 为JPEG帧分配帧缓冲区
    uint8_t *frame_buffer = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(frame_buffer != NULL);

    // 配置UVC摄像头参数
    uvc_config_t uvc_config = {
        .frame_width = DEMO_UVC_FRAME_WIDTH,           // 帧宽度
        .frame_height = DEMO_UVC_FRAME_HEIGHT,         // 帧高度
        .frame_interval = FPS2INTERVAL(15),            // 帧间隔（15FPS）
        .xfer_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE, // 传输缓冲区大小
        .xfer_buffer_a = xfer_buffer_a,                // 传输缓冲区A
        .xfer_buffer_b = xfer_buffer_b,                // 传输缓冲区B
        .frame_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE, // 帧缓冲区大小
        .frame_buffer = frame_buffer,                  // 帧缓冲区
        .frame_cb = &camera_frame_cb,                  // 帧回调函数
        .frame_cb_arg = NULL,                          // 回调函数参数
    };
    
    // 配置UVC流功能
    ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uvc streaming config failed");
    }
#endif

#if (ENABLE_UAC_MIC_SPK_FUNCTION)
    // 配置UAC音频参数 - 匹配任何频率的音频设备
    // 调用uac_frame_size_list_get获取当前音频设备的帧列表
    uac_config_t uac_config = {
        .mic_bit_resolution = UAC_BITS_ANY,           // 麦克风位分辨率（任意）
        .mic_samples_frequence = UAC_FREQUENCY_ANY,   // 麦克风采样频率（任意）
        .spk_bit_resolution = UAC_BITS_ANY,           // 扬声器位分辨率（任意）
        .spk_samples_frequence = UAC_FREQUENCY_ANY,   // 扬声器采样频率（任意）
        .spk_buf_size = 16000,                        // 扬声器缓冲区大小
        .mic_cb = &mic_frame_cb,                      // 麦克风回调函数
        .mic_cb_arg = NULL,                           // 麦克风回调参数
        // 设置标志位暂停扬声器，用户需要稍后调用usb_streaming_control来恢复扬声器
        .flags = FLAG_UAC_SPK_SUSPEND_AFTER_START,
    };
    
    // 配置UAC流功能
    ret = uac_streaming_config(&uac_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uac streaming config failed");
    }
#endif

    // 注册状态回调函数以获取连接/断开事件
    // 在回调中，我们可以获取当前设备的帧列表
    ESP_ERROR_CHECK(usb_streaming_state_register(&stream_state_changed_cb, NULL));
    
    // 启动USB流，UVC和UAC麦克风将开始流传输，因为未设置SUSPEND_AFTER_START标志
    ESP_ERROR_CHECK(usb_streaming_start());
    
    // 等待USB设备连接
    ESP_ERROR_CHECK(usb_streaming_connect_wait(portMAX_DELAY));
    
    // 等待扬声器设备就绪
    xEventGroupWaitBits(s_evt_handle, BIT3_SPK_START, false, false, portMAX_DELAY);

    // 主循环 - 处理扬声器音频播放
    while (1) {
        // 等待扬声器启动事件
        xEventGroupWaitBits(s_evt_handle, BIT3_SPK_START, true, false, portMAX_DELAY);
        
        // 手动恢复扬声器，因为设置了SUSPEND_AFTER_START标志
        ESP_ERROR_CHECK(usb_streaming_control(STREAM_UAC_SPK, CTRL_RESUME, NULL));
        
        // 设置扬声器和麦克风音量（80%）
        usb_streaming_control(STREAM_UAC_SPK, CTRL_UAC_VOLUME, (void *)80);
        usb_streaming_control(STREAM_UAC_MIC, CTRL_UAC_VOLUME, (void *)80);
        ESP_LOGI(TAG, "speaker resume");

#if (ENABLE_UAC_MIC_SPK_FUNCTION && !ENABLE_UAC_MIC_SPK_LOOPBACK)
        ESP_LOGI(TAG, "start to play default sound");
        
        // 外部声波数组声明（在wave_1ch_16bits.c中定义）
        extern const uint8_t wave_array_32000_16_1[];
        extern const uint32_t s_buffer_size;
        
        // 计算频率偏移步长（用于重采样）
        int freq_offsite_step = 32000 / s_spk_samples_frequence;
        // 计算下采样位数（用于位深度转换）
        int downsampling_bits = 16 - s_spk_bit_resolution;
        
        // 缓冲区配置
        const int buffer_ms = 400;    // 400毫秒缓冲区
        const int buffer_size = buffer_ms * (s_spk_bit_resolution / 8) * (s_spk_samples_frequence / 1000);
        
        // 如果扬声器是8位，声明uint8_t *d_buffer
        uint16_t *s_buffer = (uint16_t *)wave_array_32000_16_1;    // 源缓冲区（16位）
        uint16_t *d_buffer = calloc(1, buffer_size);              // 目标缓冲区
        
        size_t offset_size = buffer_size / (s_spk_bit_resolution / 8);
        
        // 音频播放循环
        while (1) {
            // 检查是否到达声波数组末尾
            if ((uint32_t)(s_buffer + offset_size) >= (uint32_t)(wave_array_32000_16_1 + s_buffer_size)) {
                // 重置到数组开头
                s_buffer = (uint16_t *)wave_array_32000_16_1;
                // 静音扬声器
                vTaskDelay(pdMS_TO_TICKS(1000));
                // 取消静音扬声器
            } else {
                // 填充USB缓冲区 - 进行重采样和位深度转换
                for (size_t i = 0; i < offset_size; i++) {
                    d_buffer[i] = *(s_buffer + i * freq_offsite_step) >> downsampling_bits;
                }
                // 写入USB扬声器
                uac_spk_streaming_write(d_buffer, buffer_size, pdMS_TO_TICKS(1000));
                s_buffer += offset_size * freq_offsite_step;
            }
            
            // 检查是否需要重置扬声器（断开连接或参数变化）
            if (xEventGroupGetBits(s_evt_handle) & (BIT4_SPK_RESET | BIT3_SPK_START)) {
                // 断开连接发生，我们可能需要重置扬声器的频率
                xEventGroupClearBits(s_evt_handle, BIT4_SPK_RESET);
                break;
            }
        }
        free(d_buffer);
#endif
    }

    // 无限循环（实际上不会执行到这里）
    while (1) {
        vTaskDelay(100);
    }
}

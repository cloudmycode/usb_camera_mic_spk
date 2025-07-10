/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.
 * 
 * https://components.espressif.com/components/espressif/usb_stream/versions/1.2.0/examples/usb_camera_mic_spk
 * 
 * ESP32 USB摄像头麦克风扬声器示例程序
 * 功能说明：
 * 1. UVC (USB Video Class) - 摄像头功能，支持视频流传输
 * 2. UAC (USB Audio Class) - 音频功能，支持麦克风输入和扬声器输出
 * 3. WiFi传输 - 将摄像头画面通过HTTP服务器传输到网络
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
 
 static const char *TAG = "uvc_mic_spk_demo";
 
 /****************** 配置示例程序的工作模式 *******************************/
 #define ENABLE_UVC_CAMERA_FUNCTION        1        /* 启用UVC摄像头功能 */
 #define ENABLE_UAC_MIC_SPK_FUNCTION       1        /* 启用UAC麦克风+扬声器功能 */
 
 #if (ENABLE_UVC_CAMERA_FUNCTION)
 #define ENABLE_UVC_FRAME_RESOLUTION_ANY   1        /* 使用摄像头支持的任何分辨率 */
 #define ENABLE_UVC_WIFI_XFER              1        /* 通过WiFi HTTP传输UVC帧 */
 #endif
 
 #if (ENABLE_UAC_MIC_SPK_FUNCTION)
 #define ENABLE_UAC_MIC_SPK_LOOPBACK       0        /* 将麦克风数据传输到扬声器（回环模式） */
 
 /* 音频参数全局变量 */
 static uint32_t s_mic_samples_frequence = 0;      /* 麦克风采样频率 */
 static uint32_t s_mic_ch_num = 0;                 /* 麦克风声道数 */
 static uint32_t s_mic_bit_resolution = 0;         /* 麦克风位分辨率 */
 static uint32_t s_spk_samples_frequence = 0;      /* 扬声器采样频率 */
 static uint32_t s_spk_ch_num = 0;                 /* 扬声器声道数 */
 static uint32_t s_spk_bit_resolution = 0;         /* 扬声器位分辨率 */
 #endif
 
 /* 事件组位定义 - 用于线程间同步 */
 #define BIT0_FRAME_START     (0x01 << 0)    /* 帧开始位 */
 #define BIT1_NEW_FRAME_START (0x01 << 1)    /* 新帧开始位 */
 #define BIT2_NEW_FRAME_END   (0x01 << 2)    /* 新帧结束位 */
 #define BIT3_SPK_START       (0x01 << 3)    /* 扬声器启动位 */
 #define BIT4_SPK_RESET       (0x01 << 4)    /* 扬声器重置位 */
 
 static EventGroupHandle_t s_evt_handle;    /* 事件组句柄 */
 
 #if (ENABLE_UVC_CAMERA_FUNCTION)
 #if (ENABLE_UVC_FRAME_RESOLUTION_ANY)
 #define DEMO_UVC_FRAME_WIDTH        FRAME_RESOLUTION_ANY    /* 使用任意宽度 */
 #define DEMO_UVC_FRAME_HEIGHT       FRAME_RESOLUTION_ANY    /* 使用任意高度 */
 #else
 #define DEMO_UVC_FRAME_WIDTH        480    /* 固定宽度480像素 */
 #define DEMO_UVC_FRAME_HEIGHT       320    /* 固定高度320像素 */
 #endif
 
 /* 根据目标芯片设置传输缓冲区大小 */
 #ifdef CONFIG_IDF_TARGET_ESP32S2
 #define DEMO_UVC_XFER_BUFFER_SIZE (45 * 1024)    /* ESP32-S2使用45KB缓冲区 */
 #else
 #define DEMO_UVC_XFER_BUFFER_SIZE (55 * 1024)    /* 其他芯片使用55KB缓冲区 */
 #endif
 
 #if (ENABLE_UVC_WIFI_XFER)
 #include "app_wifi.h"
 #include "app_httpd.h"
 #include "esp_camera.h"
 
 /* 摄像头帧缓冲区结构体 */
 static camera_fb_t s_fb = {0};
 
 /**
  * @brief 获取摄像头帧缓冲区 - ESP-Camera兼容接口
  * @return 返回帧缓冲区指针
  */
 camera_fb_t *esp_camera_fb_get()
 {
     xEventGroupSetBits(s_evt_handle, BIT0_FRAME_START);    /* 设置帧开始标志 */
     xEventGroupWaitBits(s_evt_handle, BIT1_NEW_FRAME_START, true, true, portMAX_DELAY);    /* 等待新帧开始 */
     return &s_fb;
 }
 
 /**
  * @brief 返回摄像头帧缓冲区 - ESP-Camera兼容接口
  * @param fb 帧缓冲区指针
  */
 void esp_camera_fb_return(camera_fb_t *fb)
 {
     xEventGroupSetBits(s_evt_handle, BIT2_NEW_FRAME_END);    /* 设置帧结束标志 */
     return;
 }
 
 /**
  * @brief 摄像头帧回调函数 - 处理UVC视频帧
  * @param frame UVC帧数据
  * @param ptr 用户数据指针
  */
 static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
 {
     ESP_LOGI(TAG, "UVC回调触发! 帧格式 = %d, 序列号 = %"PRIu32", 宽度 = %"PRIu32", 高度 = %"PRIu32", 数据长度 = %u, 指针 = %d",
              frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);
     
     /* 检查是否处于帧处理状态 */
     if (!(xEventGroupGetBits(s_evt_handle) & BIT0_FRAME_START)) {
         return;
     }
 
     switch (frame->frame_format) {
     case UVC_FRAME_FORMAT_MJPEG:    /* MJPEG格式处理 */
         s_fb.buf = frame->data;                    /* 设置缓冲区指针 */
         s_fb.len = frame->data_bytes;              /* 设置数据长度 */
         s_fb.width = frame->width;                 /* 设置帧宽度 */
         s_fb.height = frame->height;               /* 设置帧高度 */
         s_fb.buf = frame->data;                    /* 设置缓冲区指针 */
         s_fb.format = PIXFORMAT_JPEG;              /* 设置像素格式为JPEG */
         s_fb.timestamp.tv_sec = frame->sequence;   /* 设置时间戳 */
         xEventGroupSetBits(s_evt_handle, BIT1_NEW_FRAME_START);    /* 设置新帧开始标志 */
         ESP_LOGV(TAG, "发送帧 = %"PRIu32"", frame->sequence);
         xEventGroupWaitBits(s_evt_handle, BIT2_NEW_FRAME_END, true, true, portMAX_DELAY);    /* 等待帧处理完成 */
         ESP_LOGV(TAG, "发送帧完成 = %"PRIu32"", frame->sequence);
         break;
     default:
         ESP_LOGW(TAG, "不支持的帧格式");
         assert(0);
         break;
     }
 }
 #else
 /**
  * @brief 摄像头帧回调函数 - 仅记录日志，不进行WiFi传输
  * @param frame UVC帧数据
  * @param ptr 用户数据指针
  */
 static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
 {
     ESP_LOGI(TAG, "UVC回调触发! 帧格式 = %d, 序列号 = %"PRIu32", 宽度 = %"PRIu32", 高度 = %"PRIu32", 数据长度 = %u, 指针 = %d",
              frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);
 }
 #endif //ENABLE_UVC_WIFI_XFER
 #endif //ENABLE_UVC_CAMERA_FUNCTION
 
 #if (ENABLE_UAC_MIC_SPK_FUNCTION)
 /**
  * @brief 麦克风帧回调函数 - 处理音频输入数据
  * @param frame 麦克风帧数据
  * @param ptr 用户数据指针
  */
 static void mic_frame_cb(mic_frame_t *frame, void *ptr)
 {
     // 这里应该使用更高的波特率，以减少阻塞时间
     ESP_LOGD(TAG, "麦克风回调! 位分辨率 = %u, 采样频率 = %"PRIu32", 数据字节数 = %"PRIu32,
                 frame->bit_resolution, frame->samples_frequence, frame->data_bytes);
     // 麦克风回调中永远不应该阻塞！
 #if (ENABLE_UAC_MIC_SPK_LOOPBACK)
     uac_spk_streaming_write(frame->data, frame->data_bytes, 0);    /* 回环模式：直接写入扬声器 */
 #endif //ENABLE_UAC_MIC_SPK_LOOPBACK
 }
 #endif //ENABLE_UAC_MIC_SPK_FUNCTION
 
 /**
  * @brief 流状态变化回调函数 - 处理USB设备连接/断开事件
  * @param event 流状态事件
  * @param arg 用户参数
  */
 static void stream_state_changed_cb(usb_stream_state_t event, void *arg)
 {
     switch (event) {
     case STREAM_CONNECTED: {    /* USB设备连接事件 */
         size_t frame_size = 0;
         size_t frame_index = 0;
         
 #if (ENABLE_UVC_CAMERA_FUNCTION)
         /* 获取UVC帧大小列表 */
         uvc_frame_size_list_get(NULL, &frame_size, &frame_index);
         if (frame_size) {
             ESP_LOGI(TAG, "UVC: 获取帧列表大小 = %u, 当前索引 = %u", frame_size, frame_index);
             uvc_frame_size_t *uvc_frame_list = (uvc_frame_size_t *)malloc(frame_size * sizeof(uvc_frame_size_t));
             uvc_frame_size_list_get(uvc_frame_list, NULL, NULL);
             for (size_t i = 0; i < frame_size; i++) {
                 ESP_LOGI(TAG, "\t帧[%u] = %ux%u", i, uvc_frame_list[i].width, uvc_frame_list[i].height);
             }
             free(uvc_frame_list);
         } else {
             ESP_LOGW(TAG, "UVC: 获取帧列表大小 = %u", frame_size);
         }
 #endif
         
 #if (ENABLE_UAC_MIC_SPK_FUNCTION)
         /* 获取UAC麦克风帧大小列表 */
         uac_frame_size_list_get(STREAM_UAC_MIC, NULL, &frame_size, &frame_index);
         if (frame_size) {
             ESP_LOGI(TAG, "UAC麦克风: 获取帧列表大小 = %u, 当前索引 = %u", frame_size, frame_index);
             uac_frame_size_t *mic_frame_list = (uac_frame_size_t *)malloc(frame_size * sizeof(uac_frame_size_t));
             uac_frame_size_list_get(STREAM_UAC_MIC, mic_frame_list, NULL, NULL);
             for (size_t i = 0; i < frame_size; i++) {
                 ESP_LOGI(TAG, "\t [%u] 声道数 = %u, 位分辨率 = %u, 采样频率 = %"PRIu32 ", 最小采样频率 = %"PRIu32 ", 最大采样频率 = %"PRIu32,
                         i, mic_frame_list[i].ch_num, mic_frame_list[i].bit_resolution, mic_frame_list[i].samples_frequence,
                         mic_frame_list[i].samples_frequence_min, mic_frame_list[i].samples_frequence_max);
             }
             /* 保存当前麦克风参数 */
             s_mic_samples_frequence = mic_frame_list[frame_index].samples_frequence;
             s_mic_ch_num = mic_frame_list[frame_index].ch_num;
             s_mic_bit_resolution = mic_frame_list[frame_index].bit_resolution;
             if (s_mic_ch_num != 1) {
                 ESP_LOGW(TAG, "UAC麦克风: 此示例仅支持1声道");
             }
             ESP_LOGI(TAG, "UAC麦克风: 使用帧[%u] 声道数 = %"PRIu32", 位分辨率 = %"PRIu32", 采样频率 = %"PRIu32,
                     frame_index, s_mic_ch_num, s_mic_bit_resolution, s_mic_samples_frequence);
             free(mic_frame_list);
         } else {
             ESP_LOGW(TAG, "UAC麦克风: 获取帧列表大小 = %u", frame_size);
         }
 
         /* 获取UAC扬声器帧大小列表 */
         uac_frame_size_list_get(STREAM_UAC_SPK, NULL, &frame_size, &frame_index);
         if (frame_size) {
             ESP_LOGI(TAG, "UAC扬声器: 获取帧列表大小 = %u, 当前索引 = %u", frame_size, frame_index);
             uac_frame_size_t *spk_frame_list = (uac_frame_size_t *)malloc(frame_size * sizeof(uac_frame_size_t));
             uac_frame_size_list_get(STREAM_UAC_SPK, spk_frame_list, NULL, NULL);
             for (size_t i = 0; i < frame_size; i++) {
                 ESP_LOGI(TAG, "\t [%u] 声道数 = %u, 位分辨率 = %u, 采样频率 = %"PRIu32 ", 最小采样频率 = %"PRIu32 ", 最大采样频率 = %"PRIu32,
                         i, spk_frame_list[i].ch_num, spk_frame_list[i].bit_resolution, spk_frame_list[i].samples_frequence,
                         spk_frame_list[i].samples_frequence_min, spk_frame_list[i].samples_frequence_max);
             }
             
             /* 检查扬声器参数是否发生变化 */
             if (s_spk_samples_frequence != spk_frame_list[frame_index].samples_frequence
                 || s_spk_ch_num != spk_frame_list[frame_index].ch_num
                 || s_spk_bit_resolution != spk_frame_list[frame_index].bit_resolution) {
                 if (s_spk_samples_frequence) {
                     xEventGroupSetBits(s_evt_handle, BIT4_SPK_RESET);    /* 设置扬声器重置标志 */
                 }
                 /* 更新扬声器参数 */
                 s_spk_samples_frequence = spk_frame_list[frame_index].samples_frequence;
                 s_spk_ch_num = spk_frame_list[frame_index].ch_num;
                 s_spk_bit_resolution = spk_frame_list[frame_index].bit_resolution;
             }
             xEventGroupSetBits(s_evt_handle, BIT3_SPK_START);    /* 设置扬声器启动标志 */
             if (s_spk_ch_num != 1) {
                 ESP_LOGW(TAG, "UAC扬声器: 此示例仅支持1声道");
             }
             ESP_LOGI(TAG, "UAC扬声器: 使用帧[%u] 声道数 = %"PRIu32", 位分辨率 = %"PRIu32", 采样频率 = %"PRIu32,
                         frame_index, s_spk_ch_num, s_spk_bit_resolution, s_spk_samples_frequence);
             free(spk_frame_list);
         } else {
             ESP_LOGW(TAG, "UAC扬声器: 获取帧列表大小 = %u", frame_size);
         }
 #endif
         ESP_LOGI(TAG, "设备已连接");
         break;
     }
     case STREAM_DISCONNECTED:    /* USB设备断开事件 */
         ESP_LOGI(TAG, "设备已断开");
         break;
     default:
         ESP_LOGE(TAG, "未知事件");
         break;
     }
 }
 
 /**
  * @brief 主函数 - 程序入口点
  */
 void app_main(void)
 {
     /* 设置日志级别 */
     esp_log_level_set("*", ESP_LOG_INFO);
     esp_log_level_set("httpd_txrx", ESP_LOG_INFO);
     
     esp_err_t ret = ESP_FAIL;
     
     /* 创建事件组用于线程同步 */
     s_evt_handle = xEventGroupCreate();
     if (s_evt_handle == NULL) {
         ESP_LOGE(TAG, "第%u行 事件组创建失败", __LINE__);
         assert(0);
     }
 
 #if (ENABLE_UVC_CAMERA_FUNCTION)
 #if (ENABLE_UVC_WIFI_XFER)
     /* 初始化WiFi和HTTP服务器 */
     app_wifi_main();
     app_httpd_main();
 #endif //ENABLE_UVC_WIFI_XFER
     
     /* 为USB负载分配双缓冲区，传输缓冲区大小 >= 帧缓冲区大小 */
     uint8_t *xfer_buffer_a = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
     assert(xfer_buffer_a != NULL);
     uint8_t *xfer_buffer_b = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
     assert(xfer_buffer_b != NULL);
 
     /* 为JPEG帧分配帧缓冲区 */
     uint8_t *frame_buffer = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
     assert(frame_buffer != NULL);
 
     /* 配置UVC流参数 */
     uvc_config_t uvc_config = {
         /* 匹配当前摄像头的任意分辨率（默认使用第一个帧大小） */
         .frame_width = DEMO_UVC_FRAME_WIDTH,
         .frame_height = DEMO_UVC_FRAME_HEIGHT,
         .frame_interval = FPS2INTERVAL(15),    /* 15帧每秒 */
         .xfer_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
         .xfer_buffer_a = xfer_buffer_a,
         .xfer_buffer_b = xfer_buffer_b,
         .frame_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
         .frame_buffer = frame_buffer,
         .frame_cb = &camera_frame_cb,    /* 设置帧回调函数 */
         .frame_cb_arg = NULL,
     };
     
     /* 配置并启用UVC功能 */
     ret = uvc_streaming_config(&uvc_config);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "UVC流配置失败");
     }
 #endif
 
 #if (ENABLE_UAC_MIC_SPK_FUNCTION)
     /* 匹配我们找到的音频设备的任意频率
      * 调用uac_frame_size_list_get获取当前音频设备的帧列表
      */
     uac_config_t uac_config = {
         .mic_bit_resolution = UAC_BITS_ANY,        /* 任意位分辨率 */
         .mic_samples_frequence = UAC_FREQUENCY_ANY, /* 任意采样频率 */
         .spk_bit_resolution = UAC_BITS_ANY,        /* 任意位分辨率 */
         .spk_samples_frequence = UAC_FREQUENCY_ANY, /* 任意采样频率 */
         .spk_buf_size = 16000,                     /* 扬声器缓冲区大小 */
         .mic_cb = &mic_frame_cb,                   /* 设置麦克风回调函数 */
         .mic_cb_arg = NULL,
         /* 设置标志以暂停扬声器，用户稍后需要调用usb_streaming_control来恢复扬声器 */
         .flags = FLAG_UAC_SPK_SUSPEND_AFTER_START,
     };
     
     /* 配置并启用UAC功能 */
     ret = uac_streaming_config(&uac_config);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "UAC流配置失败");
     }
 #endif
     
     /* 注册状态回调以获取连接/断开事件
      * 在回调中，我们可以获取当前设备的帧列表
      */
     ESP_ERROR_CHECK(usb_streaming_state_register(&stream_state_changed_cb, NULL));
     
     /* 启动USB流，UVC和UAC麦克风将开始流式传输，因为未设置SUSPEND_AFTER_START标志 */
     ESP_ERROR_CHECK(usb_streaming_start());
     ESP_ERROR_CHECK(usb_streaming_connect_wait(portMAX_DELAY));
     
     // 等待扬声器设备就绪
     xEventGroupWaitBits(s_evt_handle, BIT3_SPK_START, false, false, portMAX_DELAY);
 
     while (1) {
         xEventGroupWaitBits(s_evt_handle, BIT3_SPK_START, true, false, portMAX_DELAY);
         
         /* 手动恢复扬声器，因为设置了SUSPEND_AFTER_START标志 */
         ESP_ERROR_CHECK(usb_streaming_control(STREAM_UAC_SPK, CTRL_RESUME, NULL));
         usb_streaming_control(STREAM_UAC_SPK, CTRL_UAC_VOLUME, (void *)80);    /* 设置扬声器音量 */
         usb_streaming_control(STREAM_UAC_MIC, CTRL_UAC_VOLUME, (void *)80);    /* 设置麦克风音量 */
         ESP_LOGI(TAG, "扬声器已恢复");
         
 #if (ENABLE_UAC_MIC_SPK_FUNCTION && !ENABLE_UAC_MIC_SPK_LOOPBACK)
         ESP_LOGI(TAG, "开始播放默认声音");
         
         /* 外部声波数组声明 */
         extern const uint8_t wave_array_32000_16_1[];
         extern const uint32_t s_buffer_size;
         
         /* 计算频率偏移步长和降采样位数 */
         int freq_offsite_step = 32000 / s_spk_samples_frequence;
         int downsampling_bits = 16 - s_spk_bit_resolution;
         
         const int buffer_ms = 400;    /* 400毫秒缓冲区 */
         const int buffer_size = buffer_ms * (s_spk_bit_resolution / 8) * (s_spk_samples_frequence / 1000);
         
         // 如果是8位扬声器，声明uint8_t *d_buffer
         uint16_t *s_buffer = (uint16_t *)wave_array_32000_16_1;    /* 源缓冲区 */
         uint16_t *d_buffer = calloc(1, buffer_size);              /* 目标缓冲区 */
         size_t offset_size = buffer_size / (s_spk_bit_resolution / 8);
         
         while (1) {
             /* 检查是否到达缓冲区末尾 */
             if ((uint32_t)(s_buffer + offset_size) >= (uint32_t)(wave_array_32000_16_1 + s_buffer_size)) {
                 s_buffer = (uint16_t *)wave_array_32000_16_1;    /* 重置到缓冲区开始 */
                 // 静音扬声器
                 vTaskDelay(pdMS_TO_TICKS(1000));
                 // 取消静音扬声器
             } else {
                 // 填充USB缓冲区
                 for (size_t i = 0; i < offset_size; i++) {
                     d_buffer[i] = *(s_buffer + i * freq_offsite_step) >> downsampling_bits;
                 }
                 // 写入USB扬声器
                 uac_spk_streaming_write(d_buffer, buffer_size, pdMS_TO_TICKS(1000));
                 s_buffer += offset_size * freq_offsite_step;
             }
             
             /* 检查是否需要重置扬声器 */
             if (xEventGroupGetBits(s_evt_handle) & (BIT4_SPK_RESET | BIT3_SPK_START)) {
                 // 发生断开连接，我们可能需要重置扬声器的频率
                 xEventGroupClearBits(s_evt_handle, BIT4_SPK_RESET);
                 break;
             }
         }
         free(d_buffer);
 #endif
     }
 
     /* 主循环 */
     while (1) {
         vTaskDelay(100);
     }
 }
 
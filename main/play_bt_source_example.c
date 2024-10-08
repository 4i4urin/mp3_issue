/* Play music from Bluetooth device

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_peripherals.h"
#include "esp_gap_bt_api.h"

#include "nvs_flash.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "board.h"
#include "mp3_decoder.h"
#include "a2dp_stream.h"
#include "fatfs_stream.h"

#define BT_CONNECT_TIMEOUT      20000

typedef uint8_t esp_peer_bdname_t[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static const char *TAG = "BLUETOOTH_SOURCE_EXAMPLE";
static esp_peer_bdname_t remote_bt_device_name;
static esp_bd_addr_t remote_bd_addr = {0};

static uint8_t device_found = 0;
static uint8_t device_connect = 0;

#define BT_REMOTE_NAME  "JBL Charge 5"
#define DEVICE_NAME     "SHISHEL_DEV"


esp_periph_set_handle_t init_sdcard(void);
audio_element_handle_t init_mp3_decoder(void);
audio_element_handle_t init_wav_decoder(void);
audio_element_handle_t init_bt_stream(void);
audio_pipeline_handle_t init_pipeline(
    audio_element_handle_t bt_stream_writer, audio_element_handle_t mp3_decoder);

int open_music_file(void);
int read_file(int file, char* buf, int len);
uint32_t send_to_mp3pipline(audio_element_handle_t el, char* buf, uint32_t buf_len);


#define SEND_SIZE (1 * 1024)

#define MP3_OUTPUT_RB_SIZE (2 * 1024)
#define INPUT_RB_SIZE 512

char buf[SEND_SIZE] = { 0 };

// #define WAV

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    esp_periph_set_handle_t set = init_sdcard();

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

#ifdef WAV
    printf("Use WAV\n");
    audio_element_handle_t mp3_decoder = init_wav_decoder();
#else
    audio_element_handle_t mp3_decoder = init_mp3_decoder();
#endif
    audio_element_handle_t bt_stream_writer = init_bt_stream();
    audio_pipeline_handle_t pipeline = init_pipeline(bt_stream_writer, mp3_decoder);
    
    ESP_LOGI(TAG, "[3.7] Create bt peripheral");
    esp_periph_handle_t bt_periph = bt_create_periph();

    ESP_LOGI(TAG, "[3.8] Start bt peripheral");
    esp_periph_start(set, bt_periph);

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    int file = open_music_file();

    int state = 1;
    while (1)
    {
        if (!device_connect || state == 0)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        if (file > 0)
            state = read_file(file, buf, SEND_SIZE);
        else
        {
            printf("ERROR: file designator is bad\n");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        send_to_mp3pipline(mp3_decoder, buf, SEND_SIZE);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        ringbuf_handle_t ring_buf = audio_element_get_output_ringbuf(bt_stream_writer);
        ringbuf_handle_t ring_buf_input = audio_element_get_input_ringbuf(bt_stream_writer);
        printf(
            "RINGBUF BLUETOOTH:\n"
            "rb_out size: %d\n"
            "rb_in size: %d\n", rb_get_size(ring_buf), rb_get_size(ring_buf_input) 
        );
    //     printf("Loop\n");
    }
    close(file);

}


esp_periph_set_handle_t init_sdcard(void)
{
    ESP_LOGI(TAG, "[ 1 ] Mount sdcard");
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    audio_board_sdcard_init(set, SD_MODE_1_LINE);
    return set;
}


audio_element_handle_t init_mp3_decoder(void)
{
    audio_element_handle_t mp3_decoder;

    ESP_LOGI(TAG, "[3.2] Create mp3 decoder to decode mp3 file");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_cfg.out_rb_size = MP3_OUTPUT_RB_SIZE;

    mp3_decoder = mp3_decoder_init(&mp3_cfg);

    ringbuf_handle_t mp3_ringbuf_in = rb_create(INPUT_RB_SIZE, 1);
    printf("INPUT Ringbuf res = %d\n", audio_element_set_input_ringbuf(mp3_decoder, mp3_ringbuf_in));

    audio_element_info_t info;
    printf("audioinf = %d\n", audio_element_getinfo(mp3_decoder, &info));
    printf(
        "Sample rate = %d\n"    "channels = %d\n"
        "bits = %d\n"           "bps = %d\n"
        "byte_pos = %lld\n"     "total_bytes = %lld\n"
        "codec_fmt = %d\n",
        info.sample_rates,      info.channels,
        info.bits,              info.bps,
        info.byte_pos,          info.total_bytes,
        info.codec_fmt);
    return mp3_decoder;
}

#include "wav_decoder.h"
audio_element_handle_t init_wav_decoder(void)
{
    audio_element_handle_t wav_decoder;

    ESP_LOGI(TAG, "[3.2] Create wav decoder to decode wav file");
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    wav_decoder = wav_decoder_init(&wav_cfg);

    ringbuf_handle_t wav_ringbuf = rb_create((2 * 1024), 1);
    printf("Ringbuf res = %d\n", audio_element_set_input_ringbuf(wav_decoder, wav_ringbuf));

    return wav_decoder;
}


static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

audio_element_handle_t init_bt_stream(void)
{
    audio_element_handle_t bt_stream_writer;

    ESP_LOGI(TAG, "[3.3] Create Bluetooth stream");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_dev_set_device_name(DEVICE_NAME);
    esp_bt_gap_register_callback(bt_app_gap_cb);

    memcpy(&remote_bt_device_name, BT_REMOTE_NAME, strlen(BT_REMOTE_NAME) + 1);
    
    a2dp_stream_config_t a2dp_config = 
    {
        .type = AUDIO_STREAM_WRITER,
        .user_callback = {0},
    };
    bt_stream_writer = a2dp_stream_init(&a2dp_config);

    ringbuf_handle_t a2dp_ringbuf = rb_create(SEND_SIZE, 1);
    printf("Ringbuf res = %d\n", audio_element_set_output_ringbuf(bt_stream_writer, a2dp_ringbuf));

    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    return bt_stream_writer;
}


audio_pipeline_handle_t init_pipeline(
    audio_element_handle_t bt_stream_writer, audio_element_handle_t mp3_decoder)
{
    audio_pipeline_handle_t pipeline;

    ESP_LOGI(TAG, "[3.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, bt_stream_writer, "bt");

    ESP_LOGI(TAG, "[3.5] Link it together mp3_decoder-->bt_stream-->[bt sink]");
    const char *link_tag[2] = {"mp3", "bt"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    return pipeline;
}


//---------------------------------------------------------------------------------

#include "ringbuf.h"
uint32_t send_to_mp3pipline(audio_element_handle_t el, char* buf, uint32_t buf_len)
{

    ringbuf_handle_t ring_buf = audio_element_get_output_ringbuf(el);
    printf(
        "RINGBUF STATE:\n"
        "bytes available = %d\n"
        "rb size: %d\n", rb_bytes_available(ring_buf), rb_get_size(ring_buf)
    );
    
    // printf("START: send to ringbuf\n");
    int32_t res = audio_element_output(el, buf, buf_len);
    printf("FINISH: send to ringbuf\n");
    if (res < 0)
    {
        printf("Something went wrong res = %d\n", res);
        return 0;
    }
    return res;
}


int open_music_file(void)
{
    printf("Try to open music files\n");

    #define FILE_NUM 3
#ifdef WAV
    const char path[FILE_NUM][25] = {
        "/sdcard/chill_music.wav",
        "/sdcard/some_song.wav",
        "/sdcard/rap_music.wav",
    };
#else
    const char path[FILE_NUM][25] = {
        "/sdcard/Punisher.mp3",
        "/sdcard/chill_music.mp3",
        "/sdcard/rap_music.mp3",
    };
#endif
    printf("Try read\n");
    int file = 0;
    for (int i = 0; i < FILE_NUM; i++)
    {
        file = open(path[i], O_RDONLY);
        if (file == -1)
        {
            printf("Failed to open. File name: %s\n", path[i]);
        } else
        {
            printf("Open file: %s\n", path[i]);
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
            
    }
    #undef FILE_NUM

    if (file == -1)
    {
        printf("ERROR: AAAAAAA\n");
        return -1;
    }
    return file;
}


int read_file(int file, char* buf, int len)
{
    int res = read(file, buf, len);
    if (res < 0)
    {
        printf("Failed to read\n");
        return -2;
    }
    return res;
}

//---------------------------------------------------------------------------------


static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param);

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
                filter_inquiry_scan_result(param);
                break;
            }
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
                if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
                    if (device_found) {
                        ESP_LOGI(TAG, "Device discovery stopped.");
                        ESP_LOGI(TAG, "a2dp connecting to peer: %s", remote_bt_device_name);
                        device_found = 0;
                        esp_a2d_source_connect(remote_bd_addr);
                    } else {
                        // not discovered, continue to discover
                        ESP_LOGI(TAG, "Device discovery failed, continue to discover...");
                        esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
                    }
                } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
                    ESP_LOGI(TAG, "Discovery started.");
                }
                break;
            }
        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
            ESP_LOGI(TAG, "Device was connected");
            device_connect = 1;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        default:
            break;
    }
    return;
}


static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len);

static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *eir = NULL;
    esp_peer_bdname_t peer_bdname;
    esp_bt_gap_dev_prop_t *p;

    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
            case ESP_BT_GAP_DEV_PROP_COD:
                cod = *(uint32_t *)(p->val);
                ESP_LOGI(TAG, "--Class of Device: 0x%" PRIx32, cod);
                break;
            case ESP_BT_GAP_DEV_PROP_RSSI:
                rssi = *(int8_t *)(p->val);
                ESP_LOGI(TAG, "--RSSI: %" PRId32, rssi);
                break;
            case ESP_BT_GAP_DEV_PROP_EIR:
                eir = (uint8_t *)(p->val);
                get_name_from_eir(eir, (uint8_t *)&peer_bdname, NULL);
                ESP_LOGI(TAG, "--Name: %s", peer_bdname);
                break;
            case ESP_BT_GAP_DEV_PROP_BDNAME:
            default:
                break;
        }
    }

    ESP_LOGI(TAG, "need device name %s", (uint8_t *)remote_bt_device_name);
    /* search for device named "peer_bdname" in its extended inquiry response */
    if (eir) {
        get_name_from_eir(eir, (uint8_t *)&peer_bdname, NULL);
        if (strcmp((char *)peer_bdname, (char *)remote_bt_device_name) != 0) {
            return;
        }

        device_found = 1;
        ESP_LOGI(TAG, "Found a target device, address %s, name %s", bda_str, (uint8_t *)peer_bdname);
        memcpy(&remote_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
        ESP_LOGI(TAG, "Cancel device discovery ...");
        esp_bt_gap_cancel_discovery();
    }
}


static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}


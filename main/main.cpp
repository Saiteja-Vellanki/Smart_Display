#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "pins_config.h"
#include "sdkconfig.h"
#include "xl9535.h"
#include <stdio.h>


#include "demos/lv_demos.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"



//#define TOUCH_MODULES_FT3267
#define TOUCH_MODULES_CST_SELF

#ifdef TOUCH_MODULES_FT3267
#define TOUCH_SLAVE_ADDRESS FT3267_SLAVE_ADDRESS
#elif defined(TOUCH_MODULES_CST_SELF)
#define TOUCH_SLAVE_ADDRESS CST820_SLAVE_ADDRESS
#else 
#error "Please confirm your touch chip model. The frame is full circle for CST820. Others are FT3267."
#endif

#include "TouchLib.h"

static const char *TAG = "Display Decode";
#define GATTS_TAG "SMARTDISP"
#define ADV_CONFIG_FLAG                           (1 << 0)
#define SCAN_RSP_CONFIG_FLAG                      (1 << 1)
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define I2C_MASTER_PORT             I2C_NUM_0

TouchLib touch;
static uint8_t adv_config_done = 0;


// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x1811 
#define GATTS_CHAR_UUID_TEST_A      0x2A44
#define GATTS_DESCR_UUID_TEST_A     0x2A3F
#define GATTS_NUM_HANDLE_TEST_A     4

#define GATTS_SERVICE_UUID_TEST_B   0x1819
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define TEST_DEVICE_NAME            "BluFyi"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;
static esp_gatt_char_prop_t b_property = 0;
esp_err_t ret;
lv_obj_t *label;
lv_obj_t *img;
SemaphoreHandle_t xSemaphore;
int x=0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = ESP_BLE_APPEARANCE_GENERIC_HID,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
   .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_B_APP_ID] = {
        .gatts_cb = gatts_profile_b_event_handler,                   /* This demo does not implement, similar as profile A */
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);



typedef struct {
  uint8_t cmd;
  uint8_t data[16];
  uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

static void logo_init(void)
{
   lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
   LV_IMG_DECLARE(Our_logo);
   ESP_LOGI(TAG, "Test decoding pictures.");
  lv_img_set_src(img, &Our_logo);
  lv_obj_center(img);
  lv_label_set_text(label, LV_SYMBOL_BLUETOOTH);
  lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
  vTaskDelay(5000 / portTICK_PERIOD_MS);

}
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if(event == ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT)
    {
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
    }
  else if(event == ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT) 
  {
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
  }
  else if(event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT)
  {
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            //ESP_LOGE(GATTS_TAG, "advertising start failed, error status = %x", param->adv_start_cmpl.status);
           
        }
        //ESP_LOGI(GATTS_TAG, "advertising start success");
  }
   else if(event == ESP_GAP_BLE_PASSKEY_REQ_EVT)
   {                          /* passkey request event */
        //ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda, true, 0x00);
   }
    else if(event == ESP_GAP_BLE_OOB_REQ_EVT) 
    {
        //ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        
    }
    else if(event == ESP_GAP_BLE_NC_REQ_EVT)
    {
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        //ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%" PRIu32, param->ble_security.key_notif.passkey);
    }  
    else if(event == ESP_GAP_BLE_SEC_REQ_EVT)
    {
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
    }
    else if(event ==ESP_GAP_BLE_PASSKEY_NOTIF_EVT)
    { ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        //ESP_LOGI(GATTS_TAG, "The passkey Notify number:%06" PRIu32, param->ble_security.key_notif.passkey);
    }    
    else if(event ==ESP_GAP_BLE_AUTH_CMPL_EVT) {
        x=0;
        esp_log_buffer_hex("addr", param->ble_security.auth_cmpl.bd_addr, ESP_BD_ADDR_LEN);
        lv_label_set_text(label, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0x0000FF), LV_PART_MAIN);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
        //ESP_LOGI(GATTS_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success) {
            x=0;
        esp_log_buffer_hex("addr", param->ble_security.auth_cmpl.bd_addr, ESP_BD_ADDR_LEN);
        lv_label_set_text(label, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
            //ESP_LOGI(GATTS_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
       
    }
   else if(event ==ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT)
   {
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            //ESP_LOGE(GATTS_TAG, "config local privacy failed, error status = %x", param->local_privacy_cmpl.status);
            
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            //ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        } else {
            adv_config_done |= ADV_CONFIG_FLAG;
        }

        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret) {
           // ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        } else {
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
        }
   }

    
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
 
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    //ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               //ESP_LOGE(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        //ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    
    char *buff="\0";
    char *p_data;
    //int data=0;
    if(event == ESP_GATTS_REG_EVT)
    {
        //ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
           // ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            //ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
           // ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
    }
    else if(event == ESP_GATTS_READ_EVT)
    {
       // ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
       
    }
     else if(event == ESP_GATTS_WRITE_EVT) 
     {
        //ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            //ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            x++;
            if(x>1)
            {
            LV_IMG_DECLARE(call_logo);
             ESP_LOGI(TAG, "Test decoding pictures.");
            lv_img_set_src(img, &call_logo);
            lv_obj_center(img);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            strcpy(buff,(char*)param->write.value);
            ESP_LOGI(GATTS_TAG, "%s",buff);
            lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);
            lv_label_set_text(label, buff);
            lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xFFFFFF), LV_PART_MAIN);
            lv_obj_align(label, LV_ALIGN_TOP_MID, 10,100);
           
            // data = atoi(p_data);
            // if(data)
            // {
            //    
            // }
            
   
            x=0;
            //  p_data = strstr(buff,"Ended");
            // if(strlen(p_data))
            // {
            //     logo_init();
            // }
            
             
            }

           
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        //ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        //ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    //ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                   // ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                     ESP_LOGI(GATTS_TAG, "Call received 3");
                }

            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
       
    }
     else if(event == ESP_GATTS_EXEC_WRITE_EVT)
    {
        //ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
    }
   else if(event == ESP_GATTS_MTU_EVT)
   {
        //ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
   }
  else if(event == ESP_GATTS_UNREG_EVT)
  {

  }
       
    else if(event == ESP_GATTS_CREATE_EVT)
    {
        //ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
           // ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
    }
    else if(event == ESP_GATTS_ADD_INCL_SRVC_EVT)
    {

    }
    else if(event ==ESP_GATTS_ADD_CHAR_EVT)
    {
        uint16_t length = 0;
        const uint8_t *prf_char;

        //ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                //param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            //ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        //ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
        for(int i = 0; i < length; i++){
            //ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            //ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        
    }
   else if(event == ESP_GATTS_ADD_CHAR_DESCR_EVT)
   {
     gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
     //ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 //param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
   }
        
     else if(event == ESP_GATTS_DELETE_EVT)
     {

     }
    
     else if(event == ESP_GATTS_START_EVT)
     {
        //ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 //param->start.status, param->start.service_handle);
     }
        
        
    else if(event ==ESP_GATTS_STOP_EVT)
    {

    }
      
   else if(event == ESP_GATTS_CONNECT_EVT){
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        //ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 //param->connect.conn_id,
                 //param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 //param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
       
    }
    else if(event == ESP_GATTS_DISCONNECT_EVT){
        //ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        lv_label_set_text(label, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
        esp_ble_gap_start_advertising(&adv_params);
}
   else if(event ==ESP_GATTS_CONF_EVT)
   {
        //ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
   }
    // case ESP_GATTS_OPEN_EVT:
    // case ESP_GATTS_CANCEL_OPEN_EVT:
    // case ESP_GATTS_CLOSE_EVT:
    // case ESP_GATTS_LISTEN_EVT:
    // case ESP_GATTS_CONGEST_EVT:
    // default:
    //     break;
   
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if(event == ESP_GATTS_REG_EVT)
    {
        //ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_B);
    }
    else if(event == ESP_GATTS_READ_EVT) {
        //ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        
    }
    else if(event == ESP_GATTS_WRITE_EVT) {
        //ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            //ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
            if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                uint16_t descr_value= param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i%0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(notify_data), notify_data, false);
                    }
                }else if (descr_value == 0x0002){
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i%0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000){
                    //ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }else{
                    //ESP_LOGE(GATTS_TAG, "unknown value");
                }

            }
        }
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        
    }
   else if(event == ESP_GATTS_EXEC_WRITE_EVT)
   {
        //ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
    }   
  else if(event == ESP_GATTS_MTU_EVT){
       // ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
}
    else if(event == ESP_GATTS_UNREG_EVT)
    {

    }
   else if(event == ESP_GATTS_CREATE_EVT)
   {
        //ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
        b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret =esp_ble_gatts_add_char( gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        b_property,
                                                        NULL, NULL);
        if (add_char_ret){
            //ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }
   }
    else if(event == ESP_GATTS_ADD_INCL_SRVC_EVT)
    {

    }
    else if(event == ESP_GATTS_ADD_CHAR_EVT)
    {
        //ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                 //param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                     NULL, NULL);
       }
    else if(event ==ESP_GATTS_ADD_CHAR_DESCR_EVT)
    {
        gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        //ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 //param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
       }
    else if(event ==ESP_GATTS_DELETE_EVT)
    {

    }
        
    else if(event ==ESP_GATTS_START_EVT)
    {
       //ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                // param->start.status, param->start.service_handle);
    }
        
        
   else if(event == ESP_GATTS_STOP_EVT)
   {

   }
        
   else if(event ==ESP_GATTS_CONNECT_EVT)
   {
        //ESP_LOGI(GATTS_TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 //param->connect.conn_id,
                 //param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 //param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
   }
    else if(event == ESP_GATTS_CONF_EVT)
    {
        //ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
   }
    // case ESP_GATTS_DISCONNECT_EVT:
    // case ESP_GATTS_OPEN_EVT:
    // case ESP_GATTS_CANCEL_OPEN_EVT:
    // case ESP_GATTS_CLOSE_EVT:
    // case ESP_GATTS_LISTEN_EVT:
    // case ESP_GATTS_CONGEST_EVT:
    
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            //ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                   // param->reg.app_id,
                    //param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

DRAM_ATTR static const lcd_init_cmd_t st7701s_init_cmds[] = { // 2.1
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3b, 0x00}, 0x02},
    {0xC1, {0x0b, 0x02}, 0x02},
    {0xC2, {0x07, 0x02}, 0x02},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01},
    {0xb0, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 0x10},
    {0xb1, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13, 0x11, 0x2a, 0x31, 0x18}, 0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x6d}, 0x01},
    {0xb1, {0x37}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x43}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1, {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20}, 0x0b},
    {0xe2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x0d},
    {0xe3, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe4, {0x22, 0x00}, 0x02},
    {0xe5, {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe7, {0x22, 0x00}, 0x02},
    {0xe8, {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 0x07},
    {0xed, {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0, 0xab, 0xff, 0xff, 0xff}, 0x10},
    {0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xef, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},
    {0x36, {0x08}, 0x01},
    {0x3a, {0x66}, 0x01},
    {0x11, {0x00}, 0x80},
    // {0xFF, {0x77, 0x01, 0x00, 0x00, 0x12}, 0x05},
    // {0xd1, {0x81}, 0x01},
    // {0xd2, {0x06}, 0x01},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}};

static void tft_init(void);

static int readRegister(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint8_t len) {
  uint8_t read_data = 0;
  i2c_master_write_read_device(I2C_MASTER_PORT, devAddr, (uint8_t *)&regAddr, 1, data, len, 1000 / portTICK_PERIOD_MS);
  return 0;
}

static int writeRegister(uint8_t devAddr, uint16_t regAddr, uint8_t *data, uint8_t len) {
  uint8_t write_buf[100] = {(uint8_t)regAddr};
  memcpy(write_buf + 1, data, len);
  i2c_master_write_to_device(I2C_MASTER_PORT, devAddr, write_buf, len + 1, 1000 / portTICK_PERIOD_MS);
  return 0;
}

static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {

  if (touch.read()) {
    TP_Point t = touch.getPoint(0);
    data->point.x = t.x;
    data->point.y = t.y;
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data,
                                   void *user_data) {
  BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
    xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
  }
#endif
  return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  xSemaphoreGive(sem_gui_ready);
  xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
  // pass the draw buffer to the driver
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

// void startup_gif(void)
// {
//   LV_IMG_DECLARE(startup);
//   img = lv_gif_create(lv_scr_act());
//   lv_gif_set_src(img, &startup);
//   lv_obj_center(img);
//   vTaskDelay(8000 / portTICK_PERIOD_MS);
// }

void taskrun(void *pvParameter)
{
   
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), LV_PART_MAIN);
  label= lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Hello !!!");
  lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  vTaskDelay(4000 / portTICK_PERIOD_MS);
  lv_label_set_text(label, "Saiteja's");
  lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  vTaskDelay(4000 / portTICK_PERIOD_MS);
  lv_label_set_text(label, "App_ver_1.0.2");
  lv_obj_set_style_text_color(lv_scr_act(), lv_color_hex(0xffffff), LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  vTaskDelay(3000 / portTICK_PERIOD_MS);
   
   logo_init();
  //xSemaphoreGive( xSemaphore );
  //vTaskDelay(50 / portTICK_PERIOD_MS);
 

  vTaskDelete(NULL);
}

void bletask(void)
{
    
 
   
 //xSemaphoreTake(xSemaphore,portMAX_DELAY);
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        //ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
   
    
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        //ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        //ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        //ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret){
        //ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        //ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    //set static passkey
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device acts as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the master;
    If your BLE device acts as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
 // vTaskDelay(50 / portTICK_PERIOD_MS);
        //vTaskDelete(NULL);
}

extern "C" void app_main(void) {
  static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;      // contains callback functions
  static lv_indev_drv_t indev_drv;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
  ESP_LOGI(TAG, "Create semaphores");
  sem_vsync_end = xSemaphoreCreateBinary();
  assert(sem_vsync_end);
  sem_gui_ready = xSemaphoreCreateBinary();
  assert(sem_gui_ready);
#endif
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = IIC_SDA_PIN,
      .scl_io_num = IIC_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master =
          {
              .clk_speed = 400 * 1000,
          },
  };
  esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);

  if (err != ESP_OK) {
    ESP_LOGI(TAG, "i2c bus creation failed");
  }
  i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);

  ESP_LOGI(TAG, "Initialize xl9535");
  xl9535_begin(0, 0, 0, I2C_MASTER_PORT);
  xl9535_pinMode(PWR_EN_PIN, OUTPUT);
  xl9535_digitalWrite(PWR_EN_PIN, 1);

  ESP_LOGI(TAG, "Initialize touch");
  xl9535_pinMode(TP_RES_PIN, OUTPUT);
  xl9535_digitalWrite(TP_RES_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl9535_digitalWrite(TP_RES_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl9535_digitalWrite(TP_RES_PIN, 1);
  touch.begin(TOUCH_SLAVE_ADDRESS, -1, readRegister, writeRegister);

  ESP_LOGI(TAG, "Initialize st7701s");
  tft_init();

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
  ESP_LOGI(TAG, "Turn off LCD backlight");
  ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT));
#endif

  ESP_LOGI(TAG, "Install RGB LCD panel driver");
  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_rgb_panel_config_t panel_config = {
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .timings =
        {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_pulse_width = 1,
            .hsync_back_porch = 30,
            .hsync_front_porch = 50,
            .vsync_pulse_width = 1,
            .vsync_back_porch = 30,
            .vsync_front_porch = 20,
            .flags =
                {
                    .pclk_active_neg = true,
                },
        },

    .data_width = 16, // RGB565 in parallel mode, thus 16bit in width

#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
    .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
    .psram_trans_align = 64,
    .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
    .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
    .de_gpio_num = EXAMPLE_PIN_NUM_DE,
    .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
    .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
    .data_gpio_nums =
        {
            // EXAMPLE_PIN_NUM_DATA12, // 2.1
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
            EXAMPLE_PIN_NUM_DATA16,
            EXAMPLE_PIN_NUM_DATA17,

            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,

            // EXAMPLE_PIN_NUM_DATA0, // 2.1
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
        },
    .flags =
        {
            .fb_in_psram = true, // allocate frame buffer in PSRAM
#if CONFIG_EXAMPLE_DOUBLE_FB
            .double_fb = true, // allocate double frame buffer
#endif                         // CONFIG_EXAMPLE_DOUBLE_FB
        },
  };
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

  ESP_LOGI(TAG, "Register event callbacks");
  esp_lcd_rgb_panel_event_callbacks_t cbs = {
      .on_vsync = example_on_vsync_event,
  };
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

  ESP_LOGI(TAG, "Initialize RGB LCD panel");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
  ESP_LOGI(TAG, "Turn on LCD backlight");
  gpio_set_level((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();
  void *buf1 = NULL;
  void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
  ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
  // initialize LVGL draw buffers
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
  ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
  buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);
  // initialize LVGL draw buffers
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 100);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

  ESP_LOGI(TAG, "Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
  disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  ESP_LOGI(TAG, "Register touch driver to LVGL");
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  ESP_LOGI(TAG, "Install LVGL tick timer");
  // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
  const esp_timer_create_args_t lvgl_tick_timer_args = {.callback = &example_increase_lvgl_tick, .name = "lvgl_tick"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if LV_USE_DEMO_WIDGETS
  ESP_LOGI(TAG, "Display LVGL Demos");
  lv_demo_widgets();
#elif LV_USE_DEMO_BENCHMARK
  ESP_LOGI(TAG, "Display LVGL Demos");
  lv_demo_benchmark();
#elif LV_USE_DEMO_STRESS
  ESP_LOGI(TAG, "Display LVGL Demos");
  lv_demo_stress();
#elif LV_USE_DEMO_KEYPAD_AND_ENCODER
  ESP_LOGI(TAG, "Display LVGL Demos");
  lv_demo_keypad_encoder();
#elif LV_USE_DEMO_MUSIC
  ESP_LOGI(TAG, "Display LVGL Demos");
  lv_demo_music();
#elif LV_USE_GIF
  

  #else
   
   

#endif

     //startup_gif();

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
 
   img = lv_img_create(lv_scr_act());
 xSemaphore = xSemaphoreCreateBinary();

if ( xTaskCreate(&taskrun, "task_run", 1024 * 20, NULL,  1, NULL) != pdPASS ) {
        printf("task run failed\r\n");
    }

    // if( xSemaphore != NULL)
    // {
    //     printf("Creating ble task");
    //     if ( xTaskCreate(&bletask, "ble_run", 1024 * 25, NULL, 5 , NULL) != pdPASS ) {
    //     printf("task run failed\r\n");
    // }
    // }
    // else
    // {
    //      printf("Failed to create Semaphore");
    // }
    bletask();

//   while (1) {
//     // raise the task priority of LVGL and/or reduce the handler period can improve the performance
            while(1){
  vTaskDelay(pdMS_TO_TICKS(2));
  lv_timer_handler();
  }
    
//     // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
//     // if (touch.read()) {
//     //   uint8_t n = touch.getPointNum();
//     //   printf("getPointNum: %d  \r\n", n);
//     //   for (uint8_t i = 0; i < n; i++) {
//     //     TP_Point t = touch.getPoint(i);
//     //     printf("[%d] point x: %d  point y: %d \r\n", i, t.x, t.y);
//     //   }
//     // }
//   }
   
}

static void lcd_send_data(uint8_t data) {
  uint8_t n;
  for (n = 0; n < 8; n++) {
    if (data & 0x80)
      xl9535_digitalWrite(LCD_SDA_PIN, 1);
    else
      xl9535_digitalWrite(LCD_SDA_PIN, 0);

    data <<= 1;
    xl9535_digitalWrite(LCD_CLK_PIN, 0);
    xl9535_digitalWrite(LCD_CLK_PIN, 1);
  }
}

static void lcd_cmd(const uint8_t cmd) {
  xl9535_digitalWrite(LCD_CS_PIN, 0);
  xl9535_digitalWrite(LCD_SDA_PIN, 0);
  xl9535_digitalWrite(LCD_CLK_PIN, 0);
  xl9535_digitalWrite(LCD_CLK_PIN, 1);
  lcd_send_data(cmd);
  xl9535_digitalWrite(LCD_CS_PIN, 1);
}

static void lcd_data(const uint8_t *data, int len) {
  uint32_t i = 0;
  if (len == 0)
    return; // no need to send anything
  do {
    xl9535_digitalWrite(LCD_CS_PIN, 0);
    xl9535_digitalWrite(LCD_SDA_PIN, 1);
    xl9535_digitalWrite(LCD_CLK_PIN, 0);
    xl9535_digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(*(data + i));
    xl9535_digitalWrite(LCD_CS_PIN, 1);
    i++;
  } while (len--);
}

static void tft_init(void) {
  int cmd = 0;
  xl9535_pinMode(LCD_SDA_PIN, OUTPUT);
  xl9535_pinMode(LCD_CLK_PIN, OUTPUT);
  xl9535_pinMode(LCD_CS_PIN, OUTPUT);
  xl9535_pinMode(LCD_RST_PIN, OUTPUT);
  // xl9535_read_all_reg();

  xl9535_digitalWrite(LCD_SDA_PIN, 1);
  xl9535_digitalWrite(LCD_CLK_PIN, 1);
  xl9535_digitalWrite(LCD_CS_PIN, 1);

  // Reset the display
  xl9535_digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl9535_digitalWrite(LCD_RST_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl9535_digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);

  while (st7701s_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(st7701s_init_cmds[cmd].cmd);
    lcd_data(st7701s_init_cmds[cmd].data, st7701s_init_cmds[cmd].databytes & 0x1F);
    if (st7701s_init_cmds[cmd].databytes & 0x80) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    cmd++;
  }
}


/**
  ******************************************************************************
  * @file    settings.h
  * @brief   Persistent settings stored in internal Flash with CRC32 protection.
  *          8AIV variant: network + per-channel analog-input configuration +
  *          service constants of the input network model.
  ******************************************************************************
  */
#ifndef APPLICATION_SETTINGS_H
#define APPLICATION_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#include "ai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_MAGIC          0x08A54A57u
#define SETTINGS_VERSION        1u

#define SETTINGS_AI_CHANNELS    AI_CHANNELS

#define SETTINGS_DEF_LED_MODE       2u    /* STATE_MACHINE */

#define SETTINGS_DEF_SLAVE_ID       1u
#define SETTINGS_DEF_TCP_PORT       502u

#define NET_MODE_STATIC            0u
#define NET_MODE_DHCP              1u
#define NET_MODE_LINKLOCAL         2u

#define SETTINGS_DEF_USE_DHCP       NET_MODE_LINKLOCAL

#define SETTINGS_DEF_IP0            192u
#define SETTINGS_DEF_IP1            168u
#define SETTINGS_DEF_IP2            1u
#define SETTINGS_DEF_IP3            10u

#define SETTINGS_DEF_MASK0          255u
#define SETTINGS_DEF_MASK1          255u
#define SETTINGS_DEF_MASK2          255u
#define SETTINGS_DEF_MASK3          0u

#define SETTINGS_DEF_GW0            192u
#define SETTINGS_DEF_GW1            168u
#define SETTINGS_DEF_GW2            1u
#define SETTINGS_DEF_GW3            1u

#define SETTINGS_DEF_WINDOW_MS      100u  /* integration window: 20 (50 Hz) or 100 (50 + 60 Hz) */

typedef enum {
    LED_MODE_ALW_OFF       = 0,
    LED_MODE_ALW_ON        = 1,
    LED_MODE_STATE_MACHINE = 2,
} led_mode_t;

typedef struct {
    uint32_t magic;
    uint16_t version;

    uint16_t led_mode;              /* led_mode_t                             */
    uint16_t modbus_tcp_port;       /* default 502                            */
    uint16_t window_ms;             /* integration window: 20 or 100          */
    uint16_t modbus_slave_id;       /* default 1                              */
    uint16_t use_dhcp;              /* net mode: 0=static,1=DHCP,2=link-local */

    uint8_t  ip[4];
    uint8_t  netmask[4];
    uint8_t  gateway[4];

    char     name[16];              /* device name, NUL-padded (discovery)    */

    ai_model_t model;               /* input-network constants (service)      */
    ai_diag_t  diag;                /* result-diagnostic thresholds (service) */
    ai_ch_cfg_t ch[SETTINGS_AI_CHANNELS];

    uint32_t crc32;                 /* CRC32 over all preceding bytes         */
} settings_t;

#define SETTINGS_NAME_LEN   16u

/** Load from Flash. Returns false and falls back to defaults if the stored
 *  record is missing or invalid; settings_config_error() then tells a
 *  corrupted record (CRC/version) from a blank sector. */
bool settings_init(void);
bool settings_config_error(void);

void settings_reset_to_defaults(void);

bool settings_save(void);
uint16_t settings_save_count(void);
uint16_t settings_save_result(void);

settings_t* settings_get(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SETTINGS_H */

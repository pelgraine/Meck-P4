/*
 * MeckUsbDrive.cpp -- USB drive mode for Meck-P4
 *
 * Settings > Experimental Features > USB drive mode shows the SD card to a
 * computer as a drive on the middle USB-C port, so files can be copied to
 * and from it without taking the card out. That port is the P4's own
 * high-speed USB pins (chip pins 49/50, USB_DM/USB_DP), wired to the USB1
 * connector on LilyGo's T-Display P4 schematics; the flashing port is a
 * separate USB-to-serial chip on UART0.
 *
 * Built on Espressif's esp_tinyusb 1.7.6~1 (the local copy in
 * overrides/esp_tinyusb, which never formats the card), with the USB
 * descriptors laid out as in ESP-IDF 5.4.1's
 * examples/peripherals/usb/device/tusb_msc.
 *
 * Only one side can use the card at a time. While the card is away, the
 * /sdcard filesystem Meck uses does not exist, so every stat() or fopen() on
 * it fails as "not found", exactly as when no card is fitted, and meck_task
 * holds its channel-message saves (meck_usb_drive_card_away()).
 *
 * Who owns the card's filesystem:
 *   - at boot, LilyGo's Sdmmc_Init() mounts it at /sdcard with ESP-IDF's
 *     esp_vfs_fat_sdmmc_mount() ("Meck's mount");
 *   - the first switch-on takes that mount down by hand, with the same steps
 *     esp_tinyusb uses for its own unmount, so the card and its SDMMC slot
 *     stay set up (esp_vfs_fat_sdcard_unmount() would shut the slot down and
 *     free the card);
 *   - from then on esp_tinyusb owns it. It gives the card to the computer
 *     when the computer sets the drive up, and mounts it back on the P4 side
 *     at its own default path, /data (not used anywhere in Meck), when the
 *     computer ejects it or the cable comes out. Switch-off mounts it back at
 *     /sdcard.
 *
 * Handing the card over safely: FatFs's unmount (f_mount(NULL, ...)) deletes
 * the volume's lock and frees its buffer without waiting, so nothing may be
 * part-way through a file when it runs. The tasks that use the card are:
 *   - meck_task (SD saves): the handover runs on meck_task itself, between
 *     its own saves;
 *   - the LVGL task (every screen that reads or writes files, the Game Boy's
 *     ROM and save files, notification tones being started): the handover
 *     holds lvgl_api_lock, which the LVGL task holds around all of its work
 *     (lvgl_ui_task in main.cpp);
 *   - the audio decoder task (chmorgan's audio_task), which closes its file
 *     itself a moment after a stop: the player is stopped first and given
 *     AUDIO_CLOSE_WAIT_MS.
 * Every later mount and unmount is esp_tinyusb's, run on the TinyUSB task
 * (usbd_defer_func), where esp_tinyusb runs its own, so they never overlap.
 *
 * USB is not started at boot. The first switch-on starts the driver; after
 * that, switch-off disconnects from the computer (tud_disconnect) and
 * switch-on reconnects (tud_connect). The driver stays installed until the
 * next restart. The task that runs the USB stack is Meck's own
 * (usb_device_task), with its stack in PSRAM: esp_tinyusb's default task
 * takes its stack from internal RAM, which failed on hardware.
 */

#include <stdio.h>
#include <string.h>
#include <sys/lock.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "meck.h"
#include "MeckAudio.h"
#include "MeckSDCard.h"
#include "t_display_p4_config.h"   // SD_BASE_PATH ("/sdcard")

// USB drive mode needs esp_tinyusb's mass-storage support (switched on in
// every build by MECK_USB_DRIVE in main/Kconfig.projbuild), and the USB port
// free: LilyGo's "Enable USB Display" option uses it for a USB screen.
#if defined(CONFIG_TINYUSB_MSC_ENABLED) && !defined(CONFIG_ENABLE_USB_DISPLAY)
#define MECK_USB_DRIVE_BUILT 1
#else
#define MECK_USB_DRIVE_BUILT 0
#endif

static volatile meck_usb_drive_state_t s_state = MECK_USB_DRIVE_OFF;
static volatile int  s_request      = 0;       // UI task -> meck_task: +1 on, -1 off
static volatile bool s_host_ejected = false;   // TinyUSB task -> UI task
static const char * volatile s_error = "";     // meck_task/UI task -> UI task

#if MECK_USB_DRIVE_BUILT

#include "esp_vfs_fat.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "device/usbd_pvt.h"   // usbd_defer_func: run a function on the TinyUSB task
#include "esp_heap_caps.h"           // MALLOC_CAP_SPIRAM
#include "freertos/idf_additions.h"  // xTaskCreatePinnedToCoreWithCaps

// The TinyUSB task is Meck's own (usb_device_task below, stack in PSRAM), so
// esp_tinyusb must not start its default one (main/Kconfig.projbuild:
// MECK_USB_DRIVE selects TINYUSB_NO_DEFAULT_TASK). Two tasks running the USB
// stack would break it.
#if !CONFIG_TINYUSB_NO_DEFAULT_TASK
#error "USB drive mode needs CONFIG_TINYUSB_NO_DEFAULT_TASK (selected by MECK_USB_DRIVE)"
#endif

extern _lock_t lvgl_api_lock;          // main.cpp: held by lvgl_ui_task around lv_timer_handler()
extern sdmmc_card_t *g_meck_sd_card;   // main.cpp: set by Sdmmc_Init() once the card mounted

#define USB_LOG(fmt, ...) printf("USB drive: " fmt "\n", ##__VA_ARGS__)

#define AUDIO_CLOSE_WAIT_MS  500    // the decoder closes its file this long after a stop, at most
#define USB_TASK_WAIT_MS     5000   // longest wait for a mount/unmount on the TinyUSB task

// The TinyUSB task: esp_tinyusb's defaults (4 KB stack, priority 5, core 1),
// but the stack in PSRAM. Internal RAM has no 4 KB block to spare by the time
// the row is used (hardware log: "create TinyUSB main task failed"), the same
// reason MeckGBC's task stack is in PSRAM. A PSRAM-stacked task must do no
// flash operations; this one only runs the USB stack, the SD card (SDMMC)
// and FAT mount/unmount.
#define USB_DEVICE_TASK_STACK  4096
#define USB_DEVICE_TASK_PRIO   5
#define USB_DEVICE_TASK_CORE   1

// ---- USB descriptors (ESP-IDF 5.4.1 examples/peripherals/usb/device/tusb_msc) ----

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

enum {
    EDPT_MSC_OUT = 0x01,
    EDPT_MSC_IN  = 0x81,
};

static tusb_desc_device_t s_device_desc = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,    // Espressif VID, as the ESP-IDF example
    .idProduct = 0x4002,   // as the ESP-IDF example
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static const uint8_t s_msc_fs_cfg_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64),
};

#if (TUD_OPT_HIGH_SPEED)
static const tusb_desc_device_qualifier_t s_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0
};

static const uint8_t s_msc_hs_cfg_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 512),
};
#endif // TUD_OPT_HIGH_SPEED

static const char s_lang_id[2] = { 0x09, 0x04 };   // English (0x0409)
static const char *s_string_desc[] = {
    s_lang_id,           // 0: supported language
    "Meck",              // 1: Manufacturer
    "Meck-P4 SD card",   // 2: Product
    "MECKP4",            // 3: Serial
};

// ---- State owned by meck_task ----

static bool s_meck_owns_fat = true;    // Meck's boot mount (Sdmmc_Init) is still in place
static bool s_storage_ready = false;   // tinyusb_msc_storage_init_sdmmc() done
static bool s_driver_installed = false;   // tinyusb_driver_install() done
static TaskHandle_t s_usb_task = NULL;     // usb_device_task, created once
static bool s_usb_started   = false;   // driver installed and usb_device_task running
static SemaphoreHandle_t s_usb_task_done = NULL;
static volatile esp_err_t s_usb_task_err = ESP_OK;

// ---- TinyUSB task ----

// Runs the USB stack: tud_task() waits on TinyUSB's event queue and handles
// each event (the computer's requests, esp_tinyusb's callbacks, and the
// functions queued with usbd_defer_func). Created only after
// tinyusb_driver_install() has initialised the stack, and never ends.
static void usb_device_task(void *arg)
{
    (void)arg;
    while (true) {
        tud_task();
    }
}

// esp_tinyusb: the card was mounted back on the P4 side (is_mounted) or
// handed to the computer (!is_mounted). While the mode is on, mounted back
// means the computer ejected the drive or the cable came out.
static void on_storage_mount_changed(tinyusb_msc_event_t *event)
{
    if (s_state == MECK_USB_DRIVE_ON) {
        s_host_ejected = event->mount_changed_data.is_mounted;
    }
}

// Run on the TinyUSB task: take the card from esp_tinyusb's P4-side mount
// (a no-op when it is not mounted).
static void usb_task_unmount(void *arg)
{
    (void)arg;
    s_usb_task_err = tinyusb_msc_storage_unmount();
    xSemaphoreGive(s_usb_task_done);
}

// Run on the TinyUSB task: give the card back to Meck at /sdcard. If the
// computer had ejected it, esp_tinyusb has it mounted at /data; that mount
// comes down first.
static void usb_task_mount_sdcard(void *arg)
{
    (void)arg;
    esp_err_t err = ESP_OK;
    if (!tinyusb_msc_storage_in_use_by_usb_host()) {
        err = tinyusb_msc_storage_unmount();
    }
    if (err == ESP_OK) {
        err = tinyusb_msc_storage_mount(SD_BASE_PATH);
    }
    s_usb_task_err = err;
    xSemaphoreGive(s_usb_task_done);
}

// ---- meck_task ----

static esp_err_t run_on_usb_task(osal_task_func_t fn)
{
    xSemaphoreTake(s_usb_task_done, 0);   // drop a late give left by an earlier timeout
    usbd_defer_func(fn, NULL, false);
    if (xSemaphoreTake(s_usb_task_done, pdMS_TO_TICKS(USB_TASK_WAIT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return s_usb_task_err;
}

// Take down Meck's boot mount of the card without touching the card or its
// SDMMC slot: the same steps as esp_tinyusb's own unmount (_unmount_sdmmc
// followed by esp_vfs_fat_unregister_path).
static esp_err_t take_down_meck_mount(sdmmc_card_t *card)
{
    const BYTE pdrv = ff_diskio_get_pdrv_card(card);
    if (pdrv == 0xff) {
        return ESP_ERR_INVALID_STATE;
    }
    char drv[3] = { (char)('0' + pdrv), ':', 0 };
    f_mount(NULL, drv, 0);
    ff_diskio_unregister(pdrv);
    return esp_vfs_fat_unregister_path(SD_BASE_PATH);
}

// Called with lvgl_api_lock held. Stops the audio player (audiobook, music
// or a notification tone) so the decoder task closes its file, and waits for
// it to do so.
static void stop_audio_for_handover(void)
{
    if (!meck_audio_ready()) return;
    const MeckAudioState st = meck_audio_get_state();
    if (st == MECK_AUDIO_STATE_IDLE) return;
    meck_audio_stop();
    vTaskDelay(pdMS_TO_TICKS(AUDIO_CLOSE_WAIT_MS));
}

static void usb_drive_fail(const char *why)
{
    USB_LOG("%s", why);
    s_error = why;
    s_state = MECK_USB_DRIVE_ERROR;
}

// Switch-on failed after the card was taken from Meck, before the TinyUSB
// task existed: nothing else can touch esp_tinyusb's mount, so the card goes
// straight back to Meck at /sdcard from here. why_card_back is the message
// shown when that works (a fixed string: the UI keeps the pointer).
static void give_back_without_usb_task(const char *why_card_back)
{
    if (tinyusb_msc_storage_mount(SD_BASE_PATH) != ESP_OK) {
        usb_drive_fail("USB could not start, and Meck could not read the SD card afterwards. "
                       "Restart the device.");
        return;
    }
    usb_drive_fail(why_card_back);
}

static void do_switch_on(void)
{
    sdmmc_card_t *card = g_meck_sd_card;
    if (!card) {
        usb_drive_fail("No SD card was found when the device started.");
        return;
    }
    if (!s_usb_task_done) {
        s_usb_task_done = xSemaphoreCreateBinary();
        if (!s_usb_task_done) {
            usb_drive_fail("Not enough memory to start USB drive mode.");
            return;
        }
    }

    // 1. esp_tinyusb's handle for the card. Memory only: nothing on the card
    //    or in Meck's mount changes here.
    if (!s_storage_ready) {
        const tinyusb_msc_sdmmc_config_t storage_cfg = {
            .card = card,
            .callback_mount_changed = on_storage_mount_changed,
            .callback_premount_changed = NULL,
            .mount_config = {
                .format_if_mount_failed = false,
                .max_files = 5,                      // as Meck's own mount (Sdmmc_Init)
                .allocation_unit_size = 16 * 1024,   // as Meck's own mount (Sdmmc_Init)
                .disk_status_check_enable = false,
                .use_one_fat = false,
            },
        };
        if (tinyusb_msc_storage_init_sdmmc(&storage_cfg) != ESP_OK) {
            usb_drive_fail("Not enough memory to start USB drive mode.");
            return;
        }
        s_storage_ready = true;
    }

    // 2. Take the card away from Meck, with every task that uses it held
    //    off (see the notes at the top of this file).
    esp_err_t err;
    _lock_acquire(&lvgl_api_lock);
    stop_audio_for_handover();
    if (s_meck_owns_fat) {
        err = take_down_meck_mount(card);
        if (err == ESP_OK) s_meck_owns_fat = false;
    } else if (s_usb_started) {
        err = run_on_usb_task(usb_task_unmount);
    } else {
        err = tinyusb_msc_storage_unmount();   // TinyUSB not running: nothing to overlap with
    }
    _lock_release(&lvgl_api_lock);
    if (err != ESP_OK) {
        USB_LOG("letting go of the card failed (%s)", esp_err_to_name(err));
        usb_drive_fail("Meck could not let go of the SD card. Restart the device before using the card again.");
        return;
    }

    // 3. Start USB the first time, reconnect it after that. The driver and
    //    the TinyUSB task are each set up once and kept until the restart.
    s_host_ejected = false;
    if (!s_driver_installed) {
        const tinyusb_config_t tusb_cfg = {
            .device_descriptor = &s_device_desc,
            .string_descriptor = s_string_desc,
            .string_descriptor_count = sizeof(s_string_desc) / sizeof(s_string_desc[0]),
            .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
            .fs_configuration_descriptor = s_msc_fs_cfg_desc,
            .hs_configuration_descriptor = s_msc_hs_cfg_desc,
            .qualifier_descriptor = &s_device_qualifier,
#else
            .configuration_descriptor = s_msc_fs_cfg_desc,
#endif // TUD_OPT_HIGH_SPEED
            .self_powered = false,     // as LilyGo's USB code: no VBUS sensing
            .vbus_monitor_io = -1,
        };
        err = tinyusb_driver_install(&tusb_cfg);
        if (err != ESP_OK) {
            USB_LOG("tinyusb_driver_install failed (%s)", esp_err_to_name(err));
            give_back_without_usb_task("USB could not start. The SD card is back with Meck.");
            return;
        }
        s_driver_installed = true;
    }
    if (!s_usb_task) {
        if (xTaskCreatePinnedToCoreWithCaps(usb_device_task, "TinyUSB", USB_DEVICE_TASK_STACK,
                                            NULL, USB_DEVICE_TASK_PRIO, &s_usb_task,
                                            USB_DEVICE_TASK_CORE, MALLOC_CAP_SPIRAM) != pdPASS) {
            s_usb_task = NULL;
            USB_LOG("TinyUSB task create failed");
            // Nothing would answer the computer: stay off the bus.
            tud_disconnect();
            give_back_without_usb_task("USB could not start (not enough memory). The SD card is back with Meck.");
            return;
        }
    }
    tud_connect();   // the first install has already connected; harmless then
    s_usb_started = true;
    s_state = MECK_USB_DRIVE_ON;
    USB_LOG("on: the SD card is on USB");
}

static void do_switch_off(void)
{
    tud_disconnect();
    const esp_err_t err = run_on_usb_task(usb_task_mount_sdcard);
    s_host_ejected = false;
    if (err != ESP_OK) {
        USB_LOG("mounting the card back failed (%s)", esp_err_to_name(err));
        usb_drive_fail("USB drive mode is off, but Meck could not read the SD card. "
             "Check the card on your computer, or restart the device.");
        return;
    }
    s_state = MECK_USB_DRIVE_OFF;
    USB_LOG("off: the SD card is back with Meck");
}

extern "C" void meck_usb_drive_service(void)
{
    const int req = s_request;
    if (req == 0) return;
    s_request = 0;
    if (req > 0 && s_state == MECK_USB_DRIVE_STARTING) {
        do_switch_on();
    } else if (req < 0 && s_state == MECK_USB_DRIVE_STOPPING) {
        do_switch_off();
    }
}

extern "C" bool meck_usb_drive_available(void)
{
    if (!g_meck_sd_card) return false;
    // Once esp_tinyusb owns the card it can go back to the computer even if
    // Meck could not read it (so it can be checked there).
    return p4_sdcard_is_mounted() || !s_meck_owns_fat;
}

extern "C" bool meck_usb_drive_host_connected(void)
{
    return s_usb_started && (s_state == MECK_USB_DRIVE_ON) && tud_mounted();
}

#else // !MECK_USB_DRIVE_BUILT

extern "C" void meck_usb_drive_service(void) {}
extern "C" bool meck_usb_drive_available(void) { return false; }
extern "C" bool meck_usb_drive_host_connected(void) { return false; }

#endif // MECK_USB_DRIVE_BUILT

// ---- UI task ----

extern "C" void meck_usb_drive_request(bool on)
{
    if (on) {
        if (s_state != MECK_USB_DRIVE_OFF && s_state != MECK_USB_DRIVE_ERROR) return;
        if (!MECK_USB_DRIVE_BUILT) {
            s_error = "USB drive mode is not available in this build.";
            s_state = MECK_USB_DRIVE_ERROR;
            return;
        }
        if (!meck_usb_drive_available()) {
            s_error = "No SD card found. Fit a card and restart the device, then try again.";
            s_state = MECK_USB_DRIVE_ERROR;
            return;
        }
        s_error = "";
        s_state = MECK_USB_DRIVE_STARTING;
        s_request = 1;
    } else {
        if (s_state != MECK_USB_DRIVE_ON) return;
        s_state = MECK_USB_DRIVE_STOPPING;
        s_request = -1;
    }
}

extern "C" meck_usb_drive_state_t meck_usb_drive_state(void) { return s_state; }

extern "C" bool meck_usb_drive_host_ejected(void)
{
    return (s_state == MECK_USB_DRIVE_ON) && s_host_ejected;
}

extern "C" const char* meck_usb_drive_error_text(void) { return s_error; }

extern "C" void meck_usb_drive_clear_error(void)
{
    if (s_state == MECK_USB_DRIVE_ERROR) {
        s_error = "";
        s_state = MECK_USB_DRIVE_OFF;
    }
}

extern "C" bool meck_usb_drive_card_away(void)
{
    const meck_usb_drive_state_t st = s_state;
    return (st == MECK_USB_DRIVE_STARTING) || (st == MECK_USB_DRIVE_ON) ||
           (st == MECK_USB_DRIVE_STOPPING);
}

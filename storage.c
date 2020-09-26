/***
** Created by Aleksey Volkov on 18.03.2020.
***/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <include/timers.h>

#include "nrf.h"
#include "nrf_soc.h"
#include "nordic_common.h"

#include "app_util.h"
#include "app_error.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_fstorage_sd.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "sensors/include/ec_sensor.h"
#include "storage.h"

#define MAGIC ((uint32_t) 0xDFDFDFDF)

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
    {
        /* Set a handler for fstorage events. */
        .evt_handler = fstorage_evt_handler,

        /* These below are the boundaries of the flash space assigned to this instance of fstorage.
         * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
         * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
         * last page of flash available to write data. */
        .start_addr = 0x5f000,
        .end_addr   = 0x5ffff,
    };

settings_t settings = { 0x00000000,0, 0, 1000, 10000, 30};
uint16_t current_version = 0;
uint32_t current_address = 0;

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
  uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
  uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
  uint32_t const code_sz         = NRF_FICR->CODESIZE;

  return (bootloader_addr != 0xFFFFFFFF ?
          bootloader_addr : (code_sz * page_sz));
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
  if (p_evt->result != NRF_SUCCESS)
  {
    NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
    return;
  }

  switch (p_evt->id)
  {
    case NRF_FSTORAGE_EVT_WRITE_RESULT:
    {
      NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                   p_evt->len, p_evt->addr);
    } break;

    case NRF_FSTORAGE_EVT_ERASE_RESULT:
    {
      NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                   p_evt->len, p_evt->addr);
    } break;

    default:
      break;
  }
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
  NRF_LOG_INFO("========| flash info |========");
  NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
  NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
  NRF_LOG_INFO("==============================");
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
  (void) sd_app_evt_wait();
}


void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
  /* While fstorage is busy, sleep and wait for an event. */
  while (nrf_fstorage_is_busy(p_fstorage))
  {
    power_manage();
  }
}

/* Initialize an fstorage instance using the nrf_fstorage_sd backend.
 * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
 * used whenever there is a SoftDevice, regardless of its status (enabled/disabled).
 *
 * returns version, if version> 0, it means that not initial values */
uint16_t storage_init()
{
  ret_code_t rc;

  nrf_fstorage_api_t * p_fs_api;
  p_fs_api = &nrf_fstorage_sd;

  rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
  APP_ERROR_CHECK(rc);

  print_flash_info(&fstorage);

  /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
  (void) nrf5_flash_end_addr_get();

  /* Find actual data. */
  current_address = fstorage.start_addr;
  for (uint32_t i = fstorage.start_addr; i < fstorage.end_addr; i += sizeof(settings_t)) {
    rc = nrf_fstorage_read(&fstorage, i, &settings, sizeof(settings_t));
    if (rc != NRF_SUCCESS)
    {
      NRF_LOG_ERROR("nrf_fstorage_read() returned: %s\n", rc);
    }

    if ((settings.magic == MAGIC) && (settings.version >= current_version))
    {
      current_version = settings.version;
      current_address = i;
    }
  }

  /* Read data. */
  rc = nrf_fstorage_read(&fstorage, current_address, &settings, sizeof(settings_t));
  if (rc != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("nrf_fstorage_read() returned: %s\n", rc);
  }

  NRF_LOG_INFO("settings addr:    %x", current_address);
  NRF_LOG_INFO("settings magic:   %x", settings.magic);
  NRF_LOG_INFO("settings version: %u", settings.version);
  NRF_LOG_INFO("settings low:     %i", settings.low_us);
  NRF_LOG_INFO("settings mid:     %i", settings.mid_us);
  NRF_LOG_INFO("settings hi:      %i", settings.hi_us);
  NRF_LOG_INFO("settings interval:%u", settings.scan_interval);
  NRF_LOG_INFO("==============================");

  return current_version;
}

void set_settings(uint16_t low, uint16_t mid, uint16_t hi)
{
  ret_code_t rc;

  /* 4 bytes aligned */
  current_address += sizeof(settings_t);
  if (current_address >= fstorage.end_addr || current_version == 0)
  {
    /* go to start in page full or fist write */
    current_address = fstorage.start_addr;

    /* erase page */
    rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
    APP_ERROR_CHECK(rc);
  }

  /* increase version */
  current_version++;

  settings.magic = MAGIC;
  settings.version = current_version;
  settings.low_us = low;
  settings.mid_us = mid;
  settings.hi_us  = hi;
  settings.scan_interval = 60; /* 10 min ToDo: adjust by ble characteristic */

  NRF_LOG_INFO("settings write addr:    %x", current_address);
  NRF_LOG_INFO("settings write version: %u", current_version);
  rc = nrf_fstorage_write(&fstorage, current_address, &settings, sizeof(settings_t), NULL);
  APP_ERROR_CHECK(rc);
}

settings_t * get_settings()
{
  return &settings;
}

void factory_settings()
{
  set_settings(0, 1000, 10000);
}
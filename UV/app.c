/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "em_cmu.h"
#include "int_enum.h"
#include "gpio.h"
#include "timer.h"
#include "i2c.h"
#include "si1132.h"

//The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

static Sensor sensors[MAX_SENSORS] = {{false, gpioPortA, 0, DEFAULT_ADDR},
									  {false, gpioPortA, 4, DEFAULT_ADDR},
									  {false, gpioPortA, 5, DEFAULT_ADDR},
									  {false, gpioPortA, 6, DEFAULT_ADDR}};
static uint16_t UV_data[MAX_SENSORS*MAX_BUFFER];
static uint8_t buffer_index = 0;
static uint8_t buffer_size = 0;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void)
{
	//INITIALIZE GPIO
	gpio_init();

	//INITIALIZE I2C
	i2c_init();

	//INITIALIZE TIMER
	timer_init_();

	//INITIALIZE SENSORS
	sensor_init();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
	/////////////////////////////////////////////////////////////////////////////
	// Put your additional application code here!                              //
	// This is called infinitely.                                              //
	// Do not call blocking functions from here!                               //
	/////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
	case sl_bt_evt_system_boot_id:
		// Extract unique ID from BT Address.
		sc = sl_bt_system_get_identity_address(&address, &address_type);
		app_assert_status(sc);

		// Pad and reverse unique ID to get System ID.
		system_id[0] = address.addr[5];
		system_id[1] = address.addr[4];
		system_id[2] = address.addr[3];
		system_id[3] = 0xFF;
		system_id[4] = 0xFE;
		system_id[5] = address.addr[2];
		system_id[6] = address.addr[1];
		system_id[7] = address.addr[0];

		sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
													 0,
													 sizeof(system_id),
													 system_id);
		app_assert_status(sc);

		// Define default connection parameters.
		sc = sl_bt_connection_set_default_parameters(
			800,	//min. connection interval: 1000 ms
			880,	//max. connection interval: 1100 ms
			1, 		//latency [maximum missed connection events]: 1 event
			550,	//timeout: 5500 ms
			0,		//min. connection event length: 0 ms
			1);		//max. connection event length: 0.675 ms
		app_assert_status(sc);

		// Set default preferred PHY to 2M PHY.
		// Note: All PHYs are accepted, just that 2M PHY is preferred
		sl_bt_connection_set_default_preferred_phy(0x02, 0xFF);

		// Create an advertising set.
		sc = sl_bt_advertiser_create_set(&advertising_set_handle);
		app_assert_status(sc);

		// Set advertising interval to 1000ms.
		sc = sl_bt_advertiser_set_timing(
			advertising_set_handle,
			1600, // min. adv. interval (milliseconds * 1.6)
			1600, // max. adv. interval (milliseconds * 1.6)
			0,   // adv. duration
			0);  // max. num. adv. events
		app_assert_status(sc);

		// Start general advertising and enable connections.
		sc = sl_bt_advertiser_start(
			advertising_set_handle,
			sl_bt_advertiser_general_discoverable,
			sl_bt_advertiser_connectable_scannable);
		app_assert_status(sc);
		break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
    	break;

    // -------------------------------
    // This event indicates that a connection was closed.
	case sl_bt_evt_connection_closed_id:
		// Stop the 1 second timer for UV measurements.
		// Reset buffer index and size
		sc = timer_sensor_stop();
		app_assert_status(sc);
		buffer_index = 0;
		buffer_size = 0;
		// Restart advertising after client has disconnected.
		sc = sl_bt_advertiser_start(
			advertising_set_handle,
			sl_bt_advertiser_general_discoverable,
			sl_bt_advertiser_connectable_scannable);
		app_assert_status(sc);
		break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

	//NOTE: This event handler also triggers when an indication acknowledge
	//is received. We do not need to check for that acknowledge since we
	//we are using notifications.
	case sl_bt_evt_gatt_server_characteristic_status_id:
		if (evt->data.evt_gatt_server_characteristic_status.characteristic
			!= gattdb_sensor_data) {
			// Exit if event was not caused by a change to sensor_data.
			break;
		}

		if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
			== gatt_notification) {
			// If gatt notifications were enabled
			// Start a 1 second timer for UV measurements.
			sc = timer_sensor_start();
			app_assert_status(sc);
		}
		else if (evt->data.evt_gatt_server_characteristic_status.client_config_flags
			== gatt_disable) {
			// If gatt notifications were disabled
			// Stop the 1 second timer for UV measurements.
			// Reset buffer index and size
			sc = timer_sensor_stop();
			app_assert_status(sc);
			buffer_index = 0;
			buffer_size = 0;
		}
		break;

		case sl_bt_evt_system_external_signal_id:
		// External signal triggered from sensor timer.
		if (evt->data.evt_system_external_signal.extsignals & INT_TIMER_SENSOR) {
			CMU_ClockDivSet(cmuClock_HCLK, 8);
			GPIO_PinOutToggle(gpioPortC, 0);

			measure_uv();
			if (buffer_size == MAX_BUFFER) {
				sl_bt_gatt_server_notify_all(
					gattdb_sensor_data,
					MAX_SENSORS*MAX_BUFFER*sizeof(uint16_t),
					(uint8_t *)UV_data);
				buffer_index = 0;
				buffer_size = 0;
			}

			GPIO_PinOutToggle(gpioPortC, 0);
			CMU_ClockDivSet(cmuClock_HCLK, 1);
		}
		else if (evt->data.evt_system_external_signal.extsignals & INT_BUTTON) {

		}
		break;

    // -------------------------------
    // Default event handler.
	default:
		break;
  }
}

void sensor_init() {
	for (uint8_t i = 0; i < MAX_SENSORS; i++) {
		if (si1132_init(sensors[i].power_port, sensors[i].power_pin, I2C0, DEFAULT_ADDR+i+1)) {
			sensors[i].addr = DEFAULT_ADDR+i+1;
			sensors[i].active = true;
			for (uint8_t o = 0; o < MAX_SENSORS*MAX_BUFFER; o += MAX_SENSORS) {
				UV_data[o+i] = 0x0000;
			}
		}
		else {
			GPIO_PinOutClear(sensors[i].power_port, sensors[i].power_pin);
			for (uint8_t o = 0; o < MAX_SENSORS*MAX_BUFFER; o += MAX_SENSORS) {
				UV_data[o+i] = 0xFFFF;
			}
		}
	}
}

void measure_uv() {
	for (uint8_t i = 0; i < MAX_SENSORS; i++) {
		if (sensors[i].active) {
			si1132_UV_start_measurement(I2C0, sensors[i].addr);
			si1132_UV_read_measurement(I2C0, sensors[i].addr, &UV_data[buffer_index]);
		}
		buffer_index++;
	}
	buffer_size++;
}

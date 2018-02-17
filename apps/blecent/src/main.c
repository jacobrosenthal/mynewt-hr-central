/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"
#include "hal/hal_bsp.h"

#if MYNEWT_VAL(SPLIT_APPLICATION)
#include "imgmgr/imgmgr.h"
#include "hal/hal_system.h"
#include "hal/hal_watchdog.h"
#include "hal/hal_gpio.h"
#endif

/* BLE */
#include "nimble/ble.h"
#include "controller/ble_ll.h"
#include "host/ble_hs.h"

/* RAM HCI transport. */
#include "transport/ram/ble_hci_ram.h"

/* Mandatory services. */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Application-specified header. */
#include "blecent.h"

/* HR LED */
#include "hr_led.h"

/* HR Button */
#include "debounce/debounce.h"
static struct os_event debounce_handle_event;
static debounce_pin_t button;

/** Log data. */
struct log blecent_log;

static int blecent_gap_event(struct ble_gap_event *event, void *arg);

/**
 * Called when service discovery of the specified peer has completed.
 */
static void
blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
{

    if (status != 0) {
        /* Service discovery failed.  Terminate the connection. */
        BLECENT_LOG(ERROR, "Error: Service discovery failed; status=%d "
                           "conn_handle=%d\n", status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    /* Service discovery has completed successfully.  Now we have a complete
     * list of services, characteristics, and descriptors that the peer
     * supports.
     */
    BLECENT_LOG(ERROR, "Service discovery complete; status=%d "
                       "conn_handle=%d\n", status, peer->conn_handle);

    /* Now perform three concurrent GATT procedures against the peer: read,
     * write, and subscribe to notifications.
     */
    hr_led_read_write_subscribe(peer);
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void
blecent_scan(void)
{
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}


/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void
blecent_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    int rc;

    /* Don't do anything if we don't care about this advertiser. */
    if (!hr_led_should_connect(disc)) {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        BLECENT_LOG(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */
    rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &disc->addr, 30000, NULL,
                         blecent_gap_event, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error: Failed to connect to device; addr_type=%d "
                           "addr=%s\n", disc->addr.type,
                           addr_str(disc->addr.val));
        return;
    }
}


/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;

    hr_led_gap_event(event, arg);

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        /* An advertisment report was received during GAP discovery. */
        print_adv_fields(&fields);

        /* Try to connect to the advertiser if it looks interesting. */
        blecent_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {
            /* Connection successfully established. */
            BLECENT_LOG(INFO, "Connection established ");

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            BLECENT_LOG(INFO, "\n");

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                BLECENT_LOG(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               blecent_on_disc_complete, NULL);
            if (rc != 0) {
                BLECENT_LOG(ERROR, "Failed to discover services; rc=%d\n", rc);
                return 0;
            }
        } else {
            /* Connection attempt failed; resume scanning. */
            BLECENT_LOG(ERROR, "Error: Connection failed; status=%d\n",
                        event->connect.status);
            blecent_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        BLECENT_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        BLECENT_LOG(INFO, "\n");

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);

        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

static void
start()
{
    const struct peer *peer;
    int rc;

    peer = peer_find_first();

    if(peer != NULL){
        ble_gap_terminate(peer->conn_handle, BLE_ERR_RD_CONN_TERM_PWROFF);
        //hr_led_stop happens on disconnect success
    }else if(ble_gap_disc_active()){

        ble_gap_disc_cancel();
        hr_led_stop();
    }else{

        blecent_scan();
        rc = hr_led_start(OS_TICKS_PER_SEC / 8);
        if(rc){
            BLECENT_LOG(ERROR, "Failed to start pwm rc=%d\n", rc);
        }
    }
}

/* todo just fix debounce to take or overload a queue to run events on */
static void
buttonPressed(debounce_pin_t *d)
{
    os_eventq_put(os_eventq_dflt_get(), &debounce_handle_event);
}

static void
button_init()
{
    int rc;

    rc = hal_timer_config(MYNEWT_VAL(BLECENT_DEBOUNCE_TIMER), 31250);
    assert(rc == 0);

    rc = debounce_init(&button, MYNEWT_VAL(BLECENT_HR_BUTTON_PIN), MYNEWT_VAL(BLECENT_HR_BUTTON_CFG), MYNEWT_VAL(BLECENT_DEBOUNCE_TIMER));
    assert(rc == 0);

    debounce_handle_event.ev_cb = &start;

#if MYNEWT_VAL(BLECENT_HR_BUTTON_VAL) == 0
    rc = debounce_start(&button, DEBOUNCE_CALLBACK_EVENT_FALL, buttonPressed, NULL);
#else
    rc = debounce_start(&button, DEBOUNCE_CALLBACK_EVENT_RISE, buttonPressed, NULL);
#endif
    assert(rc == 0);
}

#if MYNEWT_VAL(SPLIT_APPLICATION)
static void
enter_loader(struct os_event *ev)
{
    int rc;

    rc = imgmgr_state_set_pending(0,0);
    assert(rc == 0);

    hal_watchdog_tickle();
    hal_system_reset();
}
#endif

/**
 * main
 *
 * All application logic and NimBLE host work is performed in default task.
 *
 * @return int NOTE: this function should never return!
 */
int
main(void)
{
    int rc;

    /* Initialize OS */
    sysinit();

#if MYNEWT_VAL(SPLIT_APPLICATION)
    {
        /* If button is enabled, go back to loader */
        rc = hal_gpio_init_in(MYNEWT_VAL(BLECENT_LOADER_BUTTON_PIN), MYNEWT_VAL(BLECENT_LOADER_BUTTON_CFG));
        assert(rc == 0);

        if(hal_gpio_read(MYNEWT_VAL(BLECENT_LOADER_BUTTON_PIN)) == MYNEWT_VAL(BLECENT_LOADER_BUTTON_VAL))
        {
            enter_loader(NULL);
        }
    }
#endif

    /* Initialize the blecent log. */
    log_register("blecent", &blecent_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);

    /* Configure the host. */
    log_register("ble_hs", &ble_hs_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), MYNEWT_VAL(BLECENT_PEER_SVCS_MAX), MYNEWT_VAL(BLECENT_PEER_CHRS_MAX), MYNEWT_VAL(BLECENT_PEER_DSCS_MAX));
    assert(rc == 0);

    button_init();

    /* os start should never return. If it does, this should be an error */
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }

    return 0;
}

#include <pwm/pwm.h>
#include "bsp/bsp.h"

/* BLE */
#include "nimble/ble.h"
#include "controller/ble_ll.h"
#include "host/ble_hs.h"

/* RAM HCI transport. */
#include "transport/ram/ble_hci_ram.h"

/* Mandatory services. */
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "hr_led.h"

#define BLECENT_SVC_HRM_UUID                0x180D
#define BLECENT_CHR_HRM_UUID                0x2A37

#define HRM_BIT_FORMAT_POS                  0
#define HRM_CONTACT_SUPPORT_POS             1
#define HRM_EE_SUPPORT_POS                  3
#define HRM_RR_SUPPORT_POS                  4

#define HRM_BIT_FORMAT_MASK                 (0x1 << HRM_BIT_FORMAT_POS)
#define HRM_CONTACT_SUPPORT_MASK            (0x3 << HRM_CONTACT_SUPPORT_POS)
#define HRM_EE_SUPPORT_MASK                 (0x1 << HRM_EE_SUPPORT_POS)
#define HRM_RR_SUPPORT_MASK                 (0x1 << HRM_RR_SUPPORT_POS)


static struct pwm_dev *pwm;
static int32_t g_freq;

static struct os_callout led_on_callout;
static struct os_callout led_off_callout;

static int
hr_led_set_freq(int32_t freq)
{
    if (freq < OS_TICKS_PER_SEC/10) {
        return OS_EINVAL;
    }

    //adjust for led on time
    g_freq = freq - OS_TICKS_PER_SEC/10;
    return 0;
}

static void
hr_led_off()
{
    int rc;

    rc = pwm_enable_duty_cycle(pwm, 0, 0);
    if(rc){
        BLECENT_LOG(ERROR, "Error: Failed to pwm_enable_duty_cycle; "
                           "rc=%d\n", rc);
    }
    os_callout_reset(&led_on_callout, g_freq);
}

static void
hr_led_on(struct os_event *ev)
{
    int rc;

    // TODO control brightness from a config value to taste
    rc = pwm_enable_duty_cycle(pwm, 0, 55000);
    if(rc){
        BLECENT_LOG(ERROR, "Error: Failed to pwm_enable_duty_cycle; "
                           "rc=%d\n", rc);
    }

    os_callout_reset(&led_off_callout, OS_TICKS_PER_SEC/10);
}

void
hr_led_stop()
{
    os_callout_stop(&led_on_callout);
    os_callout_stop(&led_off_callout);
    if(pwm){
        os_dev_close((struct os_dev *)pwm);
    }
}

int
hr_led_start(int32_t ticks)
{
    int rc;

    rc = hr_led_set_freq(ticks);
    if(rc){
        goto error;
    }

    struct pwm_chan_cfg chan_conf = {
        .pin = MYNEWT_VAL(BLECENT_HR_LED_PIN),
        .inverted = MYNEWT_VAL(BLECENT_HR_LED_INVERTED),
        .data = NULL
    };

#if MYNEWT_VAL(SOFT_PWM)
    pwm = (struct pwm_dev *) os_dev_open("spwm", 0, NULL);
#else
    pwm = (struct pwm_dev *) os_dev_open("pwm0", 0, NULL);
#endif

    pwm_set_frequency(pwm, 300);
    
    rc = pwm_chan_config(pwm, 0, &chan_conf);
    if(rc){
        goto error;
    }

    hr_led_on(NULL);

    return 0;
error:
    return rc;
}

static int
blecent_on_subscribe(uint16_t conn_handle,
                     const struct ble_gatt_error *error,
                     struct ble_gatt_attr *attr,
                     void *arg)
{
    BLECENT_LOG(INFO, "Subscribe complete; status=%d conn_handle=%d "
                      "attr_handle=%d\n",
                error->status, conn_handle, attr->handle);

    hr_led_set_freq(OS_TICKS_PER_SEC / 5);

    return 0;
}

void
hr_led_read_write_subscribe(const struct peer *peer)
{
    const struct peer_dsc *dsc;
    uint8_t value[2];
    int rc;

    /* Subscribe to notifications for the Heart Rate Measurement characteristic.*/
    dsc = peer_dsc_find_uuid(peer,
                             BLE_UUID16_DECLARE(BLECENT_SVC_HRM_UUID),
                             BLE_UUID16_DECLARE(BLECENT_CHR_HRM_UUID),
                             BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));

    if (dsc == NULL) {
        BLECENT_LOG(ERROR, "Error: Peer lacks a CCCD for the Unread Alert "
                           "Status characteristic\n");
        goto err;
    }

    value[0] = 1;
    value[1] = 0;
    rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle,
                              value, sizeof value, blecent_on_subscribe, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error: Failed to subscribe to characteristic; "
                           "rc=%d\n", rc);
        goto err;
    }

    return;

err:
    /* Terminate the connection. */
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}


void
hr_led_init(void)
{
    os_callout_init(&led_on_callout, os_eventq_dflt_get(),
            hr_led_on, NULL);

    os_callout_init(&led_off_callout, os_eventq_dflt_get(),
            hr_led_off, NULL);
}

// HRS_SPEC_V10.pdf
// https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.heart_rate_measurement.xml
static void
hr_led_on_notify(const struct os_mbuf *om)
{
    uint16_t measurement;
    uint16_t energy_expenditure;
    uint8_t sensor_status;
    uint8_t offset = 0;
    const uint8_t *bytes = om->om_data;
    const uint8_t flags = bytes[offset++];

    /*
     *  "The value of the Sensor Contact Support bit is static while in a connection.
     *  The value of the Sensor Contact Status bit may change while in a connection."
     */
    sensor_status = (flags & HRM_CONTACT_SUPPORT_MASK) >> HRM_CONTACT_SUPPORT_POS;
    BLECENT_LOG(INFO, "sensor_contact=%d\r\n", sensor_status);

    if(sensor_status!=2){

        uint8_t format = (flags & HRM_BIT_FORMAT_MASK) >> HRM_BIT_FORMAT_POS;

        if(format){
            uint8_t lower = bytes[offset++];
            uint8_t upper = bytes[offset++] << 8;
            measurement = upper | lower;
        }else{
            measurement = bytes[offset++];
        }
        BLECENT_LOG(INFO, "measurement=%d\r\n", measurement);
        hr_led_set_freq(OS_TICKS_PER_SEC * (measurement / 60));
    }else{
        BLECENT_LOG(INFO, "sensor has no skin contact\n");
    }

    /*
     *  "If energy expended is used, it is typically only included in the eart Rate Measurement
     *  characteristic once every 10 measurements at a regular interval."
     */
    if((flags & HRM_EE_SUPPORT_MASK) >> HRM_EE_SUPPORT_POS){
        uint8_t lower = bytes[offset++];
        uint8_t upper = bytes[offset++] << 8;
        energy_expenditure = upper | lower;
        BLECENT_LOG(INFO, "energy_expenditure=%d\r\n", energy_expenditure);
    }

    /*
     *  "For a 23-octet ATT_MTU and the Heart Rate Measurement Value format set to UINT8,
     *  the maximum number of RR-Interval Values that can be notified if Energy Expended is
     *  BLUETOOTH SERVICE SPECIFICATION Page 12 of 15
     *  Heart Rate Service
     *  present is 8 and the maximum number of RR-Interval Values that can be notified if
     *  Energy Expended is not present is 9."
     */
    if((flags & HRM_RR_SUPPORT_MASK) >> HRM_RR_SUPPORT_POS){
        while(offset<om->om_len)
        {
            uint8_t lower = bytes[offset++];
            uint8_t upper = bytes[offset++] << 8;
            uint16_t rr = upper | lower;
            BLECENT_LOG(INFO, "rr=%d\r\n", rr);
        }
    }
}

/**
 * Indicates whether we should tre to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */
int
hr_led_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;
    int i;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return rc;
    }

    /* The device has to advertise support for the HRM Service
     * service (0x180d).
     */
    for (i = 0; i < fields.num_uuids16; i++) {
        if (ble_uuid_u16(&fields.uuids16[i].u) == BLECENT_SVC_HRM_UUID) {
            return 1;
        }
    }

    return 0;
}

void
hr_led_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_DISCONNECT:
        hr_led_stop();
        break;

    case BLE_GAP_EVENT_NOTIFY_RX:
        hr_led_on_notify(event->notify_rx.om);
        break;

    default:
        break;
    }
}
#include <DRIVERS/bno085_driver.h>
#include <string.h>
#include <math.h>

/* ============================================================
 * Internal constants
 * ============================================================ */

#define SHTP_HEADER_SIZE           4
#define SH2_REPORT_BASE_TIMESTAMP  0xFB
#define SH2_REPORT_TIMESTAMP_REBASE 0xF3
#define SH2_REPORT_COMMAND_RESPONSE 0xF1

/*
 * The BNO08x startup advertisement can be larger than 256 bytes.
 * Keep this receive buffer local to this driver implementation so the
 * public BNO085 structure and header API do not need to change.
 */
#define BNO085_MAX_SHTP_PACKET_SIZE       320U
#define BNO085_DISCARD_CHUNK_DATA_SIZE    60U
#define BNO085_MAX_DRAIN_PACKET_SIZE      1024U

/* Shared receive buffers: this driver is serviced from the main context. */
static uint8_t s_bno_rx_packet[BNO085_MAX_SHTP_PACKET_SIZE];
static uint8_t s_bno_rx_discard[SHTP_HEADER_SIZE + BNO085_DISCARD_CHUNK_DATA_SIZE];

/* ============================================================
 * Little-endian decoding helpers
 * ============================================================ */

static inline uint16_t read_u16(const uint8_t *p)
{
    return ((uint16_t)p[0]) | ((uint16_t)p[1] << 8);
}

static inline int16_t read_i16(const uint8_t *p)
{
    return (int16_t)read_u16(p);
}

/* ============================================================
 * SHTP sequence number (Host-to-Sensor TX)
 * ============================================================ */

static uint8_t get_next_sequence(BNO085 *dev, uint8_t channel)
{
    if (channel >= 6) return 0;
    uint8_t sequence = dev->tx_sequence[channel];
    dev->tx_sequence[channel]++;
    return sequence;
}

/* ============================================================
 * Send SHTP packet
 * ============================================================ */

static BNO_PortStatus BNO085_SendSHTP(
    BNO085 *dev,
    uint8_t channel,
    const uint8_t *payload,
    uint16_t payload_length
)
{
    uint8_t packet[BNO085_RX_BUFFER_SIZE];
    uint16_t total_length;

    if (dev == NULL || payload == NULL || dev->port.write == NULL)
        return BNO_PORT_ERROR;

    total_length = SHTP_HEADER_SIZE + payload_length;

    if (total_length > sizeof(packet))
        return BNO_PORT_ERROR;

    /* SHTP header: Length LSB, Length MSB (bit 15=0), Channel, Sequence */
    packet[0] = (uint8_t)(total_length & 0xFF);
    packet[1] = (uint8_t)((total_length >> 8) & 0x7F);
    packet[2] = channel;
    packet[3] = get_next_sequence(dev, channel);

    /* Payload */
    memcpy(&packet[4], payload, payload_length);

    /* Hardware transmit */
    BNO_PortStatus status = dev->port.write(packet, total_length);
    if (status == BNO_PORT_OK) {
        dev->diag.tx_packets++;
    } else {
        dev->diag.tx_errors++;
    }
    return status;
}

/* ============================================================
 * Soft Reset (SHTP Channel 1 - Executable)
 * ============================================================ */

BNO_PortStatus BNO085_SoftReset(BNO085 *dev)
{
    uint8_t reset_cmd = 0x01; /* Reset request */
    return BNO085_SendSHTP(dev, BNO_CHANNEL_EXECUTABLE, &reset_cmd, 1);
}

/* ============================================================
 * Send Set Feature Command (SHTP Channel 2 - Control)
 * ============================================================ */

static BNO_PortStatus BNO085_SetFeature(
    BNO085 *dev,
    uint8_t report_id,
    uint32_t interval_us
)
{
    uint8_t payload[17];
    memset(payload, 0, sizeof(payload));

    payload[0] = BNO_CMD_SET_FEATURE; /* 0xFD */
    payload[1] = report_id;
    payload[2] = 0; /* Flags */
    payload[3] = 0; /* Change sensitivity LSB */
    payload[4] = 0; /* Change sensitivity MSB */

    /* Report interval in microseconds (uint32_t, little-endian) */
    payload[5] = (uint8_t)(interval_us & 0xFF);
    payload[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    payload[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    payload[8] = (uint8_t)((interval_us >> 24) & 0xFF);

    /* Batch interval (0 = no batching) */
    payload[9] = 0;
    payload[10] = 0;
    payload[11] = 0;
    payload[12] = 0;

    /* Sensor-specific config (0 = default) */
    payload[13] = 0;
    payload[14] = 0;
    payload[15] = 0;
    payload[16] = 0;

    return BNO085_SendSHTP(dev, BNO_CHANNEL_CONTROL, payload, sizeof(payload));
}

/* ============================================================
 * Internal SHTP packet reader forward declaration
 * ============================================================ */

static BNO_PortStatus read_shtp_packet(BNO085 *dev);

/* ============================================================
 * Public initialization
 * ============================================================ */

BNO_PortStatus BNO085_Init(BNO085 *dev, const BNO_Port *port)
{
    if (dev == NULL || port == NULL)
        return BNO_PORT_ERROR;

    memset(dev, 0, sizeof(BNO085));
    dev->port = *port;
    dev->diag.i2c_addr = BNO_Port_GetAddress();

    /* Hardware reset if pin is configured */
    if (dev->port.reset != NULL)
    {
        dev->port.reset();
    }

    /* Wait for BNO085 initial boot */
    if (dev->port.delay_ms != NULL)
    {
        dev->port.delay_ms(300);
    }

    /* Send software reset and stop initialization if the write fails. */
    BNO_PortStatus status = BNO085_SoftReset(dev);
    if (status != BNO_PORT_OK)
    {
        dev->initialized = false;
        return status;
    }

    if (dev->port.delay_ms != NULL)
    {
        dev->port.delay_ms(200);
    }

    /*
     * Drain startup and advertisement packets.
     * In polling mode, BNO_PORT_TIMEOUT means that no packet is currently
     * available, so the drain is complete. Other read failures are fatal.
     */
    uint8_t consecutive_timeouts = 0U;

    for (int i = 0; i < 30; i++)
    {
        status = read_shtp_packet(dev);

        if (status == BNO_PORT_TIMEOUT)
        {
            consecutive_timeouts++;
            if (consecutive_timeouts >= 3U)
            {
                break;
            }
        }
        else if (status != BNO_PORT_OK)
        {
            dev->initialized = false;
            return status;
        }
        else
        {
            consecutive_timeouts = 0U;
        }

        if (dev->port.delay_ms != NULL)
        {
            dev->port.delay_ms(10);
        }
    }

    /* Do not report successful initialization if the sensor never replied. */
    if (dev->diag.rx_packets == 0U)
    {
        dev->initialized = false;
        return BNO_PORT_ERROR;
    }

    dev->initialized = true;
    return BNO_PORT_OK;
}

/* ============================================================
 * Interrupt notification
 * ============================================================ */

void BNO085_DataReady(BNO085 *dev)
{
    if (dev == NULL)
        return;

    dev->data_ready = true;
}

/* ============================================================
 * Enable report functions
 * ============================================================ */

BNO_PortStatus BNO085_EnableAccel(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_ACCEL, interval_us);
}

BNO_PortStatus BNO085_EnableGyro(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_GYRO, interval_us);
}

BNO_PortStatus BNO085_EnableMag(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_MAG, interval_us);
}

BNO_PortStatus BNO085_EnableLinearAccel(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_LINEAR_ACCEL, interval_us);
}

BNO_PortStatus BNO085_EnableGravity(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_GRAVITY, interval_us);
}

BNO_PortStatus BNO085_EnableRotation(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_ROTATION, interval_us);
}

BNO_PortStatus BNO085_EnableGameRotation(BNO085 *dev, uint32_t interval_us)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    return BNO085_SetFeature(dev, BNO_REPORT_GAME_ROTATION, interval_us);
}

/* ============================================================
 * Parse individual sensor events inside SHTP report payload
 * ============================================================ */

static uint16_t parse_sensor_event(BNO085 *dev, const uint8_t *data, uint16_t remaining)
{
    if (remaining < 1)
        return 0;

    uint8_t report_id = data[0];

    switch (report_id)
    {
    case SH2_REPORT_BASE_TIMESTAMP:
    case SH2_REPORT_TIMESTAMP_REBASE:
        return (remaining >= 5) ? 5 : 0;

    case SH2_REPORT_COMMAND_RESPONSE:
        return (remaining >= 16) ? 16 : (remaining >= 4 ? 4 : 0);

    case BNO_REPORT_ACCEL:
        if (remaining < 10) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            dev->data.accel.x = (float)x / 256.0f;
            dev->data.accel.y = (float)y / 256.0f;
            dev->data.accel.z = (float)z / 256.0f;
            dev->data.accel_valid = true;
            dev->diag.rx_reports++;
        }
        return 10;

    case BNO_REPORT_GYRO:
        if (remaining < 10) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            dev->data.gyro.x = (float)x / 512.0f;
            dev->data.gyro.y = (float)y / 512.0f;
            dev->data.gyro.z = (float)z / 512.0f;
            dev->data.gyro_valid = true;
            dev->diag.rx_reports++;
        }
        return 10;

    case BNO_REPORT_MAG:
        if (remaining < 10) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            dev->data.mag.x = (float)x / 16.0f;
            dev->data.mag.y = (float)y / 16.0f;
            dev->data.mag.z = (float)z / 16.0f;
            dev->data.mag_valid = true;
            dev->diag.rx_reports++;
        }
        return 10;

    case BNO_REPORT_LINEAR_ACCEL:
        if (remaining < 10) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            dev->data.linear_accel.x = (float)x / 256.0f;
            dev->data.linear_accel.y = (float)y / 256.0f;
            dev->data.linear_accel.z = (float)z / 256.0f;
            dev->data.linear_accel_valid = true;
            dev->diag.rx_reports++;
        }
        return 10;

    case BNO_REPORT_GRAVITY:
        if (remaining < 10) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            dev->data.gravity.x = (float)x / 256.0f;
            dev->data.gravity.y = (float)y / 256.0f;
            dev->data.gravity.z = (float)z / 256.0f;
            dev->data.gravity_valid = true;
            dev->diag.rx_reports++;
        }
        return 10;

    case BNO_REPORT_ROTATION:
        if (remaining < 14) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            int16_t w = read_i16(&data[10]);
            dev->data.rotation.x = (float)x / 16384.0f;
            dev->data.rotation.y = (float)y / 16384.0f;
            dev->data.rotation.z = (float)z / 16384.0f;
            dev->data.rotation.w = (float)w / 16384.0f;
            dev->data.rotation_valid = true;
            dev->diag.rx_reports++;
        }
        return 14;

    case BNO_REPORT_GAME_ROTATION:
        if (remaining < 12) return 0;
        {
            int16_t x = read_i16(&data[4]);
            int16_t y = read_i16(&data[6]);
            int16_t z = read_i16(&data[8]);
            int16_t w = read_i16(&data[10]);
            dev->data.game_rotation.x = (float)x / 16384.0f;
            dev->data.game_rotation.y = (float)y / 16384.0f;
            dev->data.game_rotation.z = (float)z / 16384.0f;
            dev->data.game_rotation.w = (float)w / 16384.0f;
            dev->data.game_rotation_valid = true;
            dev->diag.rx_reports++;
        }
        return 12;

    default:
        /* Unknown report ID: return 0 so parser can advance and resync */
        return 0;
    }
}

/* ============================================================
 * Parse all sensor reports inside an SHTP packet payload
 * ============================================================ */

static void parse_reports(BNO085 *dev, const uint8_t *data, uint16_t length)
{
    uint16_t offset = 0;
    while (offset < length)
    {
        uint16_t consumed = parse_sensor_event(dev, &data[offset], length - offset);
        if (consumed == 0)
        {
            /* Skip 1 byte to find next valid report ID */
            offset++;
        }
        else
        {
            offset += consumed;
        }
    }
}

/* ============================================================
 * Read one SHTP packet
 * ============================================================ */

static BNO_PortStatus read_shtp_packet(BNO085 *dev)
{
    BNO_PortStatus status;
    uint8_t header[SHTP_HEADER_SIZE];
    uint16_t raw_length;
    uint16_t packet_length;
    uint16_t requested_packet_length;
    uint16_t payload_length;
    uint8_t channel;

    if (dev == NULL || dev->port.read == NULL)
    {
        return BNO_PORT_ERROR;
    }

    /*
     * First read only the four-byte SHTP header to determine the packet size.
     * The BNO08x I2C interface returns the header again on the subsequent
     * read, so the second read below requests the complete packet, including
     * its four-byte header.
     */
    status = dev->port.read(header, SHTP_HEADER_SIZE);
    if (status != BNO_PORT_OK)
    {
        /* A read timeout is normal in polling mode when no packet is ready. */
        if (status != BNO_PORT_TIMEOUT)
        {
            dev->diag.rx_errors++;
        }
        return status;
    }

    memcpy(dev->diag.last_header, header, SHTP_HEADER_SIZE);

    raw_length = read_u16(header);

    /* 0xFFFF is reserved by SHTP to indicate a failed peripheral. */
    if (raw_length == 0xFFFFU)
    {
        dev->diag.rx_errors++;
        return BNO_PORT_ERROR;
    }

    /* Bit 15 is the SHTP continuation flag; bits 14:0 contain the length. */
    packet_length = (uint16_t)(raw_length & 0x7FFFU);
    dev->diag.last_packet_len = packet_length;
    dev->diag.last_channel = header[2];

    /* A zero-length packet means there is no payload to process. */
    if (packet_length == 0U)
    {
        return BNO_PORT_OK;
    }

    if (packet_length < SHTP_HEADER_SIZE)
    {
        dev->diag.rx_errors++;
        return BNO_PORT_ERROR;
    }

    payload_length = (uint16_t)(packet_length - SHTP_HEADER_SIZE);

    /*
     * Packets larger than our receive buffer are drained in bounded chunks
     * to avoid leaving the BNO08x I2C transfer partially consumed. The
     * startup advertisement is normally 284 bytes, which fits in the
     * 320-byte receive buffer above.
     */
    if (packet_length > sizeof(s_bno_rx_packet))
    {
        uint16_t remaining = payload_length;

        if (packet_length > BNO085_MAX_DRAIN_PACKET_SIZE)
        {
            dev->diag.rx_errors++;
            return BNO_PORT_ERROR;
        }

        while (remaining > 0U)
        {
            uint16_t chunk = remaining;
            if (chunk > BNO085_DISCARD_CHUNK_DATA_SIZE)
            {
                chunk = BNO085_DISCARD_CHUNK_DATA_SIZE;
            }

            /* Each partial I2C read includes the four-byte SHTP header. */
            status = dev->port.read(
                s_bno_rx_discard,
                (size_t)(SHTP_HEADER_SIZE + chunk)
            );

            if (status != BNO_PORT_OK)
            {
                if (status != BNO_PORT_TIMEOUT)
                {
                    dev->diag.rx_errors++;
                }
                return status;
            }

            remaining = (uint16_t)(remaining - chunk);
        }

        /* Valid but oversized packet was drained and intentionally ignored. */
        dev->diag.rx_packets++;
        return BNO_PORT_OK;
    }

    /* Save the length advertised by the initial header. */
    requested_packet_length = packet_length;

    /* Re-read the complete packet (header + payload), as required by SHTP/I2C. */
    status = dev->port.read(s_bno_rx_packet, requested_packet_length);
    if (status != BNO_PORT_OK)
    {
        if (status != BNO_PORT_TIMEOUT)
        {
            dev->diag.rx_errors++;
        }
        return status;
    }

    /* Use the header returned with the complete packet. */
    raw_length = read_u16(s_bno_rx_packet);
    if (raw_length == 0xFFFFU)
    {
        dev->diag.rx_errors++;
        return BNO_PORT_ERROR;
    }

    packet_length = (uint16_t)(raw_length & 0x7FFFU);
    if (packet_length < SHTP_HEADER_SIZE ||
        packet_length > requested_packet_length ||
        packet_length > sizeof(s_bno_rx_packet))
    {
        dev->diag.rx_errors++;
        return BNO_PORT_ERROR;
    }

    payload_length = (uint16_t)(packet_length - SHTP_HEADER_SIZE);
    channel = s_bno_rx_packet[2];

    memcpy(dev->diag.last_header, s_bno_rx_packet, SHTP_HEADER_SIZE);
    dev->diag.last_packet_len = packet_length;
    dev->diag.last_channel = channel;

    if (channel < 6U)
    {
        dev->rx_sequence[channel] = s_bno_rx_packet[3];
    }

    dev->diag.rx_packets++;

    /* Input reports arrive on Channel 3, Channel 4, or Channel 5. */
    if (channel == BNO_CHANNEL_INPUT ||
        channel == BNO_CHANNEL_WAKE_INPUT ||
        channel == BNO_CHANNEL_GYRO_ROTATION)
    {
        parse_reports(dev, &s_bno_rx_packet[SHTP_HEADER_SIZE], payload_length);
    }

    return BNO_PORT_OK;
}

/* ============================================================
 * Main update function
 * ============================================================ */

BNO_PortStatus BNO085_Update(BNO085 *dev)
{
    if (dev == NULL || !dev->initialized)
        return BNO_PORT_ERROR;

    /*
     * Polling mode: read_shtp_packet() performs a bounded I2C read.
     * BNO085_DataReady() remains available for a future EXTI-based version,
     * but this implementation does not gate reads on the interrupt flag.
     */
    return read_shtp_packet(dev);
}

/* ============================================================
 * Value getters
 * ============================================================ */

bool BNO085_GetAccel(BNO085 *dev, BNO_Vector3 *accel)
{
    if (dev == NULL || accel == NULL || !dev->data.accel_valid)
        return false;

    *accel = dev->data.accel;
    return true;
}

bool BNO085_GetGyro(BNO085 *dev, BNO_Vector3 *gyro)
{
    if (dev == NULL || gyro == NULL || !dev->data.gyro_valid)
        return false;

    *gyro = dev->data.gyro;
    return true;
}

bool BNO085_GetMag(BNO085 *dev, BNO_Vector3 *mag)
{
    if (dev == NULL || mag == NULL || !dev->data.mag_valid)
        return false;

    *mag = dev->data.mag;
    return true;
}

bool BNO085_GetLinearAccel(BNO085 *dev, BNO_Vector3 *linear_accel)
{
    if (dev == NULL || linear_accel == NULL || !dev->data.linear_accel_valid)
        return false;

    *linear_accel = dev->data.linear_accel;
    return true;
}

bool BNO085_GetGravity(BNO085 *dev, BNO_Vector3 *gravity)
{
    if (dev == NULL || gravity == NULL || !dev->data.gravity_valid)
        return false;

    *gravity = dev->data.gravity;
    return true;
}

bool BNO085_GetRotation(BNO085 *dev, BNO_Quaternion *q)
{
    if (dev == NULL || q == NULL || !dev->data.rotation_valid)
        return false;

    *q = dev->data.rotation;
    return true;
}

bool BNO085_GetGameRotation(BNO085 *dev, BNO_Quaternion *q)
{
    if (dev == NULL || q == NULL || !dev->data.game_rotation_valid)
        return false;

    *q = dev->data.game_rotation;
    return true;
}

float BNO085_GetEulerYaw(const BNO_Quaternion *q)
{
    if (q == NULL)
        return 0.0f;

    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    float yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / 3.141592653589793f);
    return yaw;
}

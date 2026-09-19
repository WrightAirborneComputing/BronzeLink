# ============================================================
# AIRCRAFT PICO
#
# SX1262 @ 868 MHz
#
# Receives CRSF 0x16 frames over the SX1262 and:
#   - validates them
#   - decodes channels
#   - displays RSSI/SNR
#   - forwards them to the flight controller
#   - receives and displays CRSF traffic from the flight controller
#   - detects link loss
#
# FC UART:
#   GP4 = TX -> FC RX
#   GP5 = RX <- FC TX
#   420000 baud
#
# FC transmit enabled.
# ============================================================

from machine import SPI, Pin, UART
import time


# ============================================================
# SETTINGS
# ============================================================

FREQUENCY = 868000000

FC_BAUD = 420000

LINK_TIMEOUT_MS = 1000

TELEMETRY_QUEUE_MAX = 8
TX_TIMEOUT_MS = 200
TELEMETRY_TX_INTERVAL_MS = 500   # 2 Hz return telemetry


# ============================================================
# FC UART
# ============================================================

fc_uart = UART(
    1,
    baudrate=FC_BAUD,
    bits=8,
    parity=None,
    stop=1,
    tx=Pin(4),
    rx=Pin(5)
)


# ============================================================
# SX1262
# ============================================================

spi = SPI(
    1,
    baudrate=1000000,
    polarity=0,
    phase=0,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(10),
    mosi=Pin(11),
    miso=Pin(12)
)

CS = Pin(3, Pin.OUT, value=1)
BUSY = Pin(2, Pin.IN)
RESET = Pin(15, Pin.OUT, value=1)
DIO1 = Pin(20, Pin.IN)


# ============================================================
# LOW LEVEL
# ============================================================

def wait_busy():

    start = time.ticks_ms()

    while BUSY.value():

        if time.ticks_diff(
            time.ticks_ms(),
            start
        ) > 2000:

            return False

        time.sleep_ms(1)

    return True


def command(opcode, data=b""):

    if not wait_busy():
        return False

    CS.value(0)

    spi.write(
        bytes([opcode])
    )

    if data:
        spi.write(data)

    CS.value(1)

    wait_busy()

    return True


def read_command(opcode, length):

    if not wait_busy():
        return None

    CS.value(0)

    spi.write(
        bytes([opcode])
    )

    spi.read(
        1,
        0x00
    )

    data = spi.read(
        length,
        0x00
    )

    CS.value(1)

    wait_busy()

    return data


def reset_radio():

    print(
        "Resetting SX1262..."
    )

    RESET.value(0)

    time.sleep_ms(20)

    RESET.value(1)

    time.sleep_ms(20)

    wait_busy()


# ============================================================
# RADIO
# ============================================================

def clear_irq():

    command(
        0x02,
        bytes([
            0xFF,
            0xFF
        ])
    )


def set_frequency(freq):

    frf = int(
        (freq * (1 << 25))
        / 32000000
    )

    command(
        0x86,
        bytes([
            (frf >> 24) & 0xFF,
            (frf >> 16) & 0xFF,
            (frf >> 8) & 0xFF,
            frf & 0xFF
        ])
    )


def configure_radio():

    print(
        "Configuring radio..."
    )

    # Standby RC
    command(
        0x80,
        bytes([0x00])
    )

    # LoRa
    command(
        0x8A,
        bytes([0x01])
    )

    # TCXO 3.3 V
    command(
        0x97,
        bytes([
            0x07,
            0x00,
            0x00,
            0x64
        ])
    )

    time.sleep_ms(10)

    # 868 MHz
    set_frequency(
        FREQUENCY
    )

    # DIO2 RF switch
    command(
        0x9D,
        bytes([0x01])
    )

    # SF7 / BW500 / CR4/5
    command(
        0x8B,
        bytes([
            0x07,
            0x06,
            0x01,
            0x00
        ])
    )

    # Preamble 8 / explicit / CRC
    command(
        0x8C,
        bytes([
            0x00,
            0x08,
            0x00,
            0xFF,
            0x01,
            0x00
        ])
    )

    # Buffer base
    command(
        0x8F,
        bytes([
            0x00,
            0x00
        ])
    )

    # RxDone + CRC error
    command(
        0x08,
        bytes([
            0x00,
            0x42,
            0x00,
            0x02,
            0x00,
            0x00,
            0x00,
            0x00
        ])
    )

    clear_irq()

    print(
        "Radio configured"
    )

    print(
        "Frequency:",
        FREQUENCY
    )


def start_receive():

    clear_irq()

    # Enable RX IRQ sources again after any transmit.
    # IRQ mask: RxDone (0x0002) + CRC error (0x0040).
    # Route RxDone to DIO1, as in the original configuration.
    command(
        0x08,
        bytes([
            0x00, 0x42,
            0x00, 0x02,
            0x00, 0x00,
            0x00, 0x00
        ])
    )

    # Restore RX payload length to maximum.
    command(
        0x8C,
        bytes([
            0x00, 0x08,
            0x00,
            0xFF,
            0x01,
            0x00
        ])
    )

    command(
        0x82,
        bytes([
            0xFF,
            0xFF,
            0xFF
        ])
    )


def transmit_radio_packet(payload):

    if not payload or len(payload) > 255:
        return False

    # Stop continuous receive while loading/transmitting.
    command(
        0x80,
        bytes([0x00])
    )

    clear_irq()

    # Enable TX completion IRQs while transmitting.  The original
    # radio setup enabled only RxDone + CRC error (0x0042), which
    # meant GetIrqStatus could never report TxDone and every TX was
    # incorrectly reported as failed.
    # IRQ mask: TxDone (0x0001) + Timeout (0x0200).
    # No DIO routing is needed because we poll GetIrqStatus.
    command(
        0x08,
        bytes([
            0x02, 0x01,
            0x00, 0x00,
            0x00, 0x00,
            0x00, 0x00
        ])
    )

    # LoRa packet parameters. For TX, payload length must match
    # the actual number of bytes to be transmitted.
    command(
        0x8C,
        bytes([
            0x00, 0x08,
            0x00,
            len(payload),
            0x01,
            0x00
        ])
    )

    # WriteBuffer(offset=0, payload).
    if not wait_busy():
        start_receive()
        return False

    CS.value(0)
    spi.write(bytes([0x0E, 0x00]))
    spi.write(payload)
    CS.value(1)

    if not wait_busy():
        start_receive()
        return False

    # SetTx. 0x000000 = no SX1262 internal timeout; we impose a
    # short software timeout below so the Pico cannot get stuck.
    command(
        0x83,
        bytes([0x00, 0x00, 0x00])
    )

    start = time.ticks_ms()
    sent = False

    while time.ticks_diff(time.ticks_ms(), start) < TX_TIMEOUT_MS:

        irq_data = read_command(0x12, 2)

        if irq_data is not None:
            irq = (irq_data[0] << 8) | irq_data[1]

            if irq & 0x0001:   # TxDone
                sent = True
                break

            if irq & 0x0200:   # Timeout
                break

        time.sleep_ms(1)

    clear_irq()
    start_receive()

    return sent


# ============================================================
# RSSI / SNR
# ============================================================

def get_packet_status():

    data = read_command(
        0x14,
        3
    )

    if data is None:
        return None

    rssi = -(
        data[0] // 2
    )

    snr_raw = data[1]

    if snr_raw >= 128:
        snr_raw -= 256

    snr = snr_raw / 4

    signal_rssi = -(
        data[2] // 2
    )

    return (
        rssi,
        snr,
        signal_rssi
    )


# ============================================================
# RECEIVE RADIO PACKET
# ============================================================

def receive_packet():

    irq_data = read_command(
        0x12,
        2
    )

    if irq_data is None:

        return None, None


    irq = (
        (irq_data[0] << 8)
        |
        irq_data[1]
    )


    # CRC error
    if irq & 0x0040:

        clear_irq()
        start_receive()

        return None, None


    # RxDone?
    if not (irq & 0x0002):

        return None, None


    status = read_command(
        0x13,
        2
    )

    if status is None:

        clear_irq()
        start_receive()

        return None, None


    length = status[0]
    position = status[1]


    if length == 0:

        clear_irq()
        start_receive()

        return None, None


    # ReadBuffer
    wait_busy()

    CS.value(0)

    spi.write(
        bytes([
            0x1E,
            position
        ])
    )

    spi.read(
        1,
        0x00
    )

    data = spi.read(
        length,
        0x00
    )

    CS.value(1)

    wait_busy()


    # Packet RSSI/SNR
    radio_status = get_packet_status()


    clear_irq()
    start_receive()


    return bytes(data), radio_status


# ============================================================
# CRSF
# ============================================================

def crsf_crc(data):

    crc = 0

    for byte in data:

        crc ^= byte

        for _ in range(8):

            if crc & 0x80:

                crc = (
                    (crc << 1)
                    ^ 0xD5
                ) & 0xFF

            else:

                crc = (
                    crc << 1
                ) & 0xFF

    return crc


def valid_crsf(frame):

    if len(frame) != 26:
        return False

    if frame[0] != 0xC8:
        return False

    if frame[1] != 24:
        return False

    if frame[2] != 0x16:
        return False

    return (
        crsf_crc(frame[2:25])
        ==
        frame[25]
    )


# ============================================================
# CRSF FROM FLIGHT CONTROLLER
# ============================================================

# CRSF frame format:
#   address, length, type, payload..., crc
#
# length counts bytes from type through crc, so the total
# frame size is length + 2 bytes.

fc_rx_buffer = bytearray()
fc_tx_queue = []
fc_radio_tx_count = 0
fc_radio_tx_fail_count = 0
fc_rx_frame_count = 0
fc_rx_bad_crc_count = 0


def crsf_type_name(frame_type):

    names = {
        0x02: "GPS",
        0x07: "VARIO",
        0x08: "BATTERY",
        0x09: "BARO_ALT",
        0x0B: "HEARTBEAT",
        0x14: "LINK_STATISTICS",
        0x16: "RC_CHANNELS",
        0x1E: "ATTITUDE",
        0x21: "FLIGHT_MODE",
        0x28: "DEVICE_PING",
        0x29: "DEVICE_INFO",
        0x2B: "PARAMETER_ENTRY",
        0x2C: "PARAMETER_READ",
        0x2D: "PARAMETER_WRITE",
        0x32: "COMMAND",
        0x7A: "MSP_REQ",
        0x7B: "MSP_RESP",
        0x7C: "MSP_WRITE",
        0x80: "DISPLAYPORT"
    }

    return names.get(frame_type, "UNKNOWN")


def queue_fc_frame_for_radio(frame):

    global fc_tx_queue

    # Keep the newest telemetry if the FC is producing frames faster
    # than the return radio link can carry them.
    if len(fc_tx_queue) >= TELEMETRY_QUEUE_MAX:
        fc_tx_queue.pop(0)

    fc_tx_queue.append(frame)


def send_one_queued_fc_frame():

    global fc_tx_queue
    global fc_radio_tx_count
    global fc_radio_tx_fail_count

    if not fc_tx_queue:
        return

    frame = fc_tx_queue.pop(0)

    if transmit_radio_packet(frame):
        fc_radio_tx_count += 1
        #print("PICO->GROUND[", fc_radio_tx_count, "] TYPE=", crsf_type_name(frame[2]))
    else:
        fc_radio_tx_fail_count += 1
        #print("PICO->GROUND TX FAILED",fc_radio_tx_fail_count)


def print_fc_crsf_frame(frame):

    global fc_rx_frame_count

    fc_rx_frame_count += 1

    frame_type = frame[2]

    if(False):
        print(
            "FC->PICO",
            fc_rx_frame_count,
            "| ADDR 0x%02X" % frame[0],
            "| TYPE 0x%02X" % frame_type,
            crsf_type_name(frame_type),
            "| LEN",
            len(frame),
            "|",
            " ".join("%02X" % b for b in frame)
        )


def process_fc_uart():

    global fc_rx_buffer
    global fc_rx_bad_crc_count

    available = fc_uart.any()

    if available:

        data = fc_uart.read(available)

        if data:
            fc_rx_buffer.extend(data)

    while len(fc_rx_buffer) >= 2:

        # CRSF length is normally 2..62. Anything outside that
        # range cannot be a valid frame, so discard one byte and
        # resynchronise on the stream.
        length = fc_rx_buffer[1]

        if length < 2 or length > 62:
            fc_rx_buffer = bytearray(fc_rx_buffer[1:])
            continue

        total_length = length + 2

        if len(fc_rx_buffer) < total_length:
            return

        frame = bytes(fc_rx_buffer[:total_length])

        if crsf_crc(frame[2:-1]) == frame[-1]:

            print_fc_crsf_frame(frame)
            queue_fc_frame_for_radio(frame)
            fc_rx_buffer = bytearray(fc_rx_buffer[total_length:])

        else:

            fc_rx_bad_crc_count += 1

            # Discard a single byte so that a valid frame beginning
            # later in the buffer can still be found.
            fc_rx_buffer = bytearray(fc_rx_buffer[1:])


# ============================================================
# DECODE CHANNELS FOR DISPLAY
# ============================================================

def decode_channels(frame):

    channels = [0] * 16

    bit_position = 0

    for channel_number in range(16):

        value = 0

        for bit in range(11):

            byte_index = (
                3
                +
                (
                    bit_position // 8
                )
            )

            bit_index = (
                bit_position % 8
            )

            if frame[byte_index] & (
                1 << bit_index
            ):

                value |= (
                    1 << bit
                )

            bit_position += 1

        channels[channel_number] = value

    return channels


# ============================================================
# MAIN
# ============================================================

print()
print("==============================")
print("AIRCRAFT PICO")
print("CRSF 868 MHz BIDIRECTIONAL AIR RADIO")
print("==============================")
print()
print(
    "Frequency:",
    FREQUENCY
)
print(
    "FC UART: GP4 TX / GP5 RX"
)
print(
    "FC baud:",
    FC_BAUD
)
print()
print(
    "FC RX + TX and radio return link ENABLED"
)
print()


reset_radio()
configure_radio()
start_receive()

print(
    "Listening..."
)
print()


packet_count = 0
last_packet_time = time.ticks_ms()

failsafe = True

last_status = time.ticks_ms()
previous_packet_time = 0
last_telemetry_tx_time = time.ticks_ms()

while True:

    # Read and display any CRSF telemetry/traffic sent by the FC.
    process_fc_uart()

    frame, radio_status = (
        receive_packet()
    )


    if frame is not None:

        if valid_crsf(frame):

            packet_count += 1

            previous_packet_time = last_packet_time
            last_packet_time = (time.ticks_ms())
            packet_ms = last_packet_time - previous_packet_time


            if failsafe:

                failsafe = False

                print()
                print(
                    "LINK RESTORED"
                )
                print()


            channels = decode_channels(
                frame
            )


            if radio_status is not None:

                rssi = radio_status[0]
                snr = radio_status[1]

            else:

                rssi = 0
                snr = 0


            if(False):
                print(
                    "RX",
                    packet_count,
                    "| CH1",
                    channels[0],
                    "| CH2",
                    channels[1],
                    "| CH3",
                    channels[2],
                    "| CH4",
                    channels[3],
                    "| CH5",
                    channels[4],
                    "| CH6",
                    channels[5],
                    "| CH7",
                    channels[6],
                    "| CH8",
                    channels[7],
                    "| CH9",
                    channels[8],
                    "| CH10",
                    channels[9],
                    "| CH11",
                    channels[10],
                    "| CH12",
                    channels[11],
                    "| CH13",
                    channels[12],
                    "| CH14",
                    channels[13],
                    "| CH15",
                    channels[14],
                    "| CH16",
                    channels[15],
                    "| RSSI",
                    rssi,
                    "dBm",
                    "| SNR",
                    snr,
                    "dB"
                )
            else:
                print("RX",packet_count,packet_ms)

            # ------------------------------------------------
            # Forward valid CRSF channel frame to flight controller.
            # The frame is already a valid CRSF frame.
            # ------------------------------------------------
            fc_uart.write(frame)

            # ------------------------------------------------
            # RETURN TELEMETRY
            #
            # Limit air->ground telemetry radio transmissions to 2 Hz
            # (one every 500 ms). RC reception/forwarding remains the
            # priority. Telemetry is still sent immediately after a valid
            # uplink packet, giving the ground radio a natural RX slot.
            # ------------------------------------------------
            now = time.ticks_ms()

            if time.ticks_diff(
                now,
                last_telemetry_tx_time
            ) >= TELEMETRY_TX_INTERVAL_MS:

                if fc_tx_queue:
                    send_one_queued_fc_frame()
                    last_telemetry_tx_time = now


    # --------------------------------------------------------
    # LINK FAILSAFE
    # --------------------------------------------------------

    age = time.ticks_diff(
        time.ticks_ms(),
        last_packet_time
    )


    if age > LINK_TIMEOUT_MS:

        if not failsafe:

            failsafe = True

            print()
            print(
                "=============================="
            )
            print(
                "FAILSAFE - LINK LOST"
            )
            print(
                "=============================="
            )
            print()


    # Status every 5 seconds
    now = time.ticks_ms()

    if time.ticks_diff(
        now,
        last_status
    ) >= 5000:

        print(
            "Radio RX packets:",
            packet_count,
            "| FC RX frames:",
            fc_rx_frame_count,
            "| FC bad CRC:",
            fc_rx_bad_crc_count,
            "| Radio TX:",
            fc_radio_tx_count,
            "| TX fail:",
            fc_radio_tx_fail_count,
            "| TX queued:",
            len(fc_tx_queue),
            "| Failsafe:",
            failsafe
        )

        last_status = now


    time.sleep_ms(2)



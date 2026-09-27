from machine import SPI, Pin, UART
import time

# ============================================================
# GROUND PICO
#
# Pico 2W -> UART -> Ground Pico -> SX1262 -> 868 MHz
#
# UART:
#   GP4 = TX
#   GP5 = RX
#
# Pico 2W wiring:
#   Pico 2W GP4 TX -> Ground Pico GP5 RX
#   Pico 2W GP5 RX <- Ground Pico GP4 TX
#   GND            -> GND
#
# SX1262:
#   GP10 = SCK
#   GP11 = MOSI
#   GP12 = MISO
#   GP3  = CS
#   GP2  = BUSY
#   GP15 = RESET
#   GP20 = DIO1
#
# RADIO FREQUENCY:
#   868 MHz
# ============================================================


# ============================================================
# SETTINGS
# ============================================================

FREQUENCY = 868000000

UART_BAUD = 115200

# Maximum radio transmission rate.
# This avoids building up old control frames.
RADIO_INTERVAL_MS = 100      # 10 Hz ground-master RC slots
TELEMETRY_REPLY_WINDOW_MS = 60  # listen for aircraft reply after RC TX
TX_TIMEOUT_MS = 200


# ============================================================
# UART FROM PICO 2W
# ============================================================

uart = UART(
    1,
    baudrate=UART_BAUD,
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

# Onboard telemetry/link LED.
LED = Pin("LED", Pin.OUT)
LED.value(0)


# ============================================================
# SX1262 LOW LEVEL
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

    print("Resetting SX1262...")

    RESET.value(0)
    time.sleep_ms(20)

    RESET.value(1)
    time.sleep_ms(20)

    if not wait_busy():

        print("BUSY TIMEOUT")


# ============================================================
# RADIO HELPERS
# ============================================================

def clear_irq():

    command(
        0x02,
        bytes([
            0xFF,
            0xFF
        ])
    )



def set_tx_irq():
    # TxDone + Timeout, routed to DIO1
    command(
        0x08,
        bytes([
            0x02, 0x01,   # IRQ mask: Timeout | TxDone
            0x00, 0x01,   # DIO1: TxDone
            0x00, 0x00,   # DIO2
            0x00, 0x00    # DIO3
        ])
    )


def set_rx_irq():
    # RxDone + CRC error, RxDone routed to DIO1
    command(
        0x08,
        bytes([
            0x00, 0x42,   # IRQ mask: RxDone | CRC error
            0x00, 0x02,   # DIO1: RxDone
            0x00, 0x00,   # DIO2
            0x00, 0x00    # DIO3
        ])
    )


def start_receive():
    # Restore maximum payload length for variable-length incoming packets.
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

    set_rx_irq()
    clear_irq()

    # Continuous RX
    command(
        0x82,
        bytes([
            0xFF,
            0xFF,
            0xFF
        ])
    )


def get_packet_status():
    data = read_command(0x14, 3)

    if data is None:
        return None

    rssi = -(data[0] // 2)

    snr_raw = data[1]
    if snr_raw >= 128:
        snr_raw -= 256
    snr = snr_raw / 4

    signal_rssi = -(data[2] // 2)

    return rssi, snr, signal_rssi


def receive_radio_packet():
    irq_data = read_command(0x12, 2)

    if irq_data is None:
        return None, None

    irq = (irq_data[0] << 8) | irq_data[1]

    # CRC error
    if irq & 0x0040:
        clear_irq()
        start_receive()
        return None, None

    # No complete packet yet
    if not (irq & 0x0002):
        return None, None

    status = read_command(0x13, 2)

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

    if not wait_busy():
        clear_irq()
        start_receive()
        return None, None

    CS.value(0)
    spi.write(bytes([0x1E, position]))
    spi.read(1, 0x00)
    data = spi.read(length, 0x00)
    CS.value(1)

    wait_busy()

    radio_status = get_packet_status()

    clear_irq()
    start_receive()

    return bytes(data), radio_status


def valid_crsf_any(frame):
    # General CRSF frame:
    # address, length, type, payload..., CRC
    if frame is None or len(frame) < 4:
        return False

    length = frame[1]
    if length < 2 or length > 62:
        return False

    if len(frame) != length + 2:
        return False

    return crsf_crc(frame[2:-1]) == frame[-1]


def crsf_type_name(frame_type):
    names = {
        0x02: "GPS",
        0x03: "VARIO",
        0x07: "VARIO",
        0x08: "BATTERY",
        0x09: "BARO_ALT",
        0x0B: "HEARTBEAT",
        0x14: "LINK_STATISTICS",
        0x16: "RC_CHANNELS",
        0x1E: "ATTITUDE",
        0x21: "FLIGHT_MODE",
    }
    return names.get(frame_type, "TYPE_%02X" % frame_type)


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


# ============================================================
# RADIO CONFIGURATION
# ============================================================

def configure_radio():

    print("Configuring SX1262...")

    # --------------------------------------------------------
    # Standby
    # --------------------------------------------------------

    command(
        0x80,
        bytes([0x00])
    )

    # --------------------------------------------------------
    # LoRa
    # --------------------------------------------------------

    command(
        0x8A,
        bytes([0x01])
    )

    # --------------------------------------------------------
    # TCXO = 3.3 V
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # 868 MHz
    # --------------------------------------------------------

    set_frequency(FREQUENCY)

    # --------------------------------------------------------
    # DIO2 = RF switch
    # --------------------------------------------------------

    command(
        0x9D,
        bytes([0x01])
    )

    # --------------------------------------------------------
    # SF7 / BW500 / CR4/5
    # --------------------------------------------------------

    command(
        0x8B,
        bytes([
            0x07,
            0x06,
            0x01,
            0x00
        ])
    )

    # --------------------------------------------------------
    # Preamble 8 / Explicit header / CRC ON
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # Buffer base
    # --------------------------------------------------------

    command(
        0x8F,
        bytes([
            0x00,
            0x00
        ])
    )

    # --------------------------------------------------------
    # PA configuration
    # --------------------------------------------------------

    command(
        0x95,
        bytes([
            0x04,
            0x07,
            0x00,
            0x01
        ])
    )

    # --------------------------------------------------------
    # TX power = 14 dBm
    # --------------------------------------------------------

    command(
        0x8E,
        bytes([
            14,
            0x04
        ])
    )

    # --------------------------------------------------------
    # IRQ configuration is switched dynamically between TX and RX.
    # Start in receive configuration.
    # --------------------------------------------------------

    set_rx_irq()
    clear_irq()

    print("Radio configured")
    print("Frequency:", FREQUENCY)
    print()


# ============================================================
# CRSF CRC
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


# ============================================================
# CHECK CRSF FRAME
#
# Expected 26-byte RC frame:
#
# C8 18 16 [22 bytes] CRC
# ============================================================

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
# UART BUFFER
#
# We deliberately keep only the newest valid CRSF frame.
# ============================================================

rx_buffer = bytearray()

latest_frame = None


def collect_uart_frames():

    global rx_buffer
    global latest_frame

    # --------------------------------------------------------
    # Read everything currently available
    # --------------------------------------------------------

    while uart.any():

        data = uart.read()

        if data:

            rx_buffer.extend(data)

    # --------------------------------------------------------
    # Extract as many complete frames as possible.
    # Keep ONLY the newest one.
    # --------------------------------------------------------

    while len(rx_buffer) >= 26:

        # Find CRSF start byte
        if rx_buffer[0] != 0xC8:

            rx_buffer = rx_buffer[1:]

            continue

        # Length must be 24
        if rx_buffer[1] != 24:

            rx_buffer = rx_buffer[1:]

            continue

        # Need full 26-byte frame
        frame = bytes(
            rx_buffer[:26]
        )

        rx_buffer = rx_buffer[26:]

        # Check frame
        if valid_crsf(frame):

            latest_frame = frame


# ============================================================
# RADIO TRANSMIT
# ============================================================

def send_radio_packet(data):

    # Payload must fit
    if len(data) > 255:

        return False

    # Stop any receive operation and configure TX completion IRQ.
    command(
        0x80,
        bytes([0x00])
    )

    set_tx_irq()
    clear_irq()

    # --------------------------------------------------------
    # Packet parameters with actual length
    # --------------------------------------------------------

    command(
        0x8C,
        bytes([
            0x00,
            0x08,
            0x00,
            len(data),
            0x01,
            0x00
        ])
    )

    clear_irq()

    # --------------------------------------------------------
    # WriteBuffer
    # --------------------------------------------------------

    if not wait_busy():

        return False

    CS.value(0)

    spi.write(
        bytes([
            0x0E,
            0x00
        ])
    )

    spi.write(data)

    CS.value(1)

    wait_busy()

    # --------------------------------------------------------
    # Start TX
    # --------------------------------------------------------

    command(
        0x83,
        bytes([
            0xFF,
            0xFF,
            0xFF
        ])
    )

    start = time.ticks_ms()

    while True:

        irq_data = read_command(
            0x12,
            2
        )

        if irq_data is None:

            start_receive()
            return False

        irq = (
            (irq_data[0] << 8)
            |
            irq_data[1]
        )

        # TX Done
        if irq & 0x0001:

            clear_irq()
            start_receive()

            return True

        # TX timeout
        if irq & 0x0200:

            clear_irq()
            start_receive()

            return False

        # Safety timeout
        if time.ticks_diff(
            time.ticks_ms(),
            start
        ) > TX_TIMEOUT_MS:

            clear_irq()
            start_receive()

            return False

        time.sleep_ms(1)


# ============================================================
# CRSF LINK STATISTICS FOR HANDSET RSSI DISPLAY
# ============================================================

def make_link_statistics_frame(rssi_dbm, snr_db):
    """
    Create a standard CRSF LINK_STATISTICS (0x14) frame.

    The ground SX1262 measures the aircraft->ground packet RSSI.
    CRSF encodes RSSI as a positive magnitude:
        -67 dBm -> 67
    """
    rssi_mag = max(0, min(255, -int(round(rssi_dbm))))

    # Signed SNR byte, in dB, two's-complement.
    snr_int = max(-128, min(127, int(round(snr_db))))
    snr_byte = snr_int & 0xFF

    payload = bytes([
        rssi_mag,   # uplink RSSI antenna 1
        rssi_mag,   # uplink RSSI antenna 2
        100,        # uplink link quality
        snr_byte,   # uplink SNR
        0,          # active antenna
        0,          # RF mode
        0,          # uplink TX power
        rssi_mag,   # downlink RSSI
        100,        # downlink link quality
        snr_byte    # downlink SNR
    ])

    body = bytes([0x14]) + payload
    crc = crsf_crc(body)

    return bytes([
        0xC8,
        len(body) + 1
    ]) + body + bytes([crc])


# ============================================================
# SYNCHRONISED TELEMETRY REPLY WINDOW
# ============================================================

def receive_telemetry_reply(timeout_ms):
    # send_radio_packet() has already returned the radio to continuous RX.
    # Wait only inside the aircraft's defined reply slot.  This prevents an
    # asynchronous aircraft transmission from colliding with a later RC slot.
    start = time.ticks_ms()

    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:

        telemetry, telemetry_status = receive_radio_packet()

        if telemetry is not None:
            return telemetry, telemetry_status

        time.sleep_ms(1)

    return None, None


# ============================================================
# MAIN
# ============================================================

print()
print("==============================")
print("GROUND PICO")
print("CRSF 868 MHz SYNCHRONISED GROUND MASTER")
print("==============================")
print()
print("UART:")
print("GP4 TX")
print("GP5 RX")
print("115200 baud")
print()
print("Radio:")
print("SX1262 @ 868 MHz")
print()


reset_radio()

configure_radio()


print("Ground-master 10 Hz RC; delayed aircraft reply enabled...")
print("RC slot:", RADIO_INTERVAL_MS, "ms")
print("Telemetry reply window:", TELEMETRY_REPLY_WINDOW_MS, "ms")
print()


latest_frame = None

tx_count = 0
valid_count = 0
telemetry_rx_count = 0
telemetry_bad_count = 0
last_telemetry_rx_time = time.ticks_ms()

next_tx_time = time.ticks_add(time.ticks_ms(), RADIO_INTERVAL_MS)

last_report = time.ticks_ms()


while True:

    # --------------------------------------------------------
    # Continuously collect incoming CRSF frames.
    # --------------------------------------------------------

    collect_uart_frames()


    # --------------------------------------------------------
    # Transmit the newest frame at controlled rate.
    # --------------------------------------------------------

    now = time.ticks_ms()

    if latest_frame is not None:

        # Absolute slot scheduler: the reply window does NOT get added
        # to the RC period.  A new RC slot is due every 100 ms.
        if time.ticks_diff(
            now,
            next_tx_time
        ) >= 0:

            frame_to_send = latest_frame

            # Clear it so we don't retransmit an old frame
            latest_frame = None

            if send_radio_packet(
                frame_to_send
            ):

                tx_count += 1
                valid_count += 1

                # The aircraft is permitted to transmit only now, directly
                # after receiving this RC packet.  Listen for that one reply
                # before returning to the next ground-master RC slot.
                telemetry, telemetry_status = receive_telemetry_reply(
                    TELEMETRY_REPLY_WINDOW_MS
                )

                if telemetry is not None:

                    if valid_crsf_any(telemetry):

                        telemetry_rx_count += 1
                        last_telemetry_rx_time = time.ticks_ms()
                        LED.value(1)

                        # Pass the original CRSF frame unchanged to Pico 2W.
                        uart.write(telemetry)

                        frame_type = telemetry[2]

                        if telemetry_status is not None:
                            trssi, tsnr, _ = telemetry_status

                            # Follow it with a standard CRSF LINK_STATISTICS
                            # frame containing the RSSI measured here at ground.
                            uart.write(
                                make_link_statistics_frame(
                                    trssi,
                                    tsnr
                                )
                            )
                            print(
                                "AIR->GROUND",
                                telemetry_rx_count,
                                "| TYPE 0x%02X" % frame_type,
                                crsf_type_name(frame_type),
                                "| LEN",
                                len(telemetry),
                                "| RSSI",
                                trssi,
                                "dBm | SNR",
                                tsnr,
                                "dB"
                            )
                        else:
                            print(
                                "AIR->GROUND",
                                telemetry_rx_count,
                                "| TYPE 0x%02X" % frame_type,
                                crsf_type_name(frame_type),
                                "| LEN",
                                len(telemetry)
                            )

                    else:
                        telemetry_bad_count += 1

            else:

                print(
                    "RADIO TX FAILED"
                )

            # Advance from the scheduled slot, not from the end of the
            # telemetry receive window.  This prevents cadence drift.
            next_tx_time = time.ticks_add(
                next_tx_time,
                RADIO_INTERVAL_MS
            )

            # If execution was delayed by more than a complete slot,
            # skip stale slots rather than sending a burst of old RC frames.
            now_after_slot = time.ticks_ms()
            while time.ticks_diff(now_after_slot, next_tx_time) >= 0:
                next_tx_time = time.ticks_add(
                    next_tx_time,
                    RADIO_INTERVAL_MS
                )


    # Link LED follows valid returned telemetry.
    if time.ticks_diff(
        time.ticks_ms(),
        last_telemetry_rx_time
    ) > 1000:
        LED.value(0)

    # --------------------------------------------------------
    # Status once per second
    # --------------------------------------------------------

    now = time.ticks_ms()

    if time.ticks_diff(
        now,
        last_report
    ) >= 1000:

        print(
            "TX:",
            tx_count,
            "| TELEM RX:",
            telemetry_rx_count,
            "| TELEM bad:",
            telemetry_bad_count,
            "| UART buffer:",
            len(rx_buffer),
            "| slot:",
            RADIO_INTERVAL_MS,
            "ms"
        )

        last_report = now


    time.sleep_ms(1)


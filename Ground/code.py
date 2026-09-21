# ============================================================
# RASPBERRY PI PICO 2 - CircuitPython
#
# Turtle Beach Rematch Core -> CRSF RC Channels Packed
# plus returned CRSF telemetry from the Ground Radio Pico,
# with ST7789 LCD telemetry/instrumentation.
#
# USB HOST:
#   GP2 = USB D+
#   GP3 = USB D-
#   A small USB 2.0 hub is used between Pico 2 and handset.
#
# UART TO GROUND RADIO PICO:
#   GP5 TX -> Ground Radio Pico GP5 RX
#   GP4 RX <- Ground Radio Pico GP4 TX
#   GND    -> GND
#   115200 baud
#
# CRSF CHANNEL MAP:
#   CH1  Roll      = Right stick X
#   CH2  Pitch     = Right stick Y
#   CH3  Throttle  = Left stick Y
#   CH4  Yaw       = Left stick X
#   CH5  Menu      = top-right centre button
#   CH6  View      = top-left centre button
#   CH7  LB        = left shoulder
#   CH8  RB        = right shoulder
#   CH9  LT        = left trigger, analogue
#   CH10 RT        = right trigger, analogue
#   CH11 Y
#   CH12 B
#   CH13 A
#   CH14 X
#   CH15 D-pad Up
#   CH16 D-pad Down
#
# The Xbox/Home button and L3/R3 stick-clicks are decoded and
# displayed as events, but are not assigned to CRSF channels.
# ============================================================

import time
import array
import board
import busio
import digitalio
import supervisor
import usb_host
import usb.core
import displayio
import terminalio
import fourwire
from adafruit_display_text import label
import adafruit_st7789


# ============================================================
# TURTLE BEACH REMATCH CORE / XBOX GIP
# ============================================================

VID = 0x10F5
PID = 0x7122

EP_IN = 0x81
EP_OUT = 0x02
PACKET_SIZE = 64

POWER_ON = bytearray([0x05, 0x20, 0x01, 0x01, 0x00])
LED_ON = bytearray([0x0A, 0x20, 0x02, 0x03, 0x00, 0x01, 0x14])
AUTH_DONE = bytearray([0x06, 0x20, 0x03, 0x02, 0x01, 0x00])


# ============================================================
# CRSF
# ============================================================

CRSF_ADDRESS = 0xC8
CRSF_TYPE = 0x16

CRSF_MIN = 172
CRSF_MID = 992
CRSF_MAX = 1811

# Send the latest channel state to the radio Pico at a fixed 25 Hz.
CRSF_TX_INTERVAL = 0.1

# Small centre deadband for the Hall sticks.
# Raw stick range is approximately -32768..+32767.
STICK_DEADZONE = 1000


# ============================================================
# LED
# ============================================================

led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
led.value = False
last_led_time = time.monotonic()


# ============================================================
# UART TO GROUND RADIO PICO
# ============================================================

uart = busio.UART(
    tx=board.GP4,
    rx=board.GP5,
    baudrate=115200,
    timeout=0
)




# ============================================================
# 1.14" LCD HAT - ST7789 240 x 135
#
# SB Components pinout:
#   GP10 = CLK
#   GP11 = DIN / MOSI
#   GP8  = DC
#   GP9  = CS
#   GP12 = RESET
#   GP13 = BACKLIGHT
# ============================================================

displayio.release_displays()

lcd_spi = busio.SPI(
    clock=board.GP10,
    MOSI=board.GP11
)

lcd_backlight = digitalio.DigitalInOut(board.GP13)
lcd_backlight.direction = digitalio.Direction.OUTPUT
lcd_backlight.value = True

# LCD HAT SELECT button: GP19, active low.
# Press after the handset has been plugged in/buzzed to perform the
# CircuitPython soft reload that we know makes USB enumeration reliable.
reload_button = digitalio.DigitalInOut(board.GP19)
reload_button.direction = digitalio.Direction.INPUT
reload_button.pull = digitalio.Pull.UP

# Avoid a reload loop if SELECT is still held when code.py restarts.
while not reload_button.value:
    time.sleep(0.02)

reload_button_armed = True

lcd_bus = fourwire.FourWire(
    lcd_spi,
    command=board.GP8,
    chip_select=board.GP9,
    reset=board.GP12
)

display = adafruit_st7789.ST7789(
    lcd_bus,
    width=240,
    height=135,
    rowstart=40,
    colstart=53,
    rotation=270
)

lcd_group = displayio.Group()
display.root_group = lcd_group

def make_label(text, x, y, scale=1):
    item = label.Label(
        terminalio.FONT,
        text=text,
        scale=scale,
        color=0xFFFFFF
    )
    item.x = x
    item.y = y
    lcd_group.append(item)
    return item

lcd_title   = make_label("MANTA GROUND", 4, 8, 1)
lcd_handset = make_label("HANDSET: NOT CONNECTED", 4, 24, 1)
lcd_alt     = make_label("ALT: ---.- m", 4, 40, 1)
lcd_bat     = make_label("BAT:--.-V --.-A ---%", 4, 56, 1)
lcd_air     = make_label("AIR: --.- m/s", 4, 72, 1)
lcd_hb      = make_label("HEARTBEATS: 0", 4, 88, 1)
lcd_crsf    = make_label("TX:0 RX:0 BAD:0", 4, 104, 1)
lcd_link    = make_label("SIGNAL LOST", 4, 122, 1)

last_lcd_update = 0.0
LCD_UPDATE_INTERVAL = 0.20
TELEMETRY_SIGNAL_TIMEOUT = 1.0

telemetry_altitude_m = None
telemetry_voltage_v = None
telemetry_current_a = None
telemetry_remaining_pct = None
telemetry_airspeed_ms = None
telemetry_heartbeat_count = 0
last_valid_telemetry_time = None
last_telemetry_type = None


# ============================================================
# USB HOST
# ============================================================

usb_port = usb_host.Port(board.GP2, board.GP3)

device = None
handset_usb_found = False
handset_connected = False

rx_buffer = array.array(
    "B",
    [0] * PACKET_SIZE
)


# ============================================================
# BUTTON EVENT DISPLAY
# ============================================================

previous_buttons = {}


def show_button(name, pressed):
    old = previous_buttons.get(name)

    if old is None:
        previous_buttons[name] = pressed
        return

    if pressed != old:
        print(name, "PRESSED" if pressed else "RELEASED")
        previous_buttons[name] = pressed


# ============================================================
# LED SERVICE
# ============================================================

def service_led():
    global last_led_time

    now = time.monotonic()

    if now - last_led_time >= 0.5:
        led.value = not led.value
        last_led_time = now


# ============================================================
# LCD HAT SELECT -> CIRCUITPYTHON SOFT RELOAD
# ============================================================

def service_reload_button():
    global reload_button_armed

    if reload_button.value:
        reload_button_armed = True
        return

    if not reload_button_armed:
        return

    time.sleep(0.05)  # debounce

    if not reload_button.value:
        reload_button_armed = False
        print("LCD SELECT pressed - soft reload")
        lcd_handset.text = "HANDSET: RELOADING..."
        time.sleep(0.10)
        supervisor.reload()


# ============================================================
# XBOX GIP INITIALISATION
# ============================================================

def gip_send(dev, packet):
    try:
        dev.write(
            EP_OUT,
            packet,
            timeout=100
        )
        return True

    except Exception as e:
        print("GIP TX error:", repr(e))
        return False


def initialise_gip(dev):
    print("Initialising Xbox GIP...")

    gip_send(dev, POWER_ON)
    time.sleep(0.05)

    gip_send(dev, LED_ON)
    time.sleep(0.05)

    gip_send(dev, AUTH_DONE)
    time.sleep(0.30)

    print("GIP ready")


# ============================================================
# FIND CONTROLLER
# ============================================================

def find_controller():
    global device
    global handset_usb_found
    global handset_connected

    dev = usb.core.find(
        idVendor=VID,
        idProduct=PID
    )

    if dev is None:
        handset_usb_found = False
        handset_connected = False
        return False

    print()
    print(
        "Turtle Beach controller found VID=%04X PID=%04X"
        % (dev.idVendor, dev.idProduct)
    )

    try:
        print("Product:", dev.product)
    except Exception:
        pass

    try:
        dev.set_configuration()
        print("Configuration set")

    except Exception as e:
        print("set_configuration:", repr(e))

    initialise_gip(dev)

    device = dev
    handset_usb_found = True
    handset_connected = False

    print("USB active - waiting for valid GIP input")
    print()

    return True


# ============================================================
# BASIC HELPERS
# ============================================================

def clamp(value, minimum, maximum):
    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def s16le(data, offset):
    value = data[offset] | (data[offset + 1] << 8)

    if value & 0x8000:
        value -= 65536

    return value


def u16le(data, offset):
    return data[offset] | (data[offset + 1] << 8)


# ============================================================
# TURTLE STICK -> CRSF
#
# Input: -32768 .. +32767
# Output: 172 .. 1811, centre 992
#
# The asymmetric CRSF centre is handled explicitly so that raw
# zero maps exactly to CRSF_MID.
# ============================================================

def stick_to_crsf(value, invert=False):
    if abs(value) <= STICK_DEADZONE:
        value = 0

    value = clamp(value, -32768, 32767)

    if invert:
        value = -value
        value = clamp(value, -32768, 32767)

    if value == 0:
        return CRSF_MID

    if value < 0:
        return CRSF_MID + (
            value * (CRSF_MID - CRSF_MIN) // 32768
        )

    return CRSF_MID + (
        value * (CRSF_MAX - CRSF_MID) // 32767
    )


# ============================================================
# TURTLE TRIGGER -> CRSF
#
# Trigger input is 0..1023.
# ============================================================

def trigger_to_crsf(value):
    value = clamp(value, 0, 1023)

    return CRSF_MIN + (
        value * (CRSF_MAX - CRSF_MIN) // 1023
    )


# ============================================================
# CRSF CRC-8/DVB-S2
# Polynomial 0xD5
# ============================================================

def crsf_crc(data):
    crc = 0

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


# ============================================================
# RETURN CRSF TELEMETRY FROM GROUND RADIO PICO
# ============================================================

telemetry_buffer = bytearray()
telemetry_frame_count = 0
telemetry_bad_crc_count = 0


def crsf_type_name(frame_type):
    names = {
        0x02: "GPS",
        0x07: "VARIO",
        0x08: "BATTERY",
        0x09: "BARO_ALT",
        0x0A: "AIRSPEED",
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


def be_u16(data, offset):
    return (data[offset] << 8) | data[offset + 1]


def be_u24(data, offset):
    return (data[offset] << 16) | (data[offset + 1] << 8) | data[offset + 2]


def decode_telemetry_frame(frame):
    global telemetry_altitude_m
    global telemetry_voltage_v
    global telemetry_current_a
    global telemetry_remaining_pct
    global telemetry_airspeed_ms
    global telemetry_heartbeat_count
    global last_valid_telemetry_time
    global last_telemetry_type

    if len(frame) < 4:
        return

    frame_type = frame[2]
    last_telemetry_type = frame_type
    last_valid_telemetry_time = time.monotonic()

    # CRSF GPS (0x02): payload is:
    # lat(4), lon(4), groundspeed(2), heading(2), altitude(2), sats(1).
    # Payload starts at complete-frame byte 3, so altitude is bytes 15..16.
    if frame_type == 0x02 and len(frame) >= 18:
        telemetry_altitude_m = float(be_u16(frame, 15) - 1000)

    # CRSF Battery (0x08): voltage 0.1 V, current 0.1 A, capacity u24 mAh.
    elif frame_type == 0x08 and len(frame) >= 12:
        telemetry_voltage_v = be_u16(frame, 3) / 10.0
        telemetry_current_a = be_u16(frame, 5) / 10.0
        telemetry_remaining_pct = frame[10]

    # CRSF Barometric altitude (0x09). Prefer this over GPS altitude
    # whenever it arrives. The packed form follows the CRSF convention.
    elif frame_type == 0x09 and len(frame) >= 6:
        raw = be_u16(frame, 3)
        if raw & 0x8000:
            telemetry_altitude_m = float((raw & 0x7FFF) - 10000)
        else:
            telemetry_altitude_m = raw / 10.0 - 1000.0

    # CRSF Airspeed (0x0A): unsigned 16-bit big-endian, 0.1 m/s.
    elif frame_type == 0x0A and len(frame) >= 6:
        telemetry_airspeed_ms = be_u16(frame, 3) / 10.0

    elif frame_type == 0x0B:
        telemetry_heartbeat_count += 1


def update_lcd(force=False):
    global last_lcd_update

    now = time.monotonic()
    if not force and (now - last_lcd_update) < LCD_UPDATE_INTERVAL:
        return

    last_lcd_update = now

    link_ok = (
        last_valid_telemetry_time is not None
        and (now - last_valid_telemetry_time) <= TELEMETRY_SIGNAL_TIMEOUT
    )

    # Clear live values on link loss, but retain accumulated heartbeat/count data.
    if link_ok:
        alt = telemetry_altitude_m
        volt = telemetry_voltage_v
        curr = telemetry_current_a
        remain = telemetry_remaining_pct
        air = telemetry_airspeed_ms
    else:
        alt = volt = curr = remain = air = None

    if handset_connected:
        lcd_handset.text = "HANDSET: CONNECTED"
    elif handset_usb_found:
        lcd_handset.text = "HANDSET: USB FOUND"
    else:
        lcd_handset.text = "HANDSET: NOT CONNECTED"

    lcd_alt.text = "ALT: %6.1f m" % alt if alt is not None else "ALT: ---.- m"

    if volt is not None or curr is not None:
        vtxt = "%4.1f" % volt if volt is not None else "--.-"
        ctxt = "%4.1f" % curr if curr is not None else "--.-"
        rtxt = "%3d%%" % remain if remain is not None else "---%"
        lcd_bat.text = "BAT:%sV %sA %s" % (vtxt, ctxt, rtxt)
    else:
        lcd_bat.text = "BAT:--.-V --.-A ---%"

    lcd_air.text = "AIR: %4.1f m/s" % air if air is not None else "AIR: --.- m/s"
    lcd_hb.text = "HEARTBEATS: %d" % telemetry_heartbeat_count
    lcd_crsf.text = "TX:%d RX:%d BAD:%d" % (packet_count, telemetry_frame_count, telemetry_bad_crc_count)
    lcd_link.text = "LINK OK" if link_ok else "SIGNAL LOST"


def print_telemetry_frame(frame):
    global telemetry_frame_count

    telemetry_frame_count += 1
    frame_type = frame[2]
    decode_telemetry_frame(frame)

    print(
        "GROUND->HANDSET",
        telemetry_frame_count,
        "| ADDR 0x%02X" % frame[0],
        "| TYPE 0x%02X" % frame_type,
        crsf_type_name(frame_type),
        "| LEN",
        len(frame),
        "|",
        " ".join("%02X" % b for b in frame)
    )


def process_telemetry_uart():
    global telemetry_buffer
    global telemetry_bad_crc_count

    waiting = uart.in_waiting

    if waiting:
        data = uart.read(waiting)

        if data:
            telemetry_buffer.extend(data)

    while len(telemetry_buffer) >= 2:
        length = telemetry_buffer[1]

        if length < 2 or length > 62:
            telemetry_buffer = telemetry_buffer[1:]
            continue

        total_length = length + 2

        if len(telemetry_buffer) < total_length:
            return

        frame = bytes(
            telemetry_buffer[:total_length]
        )

        if crsf_crc(frame[2:-1]) == frame[-1]:
            print_telemetry_frame(frame)
            telemetry_buffer = telemetry_buffer[total_length:]

        else:
            telemetry_bad_crc_count += 1

            print(
                "GROUND->HANDSET BAD CRC",
                telemetry_bad_crc_count
            )

            telemetry_buffer = telemetry_buffer[1:]


# ============================================================
# CREATE CRSF 0x16 FRAME
# 16 channels x 11 bits = 22 bytes
# C8 18 16 [22 bytes] CRC
# ============================================================

def make_crsf_frame(channels):
    if len(channels) != 16:
        raise ValueError("Need exactly 16 channels")

    payload = bytearray(22)
    bit_position = 0

    for channel in channels:
        value = channel & 0x07FF

        for bit in range(11):
            if value & (1 << bit):
                byte_index = bit_position // 8
                bit_index = bit_position % 8
                payload[byte_index] |= 1 << bit_index

            bit_position += 1

    body = bytearray(23)
    body[0] = CRSF_TYPE

    for i in range(22):
        body[i + 1] = payload[i]

    crc = crsf_crc(body)

    frame = bytearray(26)
    frame[0] = CRSF_ADDRESS
    frame[1] = 24

    for i in range(23):
        frame[i + 2] = body[i]

    frame[25] = crc

    return bytes(frame)


# ============================================================
# TURTLE BEACH INPUT REPORT -> 16 CRSF CHANNELS
# ============================================================

def decode_report(data):
    # Xbox GIP input packet is currently 36 bytes.
    if len(data) < 19:
        return None

    if data[0] != 0x20:
        return None

    # --------------------------------------------------------
    # Button fields
    # --------------------------------------------------------

    b4 = data[4]
    b5 = data[5]

    # Main face / centre buttons.
    top_right = bool(b4 & 0x04)    # Menu / Start equivalent
    top_left  = bool(b4 & 0x08)    # View / Select equivalent

    button_a = bool(b4 & 0x10)
    button_b = bool(b4 & 0x20)
    button_x = bool(b4 & 0x40)
    button_y = bool(b4 & 0x80)

    # D-pad, shoulders and stick-clicks.
    dpad_down  = bool(b5 & 0x01)
    dpad_left  = bool(b5 & 0x02)
    dpad_up    = bool(b5 & 0x04)
    dpad_right = bool(b5 & 0x08)

    left_shoulder  = bool(b5 & 0x10)
    right_shoulder = bool(b5 & 0x20)

    left_stick_button  = bool(b5 & 0x40)
    right_stick_button = bool(b5 & 0x80)

    xbox_button = bool(data[18] & 0x01)

    # --------------------------------------------------------
    # Analogue controls
    # --------------------------------------------------------

    left_trigger  = u16le(data, 6)
    right_trigger = u16le(data, 8)

    left_x  = s16le(data, 10)
    left_y  = s16le(data, 12)
    right_x = s16le(data, 14)
    right_y = s16le(data, 16)

    # --------------------------------------------------------
    # Display button actions
    # --------------------------------------------------------

    show_button("A", button_a)
    show_button("B", button_b)
    show_button("X", button_x)
    show_button("Y", button_y)

    show_button("D-PAD UP", dpad_up)
    show_button("D-PAD DOWN", dpad_down)
    show_button("D-PAD LEFT", dpad_left)
    show_button("D-PAD RIGHT", dpad_right)

    show_button("LEFT SHOULDER", left_shoulder)
    show_button("RIGHT SHOULDER", right_shoulder)

    show_button("LEFT STICK BUTTON", left_stick_button)
    show_button("RIGHT STICK BUTTON", right_stick_button)

    show_button("TOP LEFT / VIEW", top_left)
    show_button("TOP RIGHT / MENU", top_right)
    show_button("XBOX / HOME", xbox_button)

    # --------------------------------------------------------
    # CRSF channels
    #
    # Keep the same axis inversion convention as the previous
    # PS3 controller version:
    #   Roll       not inverted
    #   Pitch      inverted
    #   Throttle   inverted
    #   Yaw        not inverted
    # --------------------------------------------------------

    channels = [
        stick_to_crsf(right_x, False),     # CH1 Roll
        stick_to_crsf(right_y, True),      # CH2 Pitch
        stick_to_crsf(left_y, True),       # CH3 Throttle
        stick_to_crsf(left_x, False),      # CH4 Yaw

        CRSF_MAX if top_right else CRSF_MIN,        # CH5 Menu / Start
        CRSF_MAX if top_left else CRSF_MIN,         # CH6 View / Select
        CRSF_MAX if left_shoulder else CRSF_MIN,    # CH7 LB
        CRSF_MAX if right_shoulder else CRSF_MIN,   # CH8 RB

        trigger_to_crsf(left_trigger),              # CH9 LT analogue
        trigger_to_crsf(right_trigger),             # CH10 RT analogue

        CRSF_MAX if button_y else CRSF_MIN,          # CH11 Y / Triangle
        CRSF_MAX if button_b else CRSF_MIN,          # CH12 B / Circle
        CRSF_MAX if button_a else CRSF_MIN,          # CH13 A / Cross
        CRSF_MAX if button_x else CRSF_MIN,          # CH14 X / Square

        CRSF_MAX if dpad_up else CRSF_MIN,           # CH15 Up
        CRSF_MAX if dpad_down else CRSF_MIN          # CH16 Down
    ]

    return channels


# ============================================================
# MAIN
# ============================================================

print()
print("============================================")
print("PICO 2 - TURTLE BEACH -> CRSF + TELEMETRY")
print("============================================")
print()
print("USB host: GP2 D+ / GP3 D-")
print("UART:     GP0 TX / GP1 RX")
print("Baud:     115200")
print("LCD:      ST7789 240x135 on GP8/9/10/11/12/13")
print()
print("Waiting for Turtle Beach controller...")
print()

packet_count = 0
tx_error_count = 0
tx_count_at_last_debug = 0
last_debug = time.monotonic()
last_crsf_tx = time.monotonic()

# Safe initial state used until the first valid handset report arrives.
# Primary stick axes are centred; all auxiliary channels are low.
latest_channels = [
    CRSF_MID, CRSF_MID, CRSF_MID, CRSF_MID,
    CRSF_MIN, CRSF_MIN, CRSF_MIN, CRSF_MIN,
    CRSF_MIN, CRSF_MIN, CRSF_MIN, CRSF_MIN,
    CRSF_MIN, CRSF_MIN, CRSF_MIN, CRSF_MIN
]

update_lcd(force=True)


while True:
    service_reload_button()
    service_led()

    # --------------------------------------------------------
    # Receive/display returned CRSF telemetry from radio Pico
    # --------------------------------------------------------

    process_telemetry_uart()
    update_lcd()

    # --------------------------------------------------------
    # Find / reconnect controller
    # --------------------------------------------------------

    if device is None:
        try:
            find_controller()

        except Exception as e:
            print("USB search error:", repr(e))
            device = None

        if device is None:
            # Do not stop the CRSF scheduler merely because the handset is
            # absent. The safe/latest state is still transmitted at 25 Hz.
            time.sleep(0.005)

    # --------------------------------------------------------
    # Read Xbox GIP input report
    # --------------------------------------------------------

    if device is not None:
        try:
            n = device.read(
                EP_IN,
                rx_buffer,
                timeout=5
            )

            if n:
                data = bytes(rx_buffer[:n])
                channels = decode_report(data)

                if channels is not None:
                    handset_usb_found = True
                    handset_connected = True

                    # Only update the demanded channel state here. Actual
                    # CRSF transmission is performed by the fixed-rate
                    # scheduler below.
                    latest_channels = channels

        except usb.core.USBTimeoutError:
            pass

        except Exception as e:
            print("USB read error:", repr(e))
            device = None
            handset_usb_found = False
            handset_connected = False

    # --------------------------------------------------------
    # Fixed-rate CRSF UART transmitter: 25 Hz / every 40 ms
    # --------------------------------------------------------

    now = time.monotonic()

    if now - last_crsf_tx >= CRSF_TX_INTERVAL:
        # Advance by one interval rather than setting to now. This avoids
        # slow long-term drift while still preventing catch-up bursts.
        last_crsf_tx += CRSF_TX_INTERVAL
        if now - last_crsf_tx >= CRSF_TX_INTERVAL:
            last_crsf_tx = now

        frame = make_crsf_frame(latest_channels)

        try:
            written = uart.write(frame)

            # CircuitPython normally returns the number of bytes written.
            # Count None as success too, for compatibility with ports that
            # do not provide a byte count.
            if written is None or written == len(frame):
                packet_count += 1
            else:
                tx_error_count += 1
                print("UART SHORT WRITE:", written, "of", len(frame))

        except Exception as e:
            tx_error_count += 1
            print("UART TX ERROR:", repr(e))

    # --------------------------------------------------------
    # Once-per-second instrumentation
    # --------------------------------------------------------

    now = time.monotonic()

    if now - last_debug >= 1.0:
        elapsed = now - last_debug
        tx_delta = packet_count - tx_count_at_last_debug
        tx_rate = tx_delta / elapsed if elapsed > 0 else 0.0

        link_age = (
            now - last_valid_telemetry_time
            if last_valid_telemetry_time is not None
            else 999.0
        )
        type_text = (
            "0x%02X %s" % (last_telemetry_type, crsf_type_name(last_telemetry_type))
            if last_telemetry_type is not None
            else "---"
        )

        print(
            "UART TX %.1f/s" % tx_rate,
            "| HANDSET", "CONNECTED" if handset_connected else ("USB_FOUND" if handset_usb_found else "NONE"),
            "| TOTAL", packet_count,
            "| TXERR", tx_error_count,
            "| TELEM RX", telemetry_frame_count,
            "| BAD", telemetry_bad_crc_count,
            "| LAST", type_text,
            "| AGE %.2fs" % link_age,
            "| CH1-4",
            latest_channels[0], latest_channels[1],
            latest_channels[2], latest_channels[3],
            "| LT/RT", latest_channels[8], latest_channels[9]
        )

        tx_count_at_last_debug = packet_count
        last_debug = now
        update_lcd(force=True)

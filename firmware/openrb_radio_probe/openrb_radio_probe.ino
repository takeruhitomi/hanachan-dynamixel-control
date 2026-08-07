// OpenRB-150 radio probe.
//
// Finds the UART baud rate of the wireless module on the 4-pin Serial2 port.
// The dongle on the PC side is transparent, so it answers nothing by itself;
// the only way to learn the link speed is to have both ends try rates until
// bytes survive the trip. This sketch drives the OpenRB half of that search
// and reports over USB, which stays usable no matter what the radio is doing.
//
// Flash this, then run the PC-side scanner:
//   uv run python firmware/radio_scan.py --usb COM6 --radio COM7
//
// USB console protocol (one command per line, 115200 or any rate, USB CDC
// ignores it):
//   B<baud>   reopen Serial2 at that baud, e.g. B57600
//   R         report what arrived on Serial2 since the last report
//   T<text>   transmit text out of Serial2
//   ?         print the current baud
//
// Nothing here touches the DYNAMIXEL bus, so the servos cannot move.

#define USB_SERIAL Serial
#define RADIO_SERIAL Serial2

const uint32_t DEFAULT_RADIO_BAUD = 57600;
const size_t CAPTURE_SIZE = 512;

static uint32_t radio_baud = DEFAULT_RADIO_BAUD;
static uint8_t capture[CAPTURE_SIZE];
static size_t capture_len = 0;

static void openRadio(uint32_t baud) {
  RADIO_SERIAL.end();
  delay(5);
  RADIO_SERIAL.begin(baud);
  delay(5);
  while (RADIO_SERIAL.available()) {
    RADIO_SERIAL.read();
  }
  radio_baud = baud;
  capture_len = 0;
}

static void reportCapture() {
  USB_SERIAL.print("BAUD ");
  USB_SERIAL.print(radio_baud);
  USB_SERIAL.print(" BYTES ");
  USB_SERIAL.print(capture_len);
  USB_SERIAL.print(" HEX ");
  for (size_t i = 0; i < capture_len; i++) {
    if (capture[i] < 0x10) {
      USB_SERIAL.print('0');
    }
    USB_SERIAL.print(capture[i], HEX);
  }
  USB_SERIAL.print(" ASCII ");
  for (size_t i = 0; i < capture_len; i++) {
    char c = (char)capture[i];
    USB_SERIAL.print((c >= 32 && c < 127) ? c : '.');
  }
  USB_SERIAL.println();
  capture_len = 0;
}

static void handleCommand(const String &line) {
  if (line.length() == 0) {
    return;
  }
  char op = line.charAt(0);
  if (op == 'B' || op == 'b') {
    uint32_t baud = (uint32_t)line.substring(1).toInt();
    if (baud == 0) {
      USB_SERIAL.println("ERR bad baud");
      return;
    }
    openRadio(baud);
    USB_SERIAL.print("OK baud ");
    USB_SERIAL.println(radio_baud);
  } else if (op == 'R' || op == 'r') {
    reportCapture();
  } else if (op == 'T' || op == 't') {
    String payload = line.substring(1);
    RADIO_SERIAL.print(payload);
    RADIO_SERIAL.flush();
    USB_SERIAL.print("OK sent ");
    USB_SERIAL.println(payload.length());
  } else if (op == '?') {
    USB_SERIAL.print("OK baud ");
    USB_SERIAL.println(radio_baud);
  } else {
    USB_SERIAL.println("ERR unknown command");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  USB_SERIAL.begin(115200);
  openRadio(DEFAULT_RADIO_BAUD);
  // No waiting on the USB host: the board must keep working when run headless.
  USB_SERIAL.println("HANA_RADIO_PROBE_1");
}

void loop() {
  while (RADIO_SERIAL.available()) {
    uint8_t value = (uint8_t)RADIO_SERIAL.read();
    if (capture_len < CAPTURE_SIZE) {
      capture[capture_len++] = value;
    }
    digitalWrite(LED_BUILTIN, HIGH);
  }

  static String line;
  while (USB_SERIAL.available()) {
    char c = (char)USB_SERIAL.read();
    if (c == '\n' || c == '\r') {
      handleCommand(line);
      line = "";
    } else if (line.length() < 120) {
      line += c;
    }
  }

  static uint32_t last_blink = 0;
  if (millis() - last_blink > 100) {
    last_blink = millis();
    digitalWrite(LED_BUILTIN, LOW);
  }
}

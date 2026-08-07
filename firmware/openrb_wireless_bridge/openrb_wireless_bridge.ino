// OpenRB-150 USB + wireless to DYNAMIXEL bridge.
//
// Replaces the stock usb_to_dynamixel sketch. It keeps the USB port working
// exactly as before and adds the 4-pin Serial2 port, so a transparent radio
// module plugged in there can drive the same DYNAMIXEL bus from the PC.
//
//   Serial   USB CDC          host A
//   Serial2  4-pin UART port  host B (wireless module)
//   Serial1  DYNAMIXEL bus    OpenRB-150 handles the half-duplex direction in
//                             hardware, so plain byte forwarding is enough
//                             (Dynamixel2Arduino uses DXL_DIR_PIN = -1 here).
//
// Set RADIO_BAUD to the UART rate the wireless module is configured for. The
// modules are transparent, so a mismatch produces framing noise rather than an
// error. Use firmware/openrb_radio_probe to find the rate if it is unknown.
//
// If the radio is misbehaving, set ENABLE_RADIO to 0 to get a plain USB
// bridge; that is the quickest way to tell a radio problem from a bus problem.

#define ENABLE_RADIO 1

#define USB_SERIAL Serial
#define RADIO_SERIAL Serial2
#define DXL_SERIAL Serial1

const uint32_t DXL_BAUD = 1000000;
const uint32_t RADIO_BAUD = 57600;

// A sync read across ~21 servos returns a few hundred bytes in about 3 ms at
// 1 Mbps, while the radio needs tens of milliseconds to pass them on. Without
// somewhere to park them the DYNAMIXEL receive buffer overruns and the reply
// is silently truncated.
const size_t REPLY_BUFFER_SIZE = 1024;

// One host owns the bus until it goes quiet. Without this the two ports
// interleave their bytes into the middle of each other's packets, which is
// guaranteed corruption as soon as the radio delivers anything at all -- and a
// radio running at the wrong baud delivers noise continuously.
const uint32_t OWNER_RELEASE_MS = 15;

// If the active host stops draining, drop the backlog rather than wedge the
// bridge forever.
const uint32_t REPLY_STALE_MS = 250;

enum Host { HOST_NONE, HOST_USB, HOST_RADIO };

static uint8_t reply_buffer[REPLY_BUFFER_SIZE];
static size_t reply_head = 0;
static size_t reply_tail = 0;
static Host owner = HOST_NONE;
static uint32_t last_host_byte_ms = 0;
static uint32_t reply_pending_since = 0;

static inline size_t replyCount() {
  return (reply_head + REPLY_BUFFER_SIZE - reply_tail) % REPLY_BUFFER_SIZE;
}

static inline bool replyPush(uint8_t value) {
  size_t next = (reply_head + 1) % REPLY_BUFFER_SIZE;
  if (next == reply_tail) {
    return false;  // full; dropping beats blocking the bus
  }
  reply_buffer[reply_head] = value;
  reply_head = next;
  return true;
}

static void discard(Stream &source) {
  while (source.available() > 0) {
    source.read();
  }
}

// Pick the host allowed to drive the bus. A new one may only take over after
// the current one has been silent, so a packet is never split.
static void updateOwner() {
  if (owner != HOST_NONE && (millis() - last_host_byte_ms) < OWNER_RELEASE_MS) {
    return;
  }
  if (USB_SERIAL.available() > 0) {
    owner = HOST_USB;
  } else if (ENABLE_RADIO && RADIO_SERIAL.available() > 0) {
    owner = HOST_RADIO;
  } else {
    owner = HOST_NONE;
  }
}

static void forwardToDxl() {
  Stream *source = nullptr;
  if (owner == HOST_USB) {
    source = &USB_SERIAL;
#if ENABLE_RADIO
    discard(RADIO_SERIAL);  // whatever the radio is saying, it is not our turn
#endif
  } else if (owner == HOST_RADIO) {
#if ENABLE_RADIO
    source = &RADIO_SERIAL;
#endif
    discard(USB_SERIAL);
  }
  if (source == nullptr) {
    return;
  }

  while (source->available() > 0 && DXL_SERIAL.availableForWrite() > 0) {
    DXL_SERIAL.write((uint8_t)source->read());
    last_host_byte_ms = millis();
  }
}

static void drainDxl() {
  while (DXL_SERIAL.available() > 0) {
    if (!replyPush((uint8_t)DXL_SERIAL.read())) {
      return;
    }
  }
  if (replyCount() > 0 && reply_pending_since == 0) {
    reply_pending_since = millis();
  }
}

static void pushReplies() {
  if (replyCount() == 0) {
    reply_pending_since = 0;
    return;
  }

  Stream *sink = nullptr;
  if (owner == HOST_RADIO) {
#if ENABLE_RADIO
    sink = &RADIO_SERIAL;
#endif
  } else {
    // Deliberately not gated on the USB DTR state: a host that opens the port
    // without asserting DTR would otherwise never receive a single reply.
    sink = &USB_SERIAL;
  }
  if (sink == nullptr) {
    reply_tail = reply_head;
    reply_pending_since = 0;
    return;
  }

  int room = sink->availableForWrite();
  while (room > 0 && reply_tail != reply_head) {
    sink->write(reply_buffer[reply_tail]);
    reply_tail = (reply_tail + 1) % REPLY_BUFFER_SIZE;
    room--;
  }

  if (reply_tail == reply_head) {
    reply_pending_since = 0;
  } else if (reply_pending_since != 0 &&
             (millis() - reply_pending_since) > REPLY_STALE_MS) {
    reply_tail = reply_head;
    reply_pending_since = 0;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // The DYNAMIXEL power FET comes up off after every reset on OpenRB-150, and
  // nothing in the core or in Serial1.begin() turns it on. Without this the
  // servos have no power and every ping times out. The board's red DXL LED
  // lights up once it is on.
  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  delay(300);  // let the bus rail settle before talking to it

  USB_SERIAL.begin(DXL_BAUD);  // USB CDC ignores the rate; kept for clarity
#if ENABLE_RADIO
  RADIO_SERIAL.begin(RADIO_BAUD);
#endif
  DXL_SERIAL.begin(DXL_BAUD);
}

void loop() {
  updateOwner();
  forwardToDxl();
  drainDxl();
  pushReplies();

  // Lit while a host is talking, so a dead link is visible at a glance.
  digitalWrite(LED_BUILTIN, (millis() - last_host_byte_ms < 200) ? HIGH : LOW);
}

// Real-time 3D Suzuki GSX-250R on the Waveshare ESP32-S3-Touch-AMOLED-1.8 (368x448 AMOLED).
// Handles both hardware revisions: V1 = SH8601 display + FT3168 touch (0x38),
// V2 = CO5300 display + CST820 touch (0x15). Detected at boot from the touch chip's I2C address.
//
// The mesh comes from a Higgsfield (Tripo image-to-3D) model, decimated to 4000 triangles with one
// colour per face by tools/convert_model.py. It is drawn by a small software rasterizer
// (z-buffer, Gouraud lighting, one light) into two PSRAM canvases: core 1 renders one while
// core 0 pushes the other to the panel.
//
// Drag to rotate (it keeps spinning when you let go). Left alone, it turns by itself.
// Tilt the board to swing the view. BOOT toggles tilt control.

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "model.h"
#include "types.h"

// ---------- hardware ----------
#define LCD_CS 12
#define LCD_SCK 11
#define LCD_D0 4
#define LCD_D1 5
#define LCD_D2 6
#define LCD_D3 7
#define I2C_SDA 15
#define I2C_SCL 14
#define BOOT_BTN 0
#define TOUCH_ADDR_V1 0x38  // FT3168
#define TOUCH_ADDR_V2 0x15  // CST820 (same register layout from 0x02)
#define IMU_ADDR 0x6B
#define EXPANDER_ADDR 0x20

#define W 368
#define H 448

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_OLED *panel = nullptr;  // SH8601 or CO5300, picked in setup()
Arduino_Canvas *canvas[2];
uint8_t touchAddr = TOUCH_ADDR_V2;

constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }
const uint16_t C_TEXT = rgb(235, 235, 245), C_DIM = rgb(90, 90, 105), C_ACCENT = rgb(40, 120, 255);

// ---------- view ----------
#define CAM_DIST 3.0f   // camera distance in model units (model's longest half-extent = 1)
#define FOCAL 470.0f    // pixels
#define CENTER_Y 236    // screen y of the model centre (leaves room for the title)
#define AMBIENT 0.35f
#define DIFFUSE 0.90f
#define DEPTH_NEAR (CAM_DIST - 1.4f)
#define DEPTH_FAR (CAM_DIST + 1.4f)

float yaw = -0.9f, pitch = 0.22f;  // current orientation
float yawVel = 0, pitchVel = 0;    // rad/s, from the last drag
float tiltYaw = 0, tiltPitch = 0;  // IMU offsets
bool tiltOn = true;

PV pv[MODEL_NV];
uint16_t *zbuf;  // W*H depth buffer in PSRAM, 0xFFFF = empty

Box lastBox[2] = { { 0, 0, W, H }, { 0, 0, W, H } };  // area drawn into each canvas last time

// ---------- I2C: touch / IMU / expander ----------
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void expanderInit() {
  // TCA9554: pins 0-2 drive the LCD / touch reset lines. Pulse them low then high.
  if (!i2cWrite(EXPANDER_ADDR, 0x03, 0xF8)) return;
  i2cWrite(EXPANDER_ADDR, 0x01, 0x00);
  delay(20);
  i2cWrite(EXPANDER_ADDR, 0x01, 0x07);  // P0 LCD_RST, P1 display power, P2 TOUCH_RST all high
  delay(150);
}

bool readTouch(int &x, int &y) {
  uint8_t b[5];
  if (!i2cRead(touchAddr, 0x02, b, 5)) return false;
  uint8_t n = b[0] & 0x0F;
  if (n == 0 || n > 2) return false;
  x = ((b[1] & 0x0F) << 8) | b[2];
  y = ((b[3] & 0x0F) << 8) | b[4];
  return x < W && y < H;
}

bool imuOk = false;

void imuInit() {
  uint8_t id;
  if (!i2cRead(IMU_ADDR, 0x00, &id, 1) || id != 0x05) return;
  i2cWrite(IMU_ADDR, 0x02, 0x40);  // CTRL1: register auto-increment
  i2cWrite(IMU_ADDR, 0x03, 0x15);  // CTRL2: accel +-4g
  i2cWrite(IMU_ADDR, 0x08, 0x01);  // CTRL7: accel on
  imuOk = true;
}

bool readAccel(float &ax, float &ay, float &az) {
  uint8_t b[6];
  if (!imuOk || !i2cRead(IMU_ADDR, 0x35, b, 6)) return false;
  ax = (int16_t)(b[0] | b[1] << 8) / 8192.0f;
  ay = (int16_t)(b[2] | b[3] << 8) / 8192.0f;
  az = (int16_t)(b[4] | b[5] << 8) / 8192.0f;
  return true;
}

// ---------- rasterizer ----------
// Project every vertex and light it. Returns the screen box the model covers.
Box transformVertices(float yw, float pt) {
  float cy = cosf(yw), sy = sinf(yw), cp = cosf(pt), sp = sinf(pt);
  // R = Rx(pitch) * Ry(yaw)
  float m00 = cy, m01 = 0, m02 = sy;
  float m10 = sp * sy, m11 = cp, m12 = -sp * cy;
  float m20 = -cp * sy, m21 = sp, m22 = cp * cy;
  const float vs = 1.0f / 32767, ns = 1.0f / 127;
  const float lx = -0.40f, ly = 0.60f, lz = 0.70f, ll = 1.0f / sqrtf(lx * lx + ly * ly + lz * lz);
  const float zq = 32767.0f / (DEPTH_FAR - DEPTH_NEAR);
  float bx0 = W, by0 = H, bx1 = 0, by1 = 0;
  for (int k = 0; k < MODEL_NV; k++) {
    float x = MODEL_V[k * 3] * vs, y = MODEL_V[k * 3 + 1] * vs, z = MODEL_V[k * 3 + 2] * vs;
    float rx = m00 * x + m01 * y + m02 * z;
    float ry = m10 * x + m11 * y + m12 * z;
    float rz = m20 * x + m21 * y + m22 * z;
    float d = CAM_DIST - rz, inv = FOCAL / d;
    PV &p = pv[k];
    p.x = W * 0.5f + rx * inv;
    p.y = CENTER_Y - ry * inv;
    p.z = (d - DEPTH_NEAR) * zq;
    float nx = MODEL_N[k * 3] * ns, ny = MODEL_N[k * 3 + 1] * ns, nz = MODEL_N[k * 3 + 2] * ns;
    float qx = m00 * nx + m01 * ny + m02 * nz;
    float qy = m10 * nx + m11 * ny + m12 * nz;
    float qz = m20 * nx + m21 * ny + m22 * nz;
    if (qz < 0) { qx = -qx; qy = -qy; qz = -qz; }  // two-sided: the mesh has mixed winding
    float l = (qx * lx + qy * ly + qz * lz) * ll;
    p.i = (AMBIENT + DIFFUSE * (l > 0 ? l : 0)) * 256.0f;
    if (p.x < bx0) bx0 = p.x;
    if (p.x > bx1) bx1 = p.x;
    if (p.y < by0) by0 = p.y;
    if (p.y > by1) by1 = p.y;
  }
  Box b = { max(0, (int)bx0 - 2), max(0, (int)by0 - 2), min(W, (int)bx1 + 3), min(H, (int)by1 + 3) };
  return b;
}

static inline float edgeStep(float dx, float dy) { return (dy > 1e-6f || dy < -1e-6f) ? dx / dy : 0; }

// Scanline fill with linear depth and intensity. col is the face's RGB565 base colour.
void IRAM_ATTR drawTri(uint16_t *fb, const PV *a, const PV *b, const PV *c, uint16_t col) {
  if (a->y > b->y) { const PV *t = a; a = b; b = t; }
  if (b->y > c->y) { const PV *t = b; b = c; c = t; }
  if (a->y > b->y) { const PV *t = a; a = b; b = t; }
  float area = (b->x - a->x) * (c->y - a->y) - (c->x - a->x) * (b->y - a->y);
  if (area > -0.01f && area < 0.01f) return;
  float ia = 1.0f / area;
  float dzdx = ((b->z - a->z) * (c->y - a->y) - (c->z - a->z) * (b->y - a->y)) * ia;
  float dzdy = ((c->z - a->z) * (b->x - a->x) - (b->z - a->z) * (c->x - a->x)) * ia;
  float didx = ((b->i - a->i) * (c->y - a->y) - (c->i - a->i) * (b->y - a->y)) * ia;
  float didy = ((c->i - a->i) * (b->x - a->x) - (b->i - a->i) * (c->x - a->x)) * ia;
  int dz = (int)(dzdx * 65536.0f), di = (int)(didx * 65536.0f);

  float s02 = edgeStep(c->x - a->x, c->y - a->y);
  float s01 = edgeStep(b->x - a->x, b->y - a->y);
  float s12 = edgeStep(c->x - b->x, c->y - b->y);
  int y0 = max(0, (int)ceilf(a->y - 0.5f)), y2 = min(H, (int)ceilf(c->y - 0.5f));
  int r5 = col >> 11, g6 = (col >> 5) & 0x3F, b5 = col & 0x1F;

  for (int y = y0; y < y2; y++) {
    float yc = y + 0.5f;
    float xa = a->x + (yc - a->y) * s02;
    float xb = yc < b->y ? a->x + (yc - a->y) * s01 : b->x + (yc - b->y) * s12;
    if (xa > xb) { float t = xa; xa = xb; xb = t; }
    int xs = max(0, (int)ceilf(xa - 0.5f)), xe = min(W, (int)ceilf(xb - 0.5f));
    if (xs >= xe) continue;
    float fx = xs + 0.5f - a->x, fy = yc - a->y;
    int zf = (int)((a->z + dzdx * fx + dzdy * fy) * 65536.0f);
    int if_ = (int)((a->i + didx * fx + didy * fy) * 65536.0f);
    uint16_t *zr = zbuf + y * W, *fr = fb + y * W;
    for (int x = xs; x < xe; x++, zf += dz, if_ += di) {
      int zz = zf >> 16;
      if (zz < 0) zz = 0;
      if (zz >= zr[x]) continue;
      zr[x] = zz;
      int ii = if_ >> 16;
      if (ii < 0) ii = 0;
      int r = (r5 * ii) >> 8, g = (g6 * ii) >> 8, bl = (b5 * ii) >> 8;
      if (r > 31) r = 31;
      if (g > 63) g = 63;
      if (bl > 31) bl = 31;
      fr[x] = (r << 11) | (g << 5) | bl;
    }
  }
}

void clearBox(uint16_t *buf, const Box &b, uint16_t v) {
  if (b.x1 <= b.x0) return;
  for (int y = b.y0; y < b.y1; y++) {
    uint16_t *p = buf + y * W + b.x0;
    if (v == 0) memset(p, 0, (b.x1 - b.x0) * 2);
    else for (int x = b.x0; x < b.x1; x++) p[x - b.x0] = v;
  }
}

void renderModel(int idx, float yw, float pt) {
  uint16_t *fb = canvas[idx]->getFramebuffer();
  Box cur = transformVertices(yw, pt);
  // erase whatever this canvas showed two frames ago, and reset depth where we will draw
  Box &old = lastBox[idx];
  Box u = { min(old.x0, cur.x0), min(old.y0, cur.y0), max(old.x1, cur.x1), max(old.y1, cur.y1) };
  clearBox(fb, u, 0);
  clearBox(zbuf, cur, 0xFFFF);
  old = cur;
  for (int f = 0; f < MODEL_NF; f++) {
    const uint16_t *t = MODEL_F + f * 3;
    drawTri(fb, &pv[t[0]], &pv[t[1]], &pv[t[2]], MODEL_C[f]);
  }
}

// ---------- display task: core 0 pushes finished frames while core 1 renders the next ----------
QueueHandle_t flushQueue;
SemaphoreHandle_t canvasFree[2];

void flushTask(void *) {
  int idx;
  for (;;) {
    if (xQueueReceive(flushQueue, &idx, portMAX_DELAY) == pdTRUE) {
      canvas[idx]->flush();
      xSemaphoreGive(canvasFree[idx]);
    }
  }
}

// ---------- main ----------
void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BTN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  expanderInit();

  // V1 boards have an FT3168 at 0x38; otherwise assume V2 (CST820 at 0x15, CO5300 panel).
  Wire.beginTransmission(TOUCH_ADDR_V1);
  bool v1 = Wire.endTransmission() == 0;
  touchAddr = v1 ? TOUCH_ADDR_V1 : TOUCH_ADDR_V2;
  if (v1) panel = new Arduino_SH8601(bus, GFX_NOT_DEFINED, 0, W, H);
  else panel = new Arduino_CO5300(bus, GFX_NOT_DEFINED, 0, W, H, 16, 0, 0, 0);
  canvas[0] = new Arduino_Canvas(W, H, panel);
  canvas[1] = new Arduino_Canvas(W, H, panel);
  bool ok = canvas[0]->begin(80000000) && canvas[1]->begin(GFX_SKIP_OUTPUT_BEGIN);
  zbuf = (uint16_t *)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM);
  for (int i = 0; i < 2; i++) canvas[i]->fillScreen(0);
  panel->setBrightness(230);
  imuInit();
  Serial.printf("board V%d display=%d zbuf=%d psram=%u imu=%d tris=%d\n", v1 ? 1 : 2, ok, zbuf != nullptr,
                ESP.getPsramSize(), imuOk, MODEL_NF);

  flushQueue = xQueueCreate(1, sizeof(int));
  for (int i = 0; i < 2; i++) canvasFree[i] = xSemaphoreCreateBinary(), xSemaphoreGive(canvasFree[i]);
  xTaskCreatePinnedToCore(flushTask, "flush", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
  static int cur = 0;
  static uint32_t lastUs = micros(), lastTouchMs = 0, fpsT = 0;
  static int frames = 0, fps = 0;
  static bool touching = false, bootPrev = true;
  static int lastX, lastY;
  static float baseX = 0, baseY = 0;
  static bool baseInit = false;

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.1f) dt = 0.1f;
  uint32_t now = millis();

  // touch: drag rotates, release keeps the spin going
  int tx, ty;
  if (readTouch(tx, ty)) {
    if (touching) {
      float dyaw = (tx - lastX) * 0.012f, dpitch = (ty - lastY) * 0.010f;
      yaw += dyaw;
      pitch += dpitch;
      if (dt > 0) {
        yawVel = yawVel * 0.5f + 0.5f * dyaw / dt;
        pitchVel = pitchVel * 0.5f + 0.5f * dpitch / dt;
      }
    } else {
      yawVel = pitchVel = 0;
    }
    touching = true;
    lastX = tx;
    lastY = ty;
    lastTouchMs = now;
  } else {
    touching = false;
    float damp = expf(-1.6f * dt);
    yawVel *= damp;
    pitchVel *= damp;
    // drift back to auto-spin a couple of seconds after the last touch
    if (now - lastTouchMs > 2500) yawVel += (0.55f - yawVel) * (1 - expf(-0.8f * dt));
    if (now - lastTouchMs > 2500) pitch += (0.22f - pitch) * (1 - expf(-0.6f * dt));
    yaw += yawVel * dt;
    pitch += pitchVel * dt;
  }
  if (pitch > 1.3f) pitch = 1.3f, pitchVel = 0;
  if (pitch < -1.3f) pitch = -1.3f, pitchVel = 0;
  if (yaw > PI) yaw -= TWO_PI;
  if (yaw < -PI) yaw += TWO_PI;

  // tilt: the view swings with how far the board is tipped from where it has been resting
  float ax, ay, az;
  if (tiltOn && readAccel(ax, ay, az)) {
    if (!baseInit) baseX = ax, baseY = ay, baseInit = true;
    float k = 1 - expf(-0.5f * dt);  // baseline follows slowly, so a held tilt eases back
    baseX += (ax - baseX) * k;
    baseY += (ay - baseY) * k;
    float s = 1 - expf(-10.0f * dt);
    tiltYaw += ((ay - baseY) * 1.6f - tiltYaw) * s;
    tiltPitch += ((ax - baseX) * 1.2f - tiltPitch) * s;
  } else {
    tiltYaw *= 0.9f;
    tiltPitch *= 0.9f;
  }

  bool boot = digitalRead(BOOT_BTN);
  if (!boot && bootPrev) tiltOn = !tiltOn, baseInit = false;
  bootPrev = boot;

  // render into the free canvas and hand it to core 0
  xSemaphoreTake(canvasFree[cur], portMAX_DELAY);
  Arduino_Canvas *g = canvas[cur];
  renderModel(cur, yaw + tiltYaw, pitch + tiltPitch);

  g->setTextColor(C_ACCENT, 0);
  g->setTextSize(2);
  g->setCursor(W / 2 - 42, 18);
  g->print("SUZUKI");
  g->setTextColor(C_TEXT, 0);
  g->setTextSize(3);
  g->setCursor(W / 2 - 81, 40);
  g->print("GSX-250R");
  g->setTextColor(C_DIM, 0);
  g->setTextSize(1);
  g->setCursor(10, H - 18);
  g->printf("%2d fps  %d tris   ", fps, MODEL_NF);
  g->setCursor(W - 88, H - 18);
  g->print(tiltOn ? "tilt: on " : "tilt: off");

  xQueueSend(flushQueue, &cur, portMAX_DELAY);
  cur ^= 1;

  frames++;
  if (now - fpsT >= 1000) {
    fps = frames;
    frames = 0;
    fpsT = now;
    Serial.printf("fps=%d yaw=%.2f pitch=%.2f tilt=%.2f,%.2f\n", fps, yaw, pitch, tiltYaw, tiltPitch);
  }
}

#include <Wire.h>
#include "SSD1306Wire.h"
#include <esp_system.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESP_I2S.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

SSD1306Wire display(0x3c, 5, 4);

uint8_t MAC_PAR[6] = {0x84, 0xCC, 0xA8, 0x5D, 0x42, 0x44};

#define REDE_ID            0x42
#define CANAL_ESPNOW       1
#define INTERVALO_PING_MS  1000
#define TIMEOUT_LINK_MS    3500

#define INTERVALO_TELA_MS  100

bool linkOk = false;
int  rssiAtual = 0;

String macCompleto = "";

#define BAT_PIN 34

const float R1_OHMS = 100000.0;
const float R2_OHMS = 100000.0;
const float FATOR_DIVISOR = (R1_OHMS + R2_OHMS) / R2_OHMS;

const float CAL_BATERIA = 1.0;

const int BAT_MIN_MV = 0;
const int BAT_MAX_MV = 4000;

#define I2S_BCLK   26
#define I2S_WS     25
#define I2S_DIN    33

#define AUDIO_RATE  16000
#define AUDIO_BLOCO 256

#define VU_DB_MIN  -60.0
#define VU_DB_MAX  -10.0

#define VU_BARRAS  10
#define VU_X0      16
#define VU_LARG    4
#define VU_ESPACO  2

#define MODO_SIMULADO  true
#define BOTAO_PIN      27
bool botaoApertado = false;

I2SClass i2s;
int32_t bufAudio[AUDIO_BLOCO];

bool  micOk   = false;
float dbAtual = -99.0;
float nivelVU = 0;
float picoVU  = 0;
uint32_t picoMs = 0;

#define TIPO_PING 1
#define TIPO_PONG 2
#define TIPO_PTT  3

typedef struct __attribute__((packed)) {
  uint8_t  rede;
  uint8_t  tipo;
  uint16_t id;
  uint32_t t_envio_us;
} struct_mensagem;

#define TIPO_TXT        4
#define TXT_MAX         40
#define MSG_EXIBIR_MS   10000
#define LARG_MSG_PX     118

typedef struct __attribute__((packed)) {
  uint8_t  rede;
  uint8_t  tipo;
  uint16_t id;
  char     texto[TXT_MAX + 1];
} struct_texto;

volatile uint32_t rttUs          = 0;
volatile uint32_t ultimoPongMs   = 0;
volatile uint32_t pongsRecebidos = 0;
volatile uint32_t falhasEnvio    = 0;
volatile int      rssiPar        = -100;

volatile bool     pongPendente   = false;
volatile uint16_t pongId         = 0;
volatile uint32_t pongT          = 0;

volatile bool     pttRemotoRaw      = false;
volatile uint32_t ultimoPttRemotoMs = 0;

char          msgRx[TXT_MAX + 1];
volatile bool msgRxPendente = false;

uint32_t pingsEnviados = 0;
bool     espnowOk = false;

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void aoEnviar(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void aoEnviar(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) falhasEnvio++;
}

void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados, int len) {
  if (len == sizeof(struct_texto)) {
    struct_texto t;
    memcpy(&t, dados, sizeof(t));
    if (t.rede == REDE_ID && t.tipo == TIPO_TXT && !msgRxPendente) {
      t.texto[TXT_MAX] = '\0';
      strcpy(msgRx, t.texto);
      msgRxPendente = true;
    }
    return;
  }

  if (len != sizeof(struct_mensagem)) return;

  struct_mensagem m;
  memcpy(&m, dados, sizeof(m));
  if (m.rede != REDE_ID) return;

  rssiPar = info->rx_ctrl->rssi;

  if (m.tipo == TIPO_PING) {
    pongId = m.id;
    pongT  = m.t_envio_us;
    pongPendente = true;
  } else if (m.tipo == TIPO_PONG) {
    rttUs = micros() - m.t_envio_us;
    ultimoPongMs = millis();
    pongsRecebidos++;
  } else if (m.tipo == TIPO_PTT) {
    pttRemotoRaw = (m.id != 0);
    ultimoPttRemotoMs = millis();
  }
}

void iniciarEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);

  uint8_t canal; wifi_second_chan_t sec;
  esp_wifi_get_channel(&canal, &sec);
  if (Serial) Serial.printf("[WIFI] Canal atual: %d\n", canal);

  if (esp_now_init() != ESP_OK) {
    if (Serial) Serial.println("[ESP-NOW] ERRO ao iniciar");
    return;
  }
  esp_now_register_send_cb(aoEnviar);
  esp_now_register_recv_cb(aoReceber);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MAC_PAR, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    if (Serial) Serial.println("[ESP-NOW] ERRO ao adicionar o par");
    return;
  }
  espnowOk = true;
  if (Serial) Serial.println("[ESP-NOW] Iniciado");
}

void gerenciarEspNow() {
  static uint32_t ultimoPing = 0;
  static uint16_t idPing = 0;
  static uint32_t pongsImpressos = 0;
  static uint32_t ultimoStatus = 0;
  uint32_t agora = millis();

  if (!espnowOk) { linkOk = false; return; }

  if (pongPendente) {
    pongPendente = false;
    struct_mensagem r = { REDE_ID, TIPO_PONG, pongId, pongT };
    esp_now_send(MAC_PAR, (uint8_t *)&r, sizeof(r));
  }

  if (agora - ultimoPing >= INTERVALO_PING_MS) {
    ultimoPing = agora;
    struct_mensagem p = { REDE_ID, TIPO_PING, ++idPing, micros() };
    esp_now_send(MAC_PAR, (uint8_t *)&p, sizeof(p));
    pingsEnviados++;
  }

  linkOk = (pongsRecebidos > 0) && (agora - ultimoPongMs < TIMEOUT_LINK_MS);
  if (linkOk) rssiAtual = rssiPar;

  if (pongsRecebidos != pongsImpressos) {
    pongsImpressos = pongsRecebidos;
    if (Serial) Serial.printf("[PONG] RTT=%.2f ms | RSSI=%d dBm | enviados=%lu recebidos=%lu falhas=%lu\n",
                              rttUs / 1000.0, rssiPar,
                              (unsigned long)pingsEnviados,
                              (unsigned long)pongsRecebidos,
                              (unsigned long)falhasEnvio);
  }

  if (!linkOk && agora - ultimoStatus >= 2000) {
    ultimoStatus = agora;
    if (Serial) Serial.printf("[STATUS] sem link | enviados=%lu recebidos=%lu falhas=%lu\n",
                              (unsigned long)pingsEnviados,
                              (unsigned long)pongsRecebidos,
                              (unsigned long)falhasEnvio);
  }
}

void lerBotao() {
  static bool     ultimaBruta = false;
  static uint32_t tMudou = 0;

  bool bruta = (digitalRead(BOTAO_PIN) == LOW);
  if (bruta != ultimaBruta) { ultimaBruta = bruta; tMudou = millis(); }
  if (millis() - tMudou >= 25) botaoApertado = bruta;
}

#define INTERVALO_PTT_MS 300
#define TIMEOUT_PTT_MS   1000

#define PTT_LIVRE      0
#define PTT_FALANDO    1
#define PTT_ESCUTANDO  2
#define PTT_CONFLITO   3

bool pttRemoto = false;

void gerenciarPtt() {
  static bool     ultimoEnviado = false;
  static uint32_t ultimoEnvioMs = 0;
  uint32_t agora = millis();

  if (!espnowOk) { pttRemoto = false; return; }

  if (botaoApertado != ultimoEnviado || agora - ultimoEnvioMs >= INTERVALO_PTT_MS) {
    ultimoEnviado = botaoApertado;
    ultimoEnvioMs = agora;
    struct_mensagem m = { REDE_ID, TIPO_PTT, (uint16_t)(botaoApertado ? 1 : 0), micros() };
    esp_now_send(MAC_PAR, (uint8_t *)&m, sizeof(m));
  }

  pttRemoto = pttRemotoRaw && (agora - ultimoPttRemotoMs < TIMEOUT_PTT_MS);
}

int estadoPtt() {
  if (botaoApertado && pttRemoto) return PTT_CONFLITO;
  if (botaoApertado)               return PTT_FALANDO;
  if (pttRemoto)                   return PTT_ESCUTANDO;
  return PTT_LIVRE;
}

char     msgTx[TXT_MAX + 1] = "";
bool     msgTxPendente = false;

char     msgExibida[TXT_MAX + 1] = "";
String   msgL1 = "", msgL2 = "";
uint32_t msgExibidaMs = 0;

bool msgAtiva() {
  return msgExibida[0] != '\0' && (millis() - msgExibidaMs < MSG_EXIBIR_MS);
}

void quebrarMensagem(const char *txt, String &l1, String &l2, int maxPx) {
  String resto = String(txt);

  int corte = resto.length();
  while (corte > 0 && display.getStringWidth(resto.substring(0, corte)) > maxPx) corte--;
  if (corte < (int)resto.length()) {
    int esp = resto.lastIndexOf(' ', corte);
    if (esp > 0) corte = esp;
  }
  l1 = resto.substring(0, corte);

  resto = resto.substring(corte);
  resto.trim();
  int c2 = resto.length();
  while (c2 > 0 && display.getStringWidth(resto.substring(0, c2)) > maxPx) c2--;
  l2 = resto.substring(0, c2);
}

void lerSerial() {
  static char buf[TXT_MAX + 1];
  static int  n = 0;
  static bool truncou = false;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (n > 0) {
        buf[n] = '\0';
        strcpy(msgTx, buf);
        msgTxPendente = true;
        Serial.printf("[MSG] pronta: \"%s\"%s -> segure o botao para enviar\n",
                      msgTx, truncou ? " (cortada em 40 caracteres)" : "");
      }
      n = 0;
      truncou = false;
    } else if (n < TXT_MAX) {
      buf[n++] = (c >= 32 && c <= 126) ? c : '?';
    } else {
      truncou = true;
    }
  }
}

void gerenciarMensagens() {
  static uint16_t idTxt = 0;
  static bool avisouConflito = false;
  static bool avisouSemLink  = false;

  if (msgRxPendente) {
    int est = estadoPtt();
    if (est == PTT_FALANDO || est == PTT_CONFLITO) {
      Serial.printf("[MSG] descartada (conflito): \"%s\"\n", msgRx);
    } else {
      strcpy(msgExibida, msgRx);
      msgExibidaMs = millis();
      display.setFont(ArialMT_Plain_10);
      quebrarMensagem(msgExibida, msgL1, msgL2, LARG_MSG_PX);
      Serial.printf("[MSG RX] %s\n", msgExibida);
    }
    msgRxPendente = false;
  }

  int est = estadoPtt();
  if (est != PTT_FALANDO) avisouSemLink = false;
  if (est != PTT_CONFLITO) avisouConflito = false;

  if (msgTxPendente) {
    if (est == PTT_FALANDO) {
      if (linkOk) {
        struct_texto t = {};
        t.rede = REDE_ID;
        t.tipo = TIPO_TXT;
        t.id   = ++idTxt;
        strncpy(t.texto, msgTx, TXT_MAX);
        esp_now_send(MAC_PAR, (uint8_t *)&t, sizeof(t));
        Serial.printf("[MSG TX] %s\n", msgTx);
        msgTxPendente = false;
      } else if (!avisouSemLink) {
        avisouSemLink = true;
        Serial.println("[MSG] sem link: mensagem retida ate o link voltar");
      }
    } else if (est == PTT_CONFLITO && !avisouConflito) {
      avisouConflito = true;
      Serial.println("[MSG] CONFLITO: mensagem retida ate o outro soltar o botao");
    }
  }
}

void zerarVU() {
  nivelVU = 0;
  picoVU  = 0;
  dbAtual = -99.0;
}

float lerTensaoBateria() {
  uint32_t soma = 0;
  for (int i = 0; i < 16; i++) soma += analogReadMilliVolts(BAT_PIN);
  float mv = soma / 16.0;
  return (mv * FATOR_DIVISOR * CAL_BATERIA) / 1000.0;
}

int calcularPorcentagem(float tensaoV) {
  int p = map((int)(tensaoV * 1000), BAT_MIN_MV, BAT_MAX_MV, 0, 100);
  return constrain(p, 0, 100);
}

void simularVU() {
  static uint32_t ultimaLeitura = 0;
  static uint32_t ultimoSorteio = 0;
  static float alvo = 0;
  uint32_t agora = millis();

  if (agora - ultimaLeitura < 30) return;
  ultimaLeitura = agora;

  if (!botaoApertado) { zerarVU(); return; }

  if (agora - ultimoSorteio >= 120) {
    ultimoSorteio = agora;
    alvo = random(30, 96);
  }
  nivelVU += (alvo - nivelVU) * 0.4;
  dbAtual = VU_DB_MIN + nivelVU * (VU_DB_MAX - VU_DB_MIN) / 100.0;

  if (nivelVU >= picoVU) {
    picoVU = nivelVU;
    picoMs = agora;
  } else if (agora - picoMs > 800) {
    picoVU = max(nivelVU, picoVU - 1.5f);
  }
}

void atualizarVU() {
  if (MODO_SIMULADO) { simularVU(); return; }
  if (!micOk) return;

  size_t bytes = i2s.readBytes((char *)bufAudio, sizeof(bufAudio));
  int n = bytes / sizeof(int32_t);

  if (!botaoApertado) { zerarVU(); return; }
  if (n <= 0) return;

  double soma = 0;
  for (int i = 0; i < n; i++) soma += (bufAudio[i] >> 8);
  double media = soma / n;

  double somaQuad = 0;
  for (int i = 0; i < n; i++) {
    double s = (bufAudio[i] >> 8) - media;
    somaQuad += s * s;
  }
  float rms = sqrt(somaQuad / n);

  dbAtual = 20.0 * log10((rms + 1.0) / 8388608.0);

  float alvo = (dbAtual - VU_DB_MIN) * 100.0 / (VU_DB_MAX - VU_DB_MIN);
  alvo = constrain(alvo, 0.0, 100.0);

  if (alvo > nivelVU) nivelVU = alvo;
  else                nivelVU = nivelVU * 0.85 + alvo * 0.15;

  uint32_t agora = millis();
  if (nivelVU >= picoVU) {
    picoVU = nivelVU;
    picoMs = agora;
  } else if (agora - picoMs > 800) {
    picoVU = max(nivelVU, picoVU - 1.5f);
  }
}

void desenharIconeMicrofone() {
  display.fillRect(4, 51, 3, 1);
  display.fillRect(3, 52, 5, 6);
  display.fillRect(4, 58, 3, 1);
  display.drawVerticalLine(1, 54, 5);
  display.drawVerticalLine(9, 54, 5);
  display.drawHorizontalLine(2, 59, 7);
  display.drawVerticalLine(5, 60, 2);
  display.drawHorizontalLine(3, 62, 5);
}

void desenharVU() {
  desenharIconeMicrofone();

  if (!botaoApertado || pttRemoto) return;

  if (!micOk) {
    display.setFont(ArialMT_Plain_10);
    display.drawString(16, 52, "nao iniciado");
    return;
  }

  int acesas = (int)(nivelVU * VU_BARRAS / 100.0 + 0.5);
  int pico   = (int)(picoVU  * VU_BARRAS / 100.0 + 0.5);

  for (int i = 0; i < VU_BARRAS; i++) {
    int h = 2 + (i * 9) / (VU_BARRAS - 1);
    int x = VU_X0 + i * (VU_LARG + VU_ESPACO);
    int y = 63 - h;

    if (i < acesas) {
      display.fillRect(x, y, VU_LARG, h);
    } else if (pico > acesas && i == pico - 1) {
      display.drawRect(x, y, VU_LARG, h);
    } else {
      display.drawHorizontalLine(x, 62, VU_LARG);
    }
  }

  display.setFont(ArialMT_Plain_10);
  display.drawString(92, 52, String((int)dbAtual));
}

String fmtContador(uint32_t n) {
  if (n < 1000)  return String(n);
  if (n < 99000) return String(n / 1000) + "k";
  return "99k";
}

void desenharContadores() {
  const int xi = 98;
  const int xt = 108;
  const int linhas[3] = {18, 29, 40};

  int y0 = linhas[0] + 2;
  display.drawVerticalLine(xi + 3, y0, 8);
  display.drawLine(xi, y0 + 3, xi + 3, y0);
  display.drawLine(xi + 6, y0 + 3, xi + 3, y0);
  display.drawString(xt, linhas[0], fmtContador(pingsEnviados));

  y0 = linhas[1] + 2;
  display.drawLine(xi, y0, xi + 6, y0 + 6);
  display.drawLine(xi + 6, y0, xi, y0 + 6);
  display.drawString(xt, linhas[1], fmtContador(falhasEnvio));

  y0 = linhas[2] + 2;
  display.drawVerticalLine(xi + 3, y0, 8);
  display.drawLine(xi, y0 + 4, xi + 3, y0 + 7);
  display.drawLine(xi + 6, y0 + 4, xi + 3, y0 + 7);
  display.drawString(xt, linhas[2], fmtContador(pongsRecebidos));
}

void desenharMensagem() {
  display.setColor(BLACK);
  display.fillRect(0, 17, 128, 23);
  display.setColor(WHITE);
  display.drawString(0, 18, ">");
  display.drawString(8, 18, msgL1);
  display.drawString(8, 29, msgL2);
}

void desenharEstadoPtt() {
  int est = estadoPtt();

  if (est == PTT_CONFLITO) {
    if ((millis() / 300) % 2) {
      display.fillRect(0, 40, 94, 11);
      display.setColor(BLACK);
    }
    display.drawString(0, 40, "!! CONFLITO !!");
    display.setColor(WHITE);
  } else if (est == PTT_FALANDO) {
    display.drawString(0, 40, "PTT: FALANDO");
  } else if (est == PTT_ESCUTANDO) {
    display.drawString(0, 40, "PTT: ESCUTANDO");
  } else {
    display.drawString(0, 40, "PTT: LIVRE");
  }
}

void desenharTela() {
  float v = lerTensaoBateria();
  int pct = calcularPorcentagem(v);

  display.clear();
  display.setFont(ArialMT_Plain_10);

  display.fillTriangle(5, 0, 8, 0, 4, 7);
  display.fillTriangle(4, 6, 7, 6, 1, 14);
  display.drawString(14, 2, String(v, 2) + "V");

  if (linkOk) {
    int r = rssiAtual;
    if (r > -90) display.fillRect(70, 10, 2, 4);
    if (r > -75) display.fillRect(74, 7, 2, 7);
    if (r > -60) display.fillRect(78, 4, 2, 10);
  } else {
    display.drawString(68, 2, "!");
  }

  display.drawRect(100, 4, 22, 10);
  display.fillRect(122, 7, 2, 4);
  int larg = map(pct, 0, 100, 0, 18);
  if (larg > 0) display.fillRect(102, 6, larg, 6);
  display.drawHorizontalLine(0, 16, 128);

  if (linkOk) {
    display.drawString(0, 18, "Link OK  " + String(rssiAtual) + " dBm");
    display.drawString(0, 29, "RTT: " + String(rttUs / 1000.0, 1) + " ms");
  } else {
    display.drawString(0, 18, "offline");
    int n = (millis() / 400) % 4;
    String pontos = "";
    for (int i = 0; i < n; i++) pontos += ".";
    display.drawString(0, 30, "Aguardando par" + pontos);
  }

  desenharEstadoPtt();
  if (msgAtiva() && estadoPtt() != PTT_CONFLITO) desenharMensagem();
  else                                           desenharContadores();
  desenharVU();
  display.display();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  analogSetPinAttenuation(BAT_PIN, ADC_11db);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 500)) delay(10);

  display.init();

  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) macCompleto += "0";
    macCompleto += String(mac[i], HEX);
    if (i < 5) macCompleto += ":";
  }
  macCompleto.toUpperCase();
  if (Serial) Serial.println("[MAC] Esta placa: " + macCompleto);

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "INICIALIZANDO...");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 24, "ENDERECO MAC:");
  display.drawString(0, 42, macCompleto);
  display.display();
  delay(5000);

  iniciarEspNow();

  pinMode(BOTAO_PIN, INPUT_PULLUP);

  if (MODO_SIMULADO) {
    micOk = true;
    if (Serial) Serial.printf("[MIC] MODO SIMULADO: aperte o botao no GPIO %d\n", BOTAO_PIN);
  } else {
    i2s.setPins(I2S_BCLK, I2S_WS, -1, I2S_DIN);
    micOk = i2s.begin(I2S_MODE_STD, AUDIO_RATE,
                      I2S_DATA_BIT_WIDTH_32BIT,
                      I2S_SLOT_MODE_MONO,
                      I2S_STD_SLOT_LEFT);
    if (Serial) Serial.println(micOk ? "[I2S] Microfone iniciado" : "[I2S] ERRO ao iniciar");
  }

  if (Serial) Serial.println("[MSG] Digite a mensagem no Serial Monitor (Nova linha) e segure o botao para enviar");
}

void loop() {
  static uint32_t ultimaTela = 0;
  uint32_t agora = millis();

  lerBotao();
  gerenciarEspNow();
  gerenciarPtt();
  lerSerial();
  gerenciarMensagens();
  atualizarVU();

  if (agora - ultimaTela >= INTERVALO_TELA_MS) {
    ultimaTela = agora;
    desenharTela();
  }
}

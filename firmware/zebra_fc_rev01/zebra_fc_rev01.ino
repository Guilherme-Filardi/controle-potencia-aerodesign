/*
   ZEBRA AERODESIGN - CONTROLADOR DE POTENCIA (rev01, sem placa)
   ===========================================================================
   Firmware completo: passthrough SBUS + telemetria S.Port + malha de controle
   de potencia (feedforward + PID). Limita a potencia eletrica em 575 W a
   100% de stick e mantem a curva stick->potencia invariante durante o voo,
   compensando a descarga da bateria.

   O QUE MUDOU DA rev00 PARA A rev01
   ---------------------------------
   A rev00 rodava com o modelo antigo da planta, P proporcional a V, que os
   proprios dados FULLBAT/HALFBAT rejeitaram. Com bateria plena o feedforward
   comandava cerca de 5 pontos percentuais a mais que o correto, entregando
   ~666 W contra 575 W de alvo: ~66 W acima do limite, ~33 pontos de
   penalidade, no instante exato em que o wattimetro registra o pico.

     - MODELO_K   34.01  -> 0.4620
     - MODELO_N   2.247  -> 2.388
     - feedforward mudou de ESTRUTURA (ver a funcao feedforward())
     - KP         0.02   -> 0.019   (K_planta passou de 15,5 para 16,0 W/%)
     - KI         0.10   -> 0.097   (mesma constante de tempo de projeto)
     - comentarios da telemetria e do R2 corrigidos
     - radio 2,4 GHz desligado explicitamente no boot

   A CURVA_REF_W nao mudou, e nao deveria mudar: ela vem dos dados medidos do
   ensaio FULLBAT comprimidos por 575/807, e nao do modelo.

   Esta e a versao SEM PLACA, para bancada com ligacao direta. Tres linhas de
   sinal. A versao para a placa impressa e outro arquivo.

   Hardware : ESP32 DevKit V1
   Lib SBUS : Bolder Flight Systems SBUS v8.1.4 (modificada por Guilherme Filardi)

   "IF TURTLES CAN BE NINJAS, ZEBRAS CAN ALSO FLY."

   Ligacoes (tres linhas independentes, um fio cada + GND comum):
     [1] Saida SBUS do Rx -> GPIO16 (Serial2 / UART2)   throttle entra
     [2] GPIO18 (PWM)     -> entrada Master do ESC      throttle sai
     [3] S.Port do ESC    -> GPIO4  (UART1)             telemetria entra (sniffer)

   ATENCAO: esta versao tem arming e aciona motor de verdade. O sistema so
   arma com o stick no minimo por 2 s continuos apos o link SBUS subir.
   Qualquer failsafe desarma e zera o controlador.
*/

#include "sbus.h"
#include <ESP32Servo.h>
#include <math.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_bt.h"

// ===========================================================================
//  GANHOS DO CONTROLADOR  >>> MEXER AQUI NA SINTONIA <<<
// ===========================================================================
// Valores de projeto derivados de K_planta = 16,0 W/% na regiao de operacao
// (Tabela 4.5 da monografia: 17,0 W/% com bateria plena, 14,9 W/% no fim do
// envelope; variacao de 13%, pequena o bastante para tratar como constante).
// Faixas de teste no plano de sintonia: KP 0,010-0,038 | KI 0,05-0,2 | KD 0-0,01.
//
//   KP = beta / K_planta, com beta = 0,3 (fracao do erro corrigida por acao)
//        0,3 / 16,0 = 0,019 %/W
//        verificacao: erro de 50 W -> correcao de 0,95% -> ~15 W, 30% do erro
//
//   KI mantem a constante de tempo de projeto da rev00: o produto Ki*K_planta
//        era 0,10 * 15,5 = 1,55 1/s (tau = 0,65 s). Com K_planta = 16,0:
//        1,55 / 16,0 = 0,097 %/(W*s)
//        verificacao: sob a perturbacao de 2,77 W/s medida no ensaio de
//        descarga, o erro residual em regime fica em 2,77/1,55 = 1,8 W,
//        0,31% do alvo de 575 W
float KP = 0.019f;   // ganho proporcional [%/W]
float KI = 0.097f;   // ganho integral     [%/(W*s)]
float KD = 0.00f;    // ganho derivativo   [%*s/W] - zerado por projeto;
                     // derivada da MEDICAO, filtrada (ver secao PID abaixo)

const float FC_DERIV_HZ  = 1.5f;    // corte do filtro da derivada [Hz]
const float INTEG_LIM    = 20.0f;   // teto do acumulador integral [+-% de throttle]

// ---- Modelo da planta (16 pontos, R2 = 0,99957, RMSE 4,8 W) ----
// P = MODELO_K * [ (thr/100) * V ]^MODELO_Q   [P em W, V em volts, thr em %]
//
// EXPOENTE UNICO aplicado ao PRODUTO (thr/100)*V, e nao expoentes separados
// para comando e tensao. Isso decorre da fisica: o ESC aplica V_ef = delta*V,
// o motor responde a V_ef^0,80 e a helice consome N^3,07. O motor nao
// distingue a origem da tensao efetiva, entao tudo a jusante depende do
// produto. A composicao 0,80 * 3,07 = 2,46 preve o expoente medido de 2,388
// por um caminho independente do ajuste.
//
// O modelo anterior, P = 34,01 * V * (thr/100)^2,247, tinha RMSE de 26,4 W e
// vies de sinais opostos entre os ensaios (-5,7% com bateria plena, +7,0% com
// bateria parcial), que e a assinatura de erro de estrutura, nao de ruido.
//
// Valido para comando >= 40%: residuos dentro de +-3%. Em 30% chega a -8% e
// abaixo de 20% cresce rapido, mas em valor absoluto fica abaixo de 5 W.
const float MODELO_K = 0.4620f;
const float MODELO_Q = 2.388f;

// ---- Curva de referencia stick -> potencia alvo ----
// Forma da curva natural do conjunto, comprimida pelo fator 575/807,1 =
// 0,71243 e ancorada em 575 W a 100% de stick. Indice = stick/10,
// interpolacao linear entre pontos.
//
// ATENCAO: esta tabela vem dos DADOS MEDIDOS do ensaio FULLBAT, patamar a
// patamar, e NAO do modelo. Corrigir o modelo nao a invalida. Ela so muda se
// o valor de ancoragem mudar, e ai basta reescalar tudo pelo novo fator.
//
// A alternativa linear foi rejeitada: atribuiria a primeira metade do curso
// do manche uma parcela de potencia muito acima da que o conjunto entrega
// naturalmente, deixando a resposta nervosa em baixo comando.
//
// >>> 575 W E PROVISORIO <<< A ancoragem definitiva depende da perda entre o
// wattimetro (ponto fiscalizado, limite de 600 W) e o ESC (ponto de medicao),
// que ainda nao foi medida. A estimativa atual e que o valor final fique abaixo
// de 570 W. Rodar bancada com 575 W e seguro: o teto fisico observado no
// ensaio de descarga foi 611 W.
const float P_MAX_W = 575.0f;
const float CURVA_REF_W[11] = {
//  0%   10%  20%  30%  40%  50%   60%   70%   80%   90%   100%
    0.0f, 8.0f, 25.0f, 55.0f, 93.0f, 150.0f, 224.0f, 306.0f, 404.0f, 511.0f, 575.0f
};

// Tensao assumida pelo feedforward enquanto nenhuma telemetria valida chegou
// (o ESC so transmite ~5 s apos energizar). 6S cheia = 25,2 V: assumir tensao
// alta e conservador, o throttle calculado sai menor.
const float V_PADRAO = 25.2f;

// ===========================================================================
//  CONFIGURACAO DE HARDWARE E TEMPORIZACAO (validada na V0.3 - nao mexer)
// ===========================================================================

// ---- Pinos (fixados pela placa) ----
const int pinRX  = 16;   // SBUS in
const int pinTX  = 17;   // sem uso, mas o driver SBUS exige um pino de TX
const int pinESC = 18;   // PWM out para a entrada master do ESC

// ---- SBUS / throttle ----
// Ordem de canais FrSky e AETR: throttle e o canal 3 -> indice [2].
const int CH_THROTTLE = 2;
// Extremos nominais do SBUS. Recalibrar contra o serial se o stick nao
// varrer o curso completo.
const int SBUS_MIN = 172, SBUS_MAX = 1811;
const int PWM_MIN  = 1000, PWM_MAX = 2000;   // faixa de pulso do ESC, em us

// ---- Temporizacao / seguranca ----
const unsigned long SBUS_TIMEOUT_MS   = 100;   // sem frame novo -> corta
const unsigned long TLM_STALE_MS      = 500;   // telemetria velha -> so FF
const unsigned long ARMAR_STICK_MS    = 2000;  // stick no minimo por 2 s p/ armar
const float         STICK_ARMADO_PCT  = 3.0f;  // "minimo" = abaixo disso
const int           ESC_PWM_HZ        = 200;   // Tribunus aceita bem acima de 50 Hz
const unsigned long CONTROL_PERIOD_US = 5000;  // tick fixo de controle, 200 Hz
const unsigned long DEBUG_PERIOD_MS   = 250;   // limita o serial a 4 Hz
#define DEBUG_BANCADA 1                        // 0 = silencia o serial p/ voo

// ---- Telemetria S.Port ----
// O Tribunus nao fala o S.Port classico da FrSky. O frame foi levantado por
// engenharia reversa e validado com multimetro:
//
//     10   50 0B   LL HH   II   XX   CRC   7E
//     |    |       |       |    |    |     |
//     tipo appID   tensao  corr alto crc   delimitador
//          0x0B50  (LE)    (1B)
//
//   tensao   = (HH<<8 | LL) / 100   ->  ex: 0x0622 = 1570 = 15,70 V
//   corrente = (XX<<8 | II) / 100   ->  ex: 0x0DD4 = 3540 = 35,40 A
//
// A corrente ocupa DOIS bytes, como a tensao: a comum de ~35 A no fim da
// varredura vale 3540 centiamps e nao caberia em um byte so.
//
// Sincronismo pela assinatura 10 50 0B, e nao pelo CRC - o CRC do Tribunus
// nao bate com o algoritmo padrao do S.Port e nao vale a briga.
#define SPORT_GPIO   4
#define SPORT_UART   UART_NUM_1        // UART2 ja esta com o SBUS
#define SPORT_BAUD   57600
#define SPORT_INVERT true              // S.Port e serial invertido, como o SBUS
const uint8_t SIG0 = 0x10, SIG1 = 0x50, SIG2 = 0x0B;
const float ESCALA_V = 0.01f;          // centivolts -> volts
const float ESCALA_I = 0.01f;          // centiamps  -> amps

// ===========================================================================
//  OBJETOS E ESTADO
// ===========================================================================
bfs::SbusRx  sbus(&Serial2, pinRX, pinTX, true);   // 'true' = inversor interno
bfs::SbusData sbusData;
Servo esc;

int  throttleRaw = SBUS_MIN;           // ultimo throttle valido, unidades SBUS
bool linkVivo    = false;
unsigned long ultimoFrameBomMs = 0, ultimoTickUs = 0, ultimoDebugMs = 0;

// Snapshot da telemetria, atualizado conforme os frames chegam.
float tlmV = 0, tlmI = 0, tlmP = 0;
bool  tlmValida = false;
unsigned long tlmUltimaMs = 0;

// Buffer circular do fluxo de bytes do S.Port.
uint8_t  sbuf[1024];
int      sbufLen = 0;

// ---- Estado do controlador ----
bool  armado = false;
unsigned long stickBaixoDesdeMs = 0;

float integrador = 0.0f;   // acumulador do termo I [% de throttle]
float dPfilt     = 0.0f;   // derivada filtrada de P_med [W/s]
float tlmPAnt    = 0.0f;   // P_med da amostra anterior (p/ derivada)
float thrOutAnt  = 0.0f;   // ultima saida [%] (p/ anti-windup por clamping)

// Espelho p/ debug.
float dbgPalvo = 0, dbgFF = 0, dbgUpid = 0;
int   dbgPwmOut = PWM_MIN;
const char* dbgEstado = "BOOT";

// ===========================================================================
//  TELEMETRIA
// ===========================================================================
// Sobe a UART1 como linha invertida, somente recepcao.
void telemetriaInicia() {
  uart_config_t cfg = {
    .baud_rate  = SPORT_BAUD,
    .data_bits  = UART_DATA_8_BITS,
    .parity     = UART_PARITY_DISABLE,
    .stop_bits  = UART_STOP_BITS_1,
    .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
  };
  uart_driver_install(SPORT_UART, 2048, 0, 0, NULL, 0);
  uart_param_config(SPORT_UART, &cfg);
  // RX puro: a gente so escuta o barramento Rx<->ESC, nunca transmite.
  uart_set_pin(SPORT_UART, UART_PIN_NO_CHANGE, SPORT_GPIO,
               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (SPORT_INVERT) uart_set_line_inverse(SPORT_UART, UART_SIGNAL_RXD_INV);
  else              uart_set_line_inverse(SPORT_UART, UART_SIGNAL_INV_DISABLE);
}

// Drena o que chegou e desliza pelo buffer procurando a assinatura do frame.
// Nao bloqueia (timeout 0), entao nunca atrasa o tick de controle.
// Quando fecha um frame valido, ja calcula a derivada de P p/ o termo D:
// so aqui existe dado novo de verdade (a telemetria renova a ~5 Hz, o tick
// roda a 200 Hz - derivar no tick geraria zero + espeta).
void telemetriaAtualiza() {
  int avail = uart_read_bytes(SPORT_UART, sbuf + sbufLen, sizeof(sbuf) - sbufLen, 0);
  if (avail > 0) sbufLen += avail;

  int i = 0;
  while (i + 6 < sbufLen) {
    if (sbuf[i] == SIG0 && sbuf[i + 1] == SIG1 && sbuf[i + 2] == SIG2) {
      uint16_t vRaw = sbuf[i + 3] | (sbuf[i + 4] << 8);
      uint16_t  iRaw = sbuf[i + 5]| (sbuf[i + 6] << 8);
      float v = vRaw * ESCALA_V;
      // Filtro de sanidade: tensao de pack plausivel confirma que estamos
      // alinhados num frame, e nao numa coincidencia de bytes.
      if (v > 1.0f && v < 60.0f) {
        float pNova = v * (iRaw * ESCALA_I);

        // Derivada da medicao (nao do erro - evita o chute derivativo a
        // cada movimento de stick), com passa-baixa de 1a ordem: a corrente
        // e ruidosa e derivada amplifica exatamente esse ruido.
        if (tlmValida) {
          float dtS = (millis() - tlmUltimaMs) / 1000.0f;
          if (dtS > 0.02f && dtS < 1.0f) {
            float dBruta = (pNova - tlmPAnt) / dtS;
            float tau    = 1.0f / (2.0f * PI * FC_DERIV_HZ);
            float alfa   = dtS / (dtS + tau);
            dPfilt += alfa * (dBruta - dPfilt);
          }
        }
        tlmPAnt = pNova;

        tlmV = v;
        tlmI = iRaw * ESCALA_I;
        tlmP = pNova;
        tlmValida  = true;
        tlmUltimaMs = millis();
      }
      i += 8;                 // frame consumido
    } else {
      i++;                    // ainda desalinhado, avanca um byte
    }
  }

  // Guarda a sobra (frame parcial) e compacta pro inicio do buffer.
  int rem = sbufLen - i;
  if (rem > 0 && i > 0) memmove(sbuf, sbuf + i, rem);
  sbufLen = (rem > 0) ? rem : 0;
  if (sbufLen > (int)sizeof(sbuf) - 16) sbufLen = 0;   // trava anti-overflow
}

bool telemetriaFresca() { return tlmValida && (millis() - tlmUltimaMs < TLM_STALE_MS); }

// ===========================================================================
//  LEI DE CONTROLE
// ===========================================================================

// Stick em % (0-100) a partir do valor bruto de SBUS.
float stickPct(int s) {
  s = constrain(s, SBUS_MIN, SBUS_MAX);
  return 100.0f * (float)(s - SBUS_MIN) / (float)(SBUS_MAX - SBUS_MIN);
}

// Curva de referencia: posicao do stick -> potencia alvo [W].
// Interpolacao linear entre os pontos da tabela (passo de 10%).
float curvaReferencia(float stick) {
  stick = constrain(stick, 0.0f, 100.0f);
  int   idx  = (int)(stick / 10.0f);
  if (idx >= 10) return CURVA_REF_W[10];
  float frac = (stick - idx * 10.0f) / 10.0f;
  return CURVA_REF_W[idx] + frac * (CURVA_REF_W[idx + 1] - CURVA_REF_W[idx]);
}

// Feedforward: planta invertida.
//
//     P = k * [ (thr/100) * V ]^q      ->      thr = (100/V) * (P/k)^(1/q)
//
// A INVERSAO MUDOU DE ESTRUTURA em relacao a rev00, que calculava
// 100 * (P/(k*V))^(1/n). Ali a tensao entrava dentro da raiz; aqui ela sai
// como fator direto, porque o expoente se aplica ao produto thr*V e nao a
// cada variavel separadamente. Usar a forma antiga com as constantes novas
// daria comando errado em toda a faixa.
//
// Com V medido em tempo real, este termo sozinho compensa a descarga da
// bateria: com a tensao caindo, o comando calculado sobe para a mesma
// potencia de referencia. O PID corrige apenas o residuo do modelo.
//
// Conferencia contra a Tabela 4.5 da monografia, com P_alvo = 575 W:
//     V = 24,8 V -> 79,7%   |   V = 23,0 V -> 86,0%   |   V = 21,5 V -> 92,0%
float feedforward(float pAlvo, float v) {
  if (pAlvo <= 0.5f || v < 5.0f) return 0.0f;
  float thr = (100.0f / v) * powf(pAlvo / MODELO_K, 1.0f / MODELO_Q);
  return constrain(thr, 0.0f, 100.0f);
}

// Zera o estado da malha (usado ao desarmar e no failsafe).
void controleReset() {
  integrador = 0.0f;
  dPfilt     = 0.0f;
  thrOutAnt  = 0.0f;
}

// Um passo da lei de controle. Recebe o stick em % e devolve throttle de
// saida em %. dt e o periodo do tick em segundos.
float controlePasso(float stick, float dt) {
  float pAlvo = curvaReferencia(stick);
  bool  malhaFechada = telemetriaFresca();

  // Tensao p/ o feedforward: medida se ha telemetria em dia; senao a ultima
  // valida; se nunca chegou nada, o padrao conservador.
  float vFF = tlmValida ? tlmV : V_PADRAO;
  float ff  = feedforward(pAlvo, vFF);

  float uPID;
  if (malhaFechada) {
    float e = pAlvo - tlmP;

    // Anti-windup por clamping: o integrador congela se a saida ja esta no
    // batente e o erro empurraria mais pra dentro da saturacao.
    bool satAlta  = (thrOutAnt >= 100.0f && e > 0.0f);
    bool satBaixa = (thrOutAnt <=   0.0f && e < 0.0f);
    if (!satAlta && !satBaixa) integrador += KI * e * dt;
    integrador = constrain(integrador, -INTEG_LIM, INTEG_LIM);

    // PID: P e I sobre o erro, D sobre a medicao (sinal negativo).
    uPID = KP * e + integrador - KD * dPfilt;
  } else {
    // Telemetria velha: malha suspensa, opera so o feedforward com o
    // integrador congelado no ultimo valor (secao 10.4 do relatorio).
    uPID = integrador;
  }

  float thrOut = constrain(ff + uPID, 0.0f, 100.0f);

  thrOutAnt = thrOut;
  dbgPalvo  = pAlvo;
  dbgFF     = ff;
  dbgUpid   = uPID;
  dbgEstado = malhaFechada ? "CTRL" : "CTRL-FF";
  return thrOut;
}

// Throttle em % -> largura de pulso p/ o ESC [us].
int pctParaPwm(float thr) {
  thr = constrain(thr, 0.0f, 100.0f);
  return PWM_MIN + (int)(thr * (PWM_MAX - PWM_MIN) / 100.0f + 0.5f);
}

// ===========================================================================
//  ARMING
// ===========================================================================
// So arma com o link vivo e o stick abaixo de STICK_ARMADO_PCT por
// ARMAR_STICK_MS continuos. Evita partida de motor com stick alto ao ligar.
void armingAtualiza(float stick) {
  if (armado) return;
  if (stick < STICK_ARMADO_PCT) {
    if (stickBaixoDesdeMs == 0) stickBaixoDesdeMs = millis();
    else if (millis() - stickBaixoDesdeMs >= ARMAR_STICK_MS) {
      armado = true;
      Serial.println(F(">>> ARMADO <<<"));
    }
  } else {
    stickBaixoDesdeMs = 0;   // stick saiu do minimo, recomeca a contagem
  }
}

void desarma(const char* motivo) {
  if (armado) { Serial.print(F(">>> DESARMADO: ")); Serial.println(motivo); }
  armado = false;
  stickBaixoDesdeMs = 0;
  controleReset();
}

// ===========================================================================
//  DEBUG DE BANCADA
// ===========================================================================
void debugBancada() {
#if DEBUG_BANCADA
  if (millis() - ultimoDebugMs < DEBUG_PERIOD_MS) return;
  ultimoDebugMs = millis();
  Serial.print(F("Est:"));      Serial.print(dbgEstado);
  Serial.print(F(" | stick:")); Serial.print(stickPct(throttleRaw), 1);
  Serial.print(F("% | Palvo:"));Serial.print(dbgPalvo, 0);
  Serial.print(F("W | FF:"));   Serial.print(dbgFF, 1);
  Serial.print(F("% | uPID:")); Serial.print(dbgUpid, 2);
  Serial.print(F("% | PWM:"));  Serial.print(dbgPwmOut);
  Serial.print(F(" || TLM "));
  if (telemetriaFresca()) {
    Serial.print(tlmV, 2); Serial.print(F("V "));
    Serial.print(tlmI, 2); Serial.print(F("A "));
    Serial.print(tlmP, 1); Serial.print(F("W"));
  } else {
    Serial.print(F("(sem dado)"));
  }
  Serial.println();
#endif
}

// ===========================================================================
//  SETUP / LOOP
// ===========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // Radio do ESP32 desligado de proposito: sao 2,4 GHz a centimetros do
  // receptor. Hoje o firmware nunca chama WiFi nem BT, entao ja estaria
  // desligado por omissao; o problema e isso depender de ninguem ligar por
  // acidente numa alteracao futura. Se ligar em voo, pode ensurdecer o
  // receptor e derrubar o enlace de comando.
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();

  sbus.Begin();          // UART2 -> SBUS
  telemetriaInicia();    // UART1 -> S.Port. Duas UARTs de hardware, sem conflito.

  esc.setPeriodHertz(ESC_PWM_HZ);        // taxa antes do attach
  esc.attach(pinESC, PWM_MIN, PWM_MAX);
  esc.writeMicroseconds(PWM_MIN);        // segura em idle no boot

  // Nasce "vencido" de proposito: o watchdog mantem o ESC em idle ate o
  // primeiro frame SBUS valido aparecer.
  ultimoFrameBomMs = 0;

  Serial.println(F("=== ZEBRA FC rev01 (sem placa) :: controle de potencia ==="));
  Serial.print(F("Modelo: P = "));   Serial.print(MODELO_K, 4);
  Serial.print(F(" * (thr/100 * V)^")); Serial.println(MODELO_Q, 3);
  Serial.print(F("Alvo a 100% de stick: ")); Serial.print(P_MAX_W, 0);
  Serial.println(F(" W (PROVISORIO, ancoragem por definir)"));
  Serial.print(F("Ganhos: KP ")); Serial.print(KP, 3);
  Serial.print(F(" | KI "));      Serial.print(KI, 3);
  Serial.print(F(" | KD "));      Serial.println(KD, 3);
  Serial.println(F("Arming: stick no minimo por 2 s com link OK"));
}

void loop() {
  // (A) SBUS: pega o throttle do piloto, mas so confia em frame limpo.
  if (sbus.Read()) {
    sbusData = sbus.data();
    if (!sbusData.failsafe && !sbusData.lost_frame) {
      throttleRaw      = sbusData.ch[CH_THROTTLE];
      ultimoFrameBomMs = millis();
    }
  }

  // Telemetria roda toda passada, independente do tick de controle.
  telemetriaAtualiza();

  // (B) Watchdog do link. Este e o failsafe de verdade - pega fio de SBUS
  // morto ou Rx mudo, que o flag de frame acima nao enxerga sozinho.
  linkVivo = (millis() - ultimoFrameBomMs) <= SBUS_TIMEOUT_MS;
  if (!linkVivo) {
    desarma("failsafe SBUS");
    esc.writeMicroseconds(PWM_MIN);
    dbgPwmOut = PWM_MIN;
    dbgEstado = "FAILSAFE";
    debugBancada();
    return;
  }

  // (C) Tick de controle em taxa fixa (200 Hz).
  unsigned long agoraUs = micros();
  if (agoraUs - ultimoTickUs < CONTROL_PERIOD_US) return;
  float dt = (ultimoTickUs == 0) ? (CONTROL_PERIOD_US * 1e-6f)
                                 : (agoraUs - ultimoTickUs) * 1e-6f;
  ultimoTickUs = agoraUs;

  float stick = stickPct(throttleRaw);
  armingAtualiza(stick);

  int pwmOut;
  if (!armado) {
    // Desarmado: motor parado, malha zerada, aguardando gesto de arming.
    controleReset();
    pwmOut    = PWM_MIN;
    dbgEstado = "DESARM";
  } else if (stick < STICK_ARMADO_PCT) {
    // Stick no minimo: motor em idle sem acionar a malha (a potencia alvo
    // aqui e ~0 e o modelo nao vale nessa regiao; integrador zerado evita
    // acumular lixo parado no chao).
    controleReset();
    pwmOut    = PWM_MIN;
    dbgEstado = "IDLE";
  } else {
    pwmOut = pctParaPwm(controlePasso(stick, dt));
  }

  esc.writeMicroseconds(pwmOut);
  dbgPwmOut = pwmOut;
  debugBancada();
}

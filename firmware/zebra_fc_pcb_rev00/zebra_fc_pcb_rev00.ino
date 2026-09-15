/*
   ZEBRA AERODESIGN - CONTROLADOR DE POTENCIA (PCB rev00)
   ===========================================================================
   Versao para a PLACA do controlador. Passthrough SBUS + telemetria S.Port
   + malha de controle de potencia (feedforward + PID), com os dois LEDs de
   diagnostico que a placa traz.

   Hardware : ESP32 DevKit V1 soquetado na placa do controlador
   Lib SBUS : Bolder Flight Systems SBUS v8.1.4 (modificada por Guilherme Filardi)

   "IF TURTLES CAN BE NINJAS, ZEBRAS CAN ALSO FLY."

   O QUE ESTA VERSAO TEM A MAIS QUE A zebra_fc_rev01
   -------------------------------------------------
   A rev01 e a correcao direta do firmware de bancada, com ligacao por fio
   solto e tres linhas de sinal. Esta aqui pressupoe a placa, e por isso:

     - aciona os dois LEDs (D1 verde ARMADO em D19, D2 ambar TLM em D21)
     - maquina de estados explicita, em vez de flags soltas
     - log em CSV para a bancada, com as grandezas que o plano de sintonia
       manda registrar em todas as fases
     - registra o pico de potencia desde o arme, que e o que o wattimetro
       da competicao mede
     - confere a coerencia dos parametros no boot e recusa arme se algo nao
       fecha, em vez de acionar motor com configuracao invalida

   O MODELO E OS GANHOS SAO OS MESMOS DA rev01. Se um mudar, o outro tem de
   mudar junto.

   LIGACOES NA PLACA rev01 (quatro chicotes, GND comum)
   ----------------------------------------------------
     J1  entrada  S.Port do receptor   3 fios: SIG + 5V + GND  <- unico com 5 V
     J2  entrada  SBUS do receptor     2 fios: SIG + GND
     J3  saida    MASTER do ESC        2 fios: SIG + GND
     J4  saida    S.Port do ESC        2 fios: SIG + GND

   O barramento S.Port ATRAVESSA a placa: entra por J1, sai por J4, sem nada
   em serie. O ESP32 so escuta, pela derivacao em T de R2. Consequencia que
   nao existia na montagem por fio solto: tirar a placa agora derruba a
   telemetria entre receptor e ESC, e o piloto perde a leitura no radio.

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
//   KI mantem a constante de tempo de projeto: o produto Ki*K_planta vale
//        1,55 1/s (tau = 0,65 s). Com K_planta = 16,0: 1,55/16,0 = 0,097
//        verificacao: sob a perturbacao de 2,77 W/s medida no ensaio de
//        descarga, o erro residual em regime fica em 1,8 W, 0,31% do alvo
float KP = 0.019f;   // ganho proporcional [%/W]
float KI = 0.097f;   // ganho integral     [%/(W*s)]
float KD = 0.00f;    // ganho derivativo   [%*s/W] - zerado por projeto;
                     // derivada da MEDICAO, filtrada (ver secao PID abaixo)

const float FC_DERIV_HZ = 1.5f;    // corte do filtro da derivada [Hz]
const float INTEG_LIM   = 20.0f;   // teto do acumulador integral [+-% de throttle]

// ---- Modelo da planta (16 pontos, R2 = 0,99957, RMSE 4,8 W) ----
// P = MODELO_K * [ (thr/100) * V ]^MODELO_Q   [P em W, V em volts, thr em %]
//
// EXPOENTE UNICO aplicado ao PRODUTO (thr/100)*V, e nao expoentes separados
// para comando e tensao. Decorre da fisica: o ESC aplica V_ef = delta*V, o
// motor responde a V_ef^0,80 e a helice consome N^3,07. O motor nao distingue
// a origem da tensao efetiva, entao tudo a jusante depende do produto. A
// composicao 0,80 * 3,07 = 2,46 preve o expoente medido de 2,388 por um
// caminho independente do ajuste.
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
// >>> 575 W E PROVISORIO <<< A ancoragem definitiva depende da perda entre o
// wattimetro (ponto fiscalizado, limite de 600 W) e o ESC (ponto de medicao),
// que ainda nao foi medida. O teto fisico observado no ensaio de descarga foi
// 611 W, com o comando em 95,5%.
const float P_MAX_W = 575.0f;
const float CURVA_REF_W[11] = {
//  0%   10%   20%    30%    40%    50%     60%     70%     80%     90%    100%
    0.0f, 8.0f, 25.0f, 55.0f, 93.0f, 150.0f, 224.0f, 306.0f, 404.0f, 511.0f, 575.0f
};

// Tensao assumida pelo feedforward enquanto nenhuma telemetria valida chegou
// (o ESC so transmite ~5 s apos energizar). 6S cheia = 25,2 V: assumir tensao
// alta e conservador, o throttle calculado sai menor.
const float V_PADRAO = 25.2f;

// Envelope operacional, do ensaio de descarga: 359 amostras a comando
// elevado, nenhuma abaixo de 21,50 V. Usado so na conferencia do boot.
const float V_ENVELOPE_MIN = 21.5f;

// ===========================================================================
//  CONFIGURACAO DE HARDWARE E TEMPORIZACAO (fixada pela placa rev01)
// ===========================================================================

// ---- Pinos ----
// D16, D17 e D4 nao sao escolha livre: sao os pinos nativos das UARTs e os
// que aceitam inversao por hardware. Trocar exigiria remapear pela matriz de
// GPIO, mexendo em infraestrutura ja validada. Os pinos de LED sao livres.
const int pinRX     = 16;   // SBUS in            (U2RX)
const int pinTX     = 17;   // sem uso, mas o driver SBUS exige um pino de TX
const int pinESC    = 18;   // PWM out para a entrada master do ESC (J3)
const int pinLedArm = 19;   // D1 verde  - ARMADO
const int pinLedTlm = 21;   // D2 ambar  - TELEMETRIA

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

// ---- Saida de diagnostico ----
//   0 = silencio total, para voo
//   1 = linha legivel, 4 Hz, para acompanhar na bancada
//   2 = CSV a 10 Hz, para registrar as fases do plano de sintonia
#define MODO_LOG 1
const unsigned long LOG_LEGIVEL_MS = 250;
const unsigned long LOG_CSV_MS     = 100;

// ---- Telemetria S.Port ----
// O Tribunus nao fala o S.Port classico da FrSky. O frame foi levantado por
// engenharia reversa e validado com multimetro:
//
//     10   50 0B   LL HH   II   XX   CRC   7E
//     |    |       |       |    |    |     |
//     tipo appID   tensao  corr alto crc   delimitador
//          0x0B50  (LE)    (LE, 2 bytes)
//
//   tensao   = (HH<<8 | LL) / 100   ->  ex: 0x0622 = 1570 = 15,70 V
//   corrente = (XX<<8 | II) / 100   ->  ex: 0x0DD4 = 3540 = 35,40 A
//
// A corrente ocupa DOIS bytes, como a tensao: a corrente de ~35 A no fim da
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
//  MAQUINA DE ESTADOS
// ===========================================================================
// A rev01 decidia o estado por uma cadeia de ifs e guardava o nome numa
// string solta so para o debug. Aqui o estado e uma variavel de verdade:
// quem decide o comportamento e quem acende o LED leem a mesma coisa.
enum Estado : uint8_t {
  EST_BOOT,        // ligou, ainda sem frame SBUS
  EST_SEM_LINK,    // failsafe: sem SBUS valido dentro do timeout
  EST_BLOQUEADO,   // conferencia do boot reprovou; nunca arma
  EST_DESARMADO,   // link vivo, aguardando o gesto de arme
  EST_IDLE,        // armado, stick no minimo, motor parado
  EST_FF,          // controlando so com feedforward (telemetria ausente)
  EST_MALHA        // controlando com a malha fechada
};

Estado estado = EST_BOOT;

const char* nomeEstado(Estado e) {
  switch (e) {
    case EST_BOOT:      return "BOOT";
    case EST_SEM_LINK:  return "FAILSAFE";
    case EST_BLOQUEADO: return "BLOQUEADO";
    case EST_DESARMADO: return "DESARM";
    case EST_IDLE:      return "IDLE";
    case EST_FF:        return "CTRL-FF";
    case EST_MALHA:     return "CTRL";
  }
  return "?";
}

// ===========================================================================
//  OBJETOS E ESTADO
// ===========================================================================
bfs::SbusRx   sbus(&Serial2, pinRX, pinTX, true);   // 'true' = inversor interno
bfs::SbusData sbusData;
Servo esc;

int  throttleRaw = SBUS_MIN;           // ultimo throttle valido, unidades SBUS
bool linkVivo    = false;
unsigned long ultimoFrameBomMs = 0, ultimoTickUs = 0, ultimoLogMs = 0;

// Snapshot da telemetria, atualizado conforme os frames chegam.
float tlmV = 0, tlmI = 0, tlmP = 0;
bool  tlmValida = false;
unsigned long tlmUltimaMs = 0;

// Buffer circular do fluxo de bytes do S.Port.
uint8_t sbuf[1024];
int     sbufLen = 0;

// ---- Estado do controlador ----
bool armado = false;
unsigned long stickBaixoDesdeMs = 0;

float integrador = 0.0f;   // acumulador do termo I [% de throttle]
float dPfilt     = 0.0f;   // derivada filtrada de P_med [W/s]
float tlmPAnt    = 0.0f;   // P_med da amostra anterior (p/ derivada)
float thrOutAnt  = 0.0f;   // ultima saida [%] (p/ anti-windup por clamping)

// Pico de potencia desde o arme. O wattimetro da competicao mede exatamente
// isso, com retencao de maximo, e e sobre ele que incide a penalidade.
float picoPotenciaW = 0.0f;

// Espelho p/ log.
float dbgPalvo = 0, dbgFF = 0, dbgUpid = 0, dbgThrOut = 0;
int   dbgPwmOut = PWM_MIN;

// ===========================================================================
//  LEDS
// ===========================================================================
// D1 verde  ARMADO : apagado desarmado | aceso armado | piscando em failsafe
// D2 ambar  TLM    : apagado sem dado  | aceso com dado fresco | piscando
//                    devagar quando o dado envelheceu
//
// A placa fica deitada no meio da aeronave e os LEDs sao vistos de cima. O
// que se quer enxergar de longe, antes de encostar no aviao, e se o sistema
// armou e se a telemetria esta chegando.
bool pisca(unsigned long periodoMs) {
  return (millis() / (periodoMs / 2)) % 2 == 0;
}

void ledsAtualiza() {
  bool arm, tlm;
  switch (estado) {
    case EST_SEM_LINK:  arm = pisca(200);  break;   // rapido: perdeu o radio
    case EST_BLOQUEADO: arm = pisca(1000); break;   // lento: configuracao ruim
    case EST_IDLE:
    case EST_FF:
    case EST_MALHA:     arm = true;        break;
    default:            arm = false;       break;
  }
  if (!tlmValida)                                tlm = false;
  else if (millis() - tlmUltimaMs < TLM_STALE_MS) tlm = true;
  else                                            tlm = pisca(600);

  digitalWrite(pinLedArm, arm ? HIGH : LOW);
  digitalWrite(pinLedTlm, tlm ? HIGH : LOW);
}

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
      uint16_t iRaw = sbuf[i + 5] | (sbuf[i + 6] << 8);
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
        tlmValida   = true;
        tlmUltimaMs = millis();

        if (armado && pNova > picoPotenciaW) picoPotenciaW = pNova;
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
  int idx = (int)(stick / 10.0f);
  if (idx >= 10) return CURVA_REF_W[10];
  float frac = (stick - idx * 10.0f) / 10.0f;
  return CURVA_REF_W[idx] + frac * (CURVA_REF_W[idx + 1] - CURVA_REF_W[idx]);
}

// Feedforward: planta invertida.
//
//     P = k * [ (thr/100) * V ]^q      ->      thr = (100/V) * (P/k)^(1/q)
//
// A tensao sai como fator direto, e nao dentro da raiz, porque o expoente se
// aplica ao produto thr*V e nao a cada variavel separadamente.
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
    // integrador congelado no ultimo valor.
    uPID = integrador;
  }

  float thrOut = constrain(ff + uPID, 0.0f, 100.0f);

  thrOutAnt = thrOut;
  dbgPalvo  = pAlvo;
  dbgFF     = ff;
  dbgUpid   = uPID;
  dbgThrOut = thrOut;
  estado    = malhaFechada ? EST_MALHA : EST_FF;
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
  if (armado || estado == EST_BLOQUEADO) return;
  if (stick < STICK_ARMADO_PCT) {
    if (stickBaixoDesdeMs == 0) stickBaixoDesdeMs = millis();
    else if (millis() - stickBaixoDesdeMs >= ARMAR_STICK_MS) {
      armado = true;
      picoPotenciaW = 0.0f;          // o pico conta a partir deste arme
#if MODO_LOG
      Serial.println(F(">>> ARMADO <<<"));
#endif
    }
  } else {
    stickBaixoDesdeMs = 0;   // stick saiu do minimo, recomeca a contagem
  }
}

void desarma(const char* motivo) {
#if MODO_LOG
  if (armado) {
    Serial.print(F(">>> DESARMADO: ")); Serial.print(motivo);
    Serial.print(F(" | pico desde o arme: ")); Serial.print(picoPotenciaW, 0);
    Serial.println(F(" W"));
  }
#else
  (void)motivo;
#endif
  armado = false;
  stickBaixoDesdeMs = 0;
  controleReset();
}

// ===========================================================================
//  CONFERENCIA DO BOOT
// ===========================================================================
// Roda uma vez, antes de qualquer arme. Se algo nao fecha, o firmware entra
// em EST_BLOQUEADO e nunca aciona o motor: e melhor nao voar do que voar com
// uma tabela editada pela metade.
bool conferenciaBoot() {
  bool ok = true;

  // A curva tem de ser crescente, senao mais stick daria menos potencia.
  for (int i = 0; i < 10; i++) {
    if (CURVA_REF_W[i] >= CURVA_REF_W[i + 1]) {
      Serial.print(F("FALHA: CURVA_REF_W nao e crescente no indice ")); Serial.println(i);
      ok = false;
    }
  }

  // O topo da curva tem de ser o proprio alvo, senao um dos dois foi editado
  // sozinho e o sistema mira num valor que ninguem escolheu.
  if (fabsf(CURVA_REF_W[10] - P_MAX_W) > 0.5f) {
    Serial.print(F("FALHA: CURVA_REF_W[10] = ")); Serial.print(CURVA_REF_W[10], 1);
    Serial.print(F(" W, mas P_MAX_W = "));        Serial.print(P_MAX_W, 1);
    Serial.println(F(" W"));
    ok = false;
  }

  // O alvo maximo tem de caber no curso do comando na pior tensao do
  // envelope. Se nao couber, o atuador satura no fim do voo e a malha perde
  // autoridade justamente quando mais precisa dela.
  float thrPior = feedforward(P_MAX_W, V_ENVELOPE_MIN);
  if (thrPior >= 99.0f) {
    Serial.print(F("FALHA: a "));  Serial.print(V_ENVELOPE_MIN, 1);
    Serial.print(F(" V o alvo de ")); Serial.print(P_MAX_W, 0);
    Serial.println(F(" W satura o comando"));
    ok = false;
  } else {
    Serial.print(F("Margem de comando no fim do envelope: "));
    Serial.print(100.0f - thrPior, 1); Serial.println(F(" pontos"));
  }

  return ok;
}

// ===========================================================================
//  LOG
// ===========================================================================
#if MODO_LOG == 2
void logCsvCabecalho() {
  Serial.println(F("t_ms,estado,stick_pct,P_alvo_W,P_med_W,V_V,I_A,"
                   "thr_ff_pct,u_pid_pct,thr_out_pct,pwm_us,pico_W"));
}
#endif

void logAtualiza() {
#if MODO_LOG == 1
  if (millis() - ultimoLogMs < LOG_LEGIVEL_MS) return;
  ultimoLogMs = millis();
  Serial.print(F("Est:"));       Serial.print(nomeEstado(estado));
  Serial.print(F(" | stick:"));  Serial.print(stickPct(throttleRaw), 1);
  Serial.print(F("% | Palvo:")); Serial.print(dbgPalvo, 0);
  Serial.print(F("W | FF:"));    Serial.print(dbgFF, 1);
  Serial.print(F("% | uPID:"));  Serial.print(dbgUpid, 2);
  Serial.print(F("% | PWM:"));   Serial.print(dbgPwmOut);
  Serial.print(F(" || TLM "));
  if (telemetriaFresca()) {
    Serial.print(tlmV, 2); Serial.print(F("V "));
    Serial.print(tlmI, 2); Serial.print(F("A "));
    Serial.print(tlmP, 1); Serial.print(F("W"));
  } else {
    Serial.print(F("(sem dado)"));
  }
  if (armado) { Serial.print(F(" | pico:")); Serial.print(picoPotenciaW, 0); Serial.print(F("W")); }
  Serial.println();

#elif MODO_LOG == 2
  if (millis() - ultimoLogMs < LOG_CSV_MS) return;
  ultimoLogMs = millis();
  Serial.print(millis());            Serial.print(',');
  Serial.print(nomeEstado(estado));  Serial.print(',');
  Serial.print(stickPct(throttleRaw), 2); Serial.print(',');
  Serial.print(dbgPalvo, 1);         Serial.print(',');
  Serial.print(telemetriaFresca() ? tlmP : NAN, 1); Serial.print(',');
  Serial.print(telemetriaFresca() ? tlmV : NAN, 2); Serial.print(',');
  Serial.print(telemetriaFresca() ? tlmI : NAN, 2); Serial.print(',');
  Serial.print(dbgFF, 2);            Serial.print(',');
  Serial.print(dbgUpid, 3);          Serial.print(',');
  Serial.print(dbgThrOut, 2);        Serial.print(',');
  Serial.print(dbgPwmOut);           Serial.print(',');
  Serial.println(picoPotenciaW, 1);
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

  pinMode(pinLedArm, OUTPUT);
  pinMode(pinLedTlm, OUTPUT);
  digitalWrite(pinLedArm, LOW);
  digitalWrite(pinLedTlm, LOW);

  sbus.Begin();          // UART2 -> SBUS
  telemetriaInicia();    // UART1 -> S.Port. Duas UARTs de hardware, sem conflito.

  esc.setPeriodHertz(ESC_PWM_HZ);        // taxa antes do attach
  esc.attach(pinESC, PWM_MIN, PWM_MAX);
  esc.writeMicroseconds(PWM_MIN);        // segura em idle no boot

  // Nasce "vencido" de proposito: o watchdog mantem o ESC em idle ate o
  // primeiro frame SBUS valido aparecer.
  ultimoFrameBomMs = 0;

  Serial.println(F("=== ZEBRA FC PCB rev00 :: controle de potencia ==="));
  Serial.print(F("Modelo: P = "));          Serial.print(MODELO_K, 4);
  Serial.print(F(" * (thr/100 * V)^"));     Serial.println(MODELO_Q, 3);
  Serial.print(F("Alvo a 100% de stick: ")); Serial.print(P_MAX_W, 0);
  Serial.println(F(" W (PROVISORIO, ancoragem por definir)"));
  Serial.print(F("Ganhos: KP ")); Serial.print(KP, 3);
  Serial.print(F(" | KI "));      Serial.print(KI, 3);
  Serial.print(F(" | KD "));      Serial.println(KD, 3);

  if (conferenciaBoot()) {
    estado = EST_BOOT;
    Serial.println(F("Conferencia OK. Arming: stick no minimo por 2 s com link OK"));
#if MODO_LOG == 2
    logCsvCabecalho();
#endif
  } else {
    estado = EST_BLOQUEADO;
    Serial.println(F("*** BLOQUEADO: conferencia do boot reprovou, motor nao arma ***"));
  }
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

  // Configuracao reprovada no boot: motor parado, e so isso.
  if (estado == EST_BLOQUEADO) {
    esc.writeMicroseconds(PWM_MIN);
    dbgPwmOut = PWM_MIN;
    ledsAtualiza();
    logAtualiza();
    return;
  }

  // (B) Watchdog do link. Este e o failsafe de verdade - pega fio de SBUS
  // morto ou Rx mudo, que o flag de frame acima nao enxerga sozinho.
  linkVivo = (millis() - ultimoFrameBomMs) <= SBUS_TIMEOUT_MS;
  if (!linkVivo) {
    desarma("failsafe SBUS");
    esc.writeMicroseconds(PWM_MIN);
    dbgPwmOut = PWM_MIN;
    estado    = EST_SEM_LINK;
    ledsAtualiza();
    logAtualiza();
    return;
  }

  // (C) Tick de controle em taxa fixa (200 Hz).
  unsigned long agoraUs = micros();
  if (agoraUs - ultimoTickUs < CONTROL_PERIOD_US) { ledsAtualiza(); return; }
  float dt = (ultimoTickUs == 0) ? (CONTROL_PERIOD_US * 1e-6f)
                                 : (agoraUs - ultimoTickUs) * 1e-6f;
  ultimoTickUs = agoraUs;

  float stick = stickPct(throttleRaw);
  armingAtualiza(stick);

  int pwmOut;
  if (!armado) {
    // Desarmado: motor parado, malha zerada, aguardando gesto de arming.
    controleReset();
    pwmOut = PWM_MIN;
    estado = EST_DESARMADO;
  } else if (stick < STICK_ARMADO_PCT) {
    // Stick no minimo: motor em idle sem acionar a malha (a potencia alvo
    // aqui e ~0 e o modelo nao vale nessa regiao; integrador zerado evita
    // acumular lixo parado no chao).
    controleReset();
    pwmOut = PWM_MIN;
    estado = EST_IDLE;
  } else {
    // controlePasso decide entre EST_MALHA e EST_FF conforme a telemetria.
    pwmOut = pctParaPwm(controlePasso(stick, dt));
  }

  esc.writeMicroseconds(pwmOut);
  dbgPwmOut = pwmOut;
  ledsAtualiza();
  logAtualiza();
}

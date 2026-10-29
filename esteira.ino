 /* ===========================================================================
  SEPARADOR DE PEÇAS POR COR - VERSÃO CORRIGIDA  (Arduino Uno / Nano)
  ===========================================================================

  O QUE FOI CORRIGIDO EM RELAÇÃO AO CÓDIGO ORIGINAL
  -------------------------------------------------
  1) BOTÃO DESLIGA (D2): agora usa interrupção externa (INT0) + um "clique
     pendente". O clique fica guardado até o loop() poder tratá-lo, então ele
     NUNCA se perde, mesmo durante a calibração, durante o giro do servo ou
     no meio de uma leitura do sensor. A rotina antiga só reconhecia o botão
     se ele ainda estivesse pressionado no instante exato em que o loop()
     passava por ali -> cliques curtos eram ignorados.

  2) CALIBRAÇÃO NÃO-BLOQUEANTE: antes eram 20 x delay(30) = ~600 ms com o
     loop() totalmente parado. Qualquer botão pressionado nesse intervalo era
     perdido. Agora a calibração coleta uma amostra por volta do loop().

  3) LEDs EXPLÍCITOS: verde no D3 e vermelho no D4, com lógica clara:
        sistema LIGADO    -> verde ACESO,  vermelho APAGADO
        sistema DESLIGADO -> verde APAGADO, vermelho ACESO
     Ao energizar o Arduino os DOIS piscam 2 vezes (teste de ligação).
     Se quiser os dois acesos direto enquanto ligado, mude
     DOIS_LEDS_QUANDO_LIGADO para true.

  4) NÍVEL DO MOTOR CONFIGURÁVEL: se você usa módulo relé de acionamento
     invertido (liga com LOW), basta trocar MOTOR_LIGADO para LOW. Isso é a
     causa mais comum de "o botão desliga não funciona": o LED/LCD desligam,
     mas o motor continua girando.

  5) Diagnóstico dos botões no Monitor Serial (115200): mostra o nível de
     D8 / D2 / D7 na partida e a cada clique. Se D2 já aparecer em 0 com o
     botão solto, o fio está em curto com o GND ou no pino errado.

  LIGAÇÃO DOS BOTÕES: um lado no pino, o outro no GND (usa INPUT_PULLUP).
  ===========================================================================
*/

#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_TCS34725.h"

// ------------------------------------------------------------------ PINOS
const byte BOTAO_LIGA    = 8;
const byte BOTAO_DESLIGA = 3;   // D2 = INT0 no Uno/Nano (interrupção externa)
const byte BOTAO_RESETA  = 7;

const byte LED_VERDE     = 5;
const byte LED_VERMELHO  = 4;

const byte MOTOR         = 6;
const byte SERVO_AZUL    = 10;
const byte SERVO_VERDE   = 9;

// ------------------------------------------- NÍVEIS LÓGICOS DA SUA MONTAGEM
// Botões entre pino e GND com INPUT_PULLUP: pressionado = LOW.
const byte BOTAO_PRESSIONADO = LOW;
const byte BOTAO_SOLTO       = HIGH;

// >>> ATENÇÃO <<<  Se o motor NÃO para ao pressionar DESLIGA, troque para LOW.
// (Módulos relé costumam ser ativo-baixo: LOW = ligado, HIGH = desligado.)
const byte MOTOR_LIGADO    = HIGH;
const byte MOTOR_DESLIGADO = (MOTOR_LIGADO == HIGH ? LOW : HIGH);

// LEDs com resistor para o GND acendem com HIGH.
// Se os seus LEDs estão ligados ao 5 V (acendem com LOW), troque para LOW.
const byte LED_ACESO   = HIGH;
const byte LED_APAGADO = (LED_ACESO == HIGH ? LOW : HIGH);

// false = verde indica LIGADO e vermelho indica DESLIGADO (recomendado).
// true  = os DOIS LEDs ficam acesos enquanto o sistema estiver LIGADO.
const bool DOIS_LEDS_QUANDO_LIGADO = false;

// --------------------------------------------------------------- OBJETOS
// LCD: se não aparecer nada, troque 0x27 por 0x3F.
LiquidCrystal_I2C lcd(0x27, 16, 2);

Adafruit_TCS34725 tcs(
  TCS34725_INTEGRATIONTIME_24MS,
  TCS34725_GAIN_16X
);

Servo servoAzul;
Servo servoVerde;

// -------------------------------------------------------------- CONSTANTES
const int REPOUSO_AZUL  = 90;
const int REPOUSO_VERDE = 65;
const int DESVIO_AZUL   = 180;
const int DESVIO_VERDE  = 0;

const unsigned long TEMPO_DESVIO_AZUL_MS   = 7000;
const unsigned long TEMPO_DESVIO_VERDE_MS  = 7000;
const unsigned long ESTABILIZACAO_SERVO_MS = 300;
const unsigned long ANTIRREPIQUE_MS        = 50;
const unsigned long TEMPO_MENSAGEM_MS      = 1500;
const unsigned long INTERVALO_AMOSTRA_MS   = 30;

const byte AMOSTRAS_CALIBRACAO       = 20;
const byte LEITURAS_BASE_PARA_REARMAR = 4;
const uint16_t LIMIAR_CLEAR_MINIMO   = 5;

const float   DELTA_COR            = 0.10;
const float   DELTA_CLEAR_PCT      = 0.15;
const uint16_t DELTA_CLEAR_MINIMO  = 20;
const float   MARGEM_DOMINANCIA    = 1.12;

// ----------------------------------------------------------- ESTADO GLOBAL
unsigned long contB = 0, contR = 0, contG = 0;

bool ligado            = false;
bool sensorDisponivel  = false;
bool calibrado         = false;
bool calibrando        = false;
bool servoAtivo        = false;
bool prontoParaContar  = false;

byte leiturasBase = 0;

uint16_t clearBase = 0;
float rBase = 0, gBase = 0, bBase = 0;

unsigned long inicioDesvio     = 0;
unsigned long tempoDesvioAtual = 0;
unsigned long fimMensagem      = 0;
unsigned long fimEstabilizacao = 0;

// Acumuladores da calibração (agora não-bloqueante)
byte amostrasCalibracao  = 0;
byte amostrasValidas     = 0;
uint32_t somaClearCal    = 0;
float somaRCal = 0, somaGCal = 0, somaBCal = 0;
unsigned long proximaAmostraCal = 0;

// --------------------------------------------------------------- BOTÕES
// Estrutura com "latch": a intenção de clique é memorizada e consumida depois.
struct Botao {
  byte pino;
  bool ultimaLeitura;
  unsigned long ultimaMudanca;
  bool pressionadoConfirmado;
  volatile bool cliquePendente;
};

Botao botaoLiga    = { BOTAO_LIGA,    BOTAO_SOLTO, 0, false, false };
Botao botaoDesliga = { BOTAO_DESLIGA, BOTAO_SOLTO, 0, false, false };
Botao botaoReseta  = { BOTAO_RESETA,  BOTAO_SOLTO, 0, false, false };

Botao* const botoes[] = { &botaoLiga, &botaoDesliga, &botaoReseta };
const byte NUM_BOTOES = sizeof(botoes) / sizeof(botoes[0]);

volatile unsigned long ultimaBordaIsrDesliga = 0;

// Interrupção do botão DESLIGA: pega até um toque de 1 ms.
void isrBotaoDesliga() {
  unsigned long agora = millis();
  if (ultimaBordaIsrDesliga == 0 ||
      agora - ultimaBordaIsrDesliga >= ANTIRREPIQUE_MS) {
    ultimaBordaIsrDesliga = agora;
    botaoDesliga.cliquePendente = true;
  }
}

// Varredura complementar (funciona também em placas sem INT0 no D2).
void varreduraBotoes() {
  unsigned long agora = millis();

  for (byte i = 0; i < NUM_BOTOES; i++) {
    Botao &b = *botoes[i];
    bool leitura = digitalRead(b.pino);

    if (leitura != b.ultimaLeitura) {
      b.ultimaLeitura = leitura;
      b.ultimaMudanca = agora;
      continue;                       // espera o nível estabilizar
    }

    if (agora - b.ultimaMudanca < ANTIRREPIQUE_MS) continue;

    if (leitura == BOTAO_PRESSIONADO) {
      if (!b.pressionadoConfirmado) {
        b.pressionadoConfirmado = true;
        b.cliquePendente = true;      // <-- latch: não perde o clique
      }
    } else {
      b.pressionadoConfirmado = false; // soltou: rearma para o próximo clique
    }
  }
}

bool consumirClique(Botao &b) {
  bool clique;
  noInterrupts();
  clique = b.cliquePendente;
  b.cliquePendente = false;
  interrupts();
  return clique;
}

void imprimirEstadoBotoes(const char* evento) {
  Serial.print(F("[botoes] "));
  Serial.print(evento);
  Serial.print(F(" | D8(LIGA)="));
  Serial.print(digitalRead(BOTAO_LIGA));
  Serial.print(F(" D2(DESLIGA)="));
  Serial.print(digitalRead(BOTAO_DESLIGA));
  Serial.print(F(" D7(RESETA)="));
  Serial.println(digitalRead(BOTAO_RESETA));
}

// ------------------------------------------------------------------- LCD
void mostrarContadores() {
  char linha[17];
  snprintf(linha, sizeof(linha), "B:%lu R:%lu G:%lu", contB, contR, contG);
  lcd.setCursor(0, 0);
  lcd.print("                ");   // limpa a linha
  lcd.setCursor(0, 0);
  lcd.print(linha);
}

void mostrarEstado(const char* mensagem) {
  lcd.setCursor(0, 1);
  lcd.print("                ");   // limpa a linha toda (16 colunas)
  lcd.setCursor(0, 1);
  lcd.print(mensagem);
}

void mostrarMensagemTemporaria(const char* mensagem) {
  mostrarEstado(mensagem);
  fimMensagem = millis() + TEMPO_MENSAGEM_MS;
}

void atualizarEstadoDepoisDaMensagem() {
  if (fimMensagem == 0 || (long)(millis() - fimMensagem) < 0) return;

  fimMensagem = 0;

  if (!sensorDisponivel) {
    mostrarEstado("Erro no sensor");
  } else if (calibrando) {
    mostrarEstado("Calibrando base");
  } else if (ligado) {
    mostrarEstado("Sistema ligado");
  } else {
    mostrarEstado("Sist. desligado");
  }
}

// ------------------------------------------------------------------ LEDs
void atualizarLeds() {
  if (DOIS_LEDS_QUANDO_LIGADO && ligado) {
    digitalWrite(LED_VERDE,    LED_ACESO);
    digitalWrite(LED_VERMELHO, LED_ACESO);
    return;
  }

  // Verde (D3) = sistema LIGADO | Vermelho (D4) = sistema DESLIGADO
  digitalWrite(LED_VERDE,    ligado ? LED_ACESO   : LED_APAGADO);
  digitalWrite(LED_VERMELHO, ligado ? LED_APAGADO : LED_ACESO);
}

void testarLedsNaPartida() {
  for (byte i = 0; i < 2; i++) {
    digitalWrite(LED_VERDE,    LED_ACESO);
    digitalWrite(LED_VERMELHO, LED_ACESO);
    delay(150);
    digitalWrite(LED_VERDE,    LED_APAGADO);
    digitalWrite(LED_VERMELHO, LED_APAGADO);
    delay(150);
  }
  atualizarLeds();   // deixa o estado coerente: vermelho aceso (desligado)
}

// --------------------------------------------------------------- SERVIÇOS
void repousarServos() {
  servoAzul.write(REPOUSO_AZUL);
  servoVerde.write(REPOUSO_VERDE);

  servoAtivo = fal+
  tempoDesvioAtual = 0;
  fimEstabilizacao = millis() + ESTABILIZACAO_SERVO_MS;
}

void iniciarCalibracao() {
  calibrando = true;
  calibrado  = false;
  prontoParaContar = false;
  leiturasBase = 0;

  amostrasCalibracao = 0;
  amostrasValidas    = 0;
  somaClearCal = 0;
  somaRCal = somaGCal = somaBCal = 0;
  proximaAmostraCal = millis() + INTERVALO_AMOSTRA_MS;

  mostrarEstado("Calibrando base");
  Serial.println(F("Calibrando a base vazia (mantenha a esteira vazia)..."));
}

void finalizarCalibracao() {
  calibrando = false;

  if (amostrasValidas == 0) {
    calibrado = false;
    mostrarMensagemTemporaria("Erro calibracao");
    Serial.println(F("ERRO: nenhuma amostra valida na calibracao."));
    return;
  }

  rBase     = somaRCal / amostrasValidas;
  gBase     = somaGCal / amostrasValidas;
  bBase     = somaBCal / amostrasValidas;
  clearBase = (uint16_t)(somaClearCal / amostrasValidas);
  calibrado = true;

  prontoParaContar = false;
  leiturasBase = 0;
  fimEstabilizacao = millis() + ESTABILIZACAO_SERVO_MS;

  Serial.println(F("Base calibrada:"));
  Serial.print(F("Clear base: ")); Serial.println(clearBase);
  Serial.print(F("RGB base: "));
  Serial.print(rBase, 3); Serial.print(F(","));
  Serial.print(gBase, 3); Serial.print(F(","));
  Serial.println(bBase, 3);

  mostrarMensagemTemporaria("Base calibrada");
}

// Uma amostra por chamada: NÃO bloqueia o loop().
void atualizarCalibracao() {
  if (!calibrando || (long)(millis() - proximaAmostraCal) < 0) return;

  proximaAmostraCal = millis() + INTERVALO_AMOSTRA_MS;

  uint16_t r, g, b, claro;
  tcs.getRawData(&r, &g, &b, &claro);

  uint32_t somaRGB = (uint32_t)r + g + b;
  if (somaRGB > 0) {
    somaRCal += (float)r / somaRGB;
    somaGCal += (float)g / somaRGB;
    somaBCal += (float)b / somaRGB;
    somaClearCal += claro;
    amostrasValidas++;
  }

  amostrasCalibracao++;
  if (amostrasCalibracao >= AMOSTRAS_CALIBRACAO) finalizarCalibracao();
}

// ------------------------------------------------------- AÇÕES DOS BOTÕES
void ligarSistema() {
  if (ligado) {
    mostrarMensagemTemporaria("Ja esta ligado");
    return;
  }

  ligado = true;
  digitalWrite(MOTOR, MOTOR_LIGADO);
  atualizarLeds();
  mostrarContadores();

  if (sensorDisponivel) {
    iniciarCalibracao();     // esteira vazia embaixo do sensor!
  } else {
    mostrarMensagemTemporaria("Erro no sensor");
  }
}

void desligarSistema() {
  ligado = false;

  // A linha abaixo é a que efetivamente PARA o motor.
  digitalWrite(MOTOR, MOTOR_DESLIGADO);

  calibrando = false;        // aborta calibração em andamento

  servoAzul.write(REPOUSO_AZUL);
  servoVerde.write(REPOUSO_VERDE);
  servoAtivo = false;
  tempoDesvioAtual = 0;

  prontoParaContar = false;
  leiturasBase = 0;
  fimEstabilizacao = millis() + ESTABILIZACAO_SERVO_MS;

  atualizarLeds();
  mostrarMensagemTemporaria("Sist. desligado");

  Serial.println(F(">>> SISTEMA DESLIGADO: motor parado."));
}

void resetarContadores() {
  contB = 0;
  contR = 0;
  contG = 0;

  prontoParaContar = false;
  leiturasBase = 0;

  mostrarContadores();
  mostrarMensagemTemporaria("Sistema resetado");
  Serial.println(F("Contadores zerados."));
}

// ------------------------------------------------------------- DETECÇÃO
char identificarCor(uint16_t r, uint16_t g, uint16_t b, uint16_t claro) {
  if (claro < LIMIAR_CLEAR_MINIMO) return '?';

  uint32_t somaRGB = (uint32_t)r + g + b;
  if (somaRGB == 0) return '?';

  float rAtual = (float)r / somaRGB;
  float gAtual = (float)g / somaRGB;
  float bAtual = (float)b / somaRGB;

  float rCorrigido = rAtual - rBase;
  float gCorrigido = gAtual - gBase;
  float bCorrigido = bAtual - bBase;

  if (rCorrigido < 0) rCorrigido = 0;
  if (gCorrigido < 0) gCorrigido = 0;
  if (bCorrigido < 0) bCorrigido = 0;

  float maior = rCorrigido;
  if (gCorrigido > maior) maior = gCorrigido;
  if (bCorrigido > maior) maior = bCorrigido;

  if (maior < DELTA_COR) return '?';

  if (rCorrigido >= gCorrigido * MARGEM_DOMINANCIA &&
      rCorrigido >= bCorrigido * MARGEM_DOMINANCIA) return 'R';

  if (gCorrigido >= rCorrigido * MARGEM_DOMINANCIA &&
      gCorrigido >= bCorrigido * MARGEM_DOMINANCIA) return 'G';

  if (bCorrigido >= rCorrigido * MARGEM_DOMINANCIA &&
      bCorrigido >= gCorrigido * MARGEM_DOMINANCIA) return 'B';

  return '?';
}

bool detectarObjeto(uint16_t r, uint16_t g, uint16_t b, uint16_t claro) {
  if (!calibrado) return false;

  uint32_t somaRGB = (uint32_t)r + g + b;
  if (somaRGB == 0 || claro < LIMIAR_CLEAR_MINIMO) return false;

  float rAtual = (float)r / somaRGB;
  float gAtual = (float)g / somaRGB;
  float bAtual = (float)b / somaRGB;

  float d;
  float diferencaCor = 0;
  d = rAtual - rBase; diferencaCor += (d < 0 ? -d : d);
  d = gAtual - gBase; diferencaCor += (d < 0 ? -d : d);
  d = bAtual - bBase; diferencaCor += (d < 0 ? -d : d);

  long diferencaClear = (long)claro - (long)clearBase;
  if (diferencaClear < 0) diferencaClear = -diferencaClear;

  uint16_t limiteClear = (uint16_t)(clearBase * DELTA_CLEAR_PCT);
  if (limiteClear < DELTA_CLEAR_MINIMO) limiteClear = DELTA_CLEAR_MINIMO;

  return (diferencaCor >= DELTA_COR) ||
         ((unsigned long)diferencaClear >= limiteClear);
}

void processarCor(char cor) {
  if (cor == 'R') {
    contR++;
    Serial.println(F("Objeto vermelho contado"));
  } else if (cor == 'B') {
    contB++;
    servoVerde.write(REPOUSO_VERDE);
    servoAzul.write(DESVIO_AZUL);
    servoAtivo = true;
    inicioDesvio = millis();
    tempoDesvioAtual = TEMPO_DESVIO_AZUL_MS;
    Serial.println(F("Objeto azul contado"));
  } else if (cor == 'G') {
    contG++;
    servoAzul.write(REPOUSO_AZUL);
    servoVerde.write(DESVIO_VERDE);
    servoAtivo = true;
    inicioDesvio = millis();
    tempoDesvioAtual = TEMPO_DESVIO_VERDE_MS;
    Serial.println(F("Objeto verde contado"));
  }

  mostrarContadores();
}

// ------------------------------------------------------------------ SETUP
void setup() {
  Serial.begin(115200);

  pinMode(BOTAO_LIGA,    INPUT_PULLUP);
  pinMode(BOTAO_DESLIGA, INPUT_PULLUP);
  pinMode(BOTAO_RESETA,  INPUT_PULLUP);

  pinMode(LED_VERDE,    OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  // Escreve o nível antes de habilitar a saída: evita pulo no motor na partida.
  digitalWrite(MOTOR, MOTOR_DESLIGADO);
  pinMode(MOTOR, OUTPUT);
  digitalWrite(MOTOR, MOTOR_DESLIGADO);

  servoAzul.attach(SERVO_AZUL);
  servoVerde.attach(SERVO_VERDE);
  servoAzul.write(REPOUSO_AZUL);
  servoVerde.write(REPOUSO_VERDE);

  ligado = false;
  atualizarLeds();
  testarLedsNaPartida();       // verde D3 e vermelho D4 piscam 2x ao energizar

  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  mostrarContadores();
  mostrarEstado("Sist. desligado");

  sensorDisponivel = tcs.begin();
  if (!sensorDisponivel) {
    mostrarEstado("Erro no sensor");
    Serial.println(F("TCS34725 nao encontrado. Confira SDA=A4, SCL=A5, 3V3/GND."));
  } else {
    Serial.println(F("Sensor OK. Aguardando botao LIGA (D8)."));
    Serial.println(F("Ao ligar, deixe a esteira VAZIA sob o sensor."));
  }

  // Interrupção dedicada ao botão DESLIGA: é o que garante o desligamento.
  const int irqDesliga = digitalPinToInterrupt(BOTAO_DESLIGA);
  if (irqDesliga != NOT_AN_INTERRUPT) {
    attachInterrupt(irqDesliga, isrBotaoDesliga,
                    (BOTAO_PRESSIONADO == LOW) ? FALLING : RISING);
    Serial.println(F("Interrupcao do botao DESLIGA (D2) ativada."));
  } else {
    Serial.println(F("AVISO: D2 sem interrupcao nesta placa; so varredura."));
  }

  imprimirEstadoBotoes("partida");
  Serial.println(F("Dica: 1=solto, 0=pressionado. Pressione cada botao e confira."));
}

// ------------------------------------------------------------------- LOOP
void loop() {
  // Botões são lidos SEMPRE, no início de toda volta, sem exceção.
  varreduraBotoes();

  bool cliqueLiga    = consumirClique(botaoLiga);
  bool cliqueDesliga = consumirClique(botaoDesliga);
  bool cliqueReseta  = consumirClique(botaoReseta);

  if (cliqueLiga) {
    Serial.println(F("Clique: LIGA"));
    imprimirEstadoBotoes("LIGA");
    ligarSistema();
  }

  if (cliqueDesliga) {
    Serial.println(F("Clique: DESLIGA"));
    imprimirEstadoBotoes("DESLIGA");
    // vem depois do LIGA -> DESLIGA sempre vence.
    // O "if" evita repetir a rotina quando a interrupção E a varredura
    // registrarem o mesmo toque (o que é normal e inofensivo).
    if (ligado || calibrando) desligarSistema();
  }

  if (cliqueReseta) {
    Serial.println(F("Clique: RESETA"));
    imprimirEstadoBotoes("RESETA");
    resetarContadores();
  }

  atualizarEstadoDepoisDaMensagem();

  if (!ligado || !sensorDisponivel) return;

  if (calibrando) {
    atualizarCalibracao();   // 1 amostra por volta, sem travar o loop
    return;
  }

  if (!calibrado) return;

  if (servoAtivo) {
    if (millis() - inicioDesvio >= tempoDesvioAtual) repousarServos();
    return;
  }

  if ((long)(millis() - fimEstabilizacao) < 0) return;

  uint16_t r, g, b, claro;
  tcs.getRawData(&r, &g, &b, &claro);

  bool objetoPresente = detectarObjeto(r, g, b, claro);

  if (!objetoPresente) {
    if (leiturasBase < LEITURAS_BASE_PARA_REARMAR) leiturasBase++;
    if (leiturasBase >= LEITURAS_BASE_PARA_REARMAR) prontoParaContar = true;
    return;
  }

  leiturasBase = 0;
  if (!prontoParaContar) return;

  char cor = identificarCor(r, g, b, claro);

  Serial.print(F("R,G,B,Clear: "));
  Serial.print(r); Serial.print(F(","));
  Serial.print(g); Serial.print(F(","));
  Serial.print(b); Serial.print(F(","));
  Serial.print(claro);
  Serial.print(F(" | objeto=sim | cor="));
  Serial.println(cor);

  if (cor != '?') {
    processarCor(cor);
    prontoParaContar = false;
  }
}

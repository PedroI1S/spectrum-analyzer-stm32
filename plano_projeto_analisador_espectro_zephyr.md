# Plano de Projeto — Analisador de Espectro de Áudio em Tempo Real

**Disciplina:** Sistemas de Tempo Real (Trabalho Final — Opção 1)
**RTOS:** Zephyr
**Plataforma:** STM32F4 Discovery (STM32F407VG)
**Data:** Junho/2026

---

## 1. Visão geral

O sistema captura áudio por um microfone digital I2S, calcula a FFT do sinal em tempo real e exibe o espectro em um display OLED, funcionando de forma autônoma (sem PC). Um console/shell permite inspecionar o estado do sistema e das tarefas de tempo real. O fluxo de dados é:

```
INMP441 (I2S) ──DMA──> buffer ──> Janela + FFT (CMSIS-DSP) ──> magnitudes ──> Display OLED
```

A aplicação demonstra explicitamente uma tarefa **hard real-time** (aquisição + FFT, com taxa de amostragem sem jitter) e uma tarefa **soft real-time** (atualização da tela). Todas as tarefas usam objetos de sincronização do Zephyr — **sem polling** no código de aplicação.

---

## 2. Hardware (Bill of Materials)

| Item | Especificação | Função |
|---|---|---|
| Placa | STM32F4 Discovery (STM32F407VG, Cortex-M4F @ até 168 MHz, 128 KiB RAM utilizável / 1 MiB Flash) | Processamento |
| Microfone | INMP441 — MEMS I2S, 24 bits, 3,3 V, resposta 60 Hz–15 kHz, SNR 61 dBA | Aquisição (hard RT) |
| Display | OLED 0,96" 128×64 I²C (controlador SSD1306) | Visualização (soft RT) |
| Adaptador serial | USB↔Serial 3,3 V (FTDI/CP2102) **ou** uso de USB CDC ACM | Console/shell — ver §3 |
| Diversos | Jumpers Dupont, protoboard | Conexões |

**Periféricos já na placa (sem custo):** LED de usuário (LD4–LD6), botão azul de usuário (B1), ST-LINK embutido para gravar/debugar.

**Por que não usar o microfone PDM embarcado (MP45DT02):** ele entrega PDM, que exige um filtro de decimação PDM→PCM não disponível pronto no Zephyr para STM32. O INMP441 entrega PCM por I2S diretamente, eliminando esse trabalho.

---

## 3. Conexões e pinout

### 3.1 INMP441 → STM32F4 (I2S em modo master-receiver)

O INMP441 é **escravo** no barramento; o STM32 é **mestre** e gera SCK e WS. O microfone só transmite (RX).

| Pino INMP441 | Sinal I2S | STM32F4 (proposta — **I2S2**) | Observação |
|---|---|---|---|
| VDD | — | 3V3 | Máx. 3,63 V |
| GND | — | GND | |
| SCK | Bit clock | I2S2_CK (ex.: PB13) | Gerado pelo STM32 |
| WS | Word select (LR clock) | I2S2_WS (ex.: PB12) | Gerado pelo STM32 |
| SD | Dados (PCM) | I2S2_SD / data-in (ex.: PB15) | Entrada para o STM32 |
| L/R | Seleção de canal | GND | GND = canal esquerdo |

> **Verificar antes de soldar/conectar:** na Discovery, o I2S2 e o I2S3 têm pinos parcialmente ligados ao mic PDM embarcado (I2S2: PB10/PC3) e ao DAC CS43L22 (I2S3). Confirme no esquemático da placa e no arquivo de pinctrl do F407 (`stm32f407v(e-g)tx-pinctrl.dtsi`) quais pinos do I2S2 estão livres nos headers e não colidem. A coluna "proposta" acima é um ponto de partida, não definitiva.

### 3.2 OLED SSD1306 → STM32F4 (I²C)

| Pino OLED | STM32F4 (I²C1) | Observação |
|---|---|---|
| VCC | 3V3 | |
| GND | GND | |
| SCL | I2C1_SCL (PB6) | Endereço típico do SSD1306: 0x3C |
| SDA | I2C1_SDA (PB7 ou PB9) | Barramento pode ser compartilhado com o codec CS43L22 (0x4A) |

### 3.3 Console / shell

O Virtual COM Port do ST-LINK **não** está ligado à serial do chip na Discovery. Três opções:

- **Opção A (simples):** adaptador USB↔Serial 3,3 V nos pinos da UART2 (PA2 = TX, PA3 = RX), 115200 8N1.
- **Opção B (USB):** shell sobre **USB CDC ACM** pela porta USB OTG da placa. Custa habilitar o stack USB no Zephyr.
- **Opção C — SEGGER RTT (adotada, sem hardware extra):** o console roda por **RTT sobre o próprio ST-LINK/SWD**, usando o cabo mini-USB que já grava/depura. Zero hardware adicional e mais simples que a Opção B. Habilitado no `app/prj.conf` (`CONFIG_USE_SEGGER_RTT=y`, `CONFIG_RTT_CONSOLE=y`, `CONFIG_UART_CONSOLE=n`) e visualizado com o script `scripts/rtt-console.sh`. Ver §15.

---

## 4. Arquitetura de software

### 4.1 Tarefas (threads) e prioridades

| Tarefa | Classe RT | Prioridade | Disparo | Responsabilidade |
|---|---|---|---|---|
| `acq_fft` | **Hard** | Mais alta | Semáforo do callback de DMA | Lê buffer I2S, aplica janela, calcula FFT e magnitudes |
| `display` | **Soft** | Média | Message queue / semáforo vindos de `acq_fft` | Desenha o último espectro disponível no OLED |
| `blink` | Referência | Baixa | `k_timer` (período fixo) | Pisca LED — referência visual de temporização |
| `button` | Evento | — (work queue) | IRQ de GPIO → work item | Troca o modo de exibição |
| `shell` | Soft | Baixa | Subsistema shell (orientado a evento) | Comandos de inspeção do sistema |

### 4.2 Fluxo sem polling

1. O periférico I2S, via **DMA em duplo buffer (ping-pong)**, preenche metade do buffer e dispara um callback na ISR.
2. A ISR libera um **semáforo** (`k_sem_give`). A tarefa `acq_fft` está bloqueada em `k_sem_take` — acorda só quando há dado novo.
3. `acq_fft` processa o bloco e publica o resultado em uma **message queue** (`k_msgq`) ou em um buffer duplo protegido por semáforo.
4. A tarefa `display` está bloqueada esperando a fila; acorda, pega o frame mais recente e desenha. Se atrasar, nenhum dado de áudio é perdido (o DMA continua) — por isso é **soft**.
5. O botão gera **interrupção de GPIO**; a ISR enfileira um **work item** (`k_work`) que troca o modo. Nada de ler o botão em laço.

---

## 5. Mapeamento hard / soft real-time (ligação com o enunciado)

- **Hard real-time — Aquisição do microfone + FFT.** O requisito "duro" é a **taxa de amostragem constante e sem jitter**: o relógio do I2S é gerado por hardware e o DMA move as amostras sem intervenção da CPU. Cada bloco de N amostras tem um *deadline* — a FFT precisa terminar antes do próximo bloco encher, senão há *overrun*. Isso é medido e exibido (ver §10).
- **Soft real-time — Atualização da tela.** Atrasos de alguns milissegundos na tela são imperceptíveis e não corrompem dados. A tarefa apenas desenha o frame mais recente.

> O enunciado pede, para a Opção 1, **pelo menos** uma tarefa hard e uma soft, mais LED, botão e shell. Esta arquitetura cobre todos. (A regra de "duas tarefas soft" vale só para quem usa FreeRTOS — não é o nosso caso.)

---

## 6. Parâmetros de DSP (proposta inicial)

| Parâmetro | Valor proposto | Justificativa |
|---|---|---|
| Taxa de amostragem (Fs) | 16 kHz | Cobre a faixa útil do INMP441 (até 15 kHz); Nyquist em 8 kHz |
| Tamanho da FFT (N) | 512 pontos | Resolução de 31,25 Hz; frame de 32 ms (~31 fps no display) |
| Formato da amostra | 24 bits em slot de 32 bits | INMP441 entrega 24 bits; ler 32 e deslocar |
| Janela | Hann | Reduz vazamento espectral (*leakage*) — picos mais limpos |
| Eixo de magnitude | Logarítmico (dB) | Áudio é logarítmico; espectro fica legível e "profissional" |
| Agrupamento de bins | 32–64 barras | O display tem 128 px; agrupar (média/máx), de preferência em bandas log |
| Bin 0 (DC) | Descartar | Remove o offset, que senão domina visualmente |

**Funções CMSIS-DSP:** `arm_rfft_fast_init_f32()` (uma vez), `arm_rfft_fast_f32()` (por bloco), `arm_cmplx_mag_f32()` para magnitudes; janela Hann pré-calculada e aplicada com `arm_mult_f32()`.

**Orçamento de RAM (N=512, float):** entrada ~2 KB + saída ~2 KB + magnitudes ~1 KB + duplo buffer do DMA ~4 KB ≈ **~9 KB**, folgado nos 128 KiB do F407. Há margem para subir a N=1024 (resolução 15,6 Hz) se quiserem.

**Frequência dominante (prova de correção):** índice do maior bin → `f = índice × Fs / N`. Tocar um tom de 1 kHz deve mostrar ~1 kHz na tela — verificável na hora, sem PC.

---

## 7. Configuração Zephyr

### 7.1 Módulo CMSIS-DSP (não vem por padrão)

```sh
west config manifest.project-filter -- +cmsis-dsp
west update cmsis-dsp
```

### 7.2 `prj.conf` (Kconfig)

```conf
# FPU e DSP
CONFIG_FPU=y
CONFIG_CMSIS_DSP=y
CONFIG_CMSIS_DSP_TRANSFORM=y
CONFIG_CMSIS_DSP_COMPLEXMATH=y

# Aquisição
CONFIG_I2S=y
CONFIG_DMA=y

# Display
CONFIG_DISPLAY=y
CONFIG_SSD1306=y
CONFIG_I2C=y

# GPIO (LED + botão)
CONFIG_GPIO=y

# Shell + inspeção de tarefas (não intrusivo)
CONFIG_SHELL=y
CONFIG_KERNEL_SHELL=y
CONFIG_THREAD_NAME=y
CONFIG_THREAD_MONITOR=y
CONFIG_THREAD_RUNTIME_STATS=y
CONFIG_INIT_STACKS=y
CONFIG_THREAD_STACK_INFO=y

# (Opção B do console) shell sobre USB CDC ACM
# CONFIG_USB_DEVICE_STACK=y
# CONFIG_USB_CDC_ACM=y
```

### 7.3 Overlay de devicetree (`boards/stm32f4_disco.overlay`) — esqueleto

```dts
/* Clock do domínio I2S — ajustar mul-n/div-r para a Fs desejada */
&plli2s {
    mul-n = <192>;
    div-r = <2>;
    clocks = <&clk_hse>;
    status = "okay";
};

/* I2S2 como entrada do microfone (confirmar pinos no esquemático) */
&i2s2 {
    status = "okay";
    pinctrl-0 = <&i2s2_ck_pb13 &i2s2_ws_pb12 &i2s2_sd_pb15>;
    pinctrl-names = "default";
    /* dmas = <&dma1 ... (canal RX do I2S2)>; dma-names = "rx"; */
};

/* OLED no I2C1 */
&i2c1 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>;

    ssd1306: ssd1306@3c {
        compatible = "solomon,ssd1306fb";
        reg = <0x3c>;
        width = <128>;
        height = <64>;
        /* demais propriedades conforme o binding do SSD1306 */
    };
};

/ {
    aliases {
        i2s-mic = &i2s2;
    };
    chosen {
        zephyr,display = &ssd1306;
    };
};
```

> Os valores de `plli2s` (`mul-n`, `div-r`) determinam a frequência de relógio do I2S e, portanto, a Fs efetiva. **Calcular esses valores para a Fs alvo é a etapa mais delicada** — ver risco R1.

---

## 8. As três telas do display (ciclo pelo botão)

1. **Espectro (principal):** barras de magnitude em dB, com *peak-hold* (tracinho que segura o máximo e cai devagar). Cabeçalho com a **frequência dominante em Hz** + nível RMS.
2. **Forma de onda:** o sinal no domínio do tempo, antes da FFT — contrasta com o espectro e mostra os dois domínios.
3. **Tempo real (diferencial):** Fs medida, tempo de cálculo da FFT (µs), fps do display, contagem de *overruns*. É o complemento on-device do comando de shell.

A tarefa de display sempre desenha o **último frame disponível**; se um frame for perdido, não há corrupção (comportamento soft RT).

---

## 9. Comandos de shell

Além dos comandos nativos do Zephyr (`kernel threads`, `kernel stacks`, `kernel uptime`), criar um comando customizado com `SHELL_CMD_REGISTER`:

- `rt status` — Fs medida, N, tempo médio/máx da FFT, fps, *overruns*, frequência dominante atual.
- `rt mode <spectrum|wave|stats>` — força o modo de exibição (espelha o botão).

> **Não intrusivo:** a impressão usa o backend do shell (thread própria) e lê variáveis atualizadas pelas tarefas; não bloqueia nem interfere no caminho hard RT.

---

## 10. Riscos e mitigações

| ID | Risco | Impacto | Mitigação |
|---|---|---|---|
| R1 | Configuração do clock I2S (PLLI2S) — erro "Could not configure I2S domain clock" | Alto — bloqueia a aquisição | Calcular `mul-n`/`div-r` para a Fs alvo; validar com osciloscópio em SCK/WS antes do software completo |
| R2 | Conflito de pinos do I2S2/I2C1 com periféricos embarcados (mic PDM, codec) | Médio | Conferir esquemático; escolher pinos/headers livres; barramento I²C pode ser compartilhado por endereço |
| R3 | Suporte do driver I2S no F4 dentro do Zephyr | Médio | Validar cedo com leitura crua de amostras; consultar discussões da comunidade Zephyr sobre I2S em STM32F4 |
| R4 | Banda do I²C limita o frame rate do OLED | Baixo | Mirar 20–30 fps; atualização parcial; I²C em modo fast (400 kHz) |
| R5 | FFT não termina antes do próximo bloco (overrun) | Médio | Medir tempo da FFT; ajustar N/Fs; prioridade alta para `acq_fft`; contar e exibir overruns |
| R6 | Console exige hardware extra na Discovery | Baixo | Adaptador USB-Serial (UART2) **ou** USB CDC ACM |

---

## 11. Cronograma por fases (marcos verificáveis)

Ajustar as semanas ao prazo real da disciplina.

| Fase | Entrega | Critério de "pronto" |
|---|---|---|
| F0 — Bring-up | Projeto Zephyr compila e `blinky` roda; shell responde | **✅ concluído:** compila/grava, LED pisca (`k_timer`), botão troca modo (IRQ→work), shell por RTT responde (`kernel thread list`, `kernel stacks`, `rt status`, `rt mode`). Esqueleto RT do §4 implementado e validado. Ver §15 |
| F1 — Display | OLED desenha texto/figuras de teste | "Hello" + barras de teste aparecem no OLED |
| F2 — Aquisição I2S | INMP441 lê amostras via DMA | Amostras coerentes (silêncio ≈ 0; tom → senoide); SCK/WS no osciloscópio |
| F3 — DSP | FFT + magnitudes + janela | Tom de 1 kHz → pico no bin correto; frequência dominante correta |
| F4 — Integração | Espectro ao vivo no OLED; tarefas hard/soft separadas | Espectro responde ao som em tempo real |
| F5 — Interação | Botão troca telas; LED como referência; comandos `rt …` | Três telas funcionam; shell mostra métricas RT |
| F6 — Robustez & doc | Medição de jitter/overruns; README; vídeo de demo | Sem overruns em operação normal; documentação pronta |

---

## 12. Divisão de trabalho (sugestão — adaptar ao grupo)

- **Pessoa A — Aquisição/DSP (hard RT):** overlay do I2S, DMA, clock PLLI2S, FFT/janela/magnitudes, métricas de tempo.
- **Pessoa B — Display (soft RT):** driver SSD1306, as três telas, renderização do espectro, peak-hold.
- **Pessoa C — Sistema/Integração:** shell e comandos `rt`, botão (work queue), LED, integração e medição de overruns, documentação.

Pares revisam o código um do outro; integração contínua a partir da F4.

---

## 13. Checklist de requisitos do enunciado

| Requisito | Entrega no projeto | ✔ |
|---|---|---|
| ≥ 1 tarefa hard real-time | `acq_fft` (aquisição + FFT) | ☐ |
| ≥ 1 tarefa soft real-time | `display` (atualização do OLED) | ☐ |
| Tarefa de console/shell com info do sistema | Shell + `kernel threads/stacks` + `rt status` | ☐ |
| Comando para info das tarefas de tempo real | `rt status` (Fs, tempo FFT, overruns, fps) | ☐ |
| Impressão no console **não intrusiva** | Thread do shell + leitura de variáveis | ☐ |
| Tarefa para piscar LED | `blink` com `k_timer` | ☐ |
| Tarefa acionada pelo botão da placa | IRQ GPIO → work queue → troca de tela | ☐ |
| Sem polling no código de aplicação | Semáforos, msgq, work queue, timers | ☐ |
| RTOS diferente do visto em aula (não FreeRTOS) | Zephyr | ☑ |

---

## 14. Referências de configuração a consultar

- Documentação do board `stm32f4_disco` (Zephyr) — pinos, console, features suportadas.
- Binding `st,stm32-i2s` e driver `i2s_stm32.c` (Zephyr) — propriedades do nó I2S.
- Binding `solomon,ssd1306fb` (Zephyr) — propriedades do display.
- Módulo CMSIS-DSP (Zephyr) — funções `arm_rfft_fast_f32`, `arm_cmplx_mag_f32`.
- Discussões da comunidade Zephyr sobre I2S em STM32F4 (clock PLLI2S, pinctrl).

> Observação: pinos exatos, valores de PLLI2S e propriedades de binding devem ser confirmados contra a versão do Zephyr em uso e o esquemático da Discovery antes da implementação.

---

## 15. Estado atual da implementação (atualizado em 27/06/2026)

Bring-up (F0) realizado no macOS (Apple Silicon). O ambiente e a comunicação com a placa estão funcionando ponta a ponta.

### 15.1 O que já foi feito

- **Ambiente de desenvolvimento montado:** venv Python em `.venv/`, `west` 1.5, árvore do Zephyr (`main`) + módulos `cmsis_6`, `hal_stm32`, `hal_nordic`, `segger`, e toolchain `arm-zephyr-eabi` (Zephyr SDK). Ferramentas de host via Homebrew: `openocd`, `stlink`, `dtc`.
- **Comunicação com a placa estabelecida** via ST-LINK/V2 embutido + OpenOCD: STM32F407VG detectado (Cortex-M4, 1 MiB flash / 192 KiB RAM).
- **`blinky` gravado e validado** (LED de usuário piscando) — confirma a cadeia compilar → gravar → executar.
- **Console por SEGGER RTT** (Opção C da §3.3) configurado, dispensando adaptador USB-serial. Script utilitário `scripts/rtt-console.sh` abre o terminal.
- **`app/` compila e roda na `stm32f4_disco`** usando os aliases `led0`/`sw0` da placa (o overlay `app/boards/stm32f4_disco.overlay` ficou como placeholder documentado para os periféricos das fases seguintes).
- **Esqueleto de tempo real (§4) implementado e validado na placa** (`app/src/main.c`):
  - `acq_fft` (hard RT, prioridade 2) bloqueia num **semáforo** dado por um `k_timer` que simula o callback de meia-transferência do DMA do I2S (ritmo de bloco ~32 ms); processa e publica um frame numa **message queue**;
  - `display` (soft RT, prioridade 7) bloqueia na message queue e desenha o último frame (sem OLED, imprime um resumo do espectro no console);
  - `blink` por `k_timer` (LED 1 Hz) e `button` por **IRQ de GPIO → work queue** (troca o modo de exibição);
  - **shell por RTT** com `kernel thread list` / `kernel stacks` e os comandos customizados `rt status` (Fs, N, tempo de FFT, fps, overruns, freq. dominante) e `rt mode <spectrum|wave|stats>`.
  - Sem polling no código de aplicação (semáforo, msgq, work queue, timers). Aquisição/FFT/display rodam com **espectro sintético** — os pontos reais estão marcados com `TODO(Fx)`. Medido na placa: fps≈32, FFT(stub)≈5 µs, 0 overruns.

### 15.2 Decisões/ajustes em relação ao plano original

- **Console + shell:** adotado **RTT sobre ST-LINK** (§3.3, Opção C) no lugar do adaptador USB-serial (Opção A) ou USB CDC ACM (Opção B). Sem hardware extra; resolve o risco **R6**. O shell roda pelo backend RTT (`CONFIG_SHELL_BACKEND_RTT`) e os logs saem pelo mesmo canal — terminal interativo via `scripts/rtt-console.sh`.
- **Módulo `segger`** adicionado à allowlist do `west.yml` (necessário para o RTT).
- **Comando para listar threads:** neste Zephyr (`main`) é `kernel thread list` (e não `kernel threads`). O down-buffer do RTT foi aumentado para 64 (`CONFIG_SEGGER_RTT_BUFFER_SIZE_DOWN`) para não truncar linhas longas coladas no terminal.

### 15.3 Como construir, gravar e ver o console

```sh
source .venv/bin/activate            # ativa o ambiente
cd ..                                # topdir do workspace west (pasta Embarcados/)
west build -b stm32f4_disco Zephyr-Project/app
west flash -d build --runner openocd # runner padrao tenta STM32CubeProgrammer; force openocd
./Zephyr-Project/scripts/rtt-console.sh   # abre o terminal RTT (porta 9090)
```

> Apenas um OpenOCD pode usar o ST-LINK por vez — encerre o servidor RTT antes de gravar.

### 15.4 Pendências imediatas (próximos passos)

- ✅ **F0 concluído** (shell por RTT + esqueleto RT validado — ver §15.1).
- **F1 — Display:** adicionar o SSD1306 (I2C1, addr 0x3C) ao `app/boards/stm32f4_disco.overlay`, `CONFIG_DISPLAY`/`CONFIG_SSD1306`/`CONFIG_I2C`, `chosen { zephyr,display }`, e trocar o stub do `display_thread` por desenho real (CFB) — começar com um "hello"/barras de teste.
- **F2 — Aquisição:** nós de I2S do INMP441 + clock PLLI2S + canal de DMA RX no overlay (riscos R1–R3); trocar o `k_timer` que dá o semáforo pelo callback real do DMA no `acq_fft`.
- **F3 — DSP:** habilitar CMSIS-DSP (`west config manifest.project-filter -- +cmsis-dsp`) e trocar o espectro sintético por janela de Hann + `arm_rfft_fast_f32` + `arm_cmplx_mag_f32`.

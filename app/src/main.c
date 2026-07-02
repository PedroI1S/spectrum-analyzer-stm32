/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Analisador de espectro de audio - esqueleto de tempo real (fase F0).
 *
 * Implementa a arquitetura de tarefas do plano (secao 4), com toda a
 * sincronizacao por objetos do Zephyr (sem polling no codigo de aplicacao):
 *
 *   acq_fft  (hard RT) - acorda por semaforo, processa um bloco e publica o
 *                        espectro numa message queue. O semaforo e dado, por
 *                        ora, por um k_timer que simula o callback de meia
 *                        transferencia do DMA do I2S (substituir na fase F2).
 *   display  (soft RT) - bloqueia na message queue e "desenha" o ultimo frame.
 *                        Sem o OLED, imprime um resumo no console (fase F1/F4
 *                        trocam isso pelo driver SSD1306).
 *   blink              - k_timer pisca o LED de usuario (referencia visual).
 *   button             - IRQ de GPIO -> work item troca o modo de exibicao.
 *   shell              - subsistema shell: `kernel threads` e comandos `rt`.
 *
 * Os trechos marcados com TODO(Fx) sao os pontos onde entram I2S/CMSIS-DSP/OLED
 * quando o circuito estiver montado.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

LOG_MODULE_REGISTER(spectrum, LOG_LEVEL_INF);

/* ===== Parametros de DSP (plano, secao 6) ===== */
#define SAMPLE_RATE_HZ   16000
#define FFT_SIZE         512
#define N_BARS           32
#define BLOCK_PERIOD_MS  ((FFT_SIZE * 1000) / SAMPLE_RATE_HZ)   /* ~32 ms */

/* ===== Aquisicao I2S =====
 * INMP441: 24 bits MSB-first num slot de 32 (canal esquerdo; L/R=GND).
 * O registrador de dados do I2S no F4 e de 16 bits: cada slot chega em DUAS
 * metades de 16 bits, e o DMA (configurado em words de 32) grava cada metade
 * zero-estendida num word proprio. Layout real de um frame no buffer:
 *   word0 = metade ALTA de L | word1 = metade BAIXA de L
 *   word2 = metade alta de R | word3 = metade baixa de R (zero, mic so no L)
 * A amostra e recombinada em software: ((hi<<16)|lo) >> 8 -> 24 bits com sinal.
 */
#define I2S_CHANNELS         2
#define I2S_WORDS_PER_FRAME  4                 /* Lhi, Llo, Rhi, Rlo */
#define I2S_BLOCK_BYTES      (FFT_SIZE * I2S_WORDS_PER_FRAME * 4)
#define I2S_BLOCK_COUNT      3
#define I2S_TIMEOUT_MS       2000

/* ===== Afinador (deteccao de pitch por autocorrelacao) =====
 * Fs REAL: PLLI2S = HSE 8M /8 *271 /2 = 135,5 MHz; o driver arredonda o
 * prescaler p/ 132 -> bit clock 1,02652 MHz -> Fs = /64 = 16039,3 Hz.
 * Usar a Fs nominal (16000) daria um erro sistematico de ~4 cents.
 */
#define FS_ACTUAL_HZ     16039.3f
#define TUNER_WIN        1024     /* 64 ms de sinal (>5 periodos do E2 = 82 Hz);
				   * janela maior (2048) estourava o deadline do
				   * bloco: ACF media ~43 ms > 32 ms */
#define TUNER_LAG_MIN    12       /* ~1337 Hz */
#define TUNER_LAG_MAX    400      /* ~40 Hz */
#define TUNER_LEVEL_GATE 150000   /* pico 24-bit minimo p/ tentar detectar */
#define TUNER_ACF_MIN_Q  0.30f    /* qualidade minima (acf_pico/energia) */

static const char *const note_names[12] = {
	"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

/* ===== Modos de exibicao (plano, secao 8 + modo afinador) ===== */
enum disp_mode { MODE_TUNER = 0, MODE_SPECTRUM, MODE_WAVE, MODE_STATS, MODE_COUNT };
static const char *const mode_name[MODE_COUNT] = {
	"tuner", "spectrum", "wave", "stats"
};
static atomic_t disp_mode = ATOMIC_INIT(MODE_TUNER);

/* ===== Frame publicado pela acq_fft para o display ===== */
struct spectrum_frame {
	uint8_t  bars[N_BARS];   /* magnitude por barra, 0..63 (altura no OLED) */
	uint16_t dominant_hz;    /* frequencia dominante do bloco */
	uint32_t seq;            /* numero do frame */
	/* afinador */
	uint16_t f0_dhz;         /* fundamental em deci-Hz (824 = 82,4 Hz) */
	int16_t  cents;          /* desvio da nota mais proxima, em cents */
	uint8_t  midi;           /* numero MIDI da nota mais proxima */
	uint8_t  note_valid;     /* 1 = ha nota detectada */
};

/* ===== Sincronizacao - nada de polling ===== */
K_MSGQ_DEFINE(spectrum_msgq, sizeof(struct spectrum_frame), 4, 4); /* acq_fft -> display */
/* buffers de DMA do I2S; o proprio i2s_read bloqueia ate o DMA entregar um bloco */
K_MEM_SLAB_DEFINE_STATIC(i2s_rx_slab, I2S_BLOCK_BYTES, I2S_BLOCK_COUNT, 4);

/* ===== Metricas de tempo real (lidas pelo shell `rt status`) ===== */
static struct {
	uint32_t fft_us_last;
	uint32_t fft_us_max;
	uint32_t frames;
	uint32_t overruns;
	uint32_t display_fps;
	uint16_t dominant_hz;
	uint32_t level;      /* nivel de audio (pico do bloco) */
	bool     i2s_ok;     /* aquisicao I2S ativa */
	int      i2s_err;    /* codigo de erro se a init do I2S falhar */
} rt;

/* ultimo frame "desenhado" pelo display; exibido sob demanda por 'rt watch' */
static struct spectrum_frame g_latest;

/* diagnostico da aquisicao I2S (comando 'rt dump') */
static volatile int32_t  g_raw[8];        /* primeiros 4 frames crus (L,R,L,R,...) */
static volatile uint32_t g_peak_l, g_peak_r;

/* ===== LED e botao (aliases fornecidos pela board) ===== */
static const struct gpio_dt_spec led    = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

/* display OLED (chosen zephyr,display -> ssd1306 no overlay) */
static const struct device *const disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/* microfone INMP441 no I2S2 (STM32 = mestre/controller, apenas RX) */
static const struct device *const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s2));

/* ---- blink: LED por k_timer (tarefa de referencia) ---- */
static void blink_timer_cb(struct k_timer *t)
{
	gpio_pin_toggle_dt(&led);
}
K_TIMER_DEFINE(blink_timer, blink_timer_cb, NULL);

/* ---- button: ISR enfileira work item (nada de logica pesada na ISR) ---- */
static void mode_work_handler(struct k_work *w)
{
	enum disp_mode nm = (atomic_get(&disp_mode) + 1) % MODE_COUNT;

	atomic_set(&disp_mode, nm);
	LOG_INF("botao: modo -> %s", mode_name[nm]);
}
K_WORK_DEFINE(mode_work, mode_work_handler);

static void button_isr(const struct device *port, struct gpio_callback *cb,
		       uint32_t pins)
{
	k_work_submit(&mode_work);
}

/* ===== Deteccao de pitch por autocorrelacao (janela deslizante) =====
 * Autocorrelacao e o metodo classico de afinador: robusta aos harmonicos
 * fortes de uma corda (o maior pico da FFT costuma ser um harmonico, nao a
 * fundamental) e com precisao sub-amostra via interpolacao parabolica.
 */
static float tuner_ring[TUNER_WIN];    /* ultimas TUNER_WIN amostras (canal L) */
static uint32_t tuner_pos, tuner_fill;

/* retorna f0 em Hz (ou 0 se nao ha nota confiavel) */
static float tuner_detect(void)
{
	static float x[TUNER_WIN];
	static float acf[TUNER_LAG_MAX + 2];

	/* lineariza o anel em ordem temporal */
	uint32_t p = tuner_pos;

	memcpy(x, &tuner_ring[p], (TUNER_WIN - p) * sizeof(float));
	memcpy(&x[TUNER_WIN - p], tuner_ring, p * sizeof(float));

	/* remove DC */
	float mean = 0.0f;

	for (int i = 0; i < TUNER_WIN; i++) {
		mean += x[i];
	}
	mean /= TUNER_WIN;
	for (int i = 0; i < TUNER_WIN; i++) {
		x[i] -= mean;
	}

	/* energia (lag 0) */
	float r0 = 0.0f;

	for (int i = 0; i < TUNER_WIN; i++) {
		r0 += x[i] * x[i];
	}
	if (r0 < 1.0f) {
		return 0.0f;
	}

	/* autocorrelacao nos lags de interesse */
	float rmax = 0.0f;

	for (int lag = TUNER_LAG_MIN; lag <= TUNER_LAG_MAX; lag++) {
		float acc = 0.0f;

		for (int i = 0; i < TUNER_WIN - lag; i++) {
			acc += x[i] * x[i + lag];
		}
		acf[lag] = acc;
		if (acc > rmax) {
			rmax = acc;
		}
	}
	if (rmax < TUNER_ACF_MIN_Q * r0) {
		return 0.0f;    /* sem periodicidade clara (ruido/silencio) */
	}

	/* fundamental = PRIMEIRO pico local >= 85% do maximo (evita cair no
	 * multiplo do periodo e no harmonico) */
	int l = 0;
	float thr = 0.85f * rmax;

	for (int lag = TUNER_LAG_MIN + 1; lag < TUNER_LAG_MAX; lag++) {
		if (acf[lag] >= thr && acf[lag] >= acf[lag - 1] &&
		    acf[lag] >= acf[lag + 1]) {
			l = lag;
			break;
		}
	}
	if (l == 0) {
		return 0.0f;
	}

	/* interpolacao parabolica p/ lag fracionario (precisao sub-amostra) */
	float den = acf[l - 1] - 2.0f * acf[l] + acf[l + 1];
	float delta = (den != 0.0f) ? 0.5f * (acf[l - 1] - acf[l + 1]) / den : 0.0f;

	if (delta < -0.5f || delta > 0.5f) {
		delta = 0.0f;
	}
	return FS_ACTUAL_HZ / ((float)l + delta);
}

/* converte f0 -> nota MIDI mais proxima + desvio em cents (A4 = 440 Hz) */
static void tuner_note(float f0, uint8_t *midi, int16_t *cents)
{
	float c_a4 = 1200.0f * log2f(f0 / 440.0f);   /* cents relativos ao A4 */
	int n = (int)lroundf(c_a4 / 100.0f);         /* semitons ate a nota */

	*midi = (uint8_t)(69 + n);
	*cents = (int16_t)lroundf(c_a4 - 100.0f * (float)n);
}

/* ===== acq_fft (hard RT) - le o I2S via DMA, processa e publica =====
 * O i2s_read bloqueia ate o DMA entregar um bloco: e o proprio relogio
 * hard RT da aquisicao (sem polling). */
static void acq_fft_thread(void *a, void *b, void *c)
{
	const struct i2s_config cfg = {
		.word_size = 32,
		.channels = I2S_CHANNELS,
		.format = I2S_FMT_DATA_FORMAT_I2S,
		.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER,
		.frame_clk_freq = SAMPLE_RATE_HZ,
		.mem_slab = &i2s_rx_slab,
		.block_size = I2S_BLOCK_BYTES,
		.timeout = I2S_TIMEOUT_MS,
	};
	uint32_t seq = 0;

	int rc;

	if (!device_is_ready(i2s_dev)) {
		rt.i2s_err = -ENODEV;
		LOG_ERR("I2S2 device nao pronto");
	} else if ((rc = i2s_configure(i2s_dev, I2S_DIR_RX, &cfg)) != 0) {
		rt.i2s_err = rc;
		LOG_ERR("i2s_configure falhou: %d", rc);
	} else if ((rc = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START)) != 0) {
		rt.i2s_err = rc;
		LOG_ERR("i2s_trigger START falhou: %d", rc);
	} else {
		rt.i2s_ok = true;
		LOG_INF("I2S2 (INMP441) capturando ~%d Hz", SAMPLE_RATE_HZ);
	}

	while (1) {
		void *block;
		size_t nbytes;

		if (!rt.i2s_ok) {
			k_sleep(K_MSEC(500));   /* sem I2S: nao trava o resto do app */
			continue;
		}

		int ret = i2s_read(i2s_dev, &block, &nbytes);  /* bloqueia ate ter bloco */

		if (ret != 0) {
			rt.overruns++;
			LOG_WRN("i2s_read: %d", ret);
			continue;
		}

		uint32_t t0 = k_cycle_get_32();
		const uint32_t *s = block;
		uint32_t frames = nbytes / (I2S_WORDS_PER_FRAME * 4);
		int32_t pk_l = 0, pk_r = 0;

		/* pico dos dois canais + janela deslizante do afinador */
		for (uint32_t i = 0; i < frames; i++) {
			const uint32_t *fr = &s[i * I2S_WORDS_PER_FRAME];
			/* recombina as metades 16-bit e extrai 24 bits com sinal */
			int32_t l = (int32_t)((fr[0] << 16) | (fr[1] & 0xFFFF)) >> 8;
			int32_t r = (int32_t)((fr[2] << 16) | (fr[3] & 0xFFFF)) >> 8;
			int32_t al = (l < 0) ? -l : l;
			int32_t ar = (r < 0) ? -r : r;

			if (al > pk_l) {
				pk_l = al;
			}
			if (ar > pk_r) {
				pk_r = ar;
			}

			tuner_ring[tuner_pos] = (float)l;
			tuner_pos = (tuner_pos + 1) & (TUNER_WIN - 1);
		}
		if (tuner_fill < TUNER_WIN) {
			tuner_fill += frames;
		}
		for (int j = 0; j < 4; j++) {
			const uint32_t *fr = &s[j * I2S_WORDS_PER_FRAME];

			g_raw[2 * j]     = (int32_t)((fr[0] << 16) | (fr[1] & 0xFFFF)) >> 8;
			g_raw[2 * j + 1] = (int32_t)((fr[2] << 16) | (fr[3] & 0xFFFF)) >> 8;
		}
		k_mem_slab_free(&i2s_rx_slab, block);

		g_peak_l = pk_l;
		g_peak_r = pk_r;
		int32_t peak = (pk_l > pk_r) ? pk_l : pk_r;

		rt.level = (uint32_t)peak;

		struct spectrum_frame f = { .seq = seq };
		uint8_t h = peak >> 17;            /* escala grosseira p/ VU 0..63 */

		if (h > 63) {
			h = 63;
		}
		for (int i = 0; i < N_BARS; i++) {
			f.bars[i] = h;
		}

		/* afinador: so tenta detectar com janela cheia e nivel suficiente */
		if (tuner_fill >= TUNER_WIN && pk_l >= TUNER_LEVEL_GATE) {
			float f0 = tuner_detect();

			if (f0 > 0.0f) {
				tuner_note(f0, &f.midi, &f.cents);
				f.f0_dhz = (uint16_t)lroundf(f0 * 10.0f);
				f.dominant_hz = (uint16_t)lroundf(f0);
				f.note_valid = 1;
			}
		}
		rt.dominant_hz = f.dominant_hz;

		rt.fft_us_last = k_cyc_to_us_floor32(k_cycle_get_32() - t0);
		rt.fft_us_max = MAX(rt.fft_us_max, rt.fft_us_last);
		rt.frames = ++seq;

		/* publica sem bloquear o caminho hard RT; se a fila encher, descarta o
		 * frame mais antigo (display e soft - pode perder). */
		if (k_msgq_put(&spectrum_msgq, &f, K_NO_WAIT) != 0) {
			struct spectrum_frame drop;

			(void)k_msgq_get(&spectrum_msgq, &drop, K_NO_WAIT);
			(void)k_msgq_put(&spectrum_msgq, &f, K_NO_WAIT);
		}
	}
}
K_THREAD_DEFINE(acq_fft, 2048, acq_fft_thread, NULL, NULL, NULL, 2, 0, 0);

/* ===== display (soft RT) - desenha o ultimo frame disponivel ===== */
static void display_thread(void *a, void *b, void *c)
{
	struct spectrum_frame f;
	int64_t win_start = k_uptime_get();
	int64_t last_draw = 0;
	uint32_t in_win = 0;
	bool have_disp = false;

	/* inicializa o OLED (se estiver conectado); senao segue so no console */
	if (device_is_ready(disp) && cfb_framebuffer_init(disp) == 0) {
		display_blanking_off(disp);
		cfb_framebuffer_set_font(disp, 0);
		cfb_framebuffer_clear(disp, true);
		cfb_print(disp, "Analisador RT", 0, 0);
		cfb_print(disp, "F1: OLED ok", 0, 16);
		cfb_framebuffer_finalize(disp);
		have_disp = true;
		LOG_INF("OLED SSD1306 pronto (I2C1 @ 0x3c)");
	} else {
		LOG_WRN("OLED nao encontrado - confira a fiacao (I2C1 PB6/PB9, 0x3c)");
	}

	while (1) {
		k_msgq_get(&spectrum_msgq, &f, K_FOREVER);  /* bloqueia ate ter frame */

		in_win++;
		int64_t now = k_uptime_get();

		if (now - win_start >= 1000) {
			rt.display_fps = in_win;
			in_win = 0;
			win_start = now;
		}
		g_latest = f;

		/* TODO(F4): desenhar as barras do espectro. Por ora, tela de status
		 * (texto) a ~5 Hz - prova a cadeia ate o OLED. */
		if (!have_disp || now - last_draw < 200) {
			continue;
		}
		last_draw = now;

		char line[32];
		enum disp_mode m = atomic_get(&disp_mode);

		cfb_framebuffer_clear(disp, false);
		if (m == MODE_TUNER) {
			cfb_print(disp, "Afinador", 0, 0);
			if (f.note_valid) {
				snprintf(line, sizeof(line), "%s%d  %u.%u Hz",
					 note_names[f.midi % 12], f.midi / 12 - 1,
					 f.f0_dhz / 10, f.f0_dhz % 10);
				cfb_print(disp, line, 0, 16);
				snprintf(line, sizeof(line), "%+d cents", f.cents);
				cfb_print(disp, line, 0, 32);
				/* barra: | = afinado, * = posicao atual (6 c/passo) */
				char bar[20] = "[-------|-------]";
				int idx = 8 + f.cents / 6;

				idx = CLAMP(idx, 1, 15);
				bar[idx] = '*';
				cfb_print(disp, bar, 0, 48);
			} else {
				cfb_print(disp, "---", 0, 16);
				cfb_print(disp, "toque uma nota", 0, 32);
			}
		} else {
			cfb_print(disp, "Analisador RT", 0, 0);
			snprintf(line, sizeof(line), "nivel: %u", rt.level);
			cfb_print(disp, line, 0, 16);
			snprintf(line, sizeof(line), "modo: %s", mode_name[m]);
			cfb_print(disp, line, 0, 32);
			snprintf(line, sizeof(line), "fps: %u", rt.display_fps);
			cfb_print(disp, line, 0, 48);
		}
		cfb_framebuffer_finalize(disp);
	}
}
K_THREAD_DEFINE(display, 2048, display_thread, NULL, NULL, NULL, 7, 0, 0);

/* ===== shell: comandos `rt status` e `rt mode` (plano, secao 9) ===== */
static int cmd_rt_status(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Fs=%d Hz  N=%d  bloco=%d ms", SAMPLE_RATE_HZ, FFT_SIZE,
		    BLOCK_PERIOD_MS);
	shell_print(sh, "frames=%u  fps(display)=%u", rt.frames, rt.display_fps);
	shell_print(sh, "fft: ultimo=%u us  max=%u us", rt.fft_us_last,
		    rt.fft_us_max);
	shell_print(sh, "overruns=%u", rt.overruns);
	shell_print(sh, "nivel(pico)=%u", rt.level);
	if (g_latest.note_valid) {
		shell_print(sh, "nota=%s%d  f0=%u.%u Hz  desvio=%+d cents",
			    note_names[g_latest.midi % 12], g_latest.midi / 12 - 1,
			    g_latest.f0_dhz / 10, g_latest.f0_dhz % 10,
			    g_latest.cents);
	} else {
		shell_print(sh, "nota=--- (sem sinal periodico acima do limiar)");
	}
	shell_print(sh, "modo=%s", mode_name[atomic_get(&disp_mode)]);
	if (rt.i2s_ok) {
		shell_print(sh, "I2S=ok  Fs real=%d.%d Hz", (int)FS_ACTUAL_HZ,
			    (int)(FS_ACTUAL_HZ * 10) % 10);
	} else {
		shell_print(sh, "I2S=OFF (erro=%d)", rt.i2s_err);
	}
	return 0;
}

static int cmd_rt_mode(const struct shell *sh, size_t argc, char **argv)
{
	for (int i = 0; i < MODE_COUNT; i++) {
		if (strcmp(argv[1], mode_name[i]) == 0) {
			atomic_set(&disp_mode, i);
			shell_print(sh, "modo -> %s", mode_name[i]);
			return 0;
		}
	}
	shell_error(sh, "modo invalido: use tuner | spectrum | wave | stats");
	return -EINVAL;
}

static int cmd_rt_watch(const struct shell *sh, size_t argc, char **argv)
{
	static const char ramp[] = " .:-=+*#%@";
	int secs = (argc > 1) ? atoi(argv[1]) : 5;

	secs = CLAMP(secs, 1, 60);
	shell_print(sh, "espectro ao vivo por %d s (Fs=%d N=%d)...", secs,
		    SAMPLE_RATE_HZ, FFT_SIZE);

	for (int k = 0; k < secs * 2; k++) {
		struct spectrum_frame f = g_latest;   /* copia do ultimo frame */

		switch ((enum disp_mode)atomic_get(&disp_mode)) {
		case MODE_SPECTRUM: {
			char line[N_BARS + 1];

			for (int i = 0; i < N_BARS; i++) {
				line[i] = ramp[(f.bars[i] * 9) / 63];
			}
			line[N_BARS] = '\0';
			shell_print(sh, "%s  dom=%u Hz", line, f.dominant_hz);
			break;
		}
		case MODE_WAVE:
			shell_print(sh, "[wave] frame %u (forma de onda - TODO F4)",
				    f.seq);
			break;
		default:
			shell_print(sh, "[stats] fps=%u fft=%u us max=%u us overruns=%u",
				    rt.display_fps, rt.fft_us_last, rt.fft_us_max,
				    rt.overruns);
			break;
		}
		k_sleep(K_MSEC(500));
	}
	shell_print(sh, "(fim)");
	return 0;
}

static int cmd_rt_tune(const struct shell *sh, size_t argc, char **argv)
{
	int secs = (argc > 1) ? atoi(argv[1]) : 10;

	secs = CLAMP(secs, 1, 60);
	shell_print(sh, "afinador por %d s (toque uma nota; A4=440 Hz)...", secs);

	for (int k = 0; k < secs * 2; k++) {
		struct spectrum_frame f = g_latest;

		if (f.note_valid) {
			char bar[20] = "[-------|-------]";
			int idx = 8 + f.cents / 6;

			idx = CLAMP(idx, 1, 15);
			bar[idx] = '*';
			shell_print(sh, "%-2s%d  %4u.%u Hz  %+4d cents  %s",
				    note_names[f.midi % 12], f.midi / 12 - 1,
				    f.f0_dhz / 10, f.f0_dhz % 10, f.cents, bar);
		} else {
			shell_print(sh, "---");
		}
		k_sleep(K_MSEC(500));
	}
	shell_print(sh, "(fim)");
	return 0;
}

static int cmd_rt_dump(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "pico canal L=%u  R=%u  (I2S=%s)", g_peak_l, g_peak_r,
		    rt.i2s_ok ? "ok" : "OFF");
	shell_print(sh, "amostras recombinadas (4 frames, 24-bit com sinal):");
	for (int j = 0; j < 8; j += 2) {
		shell_print(sh, "  L=%8d  R=%8d", (int)g_raw[j], (int)g_raw[j + 1]);
	}
	return 0;
}

static int cmd_rt_help(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Analisador de espectro - comandos:");
	shell_print(sh, "  rt status      metricas RT: Fs, N, tempo de processamento, fps, overruns");
	shell_print(sh, "  rt tune [s]    afinador ao vivo por [s] segundos (nota + Hz + cents)");
	shell_print(sh, "  rt watch [s]   espectro ao vivo por [s] segundos (padrao 5, termina sozinho)");
	shell_print(sh, "  rt dump        amostras cruas do I2S (diagnostico)");
	shell_print(sh, "  rt mode <m>    modo do display: tuner | spectrum | wave | stats");
	shell_print(sh, "  rt help        esta ajuda");
	shell_print(sh, "");
	shell_print(sh, "Uteis do Zephyr:");
	shell_print(sh, "  kernel thread list   threads: prioridade, estado, uso de pilha");
	shell_print(sh, "  kernel stacks        uso de pilha por thread");
	shell_print(sh, "  kernel uptime        tempo desde o boot");
	shell_print(sh, "  help                 lista todos os comandos do shell");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(rt_sub,
	SHELL_CMD(help, NULL, "Lista os comandos do analisador e o que fazem", cmd_rt_help),
	SHELL_CMD(status, NULL, "Metricas de tempo real", cmd_rt_status),
	SHELL_CMD_ARG(tune, NULL, "Afinador ao vivo: rt tune [s]", cmd_rt_tune, 1, 1),
	SHELL_CMD(dump, NULL, "Amostras cruas do I2S (diagnostico)", cmd_rt_dump),
	SHELL_CMD_ARG(watch, NULL, "Espectro ao vivo por N segundos: rt watch [s]",
		      cmd_rt_watch, 1, 1),
	SHELL_CMD_ARG(mode, NULL, "Troca o modo: rt mode <spectrum|wave|stats>",
		      cmd_rt_mode, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(rt, &rt_sub, "Comandos do analisador de espectro", NULL);

int main(void)
{
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	} else {
		LOG_ERR("LED (led0) nao esta pronto");
	}

	if (gpio_is_ready_dt(&button)) {
		gpio_pin_configure_dt(&button, GPIO_INPUT);
		gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
		gpio_init_callback(&button_cb_data, button_isr, BIT(button.pin));
		gpio_add_callback(button.port, &button_cb_data);
	} else {
		LOG_ERR("botao (sw0) nao esta pronto");
	}

	k_timer_start(&blink_timer, K_MSEC(500), K_MSEC(500));            /* LED 1 Hz */

	LOG_INF("analisador de espectro (F2 - aquisicao I2S): Fs=%d N=%d bloco=%d ms",
		SAMPLE_RATE_HZ, FFT_SIZE, BLOCK_PERIOD_MS);
	LOG_INF("shell pronto. Digite 'rt help' para ver os comandos disponiveis.");
	return 0;
}

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
#include <arm_math.h>

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
#define TUNER_LAG_MIN    6        /* teto ~2670 Hz; com 12 o teto era 1337 Hz e
				   * notas na borda (ex.: E6=1318 Hz) caiam uma
				   * oitava: o pico real ficava fora da busca e
				   * o detector pegava o de 2x o periodo */
#define TUNER_LAG_MAX    400      /* piso ~40 Hz */

/* sensibilidade do afinador (rt sens <alta|media|baixa>):
 * gate  = pico 24-bit minimo p/ tentar detectar (menor = pega som mais fraco)
 * min_q = qualidade minima da ACF (acf_pico/energia; maior = mais rigoroso) */
static const struct {
	const char *name;
	int32_t gate;
	float min_q;
} sens_tbl[] = {
	{ "alta",  15000,  0.25f },
	{ "media", 50000,  0.30f },
	{ "baixa", 150000, 0.35f },
};
static atomic_t sens_idx = ATOMIC_INIT(1);   /* padrao: media */

static const char *const note_names[12] = {
	"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

/* ===== Espectro (FFT real de 512 pontos, CMSIS-DSP) =====
 * Magnitudes em dB mapeadas para barras de 0..63; bandas em progressao
 * geometrica (log) - audio e logaritmico nos dois eixos. */
#define SPEC_BINS      (FFT_SIZE / 2)
#define SPEC_DB_FLOOR  80.0f     /* dB que zera a barra (piso ~ ruido) */
#define SPEC_DB_TOP    175.0f    /* dB que satura a barra (fundo de escala) */

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
	/* jitter da aquisicao: periodo entre chegadas de bloco (us).
	 * nominal = FFT_SIZE / Fs_real = 512 / 16039,3 Hz = 31922 us */
	uint32_t per_us_last;
	uint32_t per_us_min;
	uint32_t per_us_max;
	uint64_t per_us_sum;
	uint32_t per_n;
} rt;

#define BLOCK_PERIOD_NOM_US 31922U

static void jitter_reset(void)
{
	rt.per_us_min = UINT32_MAX;
	rt.per_us_max = 0;
	rt.per_us_sum = 0;
	rt.per_n = 0;
}

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
	if (rmax < sens_tbl[atomic_get(&sens_idx)].min_q * r0) {
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

/* ===== FFT do espectro (CMSIS-DSP) ===== */
static arm_rfft_fast_instance_f32 rfft;
static float hann[FFT_SIZE];
static float fft_in[FFT_SIZE];
static float fft_out[FFT_SIZE];
static float fft_mag[SPEC_BINS];
static uint16_t band_edge[N_BARS + 1];   /* limites (bins) das barras, escala log */

static void spectrum_init(void)
{
	arm_rfft_fast_init_f32(&rfft, FFT_SIZE);
	for (int i = 0; i < FFT_SIZE; i++) {
		hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
	}
	/* bandas: progressao geometrica do bin 1 ao SPEC_BINS */
	band_edge[0] = 1;
	for (int b = 1; b <= N_BARS; b++) {
		uint16_t e = (uint16_t)lroundf(
			powf((float)SPEC_BINS, (float)b / N_BARS));

		band_edge[b] = MAX(e, band_edge[b - 1] + 1); /* >=1 bin por banda */
	}
	band_edge[N_BARS] = SPEC_BINS;
}

/* FFT sobre as ultimas FFT_SIZE amostras do anel; preenche f->bars e retorna
 * a frequencia do bin dominante (Hz) */
static uint16_t spectrum_compute(struct spectrum_frame *f)
{
	/* lineariza as FFT_SIZE amostras mais recentes com janela de Hann */
	uint32_t start = (tuner_pos - FFT_SIZE) & (TUNER_WIN - 1);

	for (int i = 0; i < FFT_SIZE; i++) {
		fft_in[i] = tuner_ring[(start + i) & (TUNER_WIN - 1)] * hann[i];
	}

	arm_rfft_fast_f32(&rfft, fft_in, fft_out, 0);
	arm_cmplx_mag_f32(fft_out, fft_mag, SPEC_BINS);
	fft_mag[0] = 0.0f;                       /* descarta DC */

	/* barras: maximo da banda -> dB -> 0..63 */
	uint32_t dom_bin = 1;

	for (int b = 0; b < N_BARS; b++) {
		float m = 0.0f;

		for (int k = band_edge[b]; k < band_edge[b + 1]; k++) {
			if (fft_mag[k] > m) {
				m = fft_mag[k];
			}
			if (fft_mag[k] > fft_mag[dom_bin]) {
				dom_bin = k;
			}
		}
		float db = 20.0f * log10f(m + 1.0f);
		int h = (int)((db - SPEC_DB_FLOOR) *
			      (63.0f / (SPEC_DB_TOP - SPEC_DB_FLOOR)));

		f->bars[b] = (uint8_t)CLAMP(h, 0, 63);
	}
	return (uint16_t)lroundf((float)dom_bin * FS_ACTUAL_HZ / FFT_SIZE);
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
	uint32_t prev_cyc = 0;
	bool have_prev = false;

	int rc;

	spectrum_init();
	jitter_reset();

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
			have_prev = false;   /* nao medir periodo sobre um buraco */
			LOG_WRN("i2s_read: %d", ret);
			continue;
		}

		/* jitter: periodo entre chegadas de bloco, visto pela aplicacao
		 * (inclui latencia de escalonamento - e o que importa p/ o deadline) */
		uint32_t now_cyc = k_cycle_get_32();

		if (have_prev) {
			uint32_t per = k_cyc_to_us_floor32(now_cyc - prev_cyc);

			rt.per_us_last = per;
			rt.per_us_min = MIN(rt.per_us_min, per);
			rt.per_us_max = MAX(rt.per_us_max, per);
			rt.per_us_sum += per;
			rt.per_n++;
		}
		prev_cyc = now_cyc;
		have_prev = true;

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

		/* espectro: FFT das ultimas FFT_SIZE amostras (barras em dB) */
		uint16_t spec_dom_hz = 0;

		if (tuner_fill >= FFT_SIZE) {
			spec_dom_hz = spectrum_compute(&f);
		}

		/* afinador: so tenta detectar com janela cheia e nivel suficiente */
		if (tuner_fill >= TUNER_WIN &&
		    pk_l >= sens_tbl[atomic_get(&sens_idx)].gate) {
			float f0 = tuner_detect();

			if (f0 > 0.0f) {
				tuner_note(f0, &f.midi, &f.cents);
				f.f0_dhz = (uint16_t)lroundf(f0 * 10.0f);
				f.note_valid = 1;
			}
		}
		/* freq dominante: a do afinador (mais precisa) se ha nota;
		 * senao, o bin dominante da FFT */
		f.dominant_hz = f.note_valid ? (uint16_t)((f.f0_dhz + 5) / 10)
					     : spec_dom_hz;
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

		/* desenha a ~20 fps (I2C fast a 400 kHz aguenta ~30 fps) */
		if (!have_disp || now - last_draw < 50) {
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
		} else if (m == MODE_SPECTRUM) {
			/* 32 barras de 4 px usando a altura toda (0..63) */
			for (int b = 0; b < N_BARS; b++) {
				int h = f.bars[b];

				if (h > 0) {
					cfb_invert_area(disp, b * 4, 63 - h, 3, h);
				}
			}
		} else if (m == MODE_WAVE) {
			/* mini-osciloscopio: 16 ms de sinal (256 amostras, 2 por
			 * coluna), gatilho por cruzamento de zero (estabiliza a
			 * onda) e ganho vertical automatico pelo pico recente */
			uint32_t base = (tuner_pos - 256 - 128) & (TUNER_WIN - 1);
			uint32_t trig = 0;

			for (uint32_t i = 0; i < 128; i++) {
				float a = tuner_ring[(base + i) & (TUNER_WIN - 1)];
				float b = tuner_ring[(base + i + 1) &
						     (TUNER_WIN - 1)];

				if (a < 0.0f && b >= 0.0f) {
					trig = i + 1;
					break;
				}
			}

			float pk = MAX((float)rt.level, 20000.0f);
			float scale = 30.0f / pk;

			for (int c = 0; c < 128; c++) {
				float v = tuner_ring[(base + trig + c * 2) &
						     (TUNER_WIN - 1)];
				int y = 32 - (int)(v * scale);

				y = CLAMP(y, 0, 62);
				cfb_invert_area(disp, c, y, 1, 2);
			}
		} else {
			snprintf(line, sizeof(line), "fps:%u  ov:%u",
				 rt.display_fps, rt.overruns);
			cfb_print(disp, line, 0, 0);
			snprintf(line, sizeof(line), "proc: %u us", rt.fft_us_last);
			cfb_print(disp, line, 0, 16);
			snprintf(line, sizeof(line), "per: %u us", rt.per_us_last);
			cfb_print(disp, line, 0, 32);
			snprintf(line, sizeof(line), "jit: %u us",
				 rt.per_n ? rt.per_us_max - rt.per_us_min : 0);
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
	if (rt.per_n > 0) {
		shell_print(sh, "periodo=%u us (nominal %u)  jitter pp=%u us",
			    rt.per_us_last, BLOCK_PERIOD_NOM_US,
			    rt.per_us_max - rt.per_us_min);
	}
	shell_print(sh, "nivel(pico)=%u", rt.level);
	if (g_latest.note_valid) {
		shell_print(sh, "nota=%s%d  f0=%u.%u Hz  desvio=%+d cents",
			    note_names[g_latest.midi % 12], g_latest.midi / 12 - 1,
			    g_latest.f0_dhz / 10, g_latest.f0_dhz % 10,
			    g_latest.cents);
	} else {
		shell_print(sh, "nota=--- (sem sinal periodico acima do limiar)");
	}
	shell_print(sh, "sensibilidade=%s", sens_tbl[atomic_get(&sens_idx)].name);
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
			shell_print(sh, "[wave] frame %u nivel=%u (onda no OLED)",
				    f.seq, rt.level);
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

static int cmd_rt_jitter(const struct shell *sh, size_t argc, char **argv)
{
	if (argc > 1 && strcmp(argv[1], "reset") == 0) {
		jitter_reset();
		shell_print(sh, "estatisticas de jitter zeradas");
		return 0;
	}
	if (rt.per_n == 0) {
		shell_print(sh, "sem medidas ainda (aquisicao parada?)");
		return 0;
	}

	uint32_t avg = (uint32_t)(rt.per_us_sum / rt.per_n);

	shell_print(sh, "periodo do bloco (n=%u, nominal=%u us):", rt.per_n,
		    BLOCK_PERIOD_NOM_US);
	shell_print(sh, "  ultimo=%u  min=%u  medio=%u  max=%u us",
		    rt.per_us_last, rt.per_us_min, avg, rt.per_us_max);
	shell_print(sh, "  jitter pico-a-pico=%u us (%u%% do periodo)",
		    rt.per_us_max - rt.per_us_min,
		    (rt.per_us_max - rt.per_us_min) * 100 / BLOCK_PERIOD_NOM_US);
	shell_print(sh, "  deadline: proc max=%u us < %u us %s  overruns=%u",
		    rt.fft_us_max, BLOCK_PERIOD_NOM_US,
		    rt.fft_us_max < BLOCK_PERIOD_NOM_US ? "OK" : "ESTOURADO",
		    rt.overruns);
	shell_print(sh, "uso: rt jitter [reset]");
	return 0;
}

static int cmd_rt_sens(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		int i = atomic_get(&sens_idx);

		shell_print(sh, "sensibilidade atual: %s (gate=%d, q=%d%%)",
			    sens_tbl[i].name, sens_tbl[i].gate,
			    (int)(sens_tbl[i].min_q * 100));
		shell_print(sh, "uso: rt sens <alta|media|baixa>");
		return 0;
	}
	for (int i = 0; i < (int)ARRAY_SIZE(sens_tbl); i++) {
		if (strcmp(argv[1], sens_tbl[i].name) == 0) {
			atomic_set(&sens_idx, i);
			shell_print(sh, "sensibilidade -> %s (gate=%d)",
				    sens_tbl[i].name, sens_tbl[i].gate);
			return 0;
		}
	}
	shell_error(sh, "sensibilidade invalida: use alta | media | baixa");
	return -EINVAL;
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
	shell_print(sh, "  rt sens <n>    sensibilidade do afinador: alta | media | baixa");
	shell_print(sh, "  rt jitter      estatisticas do periodo de bloco (jitter, deadline); [reset] zera");
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
	SHELL_CMD_ARG(sens, NULL, "Sensibilidade: rt sens <alta|media|baixa>",
		      cmd_rt_sens, 1, 1),
	SHELL_CMD_ARG(jitter, NULL, "Jitter do periodo de bloco: rt jitter [reset]",
		      cmd_rt_jitter, 1, 1),
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

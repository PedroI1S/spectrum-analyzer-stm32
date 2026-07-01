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
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_REGISTER(spectrum, LOG_LEVEL_INF);

/* ===== Parametros de DSP (plano, secao 6) ===== */
#define SAMPLE_RATE_HZ   16000
#define FFT_SIZE         512
#define N_BARS           32
#define BLOCK_PERIOD_MS  ((FFT_SIZE * 1000) / SAMPLE_RATE_HZ)   /* ~32 ms */

/* ===== Modos de exibicao (plano, secao 8) ===== */
enum disp_mode { MODE_SPECTRUM = 0, MODE_WAVE, MODE_STATS, MODE_COUNT };
static const char *const mode_name[MODE_COUNT] = { "spectrum", "wave", "stats" };
static atomic_t disp_mode = ATOMIC_INIT(MODE_SPECTRUM);

/* ===== Frame de espectro publicado pela acq_fft para o display ===== */
struct spectrum_frame {
	uint8_t  bars[N_BARS];   /* magnitude por barra, 0..63 (altura no OLED) */
	uint16_t dominant_hz;    /* frequencia dominante do bloco */
	uint32_t seq;            /* numero do frame */
};

/* ===== Sincronizacao - nada de polling ===== */
K_SEM_DEFINE(block_ready_sem, 0, 1);  /* "bloco novo do DMA" -> acq_fft */
K_MSGQ_DEFINE(spectrum_msgq, sizeof(struct spectrum_frame), 4, 4); /* -> display */

/* ===== Metricas de tempo real (lidas pelo shell `rt status`) ===== */
static struct {
	uint32_t fft_us_last;
	uint32_t fft_us_max;
	uint32_t frames;
	uint32_t overruns;
	uint32_t display_fps;
	uint16_t dominant_hz;
} rt;

/* ultimo frame "desenhado" pelo display; exibido sob demanda por 'rt watch' */
static struct spectrum_frame g_latest;

/* ===== LED e botao (aliases fornecidos pela board) ===== */
static const struct gpio_dt_spec led    = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;

/* display OLED (chosen zephyr,display -> ssd1306 no overlay) */
static const struct device *const disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

/* ---- blink: LED por k_timer (tarefa de referencia) ---- */
static void blink_timer_cb(struct k_timer *t)
{
	gpio_pin_toggle_dt(&led);
}
K_TIMER_DEFINE(blink_timer, blink_timer_cb, NULL);

/* ---- surrogato do callback de DMA do I2S: marca o ritmo dos blocos ----
 * TODO(F2): remover; quem dara o semaforo sera o dma callback real do I2S2.
 */
static void block_timer_cb(struct k_timer *t)
{
	if (k_sem_count_get(&block_ready_sem) > 0) {
		rt.overruns++;   /* acq_fft ainda nao consumiu o bloco anterior */
	}
	k_sem_give(&block_ready_sem);
}
K_TIMER_DEFINE(block_timer, block_timer_cb, NULL);

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

/* ===== acq_fft (hard RT) - acorda no semaforo, processa, publica ===== */
static void acq_fft_thread(void *a, void *b, void *c)
{
	uint32_t seq = 0;

	while (1) {
		k_sem_take(&block_ready_sem, K_FOREVER);   /* bloqueia ate ter bloco */

		uint32_t t0 = k_cycle_get_32();
		struct spectrum_frame f = { .seq = seq };

		/* TODO(F2/F3): substituir pelo processamento real:
		 *   1. ler buffer I2S do INMP441 (DMA ping-pong);
		 *   2. janela de Hann (arm_mult_f32);
		 *   3. arm_rfft_fast_f32 + arm_cmplx_mag_f32 (CMSIS-DSP);
		 *   4. agrupar bins em N_BARS bandas (log) e converter para dB.
		 * Por enquanto: espectro sintetico - um pico que varre as barras. */
		uint32_t peak = (seq / 4) % N_BARS;
		for (int i = 0; i < N_BARS; i++) {
			int mag = 60 - abs((int)i - (int)peak) * 12;

			f.bars[i] = (mag < 0) ? (1 + (i % 3)) : (uint8_t)mag;
		}
		f.dominant_hz = (uint16_t)((peak * SAMPLE_RATE_HZ) / FFT_SIZE);

		uint32_t dt = k_cyc_to_us_floor32(k_cycle_get_32() - t0);

		rt.fft_us_last = dt;
		rt.fft_us_max  = MAX(rt.fft_us_max, dt);
		rt.dominant_hz = f.dominant_hz;
		rt.frames = ++seq;

		/* publica sem bloquear o caminho hard RT; se a fila encher,
		 * descarta o frame mais antigo (display e soft - pode perder). */
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

		char line[24];

		cfb_framebuffer_clear(disp, false);
		cfb_print(disp, "Analisador RT", 0, 0);
		snprintf(line, sizeof(line), "f0: %5u Hz", f.dominant_hz);
		cfb_print(disp, line, 0, 16);
		snprintf(line, sizeof(line), "modo: %s",
			 mode_name[atomic_get(&disp_mode)]);
		cfb_print(disp, line, 0, 32);
		snprintf(line, sizeof(line), "fps: %u", rt.display_fps);
		cfb_print(disp, line, 0, 48);
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
	shell_print(sh, "freq dominante=%u Hz", rt.dominant_hz);
	shell_print(sh, "modo=%s", mode_name[atomic_get(&disp_mode)]);
	shell_print(sh, "(acq_fft/display em modo stub - aguardando mic e OLED)");
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
	shell_error(sh, "modo invalido: use spectrum | wave | stats");
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

static int cmd_rt_help(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Analisador de espectro - comandos:");
	shell_print(sh, "  rt status      metricas RT: Fs, N, tempo de FFT, fps, overruns, freq dominante");
	shell_print(sh, "  rt watch [s]   espectro ao vivo por [s] segundos (padrao 5, termina sozinho)");
	shell_print(sh, "  rt mode <m>    modo de exibicao: spectrum | wave | stats");
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
	k_timer_start(&block_timer, K_MSEC(BLOCK_PERIOD_MS),
		      K_MSEC(BLOCK_PERIOD_MS));

	LOG_INF("analisador de espectro - esqueleto RT (F0): Fs=%d N=%d bloco=%d ms",
		SAMPLE_RATE_HZ, FFT_SIZE, BLOCK_PERIOD_MS);
	LOG_INF("shell pronto. Digite 'rt help' para ver os comandos disponiveis.");
	return 0;
}

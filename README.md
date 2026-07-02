# Analisador de Espectro de Áudio em Tempo Real (Zephyr / STM32F4-Discovery)

Trabalho final de Sistemas de Tempo Real. Captura áudio por microfone I2S, calcula a
FFT em tempo real e exibe o espectro num display OLED, de forma autônoma. O plano
completo está em [`plano_projeto_analisador_espectro_zephyr.md`](plano_projeto_analisador_espectro_zephyr.md).

> Este repositório partiu do template **Zephyr example-application**; a documentação
> original do template está preservada [mais abaixo](#zephyr-example-application-template-original).

## Estado atual (bring-up — atualizado em 27/06/2026)

- ✅ Ambiente Zephyr montado: venv (`.venv/`), `west`, Zephyr `main` + módulos
  (`cmsis_6`, `hal_stm32`, `hal_nordic`, `segger`) e toolchain `arm-zephyr-eabi`.
  Ferramentas de host via Homebrew: `openocd`, `stlink`, `dtc`.
- ✅ Comunicação com a placa via ST-LINK/OpenOCD (STM32F407VG detectado).
- ✅ `blinky` gravado e validado (LED pisca).
- ✅ Console por **SEGGER RTT** sobre o ST-LINK (sem adaptador USB-serial).
- ✅ `app/` compila e roda na `stm32f4_disco` (usa os aliases `led0`/`sw0` da placa).
- ✅ **F0 concluído** — esqueleto de tempo real (plano §4) implementado e validado na placa:
  threads `acq_fft` (hard) e `display` (soft) com semáforo + message queue, `blink` por
  `k_timer`, botão por IRQ→work queue, e shell por RTT com `kernel thread list`, `kernel stacks`
  e os comandos `rt status` / `rt mode <spectrum|wave|stats>`. acq_fft/display rodam com
  espectro sintético (stubs marcados `TODO(Fx)`) até o circuito existir.
- ✅ **F1 concluído** — display OLED SSD1306 (I2C1 @ 0x3c) funcionando; mostra nível/modo/fps via CFB. Console RTT limpo + comandos `rt watch [s]`, `rt dump` e `rt help`.
- ✅ **F2 (aquisição) concluído** — INMP441 no I2S2 (CK=PB13, WS=PB12, SD=PB15) via DMA a 16 kHz,
  amostras 24-bit corretas, 0 overruns. Dois bugs não-óbvios documentados no §15.5 do plano
  (include de clock errado no overlay; amostras chegam em metades de 16 bits e são recombinadas no app).
- ⏳ Próximo (F3): detector de nota/afinador por autocorrelação (nota + cents no OLED) e, depois, FFT (CMSIS-DSP). Ver §11 e §15 do plano.

## Como construir, gravar e ver o console

A partir da raiz deste repositório (`Zephyr-Project/`) — o `west` encontra o workspace
sozinho subindo até o `.west/` na pasta-pai:

```shell
source .venv/bin/activate                # ativa o ambiente Python
west build -b stm32f4_disco app          # compila o app (gera ./build)
west flash -d build --runner openocd     # grava (forca o runner openocd)
./scripts/rtt-console.sh                 # abre o terminal RTT interativo (porta 9090)
```

Notas:

- O **runner padrão** do board é o STM32CubeProgrammer (não instalado) — sempre passe
  `--runner openocd`.
- O `app/prj.conf` usa o **shell por RTT** (`CONFIG_SHELL_BACKEND_RTT=y`,
  `CONFIG_SHELL_BACKEND_SERIAL=n`); os logs saem pelo mesmo backend. O `rtt-console.sh`
  abre um terminal interativo — dá para digitar comandos do shell direto nele.
- **Só um OpenOCD por vez** segura o ST-LINK — encerre o servidor RTT (Ctrl-C no script)
  antes de gravar.

### Recriar o ambiente do zero (outra máquina)

```shell
brew install openocd stlink dtc cmake ninja gperf
python3 -m venv Zephyr-Project/.venv && source Zephyr-Project/.venv/bin/activate
pip install west
west init -l Zephyr-Project && west update
west zephyr-export && pip install -r zephyr/scripts/requirements.txt
west sdk install -t arm-zephyr-eabi
```

---

# Zephyr Example Application (template original)

<a href="https://github.com/zephyrproject-rtos/example-application/actions/workflows/build.yml?query=branch%3Amain">
  <img src="https://github.com/zephyrproject-rtos/example-application/actions/workflows/build.yml/badge.svg?event=push">
</a>
<a href="https://github.com/zephyrproject-rtos/example-application/actions/workflows/docs.yml?query=branch%3Amain">
  <img src="https://github.com/zephyrproject-rtos/example-application/actions/workflows/docs.yml/badge.svg?event=push">
</a>
<a href="https://zephyrproject-rtos.github.io/example-application">
  <img alt="Documentation" src="https://img.shields.io/badge/documentation-3D578C?logo=sphinx&logoColor=white">
</a>
<a href="https://zephyrproject-rtos.github.io/example-application/doxygen">
  <img alt="API Documentation" src="https://img.shields.io/badge/API-documentation-3D578C?logo=c&logoColor=white">
</a>

This repository contains a Zephyr example application. The main purpose of this
repository is to serve as a reference on how to structure Zephyr-based
applications. Some of the features demonstrated in this example are:

- Basic [Zephyr application][app_dev] skeleton
- [Zephyr workspace applications][workspace_app]
- [Zephyr modules][modules]
- [West T2 topology][west_t2]
- [Custom boards][board_porting]
- Custom [devicetree bindings][bindings]
- Out-of-tree [drivers][drivers]
- Out-of-tree libraries
- Example CI configuration (using GitHub Actions)
- Custom [west extension][west_ext]
- Custom [Zephyr runner][runner_ext]
- Doxygen and Sphinx documentation boilerplate

This repository is versioned together with the [Zephyr main tree][zephyr]. This
means that every time that Zephyr is tagged, this repository is tagged as well
with the same version number, and the [manifest](west.yml) entry for `zephyr`
will point to the corresponding Zephyr tag. For example, the `example-application`
v2.6.0 will point to Zephyr v2.6.0. Note that the `main` branch always
points to the development branch of Zephyr, also `main`.

[app_dev]: https://docs.zephyrproject.org/latest/develop/application/index.html
[workspace_app]: https://docs.zephyrproject.org/latest/develop/application/index.html#zephyr-workspace-app
[modules]: https://docs.zephyrproject.org/latest/develop/modules.html
[west_t2]: https://docs.zephyrproject.org/latest/develop/west/workspaces.html#west-t2
[board_porting]: https://docs.zephyrproject.org/latest/guides/porting/board_porting.html
[bindings]: https://docs.zephyrproject.org/latest/guides/dts/bindings.html
[drivers]: https://docs.zephyrproject.org/latest/reference/drivers/index.html
[zephyr]: https://github.com/zephyrproject-rtos/zephyr
[west_ext]: https://docs.zephyrproject.org/latest/develop/west/extensions.html
[runner_ext]: https://docs.zephyrproject.org/latest/develop/modules.html#external-runners

## Getting Started

Before getting started, make sure you have a proper Zephyr development
environment. Follow the official
[Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html).

### Initialization

The first step is to initialize the workspace folder (``my-workspace``) where
the ``example-application`` and all Zephyr modules will be cloned. Run the following
command:

```shell
# initialize my-workspace for the example-application (main branch)
west init -m https://github.com/zephyrproject-rtos/example-application --mr main my-workspace
# update Zephyr modules
cd my-workspace
west update
```

### Building and running

To build the application, run the following command:

```shell
cd example-application
west build -b $BOARD app
```

where `$BOARD` is the target board.

You can use the `custom_plank` board found in this
repository. Note that Zephyr sample boards may be used if an
appropriate overlay is provided (see `app/boards`).

A sample debug configuration is also provided. To apply it, run the following
command:

```shell
west build -b $BOARD app -- -DEXTRA_CONF_FILE=debug.conf
```

Once you have built the application, run the following command to flash it:

```shell
west flash
```

### Testing

To execute Twister integration tests, run the following command:

```shell
west twister -T tests --integration
```

### Documentation

A minimal documentation setup is provided for Doxygen and Sphinx. To build the
documentation first change to the ``doc`` folder:

```shell
cd doc
```

Before continuing, check if you have Doxygen installed. It is recommended to
use the same Doxygen version used in [CI](.github/workflows/docs.yml). To
install Sphinx, make sure you have a Python installation in place and run:

```shell
pip install -r requirements.txt
```

API documentation (Doxygen) can be built using the following command:

```shell
doxygen
```

The output will be stored in the ``_build_doxygen`` folder. Similarly, the
Sphinx documentation (HTML) can be built using the following command:

```shell
make html
```

The output will be stored in the ``_build_sphinx`` folder. You may check for
other output formats other than HTML by running ``make help``.

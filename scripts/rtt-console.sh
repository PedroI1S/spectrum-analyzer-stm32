#!/usr/bin/env bash
# Abre o console SEGGER RTT da STM32F4-Discovery sobre o ST-LINK (sem hardware extra).
#
# Pre-requisitos no firmware (prj.conf):
#   CONFIG_USE_SEGGER_RTT=y
#   CONFIG_RTT_CONSOLE=y
#   CONFIG_UART_CONSOLE=n
# E o modulo "segger" precisa estar no west.yml (allowlist) -> west update.
#
# Uso:
#   ./scripts/rtt-console.sh          # sobe o servidor RTT e ja conecta o terminal
#   Ctrl-C encerra.
set -e

PORT="${RTT_PORT:-9090}"

cleanup() { pkill -P $$ openocd 2>/dev/null || true; pkill openocd 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo ">> subindo servidor RTT (OpenOCD) na porta $PORT ..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "init" -c "reset run" -c "sleep 1500" \
  -c 'rtt setup 0x20000000 0x20000 "SEGGER RTT"' \
  -c "rtt start" -c "rtt server start $PORT 0" &

# espera o servidor RTT abrir a porta
for i in $(seq 1 20); do
  if nc -z localhost "$PORT" 2>/dev/null; then break; fi
  sleep 0.3
done

echo ">> conectando ao console (Ctrl-C para sair)"
echo "------------------------------------------------------------"
nc localhost "$PORT"

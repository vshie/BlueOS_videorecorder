#!/bin/bash
# Apply DeckHand-required host /boot/firmware/config.txt overrides on a
# BlueOS Pi. Normally you DO NOT need to run this — the videorecorder
# extension's ``deckhand_host_setup`` module does the same job on every
# startup and is the source of truth. This script exists only as a
# manual bootstrap for cases where the extension hasn't been installed
# yet (e.g. fresh bring-up before the docker image is fetched).
#
# Why the DeckHand PCB needs config.txt tweaks:
#   BlueOS ships blueos_startup_update.py which restores a Navigator
#   flight-controller pin map into /boot/firmware/config.txt on every
#   boot (adding overlays and gpio= lines). Some of those defaults clash
#   with what the DeckHand uses:
#
#     dtoverlay=uart3          claims GPIO 4/5 -> collides with PCA9685
#                              ~OE (GPIO 4).  We disable it: uart3-off.
#     (uart4 not enabled)      but the DeckHand's BMS talks RS-485 on
#                              UART4 (GPIO 8/9). We enable: uart4.
#     dtoverlay=spi0-led       could claim GPIO 10, our rotation sensor
#                              input. We disable it: spi0-led-off.
#     gpio=11,24,25=op,pu,dh   drives GPIO 11 (RS-485 DE) HIGH at boot.
#                              We handle this at RUNTIME via bcm_pinmux
#                              in the extension (forces ALT4 RTS4 for
#                              kernel-driven TIOCSRS485), NOT here —
#                              trying to persist a ``gpio=11=a4,pn``
#                              override in config.txt requires the
#                              ``# custom`` marker to survive the
#                              reconciler, but that same marker breaks
#                              the firmware's gpio= parser.
#
#   The reconciler's ``spi1-3cs`` default (auto-added on Navigator hosts)
#   happens to give us /dev/spidev1.0 on GPIO 20 MOSI — the same device
#   the WS2812 LED backend opens — so we don't need our own spi1-1cs.
#
# IMPORTANT: config.txt ``dtoverlay=`` lines MUST NOT carry inline ``#``
# comments. The RPi firmware parser treats the trailing "# ..." as part
# of the overlay name and silently rejects the line at boot (empirically
# confirmed 2026-07-14: with a "# custom - DeckHand: ..." inline comment,
# UART4 was never enumerated by the kernel; the same line without the
# comment enumerated UART4 immediately on the very next boot).
#
# The BlueOS reconciler does NOT strip ``dtoverlay=uart4``,
# ``dtoverlay=uart3-off``, or ``dtoverlay=spi0-led-off`` because none of
# them match its known conflict patterns for ``[pi4]``, so we can safely
# write them WITHOUT the ``# custom`` marker.

set -e

if [ -f /boot/firmware/config.txt ]; then
  CONFIG=/boot/firmware/config.txt
elif [ -f /boot/config.txt ]; then
  CONFIG=/boot/config.txt
else
  echo "ERROR: neither /boot/firmware/config.txt nor /boot/config.txt exists" >&2
  exit 1
fi

echo "== target: $CONFIG =="
BACKUP="${CONFIG}.bak-deckhand-$(date +%Y%m%d-%H%M%S)"
sudo cp "$CONFIG" "$BACKUP"
echo "== backup: $BACKUP =="

TMP=$(mktemp)
sudo cp "$CONFIG" "$TMP"
sudo chown "$USER" "$TMP"

python3 - "$TMP" <<'PY'
import re, sys
path = sys.argv[1]
lines = open(path).read().splitlines()

# Locate the first [pi4] block. Section ends at the first blank line or
# next [tag] (mirrors blueos_startup_update.get_or_append_section).
start = next((i for i, l in enumerate(lines) if re.match(r"^\[pi4\]\s*$", l)), None)
if start is None:
    lines.extend(["", "[pi4]"])
    start = len(lines) - 1
end = len(lines)
for i in range(start + 1, len(lines)):
    if lines[i] == "" or re.match(r"^\[.+\]\s*$", lines[i]):
        end = i
        break

# Canonical DeckHand overrides. Each entry: (regex-that-matches-any-variant,
# canonical-line-to-write). Canonical lines carry NO inline comment (see
# the header of this script for why); the regex matches both plain and
# legacy commented forms so a re-apply cleans up old writes in place.
WANTED = [
    (r"^dtoverlay=uart4(?:,\S+)?(?:\s+#.*)?$",     "dtoverlay=uart4"),
    (r"^dtoverlay=uart3(?:-off)?(?:\s+#.*)?$",     "dtoverlay=uart3-off"),
    (r"^dtoverlay=spi0-led(?:-off)?(?:\s+#.*)?$",  "dtoverlay=spi0-led-off"),
]

section = lines[start + 1 : end]
for regex, canonical in WANTED:
    pat = re.compile(regex)
    idx = next((i for i, l in enumerate(section) if pat.match(l)), None)
    if idx is not None:
        section[idx] = canonical
    else:
        # Insert before the first gpio= line so top-of-section reads as
        # "overlays first, gpio= lines second" — cosmetic but readable.
        insert_at = next(
            (i for i, l in enumerate(section) if l.startswith("gpio=")),
            len(section),
        )
        section.insert(insert_at, canonical)

open(path, "w").write("\n".join(lines[: start + 1] + section + lines[end:]) + "\n")
PY

sudo cp "$TMP" "$CONFIG"
rm -f "$TMP"

echo
echo "== first [pi4] block after edit =="
sudo awk '/^\[pi4\]/{flag=1; print; next} flag && (/^\s*$/||/^\[/){flag=0} flag' "$CONFIG" | head -30

echo
echo "== diff vs backup =="
sudo diff "$BACKUP" "$CONFIG" || true

cat <<MSG

Applied. Now reboot the Pi:
    sudo reboot

After reboot verify:
    ls /dev/ttyAMA*                  # should include one mapped to MMIO 7e201800 (UART4)
    ls /dev/spidev1.0                # should exist (reconciler-added spi1-3cs)
    sudo raspi-gpio get 4,8,9,10,11  # GPIO 8/9 should be ALT4 TXD4/RXD4
MSG

#!/bin/bash
# Apply DeckHand-required host /boot/config.txt overrides on a BlueOS Pi
# and make them survive the BlueOS startup reconciler.
#
# Run this ONCE on the Pi (over SSH as user `pi`) after installing the
# DeckHand PCB, then reboot. The script is idempotent — re-running it
# leaves an already-patched file unchanged.
#
# Why this is needed:
#   BlueOS ships blueos_startup_update.py which, on every Pi 4 boot,
#   restores a Navigator flight-controller pin map into /boot/config.txt
#   (adding lines and filtering user-modified variants). Three of the
#   pins that Navigator claims collide with what the DeckHand PCB uses:
#
#     dtoverlay=uart3          claims GPIO 4/5  -> collides with PCA9685 ~OE (GPIO 4)
#     dtoverlay=spi1-3cs       claims GPIO 16-21 -> collides with the
#                              release-shaft rotation sensor input (GPIO 20)
#     gpio=11,24,25=op,pu,dh   drives GPIO 11 HIGH at boot -> that's the
#                              RS-485 DE line, which asserts the transceiver
#                              driver for the entire boot window
#
# The trick this script uses:
#   The reconciler's "already exists" check is `re.match(config, line)`,
#   which is a prefix match. So a line that STARTS with the required
#   text (but continues with something benign) satisfies the reconciler.
#   The reconciler also honours a `# custom` inline comment as a
#   protection marker on user lines. Combining the two:
#
#     dtoverlay=uart3-off      # custom - <reason>
#     dtoverlay=spi1-3cs-off   # custom - <reason>
#
#   Reconciler sees a `dtoverlay=uart3`-prefixed line and doesn't re-add
#   the harmful `dtoverlay=uart3`. Pi firmware treats `uart3-off` as a
#   non-existent overlay and silently skips it, so nothing claims the
#   pins.
#
#   For the GPIO 11 line we can't use a prefix trick because we want
#   GPIO 24 and 25 to keep their required op,pu,dh state. Instead we
#   leave the reconciler's required line in place AND add a per-pin
#   override right after it:
#
#     gpio=11,24,25=op,pu,dh
#     gpio=11=ip,pn,dl         # custom - <reason>
#
#   Pi firmware processes gpio= lines top-to-bottom and per-pin settings
#   override earlier list entries for the same pin, so GPIO 11 boots as
#   input-no-pull-drive-low while 24 and 25 stay op,pu,dh.

set -e

CONFIG=/boot/config.txt
if [ ! -f "$CONFIG" ] && [ -f /boot/firmware/config.txt ]; then
  CONFIG=/boot/firmware/config.txt
fi
if [ ! -f "$CONFIG" ]; then
  echo "ERROR: neither /boot/config.txt nor /boot/firmware/config.txt exists" >&2
  exit 1
fi

echo "== target: $CONFIG =="
BACKUP="${CONFIG}.bak-deckhand-$(date +%Y%m%d-%H%M%S)"
sudo cp "$CONFIG" "$BACKUP"
echo "== backup: $BACKUP =="

# Work on a temp file for atomicity.
TMP=$(mktemp)
sudo cp "$CONFIG" "$TMP"
sudo chown "$USER" "$TMP"

# Strip any prior variants of these lines so we can re-insert cleanly.
sed -i '/^dtoverlay=uart3\(-off\)\?\(  *#.*\)\?$/d'         "$TMP"
sed -i '/^dtoverlay=spi1-3cs\(-off\)\?\(  *#.*\)\?$/d'      "$TMP"
sed -i '/^gpio=11,24,25=op,pu,dh$/d'                        "$TMP"
sed -i '/^gpio=24,25=op,pu,dh$/d'                           "$TMP"
sed -i '/^gpio=11=ip,pn,dl.*# custom.*$/d'                  "$TMP"

# Insert the desired four lines at the top of the [pi4] section.
python3 - "$TMP" <<'PY'
import re, sys
path = sys.argv[1]
lines = open(path).read().splitlines()
start = None
end = len(lines)
for i, line in enumerate(lines):
    if re.match(r"^\[pi4\]\s*$", line):
        start = i
        break
if start is None:
    lines.extend(["", "[pi4]"])
    start = len(lines) - 1
for i in range(start + 1, len(lines)):
    if re.match(r"^\[.+\]\s*$", lines[i]):
        end = i
        break
wanted = [
    "dtoverlay=uart3-off  # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE",
    "dtoverlay=spi1-3cs-off  # custom - DeckHand: GPIO 20 needed as rotation-sensor input",
    "gpio=11,24,25=op,pu,dh",
    "gpio=11=ip,pn,dl  # custom - DeckHand: RS-485 DE line must not boot HIGH",
]
new = lines[: start + 1] + wanted + lines[start + 1 : end] + lines[end:]
open(path, "w").write("\n".join(new) + "\n")
PY

sudo cp "$TMP" "$CONFIG"
rm -f "$TMP"

echo
echo "== [pi4] block after edit =="
sudo awk '/^\[pi4\]/{flag=1; print; next} /^\[/{flag=0} flag' "$CONFIG"

echo
echo "== diff vs backup =="
sudo diff "$BACKUP" "$CONFIG" || true

cat <<MSG

Applied. Now reboot the Pi:
    sudo reboot

After reboot verify the pins are in the expected state:
    raspi-gpio get 4,8,9,11,20
    # Expected (before the extension claims them):
    #   GPIO 4:  INPUT              (was ALT4/UART3-RXD; now free for PCA9685 ~OE)
    #   GPIO 8:  ALT4 TXD4          (UART4 TX)
    #   GPIO 9:  ALT4 RXD4          (UART4 RX)
    #   GPIO 11: OUTPUT level=0     (per gpio=11=ip,pn,dl -> becomes ip after firmware
    #                                completes; software then claims as OUTPUT LOW)
    #   GPIO 20: INPUT pull=DOWN    (was ALT4/SPI1_MOSI; now free for rotation sensor)
MSG

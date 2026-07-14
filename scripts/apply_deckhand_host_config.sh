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
#   (adding lines and filtering user-modified variants). Some of the
#   pins that Navigator claims collide with what the DeckHand PCB uses,
#   and some overlays we NEED aren't there by default:
#
#     dtoverlay=uart3          claims GPIO 4/5   -> collides with PCA9685 ~OE (GPIO 4)
#     dtoverlay=uart4          only enables TXD4/RXD4 (GPIO 8/9). GOOD
#                              as-is for DeckHand Rev-A firmware: we
#                              DO NOT want ,ctsrts because that would
#                              claim GPIO 10 as CTS4 and steal the
#                              rotation-sensor input pin.
#     (no SPI1 by default)     -> we need it enabled (dtoverlay=spi1-1cs)
#                              so /dev/spidev1.0 exists and the WS2812
#                              LED backend can drive GPIO 20 = SPI1 MOSI
#     gpio=11,24,25=op,pu,dh   drives GPIO 11 HIGH at boot -> that's the
#                              RS-485 DE line. We need it in ALT4 (RTS4)
#                              so the kernel PL011 driver can flip DE
#                              via TIOCSRS485 with hardware timing.
#
# The trick this script uses:
#   The reconciler's "already exists" check is `re.match(config, line)`,
#   which is a prefix match. So a line that STARTS with the required
#   text (but continues with something benign or a variant) satisfies
#   the reconciler. The reconciler also honours a `# custom` inline
#   comment as a protection marker on user lines. Combining the two:
#
#     dtoverlay=uart3-off       # custom - <reason>
#     dtoverlay=uart4           # custom - <reason>
#     dtoverlay=spi1-1cs        # custom - <reason>
#
#   For uart3 the `-off` suffix is not a real overlay, so the Pi
#   firmware silently skips the load — nothing claims GPIO 4/5.
#
#   For uart4 we use the bare overlay (no ,ctsrts) — this keeps GPIO 8/9
#   as TXD4/RXD4 and leaves GPIO 10 alone for the rotation sensor. RTS4
#   on GPIO 11 is recovered by the gpio=11=a4,pn override below.
#
#   For spi1-1cs the overlay is REAL: it enables SPI1 with a single
#   chip-select line, muxing GPIO 18/19/20/21 to ALT4 (CE0/MISO/MOSI/
#   SCLK) so /dev/spidev1.0 becomes available. The WS2812 LED backend
#   opens that device and clocks bit-encoded WS2812 data out MOSI.
#
#   For the GPIO 11 line we can't use the prefix trick because we want
#   GPIO 24 and 25 to keep their required op,pu,dh state. Instead we
#   leave the reconciler's required line in place AND add a per-pin
#   override right after it:
#
#     gpio=11,24,25=op,pu,dh
#     gpio=11=a4,pn             # custom - <reason>
#
#   Pi firmware processes gpio= lines top-to-bottom and per-pin settings
#   override earlier list entries for the same pin, so GPIO 11 boots as
#   ALT4 (RTS4) with no pull, while 24 and 25 stay op,pu,dh.

set -e

# Bookworm-based BlueOS images (1.5.x) keep the real config.txt under
# /boot/firmware/config.txt (the file at /boot/config.txt is just a stub
# that redirects readers to the new location). Older Bullseye images
# still put it at /boot/config.txt. Prefer the new location if present.
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

# Work on a temp file for atomicity.
TMP=$(mktemp)
sudo cp "$CONFIG" "$TMP"
sudo chown "$USER" "$TMP"

# Python does the heavy lifting: the reconciler considers a line
# "already there" only if it lives inside the *first* [pi4] section
# (bounded by the first blank line after [pi4]), and it strips any
# conflicting variant that isn't marked `# custom`. So we edit the
# first-section content in place — replacing conflicting lines with
# our protected variants and adding new lines just after [pi4].
python3 - "$TMP" <<'PY'
import re, sys
path = sys.argv[1]
lines = open(path).read().splitlines()

# Locate the first [pi4] block. Section ends at the first blank line
# or next [tag] (matches blueos_startup_update's get_or_append_section).
start = None
for i, line in enumerate(lines):
    if re.match(r"^\[pi4\]\s*$", line):
        start = i
        break
if start is None:
    lines.extend(["", "[pi4]"])
    start = len(lines) - 1
end = len(lines)
for i in range(start + 1, len(lines)):
    if lines[i] == "" or re.match(r"^\[.+\]\s*$", lines[i]):
        end = i
        break

# Rewrites applied to lines currently inside [start+1 .. end).
# Each entry: (line-regex-to-match, replacement-line).
REWRITES = [
    # dtoverlay=uart4[,ANYTHING] -> plain dtoverlay=uart4. We want TXD4/
    # RXD4 only on GPIO 8/9. NO ,ctsrts, because CTS4 would claim GPIO 10
    # which the Rev-A firmware needs as a plain input for the rotation
    # sensor. RTS4 on GPIO 11 is recovered by the gpio=11=a4,pn override
    # further down.
    (r"^dtoverlay=uart4(?:,\S+)?(?:\s+#.*)?$",
     "dtoverlay=uart4  # custom - DeckHand: TXD4/RXD4 only; GPIO 10 stays free for rotation sensor"),
    # dtoverlay=uart3  ->  no-op (frees GPIO 4 for PCA9685 ~OE).
    (r"^dtoverlay=uart3(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=uart3-off  # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE"),
    # dtoverlay=spi1[-Ncs][-off] -> enable spi1-1cs so /dev/spidev1.0 is
    # available for the WS2812 LED backend to clock data out GPIO 20
    # (SPI1 MOSI). Rev-A firmware requires this to be a REAL overlay
    # (no -off suffix); earlier firmware used spi1-3cs-off to KEEP
    # GPIO 20 free as a sensor input, but that role moved to GPIO 10.
    (r"^dtoverlay=spi1(?:-\d+cs)?(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=spi1-1cs  # custom - DeckHand: enable /dev/spidev1.0 for WS2812 LED on GPIO 20"),
]

def rewrite_or_none(line: str) -> str | None:
    for pat, replacement in REWRITES:
        if re.match(pat, line):
            return replacement
    return None

# Apply rewrites; also remember which replacements have already been placed
# (so we don't add them again in the append pass).
seen = set()
for i in range(start + 1, end):
    replacement = rewrite_or_none(lines[i])
    if replacement is not None:
        lines[i] = replacement
        seen.add(replacement)

# Lines we may need to append to the top of the section if they weren't
# produced by any rewrite above. These are dtoverlay= directives whose
# order doesn't matter for correctness (each overlay claims its own
# resources independently).
APPEND_AT_TOP = [
    "dtoverlay=uart4  # custom - DeckHand: TXD4/RXD4 only; GPIO 10 stays free for rotation sensor",
    "dtoverlay=uart3-off  # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE",
    "dtoverlay=spi1-1cs  # custom - DeckHand: enable /dev/spidev1.0 for WS2812 LED on GPIO 20",
]

# Recompute section body after in-place rewrites. Drop any stale variants
# of our custom lines (from a previous run of this script) so we don't
# pile up duplicates. gpio= per-pin overrides are handled *after* — Pi
# firmware processes gpio= directives top-to-bottom and later entries
# override earlier ones for the same pin, so we must position them
# after any list-form assignments (e.g. gpio=11,24,25=op,pu,dh).
def is_stale_custom(line: str) -> bool:
    return "# custom - DeckHand:" in line

section_body = [l for l in lines[start + 1 : end] if not is_stale_custom(l)]

# GPIO 11 per-pin override must come AFTER the reconciler's required
# gpio=11,24,25=op,pu,dh line. Find it in the section and inject
# right after; if it isn't present yet, append both lines together.
gpio_1124_re = re.compile(r"^gpio=[^=]*\b11\b[^=]*=op,pu,dh(\s+#.*)?$")
gpio_11_a4  = "gpio=11=a4,pn  # custom - DeckHand: force GPIO 11 to ALT4 (RTS4) instead of op,pu,dh"

found_1124 = None
for idx, l in enumerate(section_body):
    if gpio_1124_re.match(l):
        found_1124 = idx
        break
if found_1124 is not None:
    # Insert override on the very next line so it wins the pin.
    section_body.insert(found_1124 + 1, gpio_11_a4)
else:
    # Neither line present — add both at end so ordering is correct.
    section_body.append("gpio=11,24,25=op,pu,dh")
    section_body.append(gpio_11_a4)

# Add the top-of-section dtoverlay lines that aren't already there.
present_prefixes = {re.split(r"\s+#", l, maxsplit=1)[0] for l in section_body}
new_head = []
for want in APPEND_AT_TOP:
    prefix = re.split(r"\s+#", want, maxsplit=1)[0]
    if prefix not in present_prefixes:
        new_head.append(want)
        present_prefixes.add(prefix)

# Rebuild: [pi4] header, our custom head, then the (possibly-modified)
# section body, then whatever was outside.
result = (
    lines[: start + 1]
    + new_head
    + section_body
    + lines[end:]
)
open(path, "w").write("\n".join(result) + "\n")
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

After reboot verify the pins are in the expected state:
    raspi-gpio get 4,8,9,10,11,18,19,20,21
    # Expected on Rev-A firmware (before the extension claims them):
    #   GPIO 4:  INPUT              (was ALT4/UART3-RXD; now free for PCA9685 ~OE)
    #   GPIO 8:  ALT4 TXD4          (UART4 TX)
    #   GPIO 9:  ALT4 RXD4          (UART4 RX)
    #   GPIO 10: INPUT              (free for rotation sensor, lgpio claims it as alert-input)
    #   GPIO 11: ALT4 RTS4          (kernel-driven RS-485 DE via TIOCSRS485)
    #   GPIO 18: ALT4 SPI1 CE0      (claimed by spi1-1cs, unused otherwise)
    #   GPIO 19: ALT4 SPI1 MISO     (claimed by spi1-1cs, unused otherwise)
    #   GPIO 20: ALT4 SPI1 MOSI     (WS2812 LED data line via /dev/spidev1.0)
    #   GPIO 21: ALT4 SPI1 SCLK     (claimed by spi1-1cs, unused otherwise)
    #
    # Also confirm /dev/spidev1.0 exists (proves spi1-1cs loaded):
    #   ls /dev/spidev1.0
    #
    # And confirm the kernel-mode RS-485 ioctl is supported (needs Linux >= 5.14,
    # i.e. BlueOS 1.5.x on the Bookworm-based image or a Pi 5 install):
    #   uname -r     # should be 6.x, not 5.10.x
MSG

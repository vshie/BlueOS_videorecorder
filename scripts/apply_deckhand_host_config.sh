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
#   (adding lines and filtering user-modified variants). Four of the
#   pins that Navigator claims collide with what the DeckHand PCB uses:
#
#     dtoverlay=uart3          claims GPIO 4/5   -> collides with PCA9685 ~OE (GPIO 4)
#     dtoverlay=spi1-3cs       claims GPIO 16-21 -> collides with the
#                              release-shaft rotation sensor input (GPIO 20)
#     gpio=11,24,25=op,pu,dh   drives GPIO 11 HIGH at boot -> that's the
#                              RS-485 DE line, which asserts the transceiver
#                              driver for the entire boot window
#     dtoverlay=uart4          only enables TXD4/RXD4 (GPIO 8/9); we ALSO
#                              want RTS4 on GPIO 11 so the kernel PL011
#                              driver can flip DE for us in hardware
#
# The trick this script uses:
#   The reconciler's "already exists" check is `re.match(config, line)`,
#   which is a prefix match. So a line that STARTS with the required
#   text (but continues with something benign or a variant) satisfies
#   the reconciler. The reconciler also honours a `# custom` inline
#   comment as a protection marker on user lines. Combining the two:
#
#     dtoverlay=uart3-off       # custom - <reason>
#     dtoverlay=spi1-3cs-off    # custom - <reason>
#     dtoverlay=uart4,ctsrts    # custom - <reason>
#
#   For uart3 / spi1-3cs the `-off` suffix is not a real overlay, so the
#   Pi firmware silently skips the load — nothing claims those pins.
#
#   For uart4 the `,ctsrts` parameter is a REAL overlay option: it
#   enables CTS4 (GPIO 10) and RTS4 (GPIO 11) in addition to TXD4/RXD4,
#   putting GPIO 11 into ALT4 (RTS4) mode from boot. That's exactly what
#   the PL011 driver needs to drive the SN65HVD75 DE/~RE line via
#   TIOCSRS485 in kernel space, with hardware-precision timing.
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
    # dtoverlay=uart4  ->  dtoverlay=uart4,ctsrts  (adds RTS4 on GPIO 11
    # so the kernel can drive DE via TIOCSRS485 with hardware timing).
    (r"^dtoverlay=uart4(?:,ctsrts)?(?:\s+#.*)?$",
     "dtoverlay=uart4,ctsrts  # custom - DeckHand: RTS4 on GPIO 11 for kernel TIOCSRS485 DE"),
    # dtoverlay=uart3  ->  no-op (frees GPIO 4 for PCA9685 ~OE).
    (r"^dtoverlay=uart3(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=uart3-off  # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE"),
    # dtoverlay=spi0-led -> no-op (frees GPIO 10 which would otherwise
    # be forced to SPI0_MOSI even without a WS281x device).
    (r"^dtoverlay=spi0-led(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=spi0-led-off  # custom - DeckHand: GPIO 10 must not go to SPI0_MOSI"),
    # dtoverlay=spi1-3cs -> no-op (frees GPIO 16..21 including
    # GPIO 20 which the rotation-sensor input uses).
    (r"^dtoverlay=spi1-3cs(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=spi1-3cs-off  # custom - DeckHand: GPIO 20 needed as rotation-sensor input"),
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
    "dtoverlay=uart4,ctsrts  # custom - DeckHand: RTS4 on GPIO 11 for kernel TIOCSRS485 DE",
    "dtoverlay=uart3-off  # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE",
    "dtoverlay=spi0-led-off  # custom - DeckHand: GPIO 10 must not go to SPI0_MOSI",
    "dtoverlay=spi1-3cs-off  # custom - DeckHand: GPIO 20 needed as rotation-sensor input",
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
    raspi-gpio get 4,8,9,10,11,20
    # Expected (before the extension claims them):
    #   GPIO 4:  INPUT              (was ALT4/UART3-RXD; now free for PCA9685 ~OE)
    #   GPIO 8:  ALT4 TXD4          (UART4 TX)
    #   GPIO 9:  ALT4 RXD4          (UART4 RX)
    #   GPIO 10: ALT4 CTS4          (unused on the PCB but comes with the ctsrts overlay)
    #   GPIO 11: ALT4 RTS4          (kernel-driven RS-485 DE via TIOCSRS485)
    #   GPIO 20: INPUT pull=DOWN    (was ALT4/SPI1_MOSI; now free for rotation sensor)
    #
    # Also confirm the kernel-mode RS-485 ioctl is supported (needs Linux >= 5.14,
    # i.e. BlueOS 1.5.x on the Bookworm-based image or a Pi 5 install):
    #   uname -r     # should be 6.x, not 5.10.x
MSG

#!/usr/bin/env bash
set -e

VENDOR_ID="2bdf"
PRODUCT_ID="0001"
RULE_FILE="/etc/udev/rules.d/99-hikrobot-camera.rules"

echo "Creating udev rule for Hikrobot camera ${VENDOR_ID}:${PRODUCT_ID} ..."

sudo tee "$RULE_FILE" > /dev/null <<EOF
# Hikrobot MV-CS020-10UC USB camera
SUBSYSTEM=="usb", ATTR{idVendor}=="${VENDOR_ID}", ATTR{idProduct}=="${PRODUCT_ID}", MODE:="0666", TAG+="uaccess"
EOF

echo "Reloading udev rules ..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Done."

echo
echo "Please unplug and replug the Hikrobot camera."
echo "Then check with:"
echo "  lsusb | grep -i '${VENDOR_ID}:${PRODUCT_ID}'"
echo "  ls -l /dev/bus/usb/*/* | grep -i ''"

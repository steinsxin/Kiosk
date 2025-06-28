#!/bin/bash

# Ensure the autostart directory exists for the current user
AUTOSTART_DIR="$HOME/.config/autostart"
echo "Ensuring directory exists: $AUTOSTART_DIR"
mkdir -p "$AUTOSTART_DIR"

# Copy the desktop entry file to the autostart directory
echo "Copying logo_display.desktop to $AUTOSTART_DIR/"
cp ../systemd_srv/logo_display.desktop "$AUTOSTART_DIR/"

echo "logo_display installation completed."

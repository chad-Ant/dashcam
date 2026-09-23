#!/bin/bash
#
# setup_samba_footage.sh — share the dashcam footage folder on the network.
#
# Makes /media/$USER/backup/footage appear on a Windows PC as
#   \\<jetson-ip>\footage        (Explorer address bar, or "Map network drive")
# READ-ONLY: the PC can copy footage off but never delete or modify it; the
# dashcam's own loop overwrite manages disk space.  Login is the Jetson
# account with a separate Samba password (set by this script).
#
# What it does (idempotent — safe to re-run):
#   1. apt-get install samba            (needs internet the first time)
#   2. adds a [footage] share to /etc/samba/smb.conf between marker lines
#      (the original is backed up once to smb.conf.dashcam-backup)
#   3. sets the Samba password for $SHARE_USER (prompts; skipped if one exists)
#   4. opens the Samba ports in ufw when ufw is active
#   5. enables and restarts smbd
#
# Usage: ./setup_samba_footage.sh      (re-runs itself with sudo)
# Undo:  ./remove_samba_footage.sh

SHARE_NAME="footage"
SHARE_USER="${SUDO_USER:-$USER}"
FOOTAGE_DIR="/media/$SHARE_USER/backup/footage"
SMB_CONF="/etc/samba/smb.conf"
BEGIN_MARK="# >>> dashcam footage share (managed by setup_samba_footage.sh) >>>"
END_MARK="# <<< dashcam footage share <<<"

if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi
set -e

if [ "$SHARE_USER" = "root" ]; then
    echo "run this as your normal user (it uses sudo itself), not as root"; exit 1
fi
if [ ! -d "$FOOTAGE_DIR" ]; then
    echo "footage folder $FOOTAGE_DIR not found"; exit 1
fi

echo "[1/5] installing samba..."
if ! dpkg -s samba >/dev/null 2>&1; then
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y samba
else
    echo "      already installed"
fi

echo "[2/5] configuring the [$SHARE_NAME] share..."
[ -f "$SMB_CONF.dashcam-backup" ] || cp "$SMB_CONF" "$SMB_CONF.dashcam-backup"
# Drop any previous managed block, then append a fresh one.
sed -i "/^$(printf '%s' "$BEGIN_MARK" | sed 's/[][\.*^$/]/\\&/g')\$/,/^$(printf '%s' "$END_MARK" | sed 's/[][\.*^$/]/\\&/g')\$/d" "$SMB_CONF"
cat >> "$SMB_CONF" <<EOF
$BEGIN_MARK
[$SHARE_NAME]
   comment = dashcam footage (read-only)
   path = $FOOTAGE_DIR
   browseable = yes
   read only = yes
   guest ok = no
   valid users = $SHARE_USER
$END_MARK
EOF
testparm -s "$SMB_CONF" >/dev/null   # abort (set -e) on a broken config

echo "[3/5] Samba password for '$SHARE_USER'..."
if pdbedit -L 2>/dev/null | grep -q "^$SHARE_USER:"; then
    echo "      already set (change it with: sudo smbpasswd $SHARE_USER)"
else
    echo "      choose the password Windows will ask for:"
    smbpasswd -a "$SHARE_USER"
fi

echo "[4/5] firewall..."
if command -v ufw >/dev/null && ufw status | grep -q "^Status: active"; then
    ufw allow samba
else
    echo "      ufw not active — nothing to open"
fi

echo "[5/5] starting smbd..."
systemctl enable smbd >/dev/null 2>&1
systemctl restart smbd

IP=$(ip -4 -o addr show scope global | awk '$2 !~ /^(docker|br-|veth)/ {sub(/\/.*/, "", $4); print $4; exit}')
echo
echo "done. On Windows open:  \\\\${IP:-<jetson-ip>}\\$SHARE_NAME"
echo "      user: $SHARE_USER   password: the Samba password set above"

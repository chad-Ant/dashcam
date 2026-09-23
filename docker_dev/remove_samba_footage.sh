#!/bin/bash
#
# remove_samba_footage.sh — undo setup_samba_footage.sh.
#
# Removes the [footage] share block from /etc/samba/smb.conf and restarts
# smbd.  Footage is untouched.  With --purge it also deletes the Samba
# password entry, closes the ufw Samba rule and uninstalls samba.
# Safe to run more than once.
#
# Usage: ./remove_samba_footage.sh [--purge]    (re-runs itself with sudo)

SHARE_USER="${SUDO_USER:-$USER}"
SMB_CONF="/etc/samba/smb.conf"
BEGIN_MARK="# >>> dashcam footage share (managed by setup_samba_footage.sh) >>>"
END_MARK="# <<< dashcam footage share <<<"

if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

if [ -f "$SMB_CONF" ] && grep -qF "$BEGIN_MARK" "$SMB_CONF"; then
    sed -i "/^$(printf '%s' "$BEGIN_MARK" | sed 's/[][\.*^$/]/\\&/g')\$/,/^$(printf '%s' "$END_MARK" | sed 's/[][\.*^$/]/\\&/g')\$/d" "$SMB_CONF"
    echo "removed the [footage] share from $SMB_CONF"
    systemctl restart smbd 2>/dev/null
else
    echo "no managed [footage] share in $SMB_CONF"
fi

if [ "$1" = "--purge" ]; then
    pdbedit -L 2>/dev/null | grep -q "^$SHARE_USER:" && smbpasswd -x "$SHARE_USER"
    if command -v ufw >/dev/null && ufw status | grep -q "^Status: active"; then
        ufw delete allow samba 2>/dev/null
    fi
    if dpkg -s samba >/dev/null 2>&1; then
        systemctl disable --now smbd nmbd 2>/dev/null
        apt-get remove -y samba
    fi
    echo "purged Samba (password entry, firewall rule, package)"
fi
echo "done."

#!/usr/bin/env bash
# Install the NetworkManager-owned Runner field AP without deleting any
# existing netplan or NetworkManager client profiles. Run as root.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/network"
PROFILE=runner-field-ap
SSID=Runner-Paddock
ADDRESS=10.42.0.1/24
PSK_FILE=/etc/runner-field-ap.psk
NETPLAN_FILE=/etc/netplan/99-runner-network-manager.yaml
DNSMASQ_FILE=/etc/NetworkManager/dnsmasq-shared.d/90-runner-captive.conf
PORTAL_BIN=/usr/local/lib/runner/captive_portal.py
PORTAL_UNIT=/etc/systemd/system/runner-captive-portal.service
PROFILE_FILE=/etc/NetworkManager/system-connections/runner-field-ap.nmconnection
MODE="${1:-apply}"

usage() {
    echo "usage: sudo $0 [--check|--activate]" >&2
    exit 2
}

case "$MODE" in
    apply|--check|--activate) ;;
    *) usage ;;
esac

if [[ "$MODE" == "--check" ]]; then
    echo "Runner field network — drift report (no changes):"
    command -v nmcli >/dev/null || { echo "  MISSING NetworkManager/nmcli"; exit 1; }
    command -v iw >/dev/null || echo "  MISSING iw (capability diagnostics)"
    nmcli -f NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY connection show
    echo
    nmcli -f 802-11-wireless.ssid,802-11-wireless.mode,802-11-wireless.band,802-11-wireless-security.key-mgmt,ipv4.method,ipv4.addresses,ipv6.method connection show "$PROFILE"
    echo
    systemctl --no-pager --lines=0 status runner-captive-portal.service
    exit
fi

if [[ "$EUID" -ne 0 ]]; then
    echo "must run as root (sudo network/install.sh)" >&2
    exit 1
fi

if [[ -n "${RUNNER_AP_PSK:-}" ]]; then
    if (( ${#RUNNER_AP_PSK} < 8 || ${#RUNNER_AP_PSK} > 63 )); then
        echo "RUNNER_AP_PSK must contain 8 to 63 characters" >&2
        exit 1
    fi
    umask 077
    printf '%s\n' "$RUNNER_AP_PSK" >"$PSK_FILE"
elif [[ ! -s "$PSK_FILE" ]]; then
    umask 077
    openssl rand -hex 12 >"$PSK_FILE"
fi
PSK="$(tr -d '\r\n' <"$PSK_FILE")"
if (( ${#PSK} < 8 || ${#PSK} > 63 )); then
    echo "$PSK_FILE must contain one WPA passphrase of 8 to 63 characters" >&2
    exit 1
fi
chmod 0600 "$PSK_FILE"

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y network-manager dnsmasq-base iw

if ! iw list | grep -Eq '^[[:space:]]+\* AP$'; then
    echo "wlan0 radio/driver does not advertise AP mode; refusing to install" >&2
    exit 1
fi

install -D -m 0600 "$SRC/99-runner-network-manager.yaml" "$NETPLAN_FILE"
install -D -m 0644 "$SRC/runner-captive-dnsmasq.conf" "$DNSMASQ_FILE"
install -D -m 0755 "$SRC/captive_portal.py" "$PORTAL_BIN"
install -D -m 0644 "$SRC/runner-captive-portal.service" "$PORTAL_UNIT"

# Generate NetworkManager profiles for the existing netplan declarations.
# No existing YAML or NetworkManager connection is removed or rewritten.
netplan generate
systemctl enable NetworkManager.service

# Generate the keyfile without starting NetworkManager. This cannot contend
# with systemd-networkd or preempt the connection carrying this install.
profile_tmp="$(mktemp)"
trap 'shred --remove "$profile_tmp"' EXIT
nmcli --offline connection add \
    type wifi ifname wlan0 con-name "$PROFILE" ssid "$SSID" \
    connection.autoconnect yes \
    connection.interface-name wlan0 \
    connection.autoconnect-priority 100 \
    connection.autoconnect-retries 0 \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless.channel 6 \
    802-11-wireless.powersave 2 \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.group ccmp \
    802-11-wireless-security.psk "$PSK" \
    ipv4.method shared \
    ipv4.addresses "$ADDRESS" \
    ipv4.never-default yes \
    ipv6.method disabled >"$profile_tmp"
install -D -m 0600 "$profile_tmp" "$PROFILE_FILE"
shred --remove "$profile_tmp"
trap - EXIT
if systemctl is-active --quiet NetworkManager.service; then
    nmcli connection reload
fi

systemctl daemon-reload
systemctl enable --now runner-captive-portal.service

# Paddock's unit is repository-linked on the deployed Runner. Restarting only
# this browser gateway does not alter any Pi-side mode or recording runtime.
if systemctl cat runner-paddock-web.service >/dev/null 2>&1; then
    systemctl restart runner-paddock-web.service
fi

if [[ "$MODE" == "--activate" ]]; then
    # This is the explicitly disruptive path: hand rendered devices from
    # systemd-networkd to NetworkManager now, then replace the Wi-Fi client.
    netplan apply
    nmcli connection up "$PROFILE"
fi

echo
echo "Installed $SSID (${ADDRESS%/*}). The AP will be preferred at next boot."
echo "WPA passphrase: $PSK"
echo "Paddock: http://${ADDRESS%/*}:8000/"
echo "Captive landing page: http://${ADDRESS%/*}/"
if [[ "$MODE" != "--activate" ]]; then
    echo "Current networking was left active; use '$0 --activate' to switch now."
fi

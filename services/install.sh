#!/usr/bin/env bash
# Coherent Runner systemd deploy: symlink every tracked unit from this repo
# into /etc/systemd/system, install the PWM helper and the narrow polkit rule,
# reload systemd, and enable the persistent tier (but never the two
# supervisor-owned mode units). Idempotent. Run with sudo.
#
#   sudo services/install.sh            # apply
#   services/install.sh --check         # report drift only, change nothing
#
# After this, `sudo services/install.sh --restart` (or a reboot) brings up one
# consistent current-version stack in dependency order.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO/services"
SYS=/etc/systemd/system
POLKIT=/etc/polkit-1/rules.d/49-runner-mode-units.rules
PWM_BIN=/usr/local/sbin/setup-runner-pwm

# Persistent tier: symlinked AND enabled so a reboot brings the stack up.
PERSISTENT=(
    runner-pwm-setup.service
    runner-motor.service
    runner-encoder.service
    runner-battery.service
    runner-telemetry.service
    runner-foxglove.service
    runner-stop-enforcer.service
    runner-local-control.service
    runner-drive-adapter.service
    runner-command-authority.service
    runner-mode-supervisor.service
    runner-map-executor.service
    runner-paddock-web.service
)
# Supervisor-owned: symlinked so systemd can find them, but NEVER enabled --
# runner-mode-supervisor is their sole start/stop owner.
SUPERVISED=(
    runner-mode-mapping.service
    runner-mode-autonomy.service
)
# Restarted, in order, by --restart (operator/application tier only; the
# hardware tier is left alone unless traction is disconnected).
RESTART_ORDER=(
    runner-stop-enforcer.service
    runner-command-authority.service
    runner-local-control.service
    runner-drive-adapter.service
    runner-mode-supervisor.service
    runner-map-executor.service
    runner-paddock-web.service
)

MODE="apply"
[[ "${1:-}" == "--check" ]] && MODE="check"
[[ "${1:-}" == "--restart" ]] && MODE="restart"

say() { printf '  %s\n' "$*"; }

check_one() {
    local unit="$1"
    local link="$SYS/$unit"
    local target="$SRC/$unit"
    if [[ ! -e "$link" ]]; then
        say "MISSING   $unit"
    elif [[ ! -L "$link" ]]; then
        say "COPY      $unit (regular file, not a symlink to the repo)"
    elif [[ "$(readlink -f "$link")" != "$target" ]]; then
        say "WRONGLINK $unit -> $(readlink "$link")"
    else
        local state
        state="$(systemctl is-enabled "$unit" 2>/dev/null || true)"
        say "ok        $unit (is-enabled: ${state:-unknown})"
    fi
}

if [[ "$MODE" == "check" ]]; then
    echo "Runner service deploy — drift report (no changes):"
    for u in "${PERSISTENT[@]}"; do check_one "$u"; done
    for u in "${SUPERVISED[@]}"; do check_one "$u"; done
    [[ -e "$PWM_BIN" ]] && say "ok        $PWM_BIN" || say "MISSING   $PWM_BIN"
    if [[ "$EUID" -ne 0 ]]; then
        say "unknown   $POLKIT (need root to stat /etc/polkit-1/rules.d)"
    elif [[ -e "$POLKIT" ]]; then
        say "ok        $POLKIT"
    else
        say "MISSING   $POLKIT"
    fi
    exit 0
fi

if [[ "$EUID" -ne 0 ]]; then
    echo "must run as root (sudo services/install.sh)" >&2
    exit 1
fi

if [[ "$MODE" == "restart" ]]; then
    echo "Restarting the Runner operator/application tier in order:"
    for u in "${RESTART_ORDER[@]}"; do
        say "restart $u"
        systemctl restart "$u"
    done
    systemctl --no-pager --lines=0 status "${RESTART_ORDER[@]}" || true
    exit 0
fi

echo "Installing Runner units from $SRC"

install -m 0755 "$SRC/setup-runner-pwm" "$PWM_BIN"
say "installed $PWM_BIN"

install -m 0644 "$SRC/49-runner-mode-units.rules" "$POLKIT"
say "installed $POLKIT"

for unit in "${PERSISTENT[@]}" "${SUPERVISED[@]}"; do
    ln -sfn "$SRC/$unit" "$SYS/$unit"
    say "linked    $unit"
done

systemctl daemon-reload
say "daemon-reload done"

# The two runner-mode-* units carry no [Install] section on purpose, so they
# are never passed to `systemctl enable` and cannot autostart at boot;
# runner-mode-supervisor is their sole start/stop owner. Never `disable` them
# either -- that would delete the authoritative symlink just created.
systemctl enable "${PERSISTENT[@]}"
say "enabled persistent tier"

# Warn if a stale wants/requires link would autostart a mode unit anyway.
if compgen -G "$SYS/*.target.wants/runner-mode-*" > /dev/null; then
    say "WARNING: stale mode-unit wants link(s):"
    ls -1 "$SYS"/*.target.wants/runner-mode-* || true
fi

echo
echo "Done. Now either reboot, or apply the running stack with:"
echo "    sudo $0 --restart"
echo "(restart the hardware tier -- runner-motor / runner-encoder -- yourself,"
echo " with traction power disconnected, if their binaries changed.)"

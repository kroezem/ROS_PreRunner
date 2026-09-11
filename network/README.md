# Runner field Wi-Fi

The default field connection is the NetworkManager profile
`runner-field-ap`: a WPA2-only 2.4 GHz AP named **Runner-Paddock** on channel
6. Runner is always `10.42.0.1/24`; NetworkManager's `shared` IPv4 method runs
the local DHCP/DNS service for clients. Paddock is
`http://10.42.0.1:8000/`, with `http://makro-runner.local:8000/` available
when the client supports mDNS. The fixed address is the canonical fallback.

The captive service listens separately on port 80. DHCP option 114 and local
DNS answers lead OS connectivity probes to its small landing page. Its **OPEN
PADDOCK** link targets a new, normal-browser window at port 8000. Nothing
proxies or redirects Paddock's API or same-origin `/ws` traffic.

## Install

The host currently starts from netplan/systemd-networkd. This installer adds a
later netplan renderer selection, installs NetworkManager, and generates
NetworkManager connections from all existing netplan definitions. It never
deletes or rewrites an existing client profile or Tailscale configuration.

```sh
# Generate and print a random 24-character WPA passphrase on first install:
sudo network/install.sh

# Or choose the passphrase on first install / rotate it later:
sudo RUNNER_AP_PSK='choose-at-least-8-characters' network/install.sh

# Inspect profile and captive service without changing anything:
network/install.sh --check
```

The normal install leaves the connection carrying the install session active;
the AP wins by autoconnect priority at the next boot. To switch immediately:

```sh
sudo network/install.sh --activate
```

The generated passphrase is root-readable at `/etc/runner-field-ap.psk`.

## Switching modes

List the preserved client connections, then deliberately leave field mode:

```sh
nmcli -f NAME,TYPE,DEVICE connection show
sudo nmcli connection down runner-field-ap
sudo nmcli connection up '<existing client profile name>'
```

Return to field mode with:

```sh
sudo nmcli connection up runner-field-ap
```

Manually taking the AP down suppresses its autoconnect for the current boot;
its priority makes it the default again after reboot.

## AP and Tailscale

The installed `brcmfmac` device exposes one `wlan0`. At inspection time this
host did not have `iw`, so no valid AP+STA interface-combination declaration
could be verified. This setup therefore does not create a virtual station or
depend on concurrency: field AP mode intentionally replaces Wi-Fi client mode.
`tailscaled` and its configuration are left enabled and untouched. Tailscale
will normally be offline in field mode unless Runner also has an independent
uplink such as Ethernet. Switch to a preserved Wi-Fi client profile when a
tailnet connection is wanted.

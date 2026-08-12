# ROS2 Topics Visible but No Data Across Machines — Host Firewall on Unicast UDP (2026-08-12)

Symptom: the data-collection service on 227 (10.60.2.227) could **list** all
of 115's ROS2 topics (`/roh/*`, `/vr/*`, `/arm/*`) but every stream stayed
ABSENT — no data ever arrived. A coworker's laptop (even on a different
subnet, 10.60.77.x WiFi) received the same topics without any setup.

## Root cause

DDS discovery is **not** multicast-only. The handshake has two legs:

1. **Multicast** announcements (`239.255.0.1:7400/7401`, SPDP) — "I'm here".
2. **Unicast UDP** between the peers' announced ports (SEDP + user data) —
   where endpoint matching and all actual topic data flow.

227's host firewall (iptables/nftables REJECT rule) allowed inbound
multicast but **rejected inbound unicast UDP**. Multicast alone lets you
*see* topic names; without the unicast leg, discovery never completes and no
data flows — in either direction (115 also couldn't see 227's test topic).

## Evidence (how this was pinned down)

tcpdump on 115 while 227 published a test topic:

```
IP 10.60.2.227.35529 > 239.255.0.1.7400: UDP, length 280     ← 227's multicast arrives at 115
IP 10.60.2.227 > 10.60.2.115: ICMP ... unreachable - admin prohibited filter
                                                              ← 227's firewall REJECTs the unicast reply
```

`admin prohibited filter` = an iptables/nftables REJECT rule firing on the
receiving host. Outbound + established-return traffic on 227 worked (SSH,
NFS), which is why nothing else looked broken.

## Fix

Allow inbound UDP from the lab subnet on the affected machine:

```bash
sudo iptables -I INPUT -s 10.60.2.0/24 -p udp -j ACCEPT
# or, if ufw manages the ruleset:
sudo ufw allow from 10.60.2.0/24
```

Surgical variant: DDS needs UDP 7400–7500 (discovery) + the ephemeral range
32768–60999 (data) from the lab subnet.

Verify: `ros2 topic hz /roh/left/joint_state` should immediately show ~30 Hz.

## Reminders for future cross-machine debugging

- **`ros2 topic list` working is NOT proof of connectivity.** It only means
  multicast discovery announcements arrived. Always confirm data delivery
  (`ros2 topic hz` / `echo --once`) before blaming anything else.
- The one-line discriminator: topic list works + echo hangs = unicast leg
  broken (firewall, wrong announced interface) — not a network policy issue.
- A host with multiple interfaces (Docker bridges, VPNs) or any firewall is
  a suspect by default. Fresh machines with default ACCEPT "just work",
  which is exactly why the coworker's laptop was the control case.
- `ICMP admin prohibited filter` on the wire = a REJECT rule on the sending
  host of that ICMP. It names the guilty machine directly.
- If the fast check list is needed: same `ROS_DOMAIN_ID`, same RMW
  (`echo $RMW_IMPLEMENTATION` — CycloneDDS and FastDDS never inter-discover),
  `ROS_LOCALHOST_ONLY=0`, then firewall.

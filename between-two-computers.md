# B

## Bandwidth measurement

Server (IP:192.168.0.2)

```bash
iperf3 -s
```

Client

```bash
# 4 parallel TCP streams, 30-second test, 1‑second reports
# iperf3 -c SERVER_IP -P 4 -t 30 -i 1, e.g.:
iperf3 -c 192.168.0.2 -P 4 -t 30 -i 1
```

## Bridge networks

Setup: Two computers: M and S. M has two network cards but S only one. M's interface `eno1` has access to the internet. S's interface `eth0` is connected to M's second interface, `enp1s0`. The goal is to make S have access to the internet.


### Execute at M

```bash
# Ensure forwarding
sudo sysctl -w net.ipv4.ip_forward=1


# Allow forwarding between interfaces (explicitly)
sudo iptables -A FORWARD -i enp1s0 -o eno1 -j ACCEPT
sudo iptables -A FORWARD -i eno1 -o enp1s0 -m state --state RELATED,ESTABLISHED -j ACCEPT

sudo iptables -t nat -A POSTROUTING -o eno1 -j MASQUERADE
```

#### Revert with

```bash
sudo sysctl -w net.ipv4.ip_forward=0
sudo iptables -F
sudo iptables -t nat -F

# or simply reboot
```

#### And to make changes persist

```bash
# Make IP forwarding permanent
echo 'net.ipv4.ip_forward=1' | sudo tee -a /etc/sysctl.conf

# Save iptables rules
sudo apt install iptables-persistent -y
sudo iptables-save | sudo tee /etc/iptables/rules.v4
```

# D

## Discover others in subnet

Say you sit at a pc with IP address `192.168.0.200` and you need to identify which devices it can reach in its subnet:

```bash
# Scan the entire subnet, i.e. look under 192.168.0.1--192.168.0.255
nmap -sn 192.168.0.0/24
```

# F

## File exchange (simple; for one-off type of situations)

Receiver (IP:192.168.0.2)

```bash
# netcat -l -p PORT > FILE, e.g.:
netcat -l -p 1234 > file_rx.txt
```

Transmitter

```bash
# nc RX_IP PORT -q 0 < file_tx.file
netcat 192.168.0.2 1234 -q 0 < file_tx.txt
```

or

```bash
# cat file.txt | netcat RX_IP PORT -q 0, e.g.:
cat file_tx.txt | netcat 192.168.0.2 1234 -q 0
```

## File exchange (heavyweight)

Use [9001/copyparty](https://github.com/9001/copyparty?tab=readme-ov-file#quickstart) as a file server

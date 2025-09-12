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

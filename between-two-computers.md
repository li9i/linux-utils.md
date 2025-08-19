## Measure bandwidth

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

### Exchange a file

Receiver (IP:192.168.0.2)

```bash
# netcat -l PORT > FILE, e.g.:
netcat -l 1234 > file.txt
```

Transmitter

```bash
# netcat RX_IP PORT -q 0, e.g.:
netcat 192.168.0.2 -q 0
```

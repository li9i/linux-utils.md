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

## File exchange (lightweight)

Either execute the second code fence directly or add it to your `.bashrc`. Then simply navigate to the directory in which the file(s) you want to send to another party reside and execute `httphere` there. You should see something like the following. Finally simply communicate to the other party the url `http://169.4.4.14:10337`: she should be seeing the contents of the directory.

```bash
> httphere
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
100    13  100    13    0     0     65      0 --:--:-- --:--:-- --:--:--    65
Serving HTTP on http://169.4.4.14:10337 from directory [/home/afilot]
Serving HTTP on http://169.4.4.14:10337 from directory [/home/afilot]
```

```bash
httphere()
{
    script="$(basename "$0")";
    function display_help ()
    {
        echo "$script - Quicky spin up a simple HTTP server for a directory.";
        echo "Usage: $script [arguments]";
        echo;
        echo "  -h/--help - Print help and exit.";
        echo "  -p/--port - Set the port number to bind to (defaults to 10337).";
        echo "  -d/--directory - Set the root directory to serve from (defaults to current).";
        exit 0
    };
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h | --help)
                display_help
            ;;
            -p | --port)
                port="$2";
                shift
            ;;
            -d | --directory)
                directory="$2";
                shift
            ;;
            *)
                echo "Unknown option: $1";
                display_help
            ;;
        esac;
        shift;
    done;
    port=${port:-10337};
    directory=${directory:-$(pwd)};
    if [[ $port -lt 1024 ]]; then
        echo "Warning: Ports below 1024 require root privileges.";
        echo "Please run this script with 'sudo' if you intend to use port $port.";
        exit 1;
    fi;
    ip=$(curl ifconfig.me/ip);
    echo "Serving HTTP on http://$ip:$port from directory [$directory]";
    python3 - <<EOF
import os
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
    def list_directory(self, path):
        try:
            list = os.listdir(path)
        except OSError:
            self.send_error(404, "No permission to list directory")
            return None

        list.sort(key=lambda a: a.lower())

        def human_readable_size(size):
            for unit in ['bytes', 'KB', 'MB', 'GB', 'TB']:
                if size < 1024.0:
                    return f"{size:.2f} {unit}"
                size /= 1024.0
            return f"{size:.2f} PB"

        r = []
        r.append('<!DOCTYPE HTML>')
        r.append('<html><head>')
        r.append('<meta charset="utf-8">')
        r.append('<title>Directory listing for %s</title>' % self.path)
        r.append('<style>')
        r.append('table { width: 100%%; border-collapse: collapse; }')
        r.append('th, td { padding: 0px; text-align: left; }')
        r.append('th { background-color: #f4f4f4; }')
        r.append('</style>')
        r.append('</head><body>')
        r.append('<h1>Directory listing for %s</h1>' % self.path)
        r.append('<hr>')
        r.append('<table>')
        r.append('<tr><th>Name</th><th>Size</th><th>Last Modified</th></tr>')

        if self.path != '/':
            r.append('<tr><td><a href="..">Parent Directory</a></td><td></td><td></td></tr>')

        for name in list:
            fullname = os.path.join(path, name)
            displayname = name
            size = os.path.getsize(fullname)
            last_modified = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(fullname)))

            if os.path.isdir(fullname):
                displayname = name + "/"
                r.append('<tr><td><a href="%s">%s</a></td><td>DIR</td><td>%s</td></tr>' % (displayname, displayname, last_modified))
            else:
                readable_size = human_readable_size(size)
                r.append('<tr><td><a href="%s">%s</a></td><td>%s</td><td>%s</td></tr>' % (displayname, displayname, readable_size, last_modified))

        r.append('</table>')
        r.append('<hr>')
        r.append('</body></html>')
        encoded = '\n'.join(r).encode('utf-8', 'surrogateescape')

        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)
        return None

if __name__ == '__main__':
    server_address = ('0.0.0.0', $port)
    httpd = ThreadingHTTPServer(server_address, CustomHTTPRequestHandler)
    print(f"Serving HTTP on http://$ip:$port from directory [$directory]")
    httpd.serve_forever()
EOF
}
```

## File exchange (heavyweight)

Use [9001/copyparty](https://github.com/9001/copyparty?tab=readme-ov-file#quickstart) as a file server

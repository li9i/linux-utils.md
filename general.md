# L

## List installed packages in chronological order (ascending)

```bash
zcat -f /var/log/dpkg.log* 2>/dev/null | grep " install " | awk '{print $1, $2, $4}' | sort
```

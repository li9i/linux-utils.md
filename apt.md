# L

## List installed packages in chronological order (ascending)

```bash
zcat -f /var/log/dpkg.log* 2>/dev/null | grep " install " | awk '{print $1, $2, $4}' | sort
```

# S

## Split `apt-get install` to `download` stage and `install` stage

```bash
# Optional
sudo apt-get update

# Stage 1: Download package + dependencies
sudo apt-get install --download-only <package-name>

# Stage 2: Install without downloading
sudo apt-get install --no-download <package-name>
```

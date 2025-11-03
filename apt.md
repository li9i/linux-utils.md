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

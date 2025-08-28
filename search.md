# G

## grep for string in semi-known filenames

```bash
# easier to remember
grep -rEIn "sensors" --include="docker-compose*"
```

```bash
# harder to remember
find . -type f -name "docker-compose*" -exec grep "sensors" {} +
```

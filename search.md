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

## grep for string and get unique matching words

```bash
string="GetBase"
grep -rEIn "$string" . | grep -o "\w*$string\w*" | sort -u
```

## grep for string and show k preceding, n succeeding lines

```bash
string="GetBase"
k=5
n=5
grep -rEIn -A $k -B $n $string
```

## grep for string in files and open each one in separate window

```bash
grep -rl "string to search" | xargs -n1 gvim
```

## grep for string and exclude lines containing specific words

```bash
grep -rEIn "sensors" -v -e EXCLUDE_PATTERN_1 -e EXCLUDE_PATTERN_2
```

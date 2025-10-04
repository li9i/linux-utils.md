# L

## List installed packages in chronological order (ascending)

```bash
zcat -f /var/log/dpkg.log* 2>/dev/null | grep " install " | awk '{print $1, $2, $4}' | sort
```

# Passwords

## Automated Password Generator

```bash
apg -a 1 -n 5 -m 16 -x 20 -E O0l1I -M SNCL
```

Generate `n=5` passwords
- minimum `m=16` chars length
- maximum `x=20` chars length
- `E`xclude ambiguous chars
- `M`ix `S`pecial chars, `N`umbers, `C`apital letters, `L`owercase letters

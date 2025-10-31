# L

## List installed packages in chronological order (ascending)

```bash
zcat -f /var/log/dpkg.log* 2>/dev/null | grep " install " | awk '{print $1, $2, $4}' | sort
```

# P

## Generate passwords

```bash
apg -a 1 -n 5 -m 16 -x 20 -E O0l1I -M SNCL
```

Generate `n=5` passwords
- minimum `m=16` chars length
- maximum `x=20` chars length
- `E`xclude ambiguous chars
- `M`ix `S`pecial chars, `N`umbers, `C`apital letters, `L`owercase letters

# V

## Vim: enable backup, undo, swap

- keep a original state of a file after you open it
- enable ability to undo even if you close a file
- concentrate swap files

```bash
mkdir -p ~/.vim/{backup,swap,undo}
```

and place into `.vimrc` the following

```vimrc
" Enable Backup Files
set backup
set backupdir=~/.vim/backup//
set backupext=.bak

" Enable Swap Files (for crash recovery)
set swapfile
set directory=~/.vim/swap//

" Enable Undo Files (Persistent Undo)
set undofile
set undodir=~/.vim/undo//
```

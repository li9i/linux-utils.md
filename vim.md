[.vimrc](https://github.com/li9i/dotfiles/blob/master/.vimrc)

# E

## Enable backup, undo, swap

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

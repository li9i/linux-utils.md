[.vimrc](https://github.com/li9i/dotfiles/blob/master/vim/dot-vimrc)

# D

## Delete lines containing a specific pattern

```vim
:g/pattern/d
```

## Delete lines NOT containing a specific pattern

```vim
:g!/pattern/d
```

---

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

---

# J

## Join all lines

We can do this in a number of ways:

```vim
:%j    " join all lines, collapsing whitespace to a single space (like normal J).
:%j!   " join with no space inserted between lines (like gJ).
ggVGJ  " visually select the whole file, then J to join.
:5,20j " join a specific range (lines 5–20).
```

---

# P

## Paste clipboard content into the command line

When typing an ex command (e.g. `:w` and a path), it is possible to paste the path from the clipboard directly into the command line.

### Method 1: `Ctrl-R` then `+` or `*`

1. Type `:w ` (example usage)
2. Press `Ctrl-R` then `+` to paste from the system clipboard, or
   Press `Ctrl-R` then `*` to paste from the X11 primary selection
3. Press `Enter`

In other words the sequence is

```
:w <Ctrl-R>+<Enter>
```

### Method 2: `Ctrl-R Ctrl-O +` (paste literally)

If the clipboard contains special characters (like `%` which Vim would expand), use:

```
:w <Ctrl-R><Ctrl-O>+
```

This inserts the register content **literally**, without interpreting it as Vim keycodes.

> [!NOTE]
> Make sure Vim was compiled with clipboard support:
>
> ```vim
> :echo has('clipboard')
> ```
>
> Should return `1`. If it returns `0`, you either need `vim-gtk3` (Debian/Ubuntu) or `vim` with `+clipboard`.
>
> `+` is the system clipboard register, `*` is the X11 primary selection (middle-mouse).

---

# R

## Replace text and preserve case

Say you want to replace all occurrences of the word `abc` with `xyz` but `abc` also appears as `ABC` or `Abc` and you want to preserve the current case (e.g. turn `ABC` to `XYZ` and `Abc` to `Xyz`). All you need is the [vim-abolish plugin](https://github.com/tpope/vim-abolish).

### Usage

```vim
:%S/abc/xyz/g
```

>[!CAUTION]
Note the capital S

### Installation

```bash
mkdir -p ~/.vim/pack/tpope/start
cd ~/.vim/pack/tpope/start
git clone https://tpope.io/vim/abolish.git
vim -u NONE -c "helptags abolish/doc" -c q
```

---

# U

## Unwrap lines

> With `gq` you wrap a paragraph; how do you unwrap it?

The fastest, most idiomatic way to unwrap a paragraph back into a single long line is:

```vim
vipJ
```

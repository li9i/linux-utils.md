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

# R

## Run something once on boot

```bash
crontab -e
```

e.g. restart Docker container abc 10 sec after boot

```bash
# @reboot applies to both `shutdown -h` and `-r`
@reboot sleep 10; docker container restart abc
```

# S

## Suppress lines of output on command line

Say you run a command which yields lines that clutter your overview of the output, e.g.

```bash
roslaunch vdbfusion_ros vdbfusion_mapping_irr_real_v2.launch 
[...]
I1103 09:05:33.363229   622 transform.cpp:65] Transformer init success                                                                                                                                                                   
I1103 09:05:33.373440   622 vdbfusion_mapper.cpp:403] ==========> Setting Config Success, start for running                                                                                                                              
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera (parent laser_camera) at time 1762160733.659102 according to authority unknown_publisher                                                               
         at line 277 in /tmp/binarydeb/ros-noetic-tf2-0.7.10/src/buffer_core.cpp                                                                                                                                                         
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera (parent laser_camera) at time 1762160733.659102 according to authority unknown_publisher                                                               
         at line 277 in /tmp/binarydeb/ros-noetic-tf2-0.7.10/src/buffer_core.cpp                                                                                                                                                         
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame camera (parent laser_camera) at time 1762160733.659102 according to authority unknown_publisher 
```

You may suppress lines containing the words `TF_REPEATED_DATA` and `buffer_core` by appending to the command `2> >(grep -v -e PATTERN1 -e PATTERN2 ...)`:

```bash
roslaunch vdbfusion_ros vdbfusion_mapping_irr_real_v2.launch  2> >(grep -v -e TF_REPEATED_DATA -e buffer_core)
```

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

# C

## Caja

### Open terminal in this directory keyboard shortcut

Open

```bash
~/.config/caja/accels
```

and add or set the following:

```
(gtk_accel_path "<Actions>/ExtensionsMenuGroup/CajaOpenTerminal::open_terminal" "F4")
```

Apparently

> [F4 is T]he convention in Nautilus, Thunar and Dolphin.


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

---

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

---

# S

## Steghide

### Embed Secret Data into an Image
To hide a text file (`secret.txt`) inside a cover image (`cover.jpg`):

```bash
steghide embed -cf cover.jpg -ef secret.txt
```
where
```
-cf (cover file): The original image or audio file.
-ef (embed file): The secret payload file you want to hide.
```

### Extract Hidden Data

To extract the embedded file from the stego image:

```bash
steghide extract -sf cover.jpg
```
where
```
-sf (stego file): The image/audio file containing the hidden data.
```

### Useful Optional Flags

| Flag | Description | Example |
| --- | --- | --- |
| `-p` | Pass passphrase directly in command line | `steghide embed -cf cover.jpg -ef secret.txt -p "MyPassword123"` |
| `-e` | Specify encryption algorithm (e.g., `aes-256`, `blowfish`) | `steghide embed -cf cover.jpg -ef secret.txt -e aes-256` |
| `-z` | Specify compression level (1–9; `0` disables compression) | `steghide embed -cf cover.jpg -ef secret.txt -z 9` |

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

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

## Publish a package to a Launchpad PPA

A PPA is an apt repository hosted on Launchpad. You never upload a `.deb`: you upload a signed *source* package and Launchpad's build farm compiles the binary itself, per Ubuntu series, in a clean container with no network. So everything the build needs must be declared in `debian/control` (`Build-Depends`) and present in the source tree. Build it locally in a clean container of each series first, which catches a missing build dependency before Launchpad does.

Placeholders below: `<user>` Launchpad username, `<pkg>` source package name, `<KEYID>` GPG key id, `<series>` Ubuntu codename (`noble`, `resolute`, ...).

### First-time setup

#### 1. Launchpad account

1. Register at https://launchpad.net
2. Choose the username carefully. It is public, hard to change, and becomes the PPA address `ppa:<user>/<pkg>`.
3. Sign the Code of Conduct at https://launchpad.net/codeofconduct . Uploads are refused until this is done.

#### 2. GPG key

Three things must share one email address or the upload is rejected: the GPG key identity, the `debian/changelog` signer, and a confirmed email on the Launchpad account. Use the same address as `Maintainer` in `debian/control`, and one whose inbox you can read.

```bash
gpg --full-generate-key
# (1) RSA and RSA, 4096 bits, real name and that email address

gpg --list-secret-keys --keyid-format=long
gpg --keyserver keyserver.ubuntu.com --send-keys <FINGERPRINT>
```

Register it at https://launchpad.net/~/+editpgpkeys by pasting the fingerprint. Launchpad replies with an *encrypted* mail. Save its body to a file, decrypt it, open the link inside:

```bash
gpg -d confirmation.pgp
```

#### 3. Create the PPA

Go to https://launchpad.net/~<user>/+activate-ppa and set the **URL** field to exactly `<pkg>`, which fixes the address as `ppa:<user>/<pkg>`. Display name and description are free text. The PPA then lives at https://launchpad.net/~<user>/+archive/ubuntu/<pkg>

#### 4. Local tools

```bash
sudo apt update && sudo apt install -y devscripts dput
```

Gives `debuild`, `dch`, `debsign` and `dput`. The build toolchain itself is not needed on the host if the source package is built in a container.

### Per release

#### 1. Bump the version

Bump the upstream version wherever the build system declares it (e.g. `meson.build`, `configure.ac`), then add a new top entry to `debian/changelog`:

```bash
DEBEMAIL="you@example.com" DEBFULLNAME="Your Name" \
    dch -v 1.2.0-1 --distribution <series> "What changed in this release."
```

By hand, the format is exactly:

```
<pkg> (1.2.0-1) <series>; urgency=medium

  * What changed in this release.

 -- Your Name <you@example.com>  <RFC 2822 date>
```

`date -R` prints a correctly formatted date.

#### 2. Build the source package

From the source tree:

```bash
debuild -S -sa -us -uc
```

`-S` source-only, `-sa` includes the orig tarball (use `-sd` to omit it on later uploads of the same upstream version), `-us -uc` leaves signing for the next step. Output lands in the parent directory:

- `<pkg>_<upstream>.orig.tar.gz` the upstream tarball, shared by every series
- `<pkg>_<version>.debian.tar.xz` the packaging
- `<pkg>_<version>.dsc` the source description
- `<pkg>_<version>_source.changes` what you upload

Build one per series you target, changing only the changelog distribution and the version suffix (see below). Sanity check each `.changes`: `Architecture: source`, and `Distribution` matching the series.

#### 3. Sign

```bash
debsign -k <KEYID> ../<pkg>_*_source.changes
```

Signs the `.dsc`, updates the checksums in the `.changes`, then signs the `.changes`. Run it in your own terminal so the passphrase prompt works. Your signature is your authorisation to upload.

#### 4. Upload

```bash
dput ppa:<user>/<pkg> ../<pkg>_*_source.changes
```

`dput` understands the `ppa:` shorthand with no config file, and uploads anonymously: the GPG signature is the authentication, not a login. Order between series does not matter, since Launchpad accepts an identical orig tarball it already holds.

#### 5. Watch, then install

Launchpad mails **Accepted** or **Rejected** within minutes, and a rejection is nearly always a signature or version clash, named in the mail. Follow the build at https://launchpad.net/~<user>/+archive/ubuntu/<pkg>/+packages ; roughly 10 to 40 minutes, then a publishing delay. Once **Published**:

```bash
sudo add-apt-repository ppa:<user>/<pkg>
sudo apt update
sudo apt install <pkg>
```

Later releases arrive through normal `apt upgrade`. Remove the PPA with `sudo add-apt-repository --remove ppa:<user>/<pkg>`, or `ppa-purge` to also roll packages back to the archive versions.

### Version rules

Launchpad never accepts a version string it has already seen, and the orig tarball for a given upstream version is immutable once it holds one.

- **Any change outside `debian/`** (code, build files, even dropping a build dependency) is a new *upstream* version, because the orig tarball is the tree minus `debian/`. Bump the upstream version and reset the revision to `-1`, so a fresh orig is generated. Keeping the upstream version and bumping only the revision either reuses the stale orig, silently shipping the old source, or gets rejected as a different tarball under an existing name.
- **Packaging-only change** (only under `debian/`, e.g. a `Build-Depends` fix): keep the upstream version, bump the Debian revision, e.g. `1.2.0-1` then `1.2.0-2`. The orig is unchanged and reused.
- **Per-series suffix**: give every series' upload `~ubuntuYY.MM.1`, e.g. `1.2.0-1~ubuntu24.04.1` and `1.2.0-1~ubuntu26.04.1`. The `~` sorts below the plain revision and the numeric series orders correctly, so an OS upgrade pulls the newer series' build. Use the same scheme on every series.
- **Re-upload after a rejection or a failed build**: bump the trailing number of the suffix, `~ubuntu24.04.1` then `~ubuntu24.04.2`.

### More than one series

Launchpad builds each series separately, so upload one source package per series, differing only in the changelog distribution and the `~ubuntuYY.MM.1` suffix. A series must be able to carry the package: every `Build-Depends` has to exist there, and a newer series can drop one that an older one had. To stop serving a series, stop uploading for it.

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

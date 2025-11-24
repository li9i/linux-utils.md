# linux-utils.md

Notes on sweet commands/utilities in linux

Table of Contents
=================

* [`apt`](#apt)
* [Audio](#audio)
* [Between two computers](#between-two-computers)
* [Docker](#docker)
* [General](#general)
* [`git`](#git)
* [ROS 2](#ros-2)
* [Search](#search)
* [Vim](#vim)
* [YouTube](#youtube)

## [`apt`](apt.md)

- [List installed packages in chronological order](apt.md#list-installed-packages-in-chronological-order-ascending)
- [Split `apt-get install` to `download` stage and `install` stage](apt.md#split-apt-get-install-to-download-stage-and-install-stage)

## [Audio](audio.md)

- [Extract audio from video](audio.md#extract-audio-from-video)

## [Between two computers](between-two-computers.md)

- [Bandwidth Measurement](between-two-computers.md#bandwidth-measurement)
- [Bridge Networks](between-two-computers.md#bridge-networks)
- [Discover others in subnet](between-two-computers.md#discover-others-in-subnet)
- [Exchange files (netcat)](between-two-computers.md#file-exchange-simple-for-one-off-type-of-situations)
- [Exchange files (python)](between-two-computers.md#file-exchange-lightweight)
- [Exchange files (copyparty)](between-two-computers.md#file-exchange-heavyweight)

## [Docker](docker.md)

- [Cleanup unnecessary space](docker.md#cleanup-unnecessary-space)
- [Kill running containers](docker.md#kill-running-containers)
- [Reboot Docker](docker.md#reboot-docker)
- [View names of running containers](docker.md#view-names-of-running-containers)
- [View occupied space](docker.md#view-occupied-space)

## [General](general.md)

- [Generate Passwords](general.md#generate-passwords)
- [Run something once on boot](general.md#run-something-once-on-boot)
- [Suppress lines of output on command line](general.md#suppress-lines-of-output-on-command-line)

## [`git`](git.md)

- [Configuration of keys for different repos](git.md#configuration-of-keys-for-different-repos)
- [Copy commit from `<branch-b>` to `<branch-a>`](git.md#copy-commit-from-branch-b-to-branch-a)
- [List files modified or yet untracked](git.md#list-files-modified-or-yet-untracked)
- [Remove changes introduced by commit](git.md#remove-changes-introduced-by-commit)
- [Revert multiple commits in one new commit](git.md#revert-multiple-commits-in-one-commit)
- [View file modifications](git.md#view-file-modifications)
- [View ignored files](git.md#view-ignored-files)
- `diff`
  - [Find files changed between commits filtered by regex](git.md#find-files-changed-between-commits-filtered-by-regex)
- `add`
  - [Add everything except untracked files](git.md#add-everything-except-untracked-files)
  - [Forgot to add file(s) to latest commit and realised before pushing?](git.md#forgot-to-add-files-to-latest-commit-and-realised-before-pushing)
  - [Unstage everything after `add`](git.md#unstage-everything-after-add)
- `fetch`
  - [Fetch by specifying key location](git.md#fetch-by-specifying-key-location)
- `push`
  - [Push by specifying key location](git.md#push-by-specifying-key-location)
  - [Squash commits before pushing](git.md#squash-commits-before-pushing)
- `rebase`
  - [Rebase and automatically accept changes from branch](git.md#rebase-and-automatically-accept-changes-from-branch)
  - [Squash commits during rebase](git.md#squash-commits-during-rebase)
- `merge`
  - [Merge branch but don't commit changes yet](git.md#merge-branch-but-dont-commit-changes-yet)
  - [Squash commits during merge](git.md#squash-commits-during-merge)

## [ROS 2](ros2.md)

- [`colcon`](ros2.md#colcon)
- [Migrate signing key](ros2.md@migrate.signing-key)
- [Publish static transform on the fly](ros2.md#publish-static-transform-on-the-fly)
- [Release Package via Apt](ros2.md#release-package-via-apt)
- [Save map published at custom topic](ros2.md#save-map-published-at-custom-topic)
- `topic echo`
- [Echo specific field of message](ros2.md#echo-field-of-message)
- [Throttle frequency](ros2.md#throttle-frequency)

## [Search](search.md)

- [`grep` for string in subset of all files](search.md#grep-for-string-in-semi-known-filenames)
- [`grep` for string and get unique matching words](search.md#grep-for-string-and-get-unique-matching-words)
- [`grep` for string and show preceding/succeeding lines](search.md#grep-for-string-and-show-k-preceding-n-succeeding-lines)
- [`grep` for string in files and open each one in separate window](search.md#grep-for-string-in-files-and-open-each-one-in-separate-window)
- [`grep` for string and exclude lines containing specific words](search.md#grep-for-string-and-exclude-lines-containing-specific-words)

## [Vim](vim.md)

- [Enable backup, undo, swap](vim.md#vim-enable-backup-undo-swap)

## [YouTube](youtube.md)

- [Download video](youtube.md#download-video)
- [Download video but extract only audio](youtube.md#download-video-but-extract-only-audio)

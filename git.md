# C

## Configuration of keys for different repos

```bash
$ cat .ssh/config
```

```bash
# Check the Host line
# Then the remote origin url should be modified from, e.g.
# git@github.com:epfl-lasa/robetarme_ros2_wp5.2.git
# to
# git@github.com-robetarme-user:epfl-lasa/robetarme_ros2_wp5.2.git
Host github.com-robetarme-user
  HostName github.com
  User git
  IdentityFile ~/.ssh/robetarme_passphraseless_key
  IdentitiesOnly yes

# Default identity
Host github.com
  HostName github.com
  User git
  IdentityFile ~/.ssh/id_ed25519_iti581_github
  IdentitiesOnly yes
```

## Copy commit from `<branch-b>` to `<branch-a>`

Let the commit in question have hash `abc123`. Then

```bash
git checkout <branch-a>
git cherry-pick abc123
```

# F

## Fetch by specifying key location

```bash
GIT_SSH_COMMAND='ssh -i /path/to/private_key' git fetch origin
```

# L

## List files modified or yet untracked

```bash
git status --porcelain | awk '{print $2}'
```

If you add the following to `~/.gitconfig` then issuing `git changed-files` will constitute an abbreviation

```gitignore
[alias]
    changed-files = "!git status --porcelain | awk '{print $2}'"
```

# M

## Merge branch but don't commit changes yet

Useful when you need to merge changes but not incorporate all of them

```bash
# Switch to target branch (e.g. master)
git checkout target-branch

# Merge but don't commit yet. The use of --no-ff is essential
git merge --no-commit --no-ff source-branch

# Reset a specific file to its state before merge
git checkout HEAD -- path/to/file.txt

# Now commit the merge
git commit -m "Merge source-branch, excluding changes to file.txt"
```

Then discard the changes in files you don't need with

```bash
git checkout HEAD -- <file-paths>
```

# P

## Push by specifying key location

```bash
GIT_SSH_COMMAND='ssh -i /path/to/private_key' git push origin main
```

# R

## Remove changes introduced by commit

### Create a New Commit That Undoes It (`git revert`)

```bash
git revert <commit-hash>
```

### Remove from history

With interactive rebase

```bash
git rebase -i <base-branch-or-commit>
```

Locate the commit you want to remove in the editor. Then change the word `pick` to `drop`. Save and close the editor. If rebasing requires resolving conflicts then resolve them and continue the rebase with

```bash
git rebase --continue
```

## Revert multiple commits in one commit

### Commits are sequential

```bash
git revert --no-commit OLDEST_COMMIT_HASH^..NEWEST_COMMIT_HASH

git commit -m "Revert multiple commits: description of changes"
```

### Commits are scattered

```bash
# Revert specific commits (in chronological order - oldest first)
git revert --no-commit COMMIT_HASH_1 COMMIT_HASH_2 COMMIT_HASH_3

git commit -m "Revert specific commits"
```

# S

## Squash commits before pushing

You've made N incremental local commits but you want to squash them into one commit before pushing it.

```bash
git rebase -i HEAD~N
```

An editor will open with a list like:

```bash
pick 123abc Commit message 1
pick 456def Commit message 2
pick 789ghi Commit message 3
...
pick 012jkl Commit message N
```

Change all but the first entries to `squash`

```bash
pick   123abc Commit message 1
squash 456def Commit message 2
squash 789ghi Commit message 3
...
squash 012jkl Commit message N
```

Save and close the editor. Git will then prompt you to edit the new commit message. You can combine or edit the messages as needed. Then save and close the editor again. After that the commits will be squashed into one.

## Squash commits during merge

You need to merge a branch onto `master` but you want to squash all commits from `b` into one commit.

```bash
git checkout master
git merge --squash <branch>
```

This does not create a merge commit. Instead it stages all the changes from <branch> into the index. Commit the squashed changes with

```bash
git commit -m "A single commit message summarizing all changes between <branch> and master"
```

# V

## View file modifications

### Introduced in specific commit

```bash
git show <commit-hash> -- <file>
```

### Introduced by all affecting commits

```bash
git log -p --follow <file>

# List commits only
# git log --follow <file>
```

### When file is only staged

```bash
git difftool --cached <file>
```

### Side by side in command line

```bash
git diff --word-diff
```

## View ignored files

```bash
git ls-files --others --ignored --exclude-standard
```

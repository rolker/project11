# General git notes

The starting point for Project11 is the repository contained in the CCOMJHC organization on GitHub.

https://github.com/CCOMJHC/project11



## Submodules

Project11 uses git submodules to organize the multitude of repositories that make up the framework. The toplevel project is project11 and the submodules are contained as subdirectories in catkin_ws/src/. In order to prevent git from flagging directories as non being under version contol, packages that are not submodules can be placed in catkin_ws/src/local. If later such a package becomes a core part of Project11, it can be added as a submodule and moved to catkin_ws/src/.

The toplevel project keeps track of specific versions of submodules. This means that submodules should be commited and pushed before commiting the toplevel project.

### Cloning Project11 with submodules

The `--recursive` is used to automatically clone submodules when cloning a repository.

    git clone --recursive https://github.com/CCOMJHC/project11.git

If a project gets cloned without the `--recursive` flag, it can be updated to get the sobmodules.

    git submodule update --init --recursive

### Adding a submodule

    cd ~/project11
    git submodule add https://github.com/CCOMJHC/cool_package catkin_ws/src/cool_package
    
To add a relative submodule, which allows users to chose between https or ssh when cloning...

    git submodule add ../cool_package catkin_ws/src/cool_package

### Useful links

- https://codewinsarguments.co/2016/05/01/git-submodules-vs-git-subtrees/
- https://dev.to/dwd/git-submodules-revisited-1p54
- https://github.blog/2016-02-01-working-with-submodules/


## Commit messages

How to write commit messages:

https://chris.beams.io/posts/git-commit/

## git peer to peer

From https://gist.github.com/datagrok/5080545

`git config --global alias.serve "daemon --verbose --export-all --base-path=.git --reuseaddr --strict-paths .git/"`

Use your new git serve like so:

 1. Run git serve. "Ready to rumble," it will report. Git is bad-ass.
 2. Find out your IP address. Say it's 192.168.1.123.
 3. Say "hey Jane, I'm not ready/able to push these commits up to origin, but you can fetch my commits into your clone by running git fetch git://192.168.1.123/"
 4. Press ctrl+c when you don't want to serve that repo any longer.

## Merging different repos

https://stackoverflow.com/questions/13040958/merge-two-git-repositories-without-breaking-file-history/

## GitFlow

Project11 does not require the following workflow at the moment, but it may be used if desired. The use of GitHub forks already provides a different development space for each user.

https://nvie.com/posts/a-successful-git-branching-model/

# General git notes

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

## Submodules 
 
### Submodules

https://codewinsarguments.co/2016/05/01/git-submodules-vs-git-subtrees/
https://dev.to/dwd/git-submodules-revisited-1p54
https://github.blog/2016-02-01-working-with-submodules/

### Subrepos

https://github.com/ingydotnet/git-subrepo/blob/master/Intro.pod
http://blog.s-schoener.com/2019-04-20-git-subrepo/

### Merging

https://stackoverflow.com/questions/13040958/merge-two-git-repositories-without-breaking-file-history/
 

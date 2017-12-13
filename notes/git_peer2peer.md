From https://gist.github.com/datagrok/5080545

`git config --global alias.serve "daemon --verbose --export-all --base-path=.git --reuseaddr --strict-paths .git/"`

Use your new git serve like so:

 1. Run git serve. "Ready to rumble," it will report. Git is bad-ass.
 2. Find out your IP address. Say it's 192.168.1.123.
 3. Say "hey Jane, I'm not ready/able to push these commits up to origin, but you can fetch my commits into your clone by running git fetch git://192.168.1.123/"
 4. Press ctrl+c when you don't want to serve that repo any longer.

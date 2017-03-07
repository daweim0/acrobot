git pull

git add --all

if [NOT] %* == "" (
git commit -m %* 
) else ( 
git commit -m " _ "
)

git push
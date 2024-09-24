#### init
echo "# ExpressTrain" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M master
git remote add origin https://github.com/YuZane/ExpressTrain.git
git push -u origin master



## push
git remote add origin https://github.com/YuZane/ExpressTrain.git
git branch -M master
git push -u origin master


## 烧录
pyocd flash --erase chip --target MM32F5333D7PV "C:\Users\yuzhen\Desktop\ExpressTrain\ExpressTrain.hex"  --pack="C:\Users\yuzhen\Desktop\ExpressTrain\MindMotion.MM32F5330_DFP.0.5.1.pack"


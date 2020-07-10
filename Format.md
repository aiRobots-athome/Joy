# Git Commit
## 依照類別去做分類
在Title的一開始標示一下這次更新的方向

*  Feat: 如果專案裡面有加**新的功能**
*  Fix: 如果前一次的程式碼有**bug**，然後在這次**被修正**了
*  Docs: 新增任何有關**文書**的處理
*  Style: 有做任何**美觀**上的改善，比如說縮排、名稱改善
*  Refactor: 對整個程式碼做基礎性的調整，但是在**功能上沒變**，比如說辨識方法改變，但輸出的結果都是一樣的
*  Perf: 加速程式碼的更新，但是不是新功能
*  Test: 增加或修改**測試**的程式碼，
*  Chore: 其他的瑣事，如果不在上面任意類別就屬於這類，等於others

## Commit 格式

類別： 簡單的敘述這次的更新

==一行空白==(如果更新的項目很少，下面的細項即可省略)

編號. 修改細項(開頭大寫)

## Commit Example
Docs: Add Rules for git commit

1. Add rules of commiting for the team to follow
2. Follow, or I'll smash your head to the fridge

## Exception
1. 如果是系統自己跳的commit，Ex. Merge，那就可以不在這限制
2.  如果更新的項目很多，則類別寫最重要的，然後在下面的細項講清楚哪些修正
3.  持續更新中

## Refernece
[develar/commit-message-format.md](https://gist.github.com/develar/273e2eb938792cf5f86451fbac2bcd51)

[Write your commit messages in the right way](https://kaihao.dev/posts/write-your-commit-messages-the-right-way)

# Code
## Variable
少於五個字！！

* Constant: 只用作啟動後只會**更新一次(含)以下**的變數，並集中放在程式碼最上面。格式為**全部大寫**，單字間用**底線**連接 Ex. `THIS_IS_CONST_VAR`
* Global: 只用作啟動後只會**更新一次(含)以下**的變數，並集中放在程式碼最上面。格式為**全部大寫**，單字間用**底線**連接 Ex. `This_Is_Global_Var`
* Local: 用在各個function中，減少變數重疊的問題，通常是在function中用完就會被捨棄的。格式為**全部小寫**，單字間用**底線**連接 Ex. `this_is_local_var`

## Function
請盡可能重複使用自己寫過的function，**不要每次都硬刻**！！！！！！

格式為**全部小寫**，單字間用**底線**連接
ex. `this_is_func()`

## Function commit
所有的function都必須要寫註解，讓其他人比較容易閱讀

格式為:

* python single line

```python

def func():
	"""doc string"""
	.........
```


* python multiline: 

```python

def func(param1, param2, ...):
	"""
	What the function docs
	@param param1 - Perpose of param 1
	@param param2 - Perpose of param 2
	.
	.
	.
	@retval retval1 - meaning of return value retval1
	@retval retval2 - meaning of return value retval2
	"""
	.........
	return retval1, retval2
```

* c++: 

```cpp
/**
 * What the function does
 * 
 * @param param1 - Perpose of param 1
 * @param param2 - Perpose of param 2
 * .
 * .
 * .
 * @retval retval1 - meaning of return value retval1
 */
tamplate func(param1, param2, ...){
	.........
	return retval1
}
```

## Exception
如果是call別人寫好的library則可直接用，不用把別人的程式碼格式改成我們的

## Reference
[Google Python Style Guide](http://google.github.io/styleguide/pyguide.html?fbclid=IwAR2R50OEzmkUoi7YkrpIrEwURn601WzdcIjvOOaACdhccbv0MOpcBFn5XIY#s3.16-naming)

[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html#Naming)



# 暫時這樣，之後再補充
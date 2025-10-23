---
title: 「新手向」听说你也要学C/C++?
tag:
  - C/C++
index_img: 
mathjax: false
excerpt: 'B试图用一个链接来打发走所有来问B怎么学C/C++的人'
date: 2025-10-23 15:08:06
category: 笔记
---
{% note primary %}
**B,你写这个文章干啥？**  
实不相瞒，时不时刷出一个人来问我怎么学c / 怎么装vscode / vscode怎么报错 真的非常...  
~~而且装过vscode的人都知道一开始用vscode有多麻烦~~  
总之B不想隔空喊话教操作了，嗯  
---
**本文会介绍啥呢**  
1. 必要工具的安装
   * Visual Studio Code
   * Git
   * MinGW64
2. 怎么让你的vscode正常编译运行程序
3. 通过什么途径你可以学会使用C/C++

本文不会涉及C/C++语法的具体知识，因为照抄已有的成系统的资料是没有意义的事情
{% endnote %}  

{% note danger %}
**老登请自行关掉这个页面，左转备赛去**
{% endnote %}  

# 环境配置
## 下载MinGW64 (必需)
MinGW-w64 是 Minimalist GNU for Windows 的缩写，它是一个开源的 C/C++ 编译器工具链，旨在将 GNU 编译器集合（GCC）移植到 Windows 平台上。  
说人话就是，**没有这个文件你的C/C++就别想正确编译**  
{% note light %}
**选B,选B,那为啥Dev-cpp这些软件下载了就可以运行了，没听说过MinGW64啊**  
那是因为这样软件自带了MinGW64所以你不用自己再去搞一个来了   
vscode还是需要自己去配  
{% endnote %}

你可以去搜索MinGW64然后去[官网](https://www.mingw-w64.org/)自己下载  
我推荐你使用[MinGW64国内镜像站](https://files.1f0.de/mingw/),因为官网的下载速度比较慢  
![](/img/c/MinGW64-1.png)  
下载完成后解压到一个位置上(比如`C:\Program Files\`)  
![](/img/c/MinGW64-2.png)
然后找到解压后的`MinGW64`文件夹的位置，复制其中`bin`文件夹的位置  
![](/img/c/MinGW64-3.png)  
![](/img/c/MinGW64-4.png)  
关闭文件资源管理器，打开`设置`  
搜索`系统环境变量`，然后点击`编辑系统环境变量`选项进入控制面板  
![](/img/c/MinGW64-5.png)
![](/img/c/MinGW64-6.png)
配置好后关闭这些页面就好了

## 下载VScode (必需)
VScode全称`Visual Studio Code`,是目前世界上最好用的IDE工具之一  
而且它完全免费  
你可以在[官网](https://code.visualstudio.com/)上下载VScode  
如果你遇到了下载的问题可以让我给你发个安装包(如果你有我好友的话)  
{% note warning %}
跟随安装包的指引完成安装就好了，这个应该不用我截一堆图来带吧  
{% endnote %}  

{% note danger %}
**提示**  
我们用的是蓝色的`Visual Studio Code`，不是紫色的`Visual Studio`   
`Visual Studio`是用来开发软件用的，**换而言之，你都打开这篇文章了，这软件和目前的你是没有什么关系的**  
~~而且Studio没有code泛用，UI也没有code好看~~  
{% endnote %}  

安装完成后打开`VScode`，找到`扩展`，下载下面这些扩展:
* Chinese
* C/C++
* C/C++ Extension Pack
* C/C++ Themes
* C/C++ Compile Run
* Doxygen Documentation Generator
* GitHub Actions
* GitHub Copilot
* GitHub Pull Requests
* GitLens
* Django
* Dracula Theme Official
* indent-rainbow
* Keil Assistant (如果你不是RM电控组请跳过，这和你无关)

![](/img/c/VScode-1.png)  

安装完成后来测试一下能否正常编译运行  
![](/img/c/VScode-2.png)  
![](/img/c/VScode-3.png)  
![](/img/c/VScode-4.png)  
能运行Hello, World就能运行别的  

{% note warning %}
**VScode给头文件报错的自觉看过来**  
绝对是你没有好好安装`MinGW64`！！！(恼)  
翻回去自己好好看看哪一步有问题的！！！
{% endnote %}   

{% fold info @RoboMaster电控组的同学看过来 %}
Keil5的工程文件不能直接打开，但是借助插件`Keil Assisant`就可以  
![](/img/c/VScode-Keil-1.png)  
![](/img/c/VScode-Keil-2.png)  
![](/img/c/VScode-Keil-3.png)  
![](/img/c/VScode-Keil-4.png)  
值得注意的是，编译调试还是得回到Keil中去，这没办法
{% endfold %}  

## 下载Git (推荐)
`git`是目前最泛用的*分布式版本控制软件*(没有之一)  
使用git可以让你编写代码时回档到任意一个提交过的版本，而git的功能远远不止回档这么简单  
下载git可以去[官网](https://git-scm.com/install/)下载  
如果你遇到了下载的问题可以让我给你发个安装包(如果你有我好友的话)  
{% note danger %}
**请不要修改git的默认安装路径**  
否则VScode无法正确识别  
~~而且Git的体积也很小，占不了你C盘的多少空间~~  
{% endnote %}  

关于git的使用，本文不作介绍，B站上有数量非常多的讲解视频，你完全可以自行学习  

# 怎么练习使用 C/C++  

语法知识请左转[菜鸟教程](https://www.runoob.com/)，菜鸟教程足够详细且易懂了  
练习使用请右转[洛谷](https://luogu.com.cn/),为信息学竞赛服务的`OJ(Online Judge)平台`,解决你的问题差不多是*高射炮打蚊子——你就说是不是能打死蚊子*  
![注册登录后在题单处找到6组入门题单，每个题单通过3题你就会个大概了](/source/img/c/luogu.png)  

---END---
---
---
title: 分布式版本管理Git的简易介绍
tag:
  - git
index_img: /img/git/gitlogo.png
mathjax: false
excerpt: '汇集分布式管理软件git的基本概念和常用操作'
date: 2026-04-10 16:13:13
category: 笔记
---

# 什么是git 

[Git](https://git-scm.com/) 是一个开源的分布式版本控制系统，用于敏捷高效地处理任何或小或大的项目。  
通过使用git，你能够高效地实现对代码仓库的**版本控制**和**协同工作**

# 使用环境

在本地端，我们将会结合[Visual Studio Code](https://code.visualstudio.com/)来使用git。
这种*半图形化半命令行*的使用方式兼顾**直观**和**使用效率**，相比纯命令行的**Git Bash**更加易于上手  

[下载Git](https://git-scm.com/install/windows)时应该注意: **不要修改git默认下载路径**，否则可能出现下载了git但是vscode识别不到git存在的情况。

推荐在vscode中额外添加`git gragh`、`gitlens`等git相关的插件  

在远程端，CUBOT已有一个GitHub组织，我们就使用Github作为我们的远程仓库管理平台  

## 第一次使用git?
如果你是第一次使用git，那么你需要在本地配置两个信息才能开展后续的使用  
```bash
git config --global user.name "your nick name"
git config --global user.email youremail@mail.com
```
如果不给git配置你的`name`和`email`信息，git将在后续阻止你的使用  

> 你并不需要填写你真实的姓名和邮箱  
> 事实上，git只是记录两串字符串而已，并不会去验证邮箱的可用性  
> 这一步的意义在于多人协作时能够获得某一段代码的作者及其联系方式  

# 基本概念
## 仓库初始化

### 本地创建仓库 （git init）
用vscode打开我们需要进行版本管理的文件夹。如果这个文件夹下无`/.git`，*源代码管理*侧边栏将会显示两个提示按钮  
![](/img/git/git-init-1.png)  
直接点击第一个按钮就可以完成初始化。这时git会在当前文件夹下新建一个文件夹`/.git`来管理所有的版本信息  

### 远程克隆仓库 （git clone）
远程克隆仓库我们选择使用命令行  
首先去我们要克隆的仓库复制git仓库的URL  
![](/img/git/git-init-2.png)  
然后在vscode中唤出终端  
![](/img/git/git-init-3.png)  
然后在`git bash`中输入命令  
```bash
git clone <后缀为.git的URL>
```
![](/img/git/git-init-4.png)  

## 工作区、暂存区、版本库

* **工作区** 资源管理器中（包含当前仓库的`/.git`的最小文件夹下的）可见的所有文件和所有文件夹（`/.git`除外）
* **暂存区** `/.git/index`文件。在vscode中，其内容可见于*源代码管理*侧边栏页面
* **版本库** 文件夹`/.git`

### 工作区
工作区是你在本地计算机上的项目目录，你在这里进行文件的创建、修改和删除操作。工作区包含了当前项目的所有文件和子目录。  
特点：
* 显示项目的当前状态
* 文件的修改在工作区中进行，但这些修改还没有被记录到版本控制中

### 暂存区
暂存区是一个临时存储区域，它包含了即将被提交到版本库中的文件快照，在提交之前，你可以选择性地将工作区中的修改添加到暂存区。  
在vscode中，每次你修改代码时，新出现修改的文件（夹）都会在左侧改变高亮颜色。绿色代表文件被新建，蓝色代表文件被修改，红色代表文件被删除  
你可以在*源代码管理*侧边栏中看到当前工作区较上次提交出现的所有修改。此时修改并没有被存入暂存区。  
对单独的文件，你可以选择点击*放弃更改*来使文件还原到上次提交的状态，也可以点击*暂存更改*来将文件的更改加入到暂存区。  
当某个文件的更改被暂存，其高亮颜色会变深，以提示更改被暂存  

更改被暂存才会在下次提交中被git记录
### 版本库
版本库包含项目的所有版本历史记录。  
每次提交都会在版本库中创建一个新的快照，这些快照是不可变的，确保了项目的完整历史记录。  

## 提交（commit）
进行提交操作会将本地已经暂存的更改记录入本地新的版本内。  
要进行一次提交，必须填写**提交信息**才能完成提交。**提交信息**应尽可能地描述清楚这次commit中改动的意义，例如添加了什么新功能，修复了什么bug，测试了什么功能或者重构什么代码等等  

{% fold info @Angular规范 %}
这是一种常见的`git commit`规范，作为提交规范的参考，不要求完全严格执行  

`type` (`scope`) : `subject`  

> `type`  

用于说明`git commit`的类别，**只允许**使用以下标识:  
|Type|说明|
|----|----|
|feat|新功能|
|fix|完全修复某一bug|
|to|修复某一bug，最终完成该bug修复时的提交还是使用fix|
|docs|新建/更新 项目说明性文档(例如：`README.md`)|
|style|调整项目中的代码格式（不影响代码运行的变动）|
|refactor|重构|
|perf|优化|
|test|增加测试|
|chore|代码构建方式、项目辅助工具的变动|
|revert|对应指令`git revert`，回滚到上一版本|
|merge|对应指令`git merge`，代码合并|
|sync|同步主分支或分支的Bug|

> `scope` (可选)

scope用于说明 commit 影响的范围，比如数据层、控制层、视图层等等，视项目不同而不同。

> `subject` 

subject是commit目的的简短描述，不超过50个字符  

> *example*:
> ```
> fix(Holder): 修复云台会撞到机械限位的问题
> feat(USB): 添加 bsp_usb ，允许使用 usb 来和上位机通信
> ```

{% endfold %}

每次提交都会记录一个新的代码版本。如果你安装了`git gragh`，你可以在*源代码管理*下面看到本代码仓库的所有历史版本  
每次提交都会生成一个哈希值来唯一对应这次提交。这个哈希值可以被后面介绍的部分指令使用来进行和这次提交有关的操作  

## 分支（branch）
Git 分支管理是 Git 强大功能之一，能够让多个开发人员并行工作，开发新功能、修复 bug 或进行实验，而不会影响主代码库。  
几乎每一种版本控制系统都以某种形式支持分支，一个分支代表一条独立的开发线。  
使用分支意味着你可以从开发主线上分离开来，然后在不影响主线的同时继续工作。  

> Git 分支实际上是指向更改快照的指针。

### 创建分支
vscode的左下角的分支按钮可以快捷地新建、切换分支  
![](/img/git/git-branch-1.png)  
当然你也可以使用命令行
```bash
# 创建分支
git checkout -b <branchname>
# 切换分支
git checkout <branchname>
# 合并分支,将其他分支合并到当前分支
git merge <branchname>
# 删除分支
git branch -d <branchname>
```

### 分支合并
每次运行merge指令时，git都会进行检查是否出现冲突  
如果出现合并冲突，git会标记冲突文件的冲突点，并要求你手动解决每一处冲突  
![merge conflict的简单示例](/img/git/git-merge.png)

{% note danger %}
**不建议在远程仓库托管平台进行merge操作**  
当出现merge conflict时，在github上难以精确解决每一处冲突。  

推荐的做法是:
1. 抓取当前远程仓库，将要合并的分支都放入本地  
   ```bash
   git fetch
   ```
2. 在vscode中运行merge指令  
   ```bash
   git merge <branchname>
   ```
3. 在vscode中逐一解决所有merge conflict
4. 将解决好merge conflict的文件添加到暂存区
5. commit （这次commit的消息git会自动生成一个）
6. 将提交推送到远程仓库
{% endnote %} 

### “舍近求远”的意义？
**分支**的实际意义在于：
1. **保护主分支的稳定性**  
   当你需要希望实现一个新的功能时，你并不能保证你所有的改动都不会影响到原有功能的执行  
   如果你直接将未经验证的代码commit到`main`分支上，此时就会破坏`main`分支的稳定性  
   新建一个分支，先在分支上进行更改，测试稳定后再merge回`main`分支，这样就避免了`main`分支上出现不可用代码的历史  
2. **多人并行协作**  
   有时，一个项目并不是只有你一个人在写。为了避免不同人同时对`main`分支编辑造成意料之外的冲突，每一个开发者为自己正在实现的功能创建一个分支是有必要的  
   > *example*  
   > **Rh-i**和**ChoseB**正在合作编写一个项目，两人约定好了下一步的分工：**Rh-i**去写陀螺仪相关，**ChoseB**去写USB驱动  
   > 为了避免两人的代码在`main`分支上相互影响，**Rh-i**新建了分支`dev-imu`，**ChoseB**新建了分支`dev-usb`  
   > 一段时间后，两人分别完成了各自的任务。此时他们各自在本地切回`main`分支，并执行了merge操作  
   > 两人的开发过程是相对独立的，最后通过merge操作，他们将各自完成的工作都推送到了`main`分支上  
3. **实现低成本实验**  
   分支的创建和删除极其快捷，由于新建分支和删除分支本质只是新建一个指针哈希和删除一个指针哈希，这个过程几乎不占用硬盘资源。  
   当你想要做一个可能导致工程崩溃的尝试，不妨新建一个分支，在新分支内大胆尝试。一旦失败，删除分支，回退版本，恢复原样

## 远程仓库
远程仓库可以理解为“放在云端的一份仓库副本”，常见托管平台有 GitHub、GitLab、Gitee 等。  
本地仓库负责你日常开发，远程仓库负责**备份、共享和协作**。  

以 GitHub 为例：你在本地写完代码后，可以把提交推送到 GitHub
协作者再从 GitHub 拉取这些提交，就能拿到你的最新改动。  

当你通过 `git clone` 克隆一个 GitHub 仓库时，Git 会自动给这个远程地址起名为 `origin`。  
你可以把它理解为“默认远程仓库的别名”。  

常见查看命令：
```bash
# 查看当前仓库关联了哪些远程仓库
git remote -v
# 链接远程仓库
git remote origin <仓库URL>
```

### 同步本地和远程
1. **拉取远程更新到本地**（`fetch` / `pull`）  
   `git fetch` 只下载远程更新，不自动合并；  
   `git pull` = `fetch` + `merge`（会尝试自动合并到当前分支）。

2. **把本地提交上传到远程**（`push`）  
   当你本地 commit 完成后，commit按钮会变成*向远程推送提交*，点击该按钮即可快捷推送提交

### Github Fork
`Fork` 是把“别人的仓库”复制一份到你自己的 GitHub 账户下。  
它的核心价值是：你没有原仓库写权限时，也能基于原项目开发并发起协作。  

一个典型的 Fork 关系如下：
* **upstream**：原作者/组织的仓库（上游仓库）
* **origin**：你自己账户下的 Fork 仓库

常见流程：
1. 在 GitHub 页面点击 **Fork**，将目标仓库复制到自己账号下  
2. 在本地克隆你自己的 Fork 仓库（`origin`）  
3. 在本地新增上游仓库地址（`upstream`），用于同步原项目更新

```bash
# 克隆你自己的 Fork 仓库
git clone <your-fork-url>

# 进入仓库后，添加上游仓库
git remote add upstream <original-repo-url>

# 查看远程仓库配置
git remote -v
```

当原仓库有新提交时，你可以这样同步：
```bash
git fetch upstream
git checkout main
git merge upstream/main
git push origin main
```

### 拉取请求PR (Pull Request)
`PR`（Pull Request）是 GitHub 上发起“代码合并请求”的方式。  
它本质是在说：**我已经在分支上完成改动，请你审查并合并到目标分支**。  

一个标准 PR 通常包含：
* 改动描述（你做了什么）
* 改动原因（为什么这样做）
* 测试说明（你如何验证改动有效）

推荐流程：
1. 从最新的 `main` 拉出功能分支并开发  
2. 本地自测通过后，推送分支到 GitHub  
3. 在 GitHub 点击 **Compare & pull request** 发起 PR  
4. 邀请 reviewer 审查，根据评论继续修改并推送  
5. 审查通过后，由有权限的人合并 PR

```bash
# 1) 更新主分支
git checkout main
git pull origin main

# 2) 创建并切换功能分支

# 3) 开发并提交

# 4) 推送并建立上游关系
```

{% note info %}
同一个 PR 中尽量只解决一个问题，改动越聚焦，审查越高效，也越容易在出现问题时快速回滚。
{% endnote %}

### 为什么远程仓库很重要？
1. **防止本地数据丢失**：电脑损坏时，远程仓库仍保留完整历史。  
2. **方便多人协作**：所有人通过 GitHub 交换代码，不需要互传压缩包。  
3. **流程更规范**：通过 PR 审查和讨论，减少“代码能跑但问题很多”的情况。  

# 推荐 Workflow & 常见操作

## 准备工作
### Case1 新建git仓库
点击*初始化仓库*按钮  
第一次commit后点击*发布branch*按钮  
#### 强制远程推送
{% note danger %}
**该操作将覆写目标远程仓库。只有你知道你在干什么的时候才进行该操作**  
当远程仓库中的代码版本与你的代码有较大出入且当前只有你在开发该项目时，要进行远程推送，你可能需要考虑**强制远程推送**  
在`git bash`中执行命令
```bash
# 将本地分支强制推送到远程覆盖历史
# 覆盖远程分支历史
git push --force origin <branchname>
# 更安全的方式，仅当远程未被他人更新时才覆盖
git push --force-with-lease origin <branchname>
```
**强制远程推送**将导致原有远程仓库中的代码和提交历史全部丢失，因此请谨慎操作
{% endnote %}  

### Case2 克隆已有仓库
1. **Fork你要参与的远程仓库到自己Github账户**
2. 克隆你自己账户上的远程仓库
   ```bash
   git clone <URL>
   ```

### gitignore

`.gitignore`是一个特殊的文件，用来告诉 Git：**哪些文件不应被纳入版本控制**。  
典型场景包括：
1. 构建产物（如 `build/`、`dist/`）
2. 临时文件和日志（如 `*.log`）
3. 本地配置和敏感信息（如 `.env`、密钥文件）
4. IDE 配置缓存（如 `.vscode/` 中的本地化配置）

常见写法示例：
```gitignore
# ==============================================================================
# 编译生成的文件 (Build Output)
# ==============================================================================
# 忽略 CMake 和 Make 的构建目录
build/Debug/*
build/*
Debug/*

# 具体的编译中间文件 (Object files, dependency files)
*.o
*.obj
*.ko
*.d
*.su
*.gch
*.pch

# 链接器输出与可执行文件 (Linker output & Executables)
*.elf
*.hex
*.bin
*.map
*.ilk
*.exp
*.lib
*.a
*.la
*.lo
*.dll
*.so
*.so.*
*.dylib
*.exe
*.out
*.app

# 调试文件 (Debug files)
*.dSYM/
*.idb
*.pdb
*.dwo

# ==============================================================================
# 个人配置文件与IDE设置 (Personal Configs & IDEs)
# ==============================================================================
# STM32CubeMX / CubeIDE生成的工程文件 (通常不需要上传，因为有 .ioc 文件)
.mxproject
.project
.cproject

# CMake 缓存与临时文件 (如果在根目录运行了 cmake)
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
Makefile
# 但是要保留 CMakeLists.txt 和 cmake 目录下的脚本
!CMakeLists.txt
!cmake/

# ==============================================================================
# 操作系统生成的文件 (OS Files)
# ==============================================================================
.DS_Store
Thumbs.db
```

规则小结：
1. `#` 开头是注释
2. `*.log` 表示忽略所有 `.log` 文件
3. `dir/` 表示忽略整个目录
4. `!xxx` 表示“取消忽略”，例如你可以忽略整个目录后单独保留一个模板文件

```gitignore
# 忽略 config 目录下所有文件
config/*
# 但保留示例配置
!config/example.yml
```

建议做法：在仓库初始化早期就创建好 `.gitignore`，避免把无关文件提交进历史。  
如果某文件已经被 Git 跟踪，后续即使写进 `.gitignore` 也不会自动失效，需要先取消跟踪：

```bash
# 从版本控制中移除，但保留本地文件
git rm --cached <file-or-dir>
```

{% note info %}
`.gitignore` 只能影响“未被跟踪”的文件。已经提交过的文件，需先 `git rm --cached` 再提交一次，规则才会生效。
{% endnote %}

## 本地改动
推荐在功能分支上进行所有改动，不要直接在 `main` 上开发。  

标准步骤：
1. 切换到自己的开发分支（没有就新建）
2. 小步提交，每个 commit 只做一件事
3. 提交前先自测，保证代码可运行

```bash
# 从 main 切出开发分支
git checkout main
git pull origin main
git checkout -b dev-xxx
```
在*源代码管理*中选择性地暂存你的更改  
填写好提交消息后点击**提交**按钮来commit

{% note warning %}
不要把“格式化全项目”与“功能改动”混在同一个 commit 里，否则会显著增加审查难度。
{% endnote %}

### 回退历史版本
git提供了三种回退历史版本的方式  
#### 签出Checkout
`checkout` 可以把你的工作目录切换到某个历史提交（或某个分支）。  
当你 checkout 到一个提交哈希时，通常会进入 **detached HEAD** 状态：你可以查看和测试历史代码，但不应该直接在这个状态长期开发。  

```bash
# 临时切到某次历史提交（只读查看很常见）
git checkout <commit-hash>
# 回到当前开发分支
git checkout main
```

适用场景：
1. 快速验证“某次提交时程序是否正常”
2. 对比历史版本行为
3. 从历史提交拉出新分支继续开发

```bash
# 从历史提交创建新分支继续开发
git checkout -b hotfix-from-old <commit-hash>
```

#### 重置Reset
`reset` 会移动当前分支指针（HEAD）到目标提交，可同时影响暂存区和工作区。  
它适合“本地还没推送或你确定可以改写历史”的场景。  

三种常用模式：
```bash
# 仅移动 HEAD，保留暂存区和工作区改动
git reset --soft <commit-hash>

# 默认模式：移动 HEAD，并重置暂存区；工作区改动保留
git reset --mixed <commit-hash>
# 等价写法
git reset <commit-hash>

# 移动 HEAD，并重置暂存区和工作区（危险）
git reset --hard <commit-hash>
```

理解方式：
1. `--soft`：撤销 commit，但保留“已暂存”状态
2. `--mixed`：撤销 commit，也取消暂存，但代码还在文件里
3. `--hard`：连文件改动一起丢弃

{% note danger %}
`git reset --hard` 会直接丢失未提交改动。执行前建议先确认 `git status`，必要时先 `git stash` 备份。
{% endnote %}

#### 还原Revert
`revert` 不会改写历史，而是“新建一个反向提交”来抵消指定提交的改动。  
这也是协作中更推荐的回退方式，尤其是已经 push 到远程后的提交。  

常用命令：
```bash
# 还原某一次提交（会产生一个新的 commit）
git revert <commit-hash>

# 还原最近一次提交
git revert HEAD

# 一次还原一段提交（不自动提交，便于检查）
git revert --no-commit <old-hash>^..<new-hash>
git commit -m "revert: 回滚某功能改动"
```

适用场景：
1. 已推送到远程后发现某次提交有问题
2. 不希望破坏已有提交历史
3. 需要可追踪地“撤回某功能”
#### 图形化操作
利用`git graph`插件，我们能够快捷地完成上面三种操作  
进入`git graph`界面，对我们想要回退的提交右键，能够看到上述的三种操作。  
![](/img/git/git-gitgraph.png)  

## 远程推送
本地 commit 完成后，需要把分支推送到 GitHub，方便备份和协作。  

```bash
# 第一次推送该分支
git push -u origin dev-xxx

# 后续继续推送
git push
```

若推送被拒绝（常见提示：`rejected`），通常是远程分支有新提交：
```bash
git pull --rebase origin dev-xxx
git push
```

如果你是在 Fork 仓库协作，通常推送到自己的 `origin`，然后从 GitHub 页面发起到上游仓库的 PR。  

## 审核PR (前提是你有远程仓库的管理权限)
作为 reviewer，重点看三件事：
1. **正确性**：功能是否符合需求，是否引入明显 bug
2. **可维护性**：命名是否清晰，结构是否可读，是否有重复逻辑
3. **风险控制**：是否有测试说明，是否影响主流程，是否便于回滚

推荐审核流程：
1. 在 PR 页面先读标题和描述，确认改动目标
2. 按文件查看 diff，优先看核心逻辑文件
3. 本地拉分支验证（能运行、能通过关键测试）
4. 给出 review 结论：`Comment` / `Request changes` / `Approve`

```bash
# 本地拉取对方分支进行验证（示例）
git fetch origin
git checkout <contributor-branch>
```



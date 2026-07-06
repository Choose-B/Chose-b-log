# CLAUDE.md

此文件为 Claude Code (claude.ai/code) 在此仓库中工作时提供指导。

## 项目概述

这是 **B-log**——基于 [Hexo](https://hexo.io/) 构建的个人技术博客，使用 Fluid 主题，部署于 Netlify。博客内容以机器人（RoboMaster）、嵌入式开发（STM32、FreeRTOS）、电机控制和数学基础为主题。

- **站点 URL**：`http://chose-b-log.top`
- **语言**：zh-CN
- **时区**：Asia/Shanghai
- **Hexo 版本**：8.x（package.json 中声明 7.3.0，实际安装 ^8.0.0）

## 常用命令

```bash
# 安装依赖
npm install

# 本地开发服务器（默认 http://localhost:4000）
hexo server
# 或
npm run server

# 生成静态文件到 public/
hexo generate
# 或
npm run build

# 清理缓存和生成文件
hexo clean
# 或
npm run clean

# 清理 + 构建（用于 Netlify 部署）
npm run netlify

# 创建新文章
hexo new post "文章标题"

# 创建新草稿
hexo new draft "草稿标题"

# 发布草稿
hexo publish post "草稿标题"

# 部署（当前 deploy.type 为空，由 Netlify 自动部署）
hexo deploy
```

## 目录结构

```
source/
├── _posts/          # 博客文章（Hexo 自动渲染）
├── hide_posts/      # 隐藏文章——不会被 Hexo 自动渲染，仅通过直接链接访问
├── about/           # 关于页面（layout: about）
├── css/             # 自定义 CSS（当前仅含 katex.min.css）
├── Document/        # 参考资料 PDF（电机手册、通信协议等）
└── img/             # 图片资源（含 about/、c/、git/、RM-Note/ 等子目录）
scaffolds/           # hexo new 命令的文章模板
_config.yml          # Hexo 主配置（当前 theme: fluid）
_config.fluid.yml    # Fluid 主题完整配置
_config.redefine.yml # Redefine 主题备选配置
_config.landscape.yml# Landscape 主题占位配置
```

## 文章 Frontmatter 约定

所有文章使用 YAML frontmatter，重要字段如下：

```yaml
---
title: 文章标题
category: 分类名          # 单值字符串
tag:                      # YAML 列表
  - 标签1
  - 标签2
date: YYYY-MM-DD HH:mm:ss
hide: false               # true = 仅通过链接访问，不在任何列表展示
achieve: true             # false = 不在首页显示，但仍出现在归档/分类中
sticky: 1                 # 置顶权重，越大越靠前
index_img: /img/xxx.avif  # 首页封面图
mathjax: false            # 是否启用 KaTeX 数学渲染
excerpt: ''               # 手动摘要
---
```

- 草稿模板（`scaffolds/draft.md`）最简，仅含 `title` 和 `tags`
- 页面模板（`scaffolds/page.md`）有 `math` 字段（LaTeX 开关），与文章模板的 `mathjax` 字段不同，注意区分
- `achieve: false` 的文章仍然会被渲染，可以归档/分类页面中找到——相当于"不在首页时间线展示"

## 书写规范

基于 `source/hide_posts/文章全显示要素测试.md` 归纳的本博客 Markdown 书写习惯。

### 数学公式

只有在 frontmatter 中通过 `mathjax: true` 开启数学公式，才会正确渲染数学公式：

- 行内公式：`{% mathjax %}P\propto error{% endmathjax %}`
- 行间公式（块级）：
  ```
  {% mathjax %}
  ...
  {% endmathjax %}
  ```
只在发布博客是进行检查：如果有`$`字段，自行替换`{% mathjax %}`...`{% endmathjax %}`;如果有`$$`字段，自行替换为
```
{% mathjax %}
...
{% endmathjax %}
```

或直接运行转换脚本：
```bash
python3 tools/convert_math.py source/_posts/文章名.md
```

反向转换（`{% mathjax %}` → `$` / `$$`）：
```bash
python3 tools/convert_math_reverse.py source/_posts/文章名.md
```

### Fluid 主题 Tag 插件

**Note 标签**（提示框，`{% note %}` 和 `{% endnote %}` 需要各占单独一行）：

- `{% note primary %}` — 紫色主色调提示框
- `{% note secondary %}` — 灰色次要提示框
- `{% note success %}` — 绿色成功提示框
- `{% note danger %}` — 红色危险/警告提示框
- `{% note warning %}` — 黄色警告提示框
- `{% note info %}` — 蓝色信息提示框
- `{% note light %}` — 浅色提示框

**Label 标签**（行内高亮文字）：`{% label primary @高亮文字 %}`

**折叠块**：`{% fold info @折叠块标题 %} 需要折叠的内容，支持 markdown {% endfold %}`

**勾选框**：`{% cb 勾选文字, true, false %}`（第二个参数 `true`=默认勾选，`false`=默认不勾选）

**按钮**：`{% btn URL, 按钮文字, 提示文字 %}`

### 常规 Markdown

| 元素 | 语法 |
|------|------|
| 斜体 | `*text*` |
| 加粗 | `**text**` |
| 粗斜体 | `***text***` |
| 删除线 | `~~text~~` |
| 行内代码 | `` `code` `` |
| 链接 | `[text](url)` |
| 图片 | `![alt](/img/...)` |
| 表格 | 标准 `\|--\|--\|` 语法 |
| 无序列表 | `*` / `-` / `+`（支持缩进嵌套） |
| 有序列表 | `1.`（支持嵌套，编号数字本身不强制连续） |

### 代码块

使用标准围栏代码块，标注语言以获得语法高亮。常用语言标识：`python`、`c++`、`markdown`、`yaml`。

## 主题配置

当前使用 **Fluid 主题**（`hexo-theme-fluid`），核心配置在 `_config.fluid.yml`：

- **暗色模式**：已启用，默认为 auto
- **代码高亮**：highlight.js，浅色主题 `Github Gist`，深色主题 `dark`
- **数学公式**：通过 `@traptitech/markdown-it-katex` 插件渲染 KaTeX
- **LaTeX**：`hexo-math` 插件提供（文章通过 `mathjax: true` 开启）
- **评论**：Gitalk（`gitalk` 依赖已安装）
- **图片懒加载**：已启用，占位图 `/img/loading.gif`
- **导航栏**：首页、标签、分类、关于、友链
- **友链页面**：需要在 source 下创建 `links/` 目录

另有两套备选主题配置（`_config.redefine.yml`、`_config.landscape.yml`），切换主题时需要将对应配置合并到 `_config.yml` 中。

## 关键插件

| 插件 | 用途 |
|------|------|
| `hexo-generator-archive/category/index/tag` | 生成归档、分类、首页、标签页 |
| `hexo-renderer-ejs` | EJS 模板渲染 |
| `hexo-renderer-marked` | Markdown 渲染（marked 引擎） |
| `hexo-renderer-stylus` | Stylus CSS 预处理器 |
| `hexo-math` + `@traptitech/markdown-it-katex` | 数学公式渲染 |
| `hexo-tag-chart` | Chart.js 图表标签插件 |
| `gitalk` | GitHub Issue 驱动的评论系统 |

## 部署

部署在 **Netlify**，通过 GitHub 仓库连接自动触发。`package.json` 中 `netlify` 脚本（`npm run clean && npm run build`）是 Netlify 的构建命令。每次推送到 main 分支会自动部署。

GitHub Dependabot 已配置为每日检查 npm 依赖更新。

## 注意事项

- 添加新文章时，务必在 frontmatter 中填写 `category` 字段，否则归类为 `uncategorized`
- 对于大部分文章下frontmatter中`hide`、`achieve`和`sticky`字段是删去的，因此对于任意blog不要做任何涉及这三个字段的改动
- `source/hide_posts/` 中的文件不会被 Hexo 自动渲染，需手动移动到 `_posts/` 才能发布
- 图片资源统一放在 `source/img/` 下，文章引用时使用绝对路径 `/img/...`
- 更换主题时，需要同步修改 `_config.yml` 中的 `theme` 字段，并确保新主题的配置文件（如 `_config.redefine.yml`）包含必要的插件配置

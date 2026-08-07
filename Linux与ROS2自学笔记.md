# Linux + ROS 2 自学笔记(零基础版)

> 这份笔记记录了从零开始学 Linux 命令行到写出第一批 ROS 2 节点的完整过程。
> 内容基于真实的学习过程整理,包含实际踩过的坑和解决方法。
> 适合:完全没用过 Linux、想入门机器人开发的同学。
> 预计投入:约 2 小时/天,两周左右能完整走完。

---

## 怎么用这份笔记

**三条铁律,比任何内容都重要:**

1. **必须边看边敲。** 看懂 ≠ 会用。每个命令都要自己在终端敲一遍,看到输出再往下走。只看不敲的话,两天后你什么都不记得。
2. **报错是正常的,先自己读。** 遇到报错先看它说什么(英文慢慢读),再自己搜。每修好一个报错,你的能力就涨一点。直接问别人要答案,你只学会了那一条命令。
3. **建一个专用练习目录,在里面随便折腾。** 所有危险操作(删除、改权限)都在练习目录里做,搞砸了删掉重建即可。

**开始前先建练习目录:**
```bash
mkdir -p ~/linux_practice
cd ~/linux_practice
```
以后每次开始学习,先 `cd ~/linux_practice`。

---

## 前置准备:环境

这份笔记假设你已经装好了:
- **Ubuntu 24.04 LTS**(建议双系统,不建议虚拟机——后面跑 Gazebo 仿真需要显卡加速)
- **ROS 2 Jazzy**

装环境本身是个不小的工程(分区、BIOS、驱动),如果还没装,建议:
- 装系统时**语言选 English**——中文会让主目录出现"下载""文档"这类中文路径,很多工具链对中文路径兼容极差,是个著名的坑
- ROS 2 安装可以用国内的一键脚本(搜"鱼香ROS 一键安装"),比官方文档手动装省事得多,国内网络也更稳定
- 装完验证:终端输入 `ros2 --help`,能输出一大堆子命令说明就成功了

---

# 第一部分:Linux 基础

## Day 1:文件系统与导航

**目标:** 能在终端里自由"走路",看懂目录结构。

打开终端:`Ctrl + Alt + T`

### 核心命令

```bash
pwd                    # 我在哪(print working directory)
ls                     # 列出当前目录内容
ls -l                  # 详细列表(权限、大小、时间)
ls -a                  # 显示隐藏文件(以 . 开头的)
ls -lh                 # 人类可读的大小(K/M/G 而不是一堆字节数)
ls -R                  # 递归显示所有子目录
```

参数可以叠加,`ls -lah` 就是三个效果一起。

```bash
cd /                   # 去根目录
cd /home/你的用户名     # 绝对路径(从 / 开始写全)
cd ~                   # 回家目录(~ 是家的简写)
cd ..                  # 上一级(.. 代表上级目录)
cd -                   # 回到上一个待过的目录
cd                     # 不带参数也是回家
```

### Linux 目录结构(记住这几个)

```
/                根目录,一切的起点
├── home/你的用户名   你的家目录,个人文件都在这
├── etc            系统配置文件
├── usr            已安装的软件
├── opt            第三方大型软件(ROS 装在 /opt/ros/jazzy)
├── tmp            临时文件(重启会清空)
└── mnt            挂载点(U盘、其他分区)
```

### ★ 必须养成的习惯:Tab 补全

输入 `cd /opt/r` 然后**按 Tab 键**,会自动补成 `/opt/ros/`。再输 `j` 按 Tab,补成 `jazzy`。

**Tab 补全不只是省打字,更是防错**——补不出来就说明路径写错了,当场就知道。老手几乎不完整敲路径,全靠 Tab。

### 练习

1. 用 `cd` 走到 `/opt/ros/jazzy`,`ls` 看看 ROS 装了什么
2. 合上笔记,凭记忆画出 Linux 目录树的主干

**过关标准:** 不看笔记能说出 `/opt/ros/jazzy` 每一层是什么意思。

---

## Day 2:文件操作与通配符

### 创建、复制、移动、删除

```bash
mkdir test1                # 建目录
mkdir -p a/b/c             # 一次建多层嵌套(没有 -p 会报错)
touch file1.txt file2.txt  # 建空文件(可以一次建多个)

cp file1.txt backup.txt    # 复制
mv file2.txt test1/        # 移动到 test1 目录
mv file1.txt renamed.txt   # 重命名

rm renamed.txt             # 删除文件
rm -r test1                # 删除目录(-r 递归)
```

**重要概念:`mv` 既是移动也是重命名。** 目标是目录就移动过去,目标是新名字就改名。Linux 没有单独的 rename 命令。这一点在后面写批量重命名脚本时是核心。

### ⚠️ 关于 rm 的安全习惯

**Linux 的 `rm` 没有回收站,删了就是真没了,没有 Ctrl+Z。**

`rm -rf`(强制递归删除)是新手事故的头号来源。养成一个动作:

**删之前先用 `ls` 把要删的东西列出来看一眼。**

```bash
ls *.txt      # 先看清楚要删哪些
rm *.txt      # 确认无误再删
```

### 通配符(批量操作的基础)

```bash
touch img1.jpg img2.jpg img3.jpg data.txt

ls *.jpg           # * 匹配任意长度任意字符
ls img?.jpg        # ? 匹配单个字符
ls img[12].jpg     # [] 匹配方括号里任一字符
ls img[1-3].jpg    # - 表示范围
```

配合其他命令使用:
```bash
mkdir photos
cp *.jpg photos/       # 一条命令复制所有 jpg
mv *.txt photos/       # 批量移动
```

**通配符的威力和危险是一体的:** `rm *.txt` 和 `rm * .txt`(中间多个空格)是天壤之别——后者会删掉当前目录所有文件。**空格在命令行里是分隔符,多一个空格就是多一个参数。**

### 练习

用**一条命令**把所有 `.log` 文件移到 `logs` 目录:
```bash
touch sys1.log sys2.log app.log config.yaml
mkdir logs
# 你来写这条命令
```

<details>
<summary>答案</summary>

```bash
mv *.log logs/
```
</details>

**做完记得 `ls` 和 `ls logs` 验证结果——操作完看一眼结果,是个好习惯。**

---

## Day 3:权限管理

这是 Linux 里最容易一知半解的部分,但理解了就很清晰。

### 读懂权限

```bash
ls -l
```

每行开头那串 `-rwxr-xr-x` 就是权限:

```
-  rwx  r-x  r-x
│   │    │    │
│   │    │    └─ others:其他所有人
│   │    └────── group:同组用户
│   └─────────── owner:文件所有者(你)
└─────────────── 类型:- 普通文件, d 目录
```

三组,每组三位,固定顺序 **r-w-x**:
- **r** = read 读
- **w** = write 写
- **x** = execute 执行
- **-** 表示没有这个权限

### 数字权限

```
r = 4    w = 2    x = 1

rwx = 4+2+1 = 7    全部权限
rw- = 4+2+0 = 6    读写
r-x = 4+0+1 = 5    读和执行
r-- = 4+0+0 = 4    只读
--- = 0            什么都不给
```

三组各算一位,拼成三位数:

| 数字 | 含义 | 典型用途 |
|---|---|---|
| **755** | rwxr-xr-x | 可执行程序、脚本、目录 |
| **644** | rw-r--r-- | 普通文档 |
| **700** | rwx------ | 私密目录 |

### chmod 改权限

```bash
touch test.txt
ls -l test.txt

chmod 644 test.txt      # 数字法
chmod 755 test.txt
chmod +x test.txt       # 符号法:给所有人加执行权限
chmod -x test.txt       # 去掉执行权限
chmod u+w test.txt      # 只给 owner 加写权限
```

`u`=owner, `g`=group, `o`=others, `a`=all;`+` 加,`-` 减。

**`chmod +x` 是最常用的**——写完脚本必须加执行权限才能跑。

### ★ 一个反直觉的点:目录的 x 权限

- 对**文件**:x = 能不能当程序运行
- 对**目录**:x = **能不能进去(cd)**

亲手验证一下:
```bash
mkdir locked
touch locked/secret.txt
chmod 600 locked        # rw-------,没有 x
ls locked               # 能看到文件名,但报 Permission denied
cd locked               # 进不去!

chmod 700 locked        # rwx------,有 x 了
cd locked               # 现在能进了
cd ..
```

`ls locked` 那个现象很有意思:**既报错又列出了文件名**——因为 r 让你能读到"有个叫 secret.txt 的名字",但没有 x 让你无法进入目录获取详细信息。

### sudo 与 root

```bash
cd /opt/ros/jazzy
touch mytest.txt        # 报 Permission denied
```

为什么?
```bash
ls -ld /opt/ros/jazzy   # 所有者是 root
```

**root 是超级管理员,权限不受限制。** Ubuntu 不让你直接用 root 登录(太危险),而是给你 `sudo`——**临时以 root 身份执行这一条命令**。

```bash
sudo touch /opt/ros/jazzy/mytest.txt   # 成功
sudo rm /opt/ros/jazzy/mytest.txt      # 删掉
```

### ⚠️ sudo 的三条规矩

**加了 sudo,所有保护都失效了。** 系统不会再拦你。

1. **能不用 sudo 就不用。** 在自己家目录里操作,永远不需要 sudo。
2. **敲 sudo + rm 时,手指离开回车键,先把命令读一遍。** 特别看路径开头有没有多余的 `/`。有条臭名昭著的命令 `sudo rm -rf /` 会删掉整个系统。
3. **不要用 sudo 跑图形程序或编辑器**(比如 `sudo code`),会把配置文件所有者搞成 root,之后你自己反而改不了。

---

## Day 4:进程管理

前面操作的是"静态"的文件,现在看"动态"的运行中程序。

**每个运行中的程序叫一个进程(process),都有唯一编号 PID。**

```bash
ps aux                 # 列出所有进程
ps aux | grep firefox  # 只看含 firefox 的(| 是管道,Day 5 细讲)
top                    # 实时监视器,按 q 退出
```

### 前台与后台

```bash
ros2 run demo_nodes_cpp talker      # 前台运行,占住终端
# 按 Ctrl+C 终止

ros2 run demo_nodes_cpp talker &    # 结尾加 & 后台运行
jobs                                 # 查看后台任务
fg                                   # 调回前台
# 按 Ctrl+Z 暂停到后台
bg                                   # 让它在后台继续跑
```

| 操作 | 作用 |
|---|---|
| `Ctrl+C` | 终止当前前台程序 |
| `Ctrl+Z` | 暂停到后台 |
| `&` | 直接后台运行 |
| `q` | 退出 top / less 这类全屏程序 |
| `kill <PID>` | 按进程号结束 |
| `kill -9 <PID>` | 强制杀死(程序卡死时用) |

### ★ 坑点:grep 会匹配到自己

```bash
ps aux | grep talker
```

即使 talker 已经关掉了,你还是会看到一行:
```
qiking  7130  ... grep --color=auto talker
```

**这是 grep 进程匹配到了它自己**——因为它的命令行里含有 "talker" 这个词。

**判断方法:看 COMMAND 那一列,如果只剩 grep 自己,就是干净的。**

更省心的查法:
```bash
pgrep -a talker              # 没输出 = 进程不存在,不会匹配自己
ps aux | grep talker | grep -v grep   # -v 反向过滤掉 grep 自己
```

### ★ 一个 ROS 相关的现象:父子进程

跑 `ros2 run demo_nodes_cpp talker` 后查进程,你会看到**两个**:
```
7166 → /usr/bin/python3 /opt/ros/jazzy/bin/ros2 run demo_nodes_cpp talker
7169 → /opt/ros/jazzy/lib/demo_nodes_cpp/talker
```

`ros2` 命令本身是个 **Python 写的启动器**(父进程),它负责找到并启动真正的可执行文件(子进程)。**一条 `ros2 run` = 一个启动器进程 + 一个实际节点进程。**

杀进程时两个都要杀,只杀父进程有时会留下孤儿子进程。

---

## Day 5:包管理(apt)

### apt 在做什么

Windows 装软件是下载 exe 双击。Linux 不是——**软件放在"软件源"(仓库服务器)里,apt 负责去仓库找、下载、安装,还自动处理依赖。**

"依赖"是重点:你装 A,A 需要 B、C 才能跑,apt 自动把 B、C 一起装上。

### 核心命令

```bash
sudo apt update            # 刷新软件清单(不装任何东西)
apt list --upgradable      # 看有哪些可升级(不用 sudo)
sudo apt upgrade           # 升级所有可升级的包

apt search htop            # 搜索包
apt show htop              # 看包的详细信息(装之前先 show 是好习惯)

sudo apt install htop      # 安装
sudo apt remove htop       # 卸载(保留配置文件)
sudo apt purge htop        # 卸载并删除配置
sudo apt autoremove        # 清理没人需要的孤儿依赖
```

**规律:凡是要改系统的命令(install/upgrade/update)都要 sudo,凡是只查询的(list/search/show)都不用。** 这和 Day 3 的权限是一回事。

### dpkg:查已装的包

apt 管"从仓库装",dpkg 管"本机已经装了什么"。

```bash
dpkg -l | grep ros-jazzy | wc -l    # 数一下装了多少 ROS 包

dpkg -L ros-jazzy-demo-nodes-cpp    # 这个包装了哪些文件
# 你会看到 /opt/ros/jazzy/lib/demo_nodes_cpp/talker
# 这正是 ps 里看到的那个子进程!

dpkg -S /opt/ros/jazzy/lib/demo_nodes_cpp/talker   # 这个文件属于哪个包
```

**这几条命令把整条链路打通了:** apt 装包 → 文件落到 /opt/ros/jazzy/ → `ros2 run` 启动它 → `ps` 里能看到进程。

### ★ 真实遇到的报错:网络问题

```
Err:2 http://mirrors.tuna.tsinghua.edu.cn/ubuntu noble InRelease
  Temporary failure resolving 'mirrors.tuna.tsinghua.edu.cn'
```

**满屏红字别慌,先找重复出现的那一句核心信息。**

`Temporary failure resolving` = **解析失败** = 电脑没法把网址翻译成 IP = **几乎总是没联网**,不是 apt 的问题。

验证:
```bash
ping -c 3 mirrors.tuna.tsinghua.edu.cn
```
- 显示 `3 packets transmitted, 3 received` → 网通了
- 显示 `Name or service not known` → 确实没联网,去连 WiFi

**读报错的方法(这比记住这条报错更重要):**
1. 找**重复出现的核心信息**,其余都是连锁反应
2. 看关键词判断方向:`resolving failed` → 网络问题,不是软件本身
3. 看最后一行的总结,通常告诉你后果严不严重

---

## Day 6:管道与重定向(Linux 最精髓的一天)

### 查看文件内容

```bash
echo -e "apple\nbanana\ncherry" > fruits.txt   # 造个测试文件

cat fruits.txt            # 一次全显示(短文件用)
less fruits.txt           # 分页查看(长文件用),q 退出,/关键词 搜索
head -n 3 fruits.txt      # 前3行
tail -n 3 fruits.txt      # 后3行
tail -f /var/log/syslog   # 实时跟踪新增内容,Ctrl+C 退出
```

`tail -f` 以后调试 ROS 很有用:跑仿真时把日志输出到文件,另开终端实时盯着看报错。

### 三个基础工具

```bash
wc -l fruits.txt          # 数行数(wc = word count)
sort fruits.txt           # 排序(只输出结果,不改原文件)

grep an fruits.txt        # 找出含 "an" 的行
grep -i APPLE fruits.txt  # -i 忽略大小写
grep -n a fruits.txt      # -n 显示行号
grep -v a fruits.txt      # -v 反向,输出不含 a 的行
grep -rn "talker" /opt/ros/jazzy/share/demo_nodes_cpp/  # -r 递归搜目录
```

**grep 是以后用得最多的工具之一**——在几千行 ROS 日志里找 ERROR、找某个节点名,全靠它。

### ★ 管道 `|`(核心概念)

**把左边命令的输出,直接喂给右边命令当输入。数据从左往右流。**

```bash
cat fruits.txt | grep a              # 输出内容 → 筛选含a的行
cat fruits.txt | sort | head -n 3    # 内容 → 排序 → 取前3行(可以一直串)

ls /opt/ros/jazzy/share | wc -l      # ROS 装了多少个包
ls /opt/ros/jazzy/share | grep demo | wc -l   # 有多少个含 demo 的包
ps aux | grep ros                     # 看进程再筛选
history | grep git                    # 翻命令历史,"我之前那条命令怎么写的"神器
```

**为什么这套设计牛?这是 Unix 哲学:每个命令只做一件事并做好,再用管道自由组合。** ls 只管列、grep 只管筛、wc 只管数——单个都简单,但 `|` 让它们能拼出无数花样。

以后你会写出 `ros2 topic list | grep camera` 这种,一眼找出所有相机相关话题。

### 重定向 `>` `>>`

管道是"命令→命令",重定向是"命令→文件"。

```bash
ls -l > filelist.txt        # > 覆盖写入文件
echo "新一行" >> filelist.txt  # >> 追加,不覆盖
```

**⚠️ 经典事故:**
```bash
sort fruits.txt > fruits.txt   # 别这么干!
```
你以为是"排序后存回原文件",实际上 `>` 会**先把文件清空**再写入,内容全没了。正确做法:
```bash
sort fruits.txt > temp.txt && mv temp.txt fruits.txt
```
(`&&` = 前一条成功才执行后一条)

### 错误输出与 /dev/null

Linux 输出分两路:**正常输出(stdout)** 和 **错误输出(stderr)**。

```bash
ls /opt /nonexistent > out.txt
cat out.txt       # 只有 /opt 的内容,报错没进去!
```
报错走的是 stderr,`>` 只抓正常输出。

```bash
ls /nonexistent 2> err.txt      # 2> 专门重定向错误输出
ls /nonexistent 2> /dev/null    # /dev/null 是黑洞,扔进去就消失
ls /opt /nonexistent > all.txt 2>&1   # 2>&1 把错误也并到正常输出那一路
```

`2>/dev/null` 用来屏蔽不关心的报错,`2>&1` 这个写法你会在无数脚本里见到。

### ★ 最值钱的技能:查手册

**同一个参数字母在不同命令里含义完全不同!** 比如:
- `grep -n` → n 是 **number(行号)**,独立开关
- `head -n 3` → n 是 **number of lines(要几行)**,后面必须跟数字

**所以不能靠猜参数,要查:**
```bash
man grep          # 完整手册,方向键滚动,/关键词 搜索,q 退出
grep --help       # 精简版,一屏左右,查参数更快
```

**能自己查手册的人,和只会复制粘贴命令的人,差距就在这。**

### 简单编辑器 nano

```bash
nano test.txt
```
- 直接输入文字
- **`Ctrl+O` 保存**(会问文件名,回车确认)
- **`Ctrl+X` 退出**
- `Ctrl+K` 剪切当前行,`Ctrl+U` 粘贴
- `Ctrl+W` 搜索

底部一直显示快捷键提示,`^` 代表 Ctrl。

---

## Day 7:Bash 脚本(重头戏)

前面都是一条条手敲命令,现在把它们组织成**能重复运行的程序**。

### 第一个脚本

```bash
nano hello.sh
```
```bash
#!/bin/bash
# 这是注释,# 开头的行不执行
echo "Hello, this is my first script!"
echo "Current directory: $(pwd)"
echo "Today is: $(date)"
```

运行:
```bash
./hello.sh          # 报错 Permission denied!
```

**为什么?——Day 3 学的权限。** 新建文件默认没有执行(x)权限:
```bash
chmod +x hello.sh
./hello.sh          # 现在成功了
```

**两个知识点:**
1. **`#!/bin/bash`(叫 shebang)**:第一行,告诉系统用哪个解释器跑。写脚本必须有。
2. **`$(命令)`**:命令替换,把括号里命令的**输出结果**嵌进来。

**另一种运行方式(不需要 +x):**
```bash
bash hello.sh
```

**为什么要写 `./hello.sh` 而不是 `hello.sh`?** 出于安全,Linux 默认不在当前目录找可执行文件(防止误跑同名恶意脚本),`./` 明确表示"就是当前目录这个文件"。

### 变量与三个必踩的坑

```bash
#!/bin/bash
name="Bill"
count=5
echo "Name: $name"
echo "Packages: $(ls /opt/ros/jazzy/share | wc -l)"
```

**坑 1:等号两边不能有空格**
```bash
name = "Bill"    # 错!bash 以为你要执行一个叫 name 的命令
name="Bill"      # 对
```

**坑 2:单引号 vs 双引号**
```bash
echo "Hello $name"    # 双引号:变量被替换 → Hello Bill
echo 'Hello $name'    # 单引号:原样输出 → Hello $name
```
**规则:想让变量生效用双引号,想要字面量用单引号。**

**坑 3:变量一定要加双引号**
```bash
file="my file.txt"
rm $file       # 危险!空格会让它拆成 rm my 和 file.txt 两个文件
rm "$file"     # 正确
```
**永远写 `"$变量"`。** 这和 Day 2 说的"空格是分隔符"是同一个原理。

### for 循环

```bash
#!/bin/bash

# 遍历一组值
for i in 1 2 3 4 5
do
    echo "Number: $i"
done

# 遍历范围
for i in {1..5}
do
    echo "Range: $i"
done

# 遍历文件(最常用!)
for file in *.txt
do
    echo "Found: $file"
done
```

结构就四行骨架:
```
for 变量 in 列表
do
    对每个元素要做的事
done
```

### if 判断

```bash
#!/bin/bash
file="fruits.txt"

if [ -f "$file" ]; then
    echo "$file exists"
else
    echo "$file not found"
fi
```

常用判断:

| 写法 | 含义 |
|---|---|
| `[ -f "$x" ]` | x 是文件且存在 |
| `[ -d "$x" ]` | x 是目录且存在 |
| `[ -z "$x" ]` | x 是空字符串 |
| `[ "$a" = "$b" ]` | 字符串相等 |
| `[ "$a" -eq "$b" ]` | 数字相等 |
| `[ "$a" -gt "$b" ]` | 数字大于 |

**⚠️ 坑:方括号内侧必须有空格**
```bash
if [-f "$file"]     # 错
if [ -f "$file" ]   # 对
```
因为 `[` 本身是个命令,不是语法符号。

### 脚本参数

```bash
#!/bin/bash
if [ -z "$1" ]; then
    echo "Usage: bash greet.sh <name>"
    exit 1
fi
echo "Hello, $1!"
echo "You passed $# argument(s)"
```

```bash
bash greet.sh Bill
```

规则很机械,**空格分隔,从左到右数**:
```
bash greet.sh Bill Smith
     └───┬──┘ └┬┘ └─┬─┘
        $0     $1    $2
```

- `$0` = 脚本名字本身(系统自动放的,你传的参数从 $1 开始,不会顶掉它)
- `$1` `$2` ... = 第1、2个参数
- `$#` = 参数总个数
- `$@` = 所有参数的列表,可以用 for 遍历(参数数量不定时用这个)
- 第10个及以后要写 `${10}`(不加花括号会被理解成 `$1` 后面跟个 0)

### ★★ 核心练习:批量重命名脚本

**这是 Linux 阶段的过关标准。先自己写,写不出来再看答案。**

准备测试:
```bash
mkdir rename_test && cd rename_test
touch photo1.jpg photo2.jpg photo3.jpg
```

**任务:写一个脚本,把所有 `.jpg` 加上前缀 `vacation_`**

需要的零件(都学过了):
- `for file in *.jpg` —— 循环 + 通配符
- `mv 旧名 新名` —— mv 既是移动也是重命名
- `"vacation_$file"` —— 变量拼接,记得加双引号

<details>
<summary>基础版答案</summary>

```bash
#!/bin/bash
for file in *.jpg
do
    mv "$file" "vacation_$file"
    echo "Renamed: $file -> vacation_$file"
done
```
</details>

**进阶版:让脚本接收参数,变成通用工具**

目标用法:`bash batch_rename.sh jpg holiday_` → 把所有 jpg 加上 holiday_ 前缀

<details>
<summary>进阶版答案</summary>

```bash
#!/bin/bash
if [ $# -ne 2 ]; then
    echo "Usage: bash batch_rename.sh <extension> <prefix>"
    exit 1
fi

ext="$1"
prefix="$2"
count=0

for file in *."$ext"
do
    if [ -f "$file" ]; then
        mv "$file" "${prefix}${file}"
        echo "Renamed: $file -> ${prefix}${file}"
        count=$((count + 1))
    fi
done

echo "Done. $count file(s) renamed."
```

`$((...))` 是数学运算,`${prefix}${file}` 用花括号明确变量边界。
</details>

**改扩展名的技巧:**
```bash
for file in *.txt
do
    mv "$file" "${file%.txt}.md"
done
```
`${file%.txt}` 是**参数展开**:从变量末尾删掉 `.txt`。处理文件名的常用手法。

---

## Day 8:Git 版本控制

**Git 解决什么问题?** 你是不是有过 `code_v1.py`、`code_v2_final.py`、`code_final_真的最终版.py` 这种文件?Git 自动记录每次改动,随时能回到任何版本,还能看到每次改了什么。

### 一次性配置

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"     # 要和 GitHub 账号一致!
git config --global init.defaultBranch main

git config --list        # 确认
```

**邮箱要和 GitHub 账号一致**,否则你的提交在 GitHub 上会显示成"陌生人"提交的。

### 核心工作流:改 → add → commit

```bash
mkdir ~/my_project && cd ~/my_project
git init                 # 初始化仓库(会建一个隐藏的 .git 目录,别手动动它)

echo "# My Project" > README.md
git status               # 显示 Untracked files(红色)

git add README.md        # 加入暂存区
git status               # 变绿了,Changes to be committed
git add .                # . 表示当前目录所有文件

git commit -m "initial commit: add readme"   # 提交
git log --oneline        # 查看历史
```

**为什么提交要分 add 和 commit 两步?** 因为你常常改了 5 个文件,但只想把其中 3 个作为一次有意义的提交。

**颜色区分:红色 = Git 还没管(未追踪);绿色 = 已暂存待提交。**

### 日常命令

```bash
git status               # 看当前状态(最常用)
git diff                 # 看具体改了什么(提交前检查一遍是好习惯)
git log --oneline        # 精简版历史
```

commit 输出的信息怎么读:
```
[main 369d1d6] add notes
 2 files changed, 3 insertions(+)
```
- `369d1d6` = 这次提交的唯一编号
- `3 insertions(+)` = 新增了 3 行
- 修改一行,Git 算作"删1行+加1行"(它按行比对,没有"修改"的概念)

**提交节奏:每完成一件有意义的小事就 commit 一次。** 别攒一大堆最后一次性提交,那样没法只回滚其中一部分。

### ★ .gitignore(对 ROS 特别重要)

有些文件不该进仓库:编译产物、临时文件、大文件。

```bash
nano .gitignore
```
```
# ROS 2 编译产物
build/
install/
log/

# Python
__pycache__/
*.pyc

# 编辑器
.vscode/
*.swp
```

**为什么至关重要:** 用 colcon 编译 ROS 工作空间会生成 `build/`、`install/`、`log/` 三个目录,**里面成千上万个文件,可能几百 MB**。这些是自动生成的,别人拿到源码自己编译就有,传上去只会:
- 把仓库撑爆,push 都 push 不动
- 每次编译都变,`git status` 里全是噪音,你看不清自己到底改了什么

**新手最常见的 GitHub 事故就是把 build/ 传上去了。**

验证 .gitignore 生效:
```bash
mkdir build && touch build/fake.o
git status        # 应该看不到 build/
rm -rf build
```

### 连接 GitHub(SSH 方式)

```bash
ssh-keygen -t ed25519 -C "你的邮箱"     # 一路回车
cat ~/.ssh/id_ed25519.pub               # 复制输出的公钥
```

> **概念澄清:** 生成的是一对密钥。`id_ed25519`(无后缀)是**私钥**,绝对不能给任何人;`id_ed25519.pub` 是**公钥**,就是要贴给 GitHub 的。别搞反。

贴到 GitHub:头像 → Settings → SSH and GPG keys → New SSH key → 粘贴 → 保存

测试:
```bash
ssh -T git@github.com
```
显示 `Hi 用户名! You've successfully authenticated, but GitHub does not provide shell access.` **就是成功了**(这句看着像报错,其实是正常的)。

推送:
```bash
# 先在 GitHub 网页建一个空仓库,不要勾选任何初始化选项
git remote add origin git@github.com:你的用户名/仓库名.git
git branch -M main
git push -u origin main       # -u 只需第一次加,以后直接 git push
```

---

# 第二部分:ROS 2 核心概念

## 先用命令行"观察",再动手写

**这个顺序很重要。** 先看清楚系统里在跑什么、怎么通信,比一上来抄代码强得多。

### 确认环境

```bash
printenv | grep ROS      # 应该看到 ROS_DISTRO=jazzy
```

### 跑起 demo,带着问题观察

终端 1:
```bash
ros2 run demo_nodes_cpp talker
```

新开终端 2:
```bash
ros2 node list           # 输出 /talker
ros2 node info /talker
```

`node info` 的输出里,**大部分是 ROS 自动给每个节点配的基础设施**(参数系统、日志系统),talker 自己的只有一行:
```
Publishers:
  /chatter: std_msgs/msg/String
```

### ★ 三个核心概念

```
/chatter: std_msgs/msg/String
└──┬───┘  └────────┬────────┘
  话题          消息类型
```

**节点(Node)** = 一个独立运行的程序单元。ROS 的哲学是"一个节点干一件事":读雷达一个节点、做 SLAM 一个节点、控制轮子一个节点。你的机器人就是十几个节点拼起来的。

**话题(Topic)** = 一个**具名的数据通道**。节点不直接跟对方通信,而是"往某个话题发"、"从某个话题收"。

**消息类型(Message Type)** = 数据的格式约定。发布方和订阅方必须用**同一个类型**才能通信,类型不匹配就收不到(新手常见故障)。

### ★★ 最重要的设计:发布订阅解耦

**talker 根本不知道有没有人在听,listener 也不知道数据是谁发的。**

亲手验证:
```bash
# 终端2
ros2 topic list
ros2 topic info /chatter        # Subscription count: 0
ros2 topic echo /chatter        # 你等于亲手当了一次 listener!
# Ctrl+C 停掉
ros2 topic info /chatter        # 订阅数又变回 0
```

```bash
# 终端3:启动真正的 listener
ros2 run demo_nodes_py listener

# 终端4:手动往话题里发消息
ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello from me'}"
```

**去看 listener 那个终端**——它同时收到两种消息,talker 发的和你手动发的,混在一起。listener 完全不关心是谁发的。

**这种"互不认识"就是 ROS 最重要的设计:** 你可以随时增删节点,不用改任何一方的代码。想加个记录器把数据存下来?订阅那个话题就行,talker 完全不受影响。

### 查看消息类型定义

```bash
ros2 interface show std_msgs/msg/String
```
输出就一行 `string data`——所以 echo 时才显示 `data: '...'`。

### 可视化整个系统

```bash
rqt_graph
```
椭圆是节点,箭头是话题数据流向。能直观看到 `/talker → /chatter → /listener`。

(左上角下拉框选 "Nodes/Topics (all)",再点刷新)

**注意:Service 不会显示在图上**——因为 Service 的连接只在客户端发请求那一瞬间存在,平时服务端只是待命,没有持续的数据流。这进一步印证了 Topic 和 Service 的本质区别。

---

## 创建你的第一个 ROS 包

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg --dependencies rclpy std_msgs
cd my_first_pkg && tree
```

生成的结构:
```
my_first_pkg/
├── my_first_pkg/          ← Python 代码放这里(和外层同名,是 Python 包的规矩)
│   └── __init__.py
├── package.xml            ← 包的"身份证":名字、版本、依赖
├── resource/
│   └── my_first_pkg       ← 空文件,标记"这是个有效的 ROS 包"
├── setup.cfg
├── setup.py               ← 安装配置、入口点注册
└── test/                  ← 自动生成的代码规范检查,先不用管
```

**最容易搞混的一点:** 外层 `my_first_pkg/` 是**整个 ROS 包**;内层 `my_first_pkg/my_first_pkg/` 才是 **Python 代码真正住的地方**。

`setup.py` 里有个关键位置:
```python
entry_points={
    'console_scripts': [
    ],
},
```
**现在是空的**,这就是为什么 `ros2 run` 还跑不了任何东西。写完节点要回来注册。

---

## 写一个 Publisher(发布者)

```bash
cd ~/ros2_ws/src/my_first_pkg/my_first_pkg
nano battery_publisher.py
```

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        self.publisher_ = self.create_publisher(String, 'battery_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = 100

    def timer_callback(self):
        msg = String()
        msg.data = f'Battery: {self.battery_level}%'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.battery_level -= 1
        if self.battery_level < 0:
            self.battery_level = 100

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 逐段理解

**1. 导入**
- `rclpy` = ROS 的 Python 库本体
- `Node` = 所有节点的基类,你的类要继承它
- `String` = 消息类型

**2. 继承 Node 创建节点**
```python
class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
```
**注意:这不是"访问"已有的东西,是创建一个全新的节点实例。** 继承 `Node` 是为了拿到基类自带的能力(`create_publisher`、`get_logger` 等),不用自己造轮子。

`super().__init__('battery_publisher')` 里的名字,就是以后 `ros2 node list` 看到的名字。

**3. 创建发布者**
```python
self.publisher_ = self.create_publisher(String, 'battery_status', 10)
```
三个参数:消息类型、话题名、队列长度(消息缓冲区大小)。

**发布者就是"话筒"**——是节点和外界通信的出口。没有它,你的节点算出的数据只能自己留着,发不出去。这和你手动敲 `ros2 topic pub` 干的是同一件事,只是写进了代码。

**4. 定时器**
```python
self.timer = self.create_timer(1.0, self.timer_callback)
```
"每 1 秒自动调用一次 `timer_callback`"。

**★ 这是 ROS 节点和普通脚本最大的不同:** 不是从上到下执行完就退出(像 bash 脚本那样),而是**注册好回调函数,然后一直待命等事件触发**。

**5. 回调函数(每秒真正执行的内容)**
```python
def timer_callback(self):
    msg = String()                    # 造一个空"信封"
    msg.data = f'Battery: ...'        # 往信封里写内容
    self.publisher_.publish(msg)      # 投进邮筒(话题)
    self.get_logger().info(...)       # 顺便打印日志
```
为什么要"造对象再填内容"?因为 `String` 类型规定了必须有个叫 `data` 的字段(就是 `interface show` 看到的 `string data`),不能直接把字符串发出去。

`get_logger().info()` 是 ROS 专用的打印方式,比 `print()` 好——能分级别、能被日志系统统一管理。

**6. main 函数**
```python
rclpy.init(args=args)      # 启动 ROS 通信系统
node = BatteryPublisher()  # 创建节点实例
rclpy.spin(node)           # ★ 让节点"转起来",持续监听事件直到 Ctrl+C
```
**`rclpy.spin()` 不写的话,程序建完节点就直接退出了,定时器一次都不会触发。**

### ★★ 三步流程(每次加新节点都要走一遍)

**这是初学者最容易漏的地方,记牢:**

**第 1 步:在 setup.py 注册**
```python
entry_points={
    'console_scripts': [
        'battery_publisher = my_first_pkg.battery_publisher:main',
    ],
},
```

语法拆解(很机械):
```
battery_publisher = my_first_pkg.battery_publisher:main
└──────┬────────┘   └──────┬────────┘ └─────┬──────┘
  命令行里的名字        包名.文件名          函数名
```
意思是"执行 `ros2 run my_first_pkg battery_publisher` 时,去 `my_first_pkg/battery_publisher.py` 调用它的 `main` 函数"。

**第 2 步:编译**
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
```
(`--packages-select` 只编译指定的包,工作空间大了之后能省很多时间)

**第 3 步:source**
```bash
source install/setup.bash
```

**运行:**
```bash
ros2 run my_first_pkg battery_publisher
```

**⚠️ `source` 只对当前终端有效!** 每开一个新终端都要重新 source(或者加进 `~/.bashrc` 自动加载)。

---

## 写一个 Subscriber(订阅者)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.subscription = self.create_subscription(
            String,
            'battery_status',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
        level = int(msg.data.split(': ')[1].replace('%', ''))
        if level < 20:
            self.get_logger().warn(f'LOW BATTERY WARNING: {level}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 和 Publisher 对比

**相似:** 导入方式、`super().__init__(名字)`、main 函数结构完全一样。

**核心不同:**
- Publisher 用 `create_publisher` + `create_timer`(**主动定时发**)
- Subscriber 用 `create_subscription` + 回调函数(**被动等待,消息来了才触发**)

**`create_subscription` 的四个参数:** 消息类型、话题名(**必须和 publisher 完全一致**)、回调函数、队列长度。

**`listener_callback(self, msg)` 不是你主动调用的**,是 ROS 收到消息时自动调用,`msg` 就是收到的消息对象。

`get_logger().warn()` 是不同的日志级别,终端里会用黄色标出来。

**记得走三步流程(注册 → build → source),然后两个终端分别跑起来验证。**

---

## Service(服务):一问一答的通信

### 和 Topic 的本质区别

| | Topic | Service |
|---|---|---|
| 模式 | 广播式,单向,持续 | 请求-响应,一问一答 |
| 类比 | 电台广播(不知道谁在听) | 打电话问客服(等对方回答) |
| 适合 | 状态流:电量、位置、传感器数据 | 一次性操作:计算、检查、重置 |
| 是否需要回应 | 不需要 | **必须返回 response** |

### 定义 Service 接口

先建接口文件:
```
int64 required_minutes
---
bool can_complete
string message
```
`---` 上面是**请求**字段,下面是**响应**字段。

### ⚠️ 重要:接口必须放在单独的 ament_cmake 包里

**原因是纯技术限制,和 Topic/Service 的区别无关:** 生成接口对应的 Python 代码需要一套叫 `rosidl` 的代码生成工具,而它依赖 **CMake** 构建系统。

你的 `my_first_pkg` 是 `ament_python` 类型,**没有 CMakeLists.txt**,所以没法生成接口代码。

**业界习惯:不管 msg 还是 srv,自定义接口都放进一个专门的 `xxx_interfaces` 包**——这样多个包可以共享同一套接口,不产生循环依赖。

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_first_pkg_interfaces
mkdir my_first_pkg_interfaces/srv
# 把 CheckBattery.srv 放进 my_first_pkg_interfaces/srv/
```

**配置 `my_first_pkg_interfaces/package.xml`**,在 `<buildtool_depend>ament_cmake</buildtool_depend>` 下面加:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**配置 `CMakeLists.txt`**,在 `ament_package()` 之前加:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CheckBattery.srv"
)
```

编译并验证:
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg_interfaces
source install/setup.bash
ros2 interface show my_first_pkg_interfaces/srv/CheckBattery
```

能正常显示接口定义,就说明 ROS 认识你自定义的类型了。

**最后在 `my_first_pkg/package.xml` 里声明依赖:**
```xml
<depend>my_first_pkg_interfaces</depend>
```

### Service 服务端

```python
import rclpy
from rclpy.node import Node
from my_first_pkg_interfaces.srv import CheckBattery

class BatteryService(Node):
    def __init__(self):
        super().__init__('battery_service')
        self.srv = self.create_service(
            CheckBattery,
            'check_battery',
            self.check_battery_callback)
        self.current_battery = 45

    def check_battery_callback(self, request, response):
        minutes_needed = request.required_minutes
        minutes_available = self.current_battery
        if minutes_available >= minutes_needed:
            response.can_complete = True
            response.message = f'OK, battery has {minutes_available} min, task needs {minutes_needed} min'
        else:
            response.can_complete = False
            response.message = f'NOT ENOUGH, battery has {minutes_available} min, task needs {minutes_needed} min'
        self.get_logger().info(f'Request: {minutes_needed} min -> {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BatteryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**关键区别:**
- 回调函数有**两个参数**(subscriber 只有一个 `msg`):`request` 是客户端问的内容,`response` 是你要填好返回的
- `request.required_minutes`、`response.can_complete` 这些**字段名必须和 .srv 文件里完全一致**
- **必须 `return response`**——这是 Service 和 Topic 最本质的区别

### 用命令行测试

服务端跑起来后**不会有任何输出**(它只是安静地等着有人来问),这是正常的。

```bash
ros2 service list        # 应该看到 /check_battery
ros2 service call /check_battery my_first_pkg_interfaces/srv/CheckBattery "{required_minutes: 30}"
```

输出:
```
requester: making request: ...(required_minutes=30)
response:
...(can_complete=True, message='OK, battery has 45 min, task needs 30 min')
```

**记得试一个会失败的场景**(比如 60 分钟),验证两条分支都对。

### Service 客户端

```python
import sys
import rclpy
from rclpy.node import Node
from my_first_pkg_interfaces.srv import CheckBattery

class BatteryClient(Node):
    def __init__(self):
        super().__init__('battery_client')
        self.client = self.create_client(CheckBattery, 'check_battery')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, minutes):
        request = CheckBattery.Request()
        request.required_minutes = minutes
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = BatteryClient()
    minutes = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    response = node.send_request(minutes)
    node.get_logger().info(f'Result: can_complete={response.can_complete}, message="{response.message}"')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**关键点:**

**`wait_for_service`** —— 如果服务端还没启动,客户端会一直等。Topic 不需要这个(publisher 可以对着空气发消息)。

**`call_async` + `spin_until_future_complete`:**
- `call_async(request)` 发出请求,返回一个 `future`(代表"未来会有结果的占位符")
- `spin_until_future_complete(...)` **原地等**,直到服务端返回
- `future.result()` 取出真正的 response

**`sys.argv`** —— Python 里读命令行参数,和 bash 脚本的 `$1` `$2` 是同一个思想。

**客户端跑完就退出**——main 里没有 `rclpy.spin(node)`,因为问完一次拿到答案就该结束。这是和 server/subscriber 最大的行为差异。

```bash
ros2 run my_first_pkg battery_client 40
```

---

## Parameter(参数):不改代码调整行为

回想每个节点的 `node info` 里都有一串 `/xxx/get_parameters` 服务——那就是 ROS 自动配的参数系统。

**为什么重要:** 同一份代码,换个环境(仿真 vs 真机、不同机器人)只需改参数,不用重写代码。

### 改造 Publisher

```python
def __init__(self):
    super().__init__('battery_publisher')

    self.declare_parameter('publish_rate', 1.0)
    self.declare_parameter('initial_level', 100)

    rate = self.get_parameter('publish_rate').value
    self.battery_level = self.get_parameter('initial_level').value

    self.publisher_ = self.create_publisher(String, 'battery_status', 10)
    self.timer = self.create_timer(1.0 / rate, self.timer_callback)
```

- **`declare_parameter(名字, 默认值)`** —— 声明参数。**必须先声明才能用**(防止拼写错误导致的隐藏 bug)
- **`get_parameter(名字).value`** —— 读取当前值

**设计思想:参数没被指定时用默认值继续运行,而不是报错退出。** 这和 Linux 命令的设计一致(`head` 不写 `-n` 就默认 10 行)——**合理默认值 + 可选覆盖**。

### 命令行覆盖参数

```bash
# 用默认值
ros2 run my_first_pkg battery_publisher

# 覆盖参数
ros2 run my_first_pkg battery_publisher --ros-args -p publish_rate:=2.0 -p initial_level:=20
```

**怎么验证生效了?看时间戳算间隔:**
```
[1786108288.938184660] Battery: 20%
[1786108289.431887500] Battery: 19%     ← 差约 0.49 秒
[1786108289.931843155] Battery: 18%     ← 差约 0.50 秒
```
每次间隔 0.5 秒 = 每秒 2 次,`publish_rate:=2.0` 精确生效。**不要凭感觉说"好像变快了",从数字上验证。**

---

## Launch 文件:一条命令启动多个节点

**解决的痛点:** 测试时要开三四个终端,每个都要 source、手动敲 `ros2 run`。

```bash
mkdir -p ~/ros2_ws/src/my_first_pkg/launch
nano ~/ros2_ws/src/my_first_pkg/launch/battery_system.launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_pkg',
            executable='battery_publisher',
            name='battery_publisher',
            parameters=[
                {'publish_rate': 2.0},
                {'initial_level': 30}
            ]
        ),
        Node(
            package='my_first_pkg',
            executable='battery_monitor',
            name='battery_monitor'
        ),
    ])
```

**理解:**
- `generate_launch_description()` —— ROS 规定必须叫这个名字的函数
- `LaunchDescription([...])` —— 一个列表,每项是"要启动的一个东西"
- **每个 `Node(...)` 就对应你手敲的一条 `ros2 run`**,参数直接对应:
  - `package` → 包名
  - `executable` → 可执行文件名
  - `parameters` → 对应 `--ros-args -p`

**还要在 setup.py 里声明 launch 目录**(否则 ROS 找不到):

顶部加:
```python
import os
from glob import glob
```

`data_files` 列表里加一项:
```python
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
```

运行:
```bash
colcon build --packages-select my_first_pkg
source install/setup.bash
ros2 launch my_first_pkg battery_system.launch.py
```

输出会是这样:
```
[INFO] [battery_publisher-1]: process started with pid [7439]
[INFO] [battery_monitor-2]: process started with pid [7440]
[battery_publisher-1] [INFO] ...: Publishing: "Battery: 30%"
[battery_monitor-2] [INFO] ...: Received: "Battery: 30%"
```

**`[battery_publisher-1]` 这些前缀是 launch 自动加的标签**,方便在混合输出里分清谁在说话。以后同时跑十个节点时会救命。

**一次 Ctrl+C 会关掉 launch 管理的所有子进程**,不用一个个杀。

---

# 附录 A:必踩的坑(提前看,能省几小时)

## 1. ★ nano 编辑后没保存

**症状:** 明明改了代码,运行结果却和没改一样。

**原因:** 在 nano 里编辑完,**忘记按 `Ctrl+O` 保存**就直接 `Ctrl+X` 退出了,磁盘上还是旧文件。

**★ 调试第一原则:** **"代码看起来对但结果不对"时,永远不要相信自己"应该改过了"的记忆,直接 `cat` 文件看磁盘上真实存的是什么。**
```bash
cat 你的文件.py
```

保存的正确动作:`Ctrl+O` → 回车确认 → 看到底部 `[Wrote N lines]` → 再 `Ctrl+X`

## 2. ★ 新终端忘记 source

**症状:** `The passed service type is invalid`、`package not found`、找不到自己写的节点。

**原因:** 新开的终端只加载了全局 ROS 环境(`/opt/ros/jazzy`),不知道你自己工作空间的存在。

**解决:**
```bash
source ~/ros2_ws/install/setup.bash
```

**这是 ROS 开发里最常见的坑,几乎每个人都在这摔过。** 遇到莫名其妙的"找不到"类报错,第一反应就该是"这个终端 source 了吗"。

## 3. 忘记注册 entry_point 或没重新 build

**症状:** `No executable found`

**原因:** 写了 `.py` 文件但没在 setup.py 的 `console_scripts` 里注册,或者注册了但没重新 `colcon build`。

**记住三步流程:写代码 → 注册 setup.py → colcon build → source**

## 4. 空格敏感

```bash
ls /opt /nonexistent    # 两个参数
ls /opt / nonexistent   # 三个参数!含义完全不同

name = "Bill"    # 错,bash 以为要执行 name 命令
name="Bill"      # 对

if [-f "$file"]     # 错
if [ -f "$file" ]   # 对
```

## 5. 报错不一定是红色

终端的 stderr 颜色取决于配置,很多时候和正常输出一个颜色。**判断是不是报错看内容(`cannot access` / `No such file`),不是看颜色。**

## 6. grep 匹配到自己

`ps aux | grep xxx` 的结果里,那行 COMMAND 是 `grep --color=auto xxx` 的是 grep 自己。用 `pgrep -a xxx` 更干净。

---

# 附录 B:命令速查表

## Linux

| 命令 | 作用 |
|---|---|
| `pwd` | 我在哪 |
| `ls -lah` | 详细列表+隐藏文件+可读大小 |
| `cd ~` / `cd ..` / `cd -` | 回家 / 上级 / 上一个目录 |
| `mkdir -p a/b/c` | 建多层目录 |
| `cp` / `mv` / `rm -r` | 复制 / 移动或改名 / 删除 |
| `chmod 755 file` | 改权限 |
| `ps aux` / `top` / `kill PID` | 查进程 / 实时监视 / 结束进程 |
| `sudo apt install xxx` | 装软件 |
| `cat` / `less` / `head` / `tail -f` | 看文件 |
| `grep -rn "词" 目录/` | 递归搜索 |
| `wc -l` | 数行数 |
| `命令1 \| 命令2` | 管道 |
| `> 文件` / `>> 文件` | 重定向覆盖 / 追加 |
| `2>/dev/null` | 屏蔽报错 |
| `man 命令` / `命令 --help` | 查手册 |

## Git

| 命令 | 作用 |
|---|---|
| `git status` | 看状态(最常用) |
| `git add .` | 暂存所有改动 |
| `git commit -m "说明"` | 提交 |
| `git log --oneline` | 看历史 |
| `git diff` | 看改了什么 |
| `git push` | 推到远程 |

## ROS 2

| 命令 | 作用 |
|---|---|
| `ros2 node list` / `ros2 node info /节点` | 列节点 / 看节点详情 |
| `ros2 topic list` / `info` / `echo` | 列话题 / 详情 / 实时看数据 |
| `ros2 topic pub 话题 类型 "{...}"` | 手动发消息 |
| `ros2 service list` / `service call` | 列服务 / 调用服务 |
| `ros2 interface show 类型` | 看消息/服务定义 |
| `ros2 run 包名 可执行名` | 运行节点 |
| `ros2 launch 包名 xxx.launch.py` | 用 launch 启动 |
| `ros2 pkg create --build-type ament_python 包名` | 建包 |
| `colcon build --packages-select 包名` | 编译 |
| `source install/setup.bash` | 加载工作空间环境 |
| `rqt_graph` | 可视化节点关系 |

---

# 附录 C:学习建议

## 时间投入

按每天 2 小时算:
- **Linux 基础(Day 1-8):** 约 1-2 周
- **ROS 2 核心:** 约 1-2 周

基础好的话 Linux 部分可以压缩,但**别跳过 bash 脚本和 Git**——后面天天用。

## 学完这些之后

下一步方向(按顺序):
1. **URDF + Gazebo 仿真** —— 给机器人建一个物理身体,放进仿真世界
2. **tf2 坐标变换** —— 机器人各部件之间的位置关系
3. **SLAM 建图**(slam_toolbox)
4. **Nav2 自主导航**

## ⚠️ 版本相关的坑(会节省你大量时间)

1. **Gazebo 新旧版本混淆(最大的坑):** ROS 2 Jazzy 配套的是**新版 Gazebo(Harmonic,命令 `gz sim`)**,不是老的 Gazebo Classic(命令 `gazebo`,已停止维护)。网上大量 2024 年前的教程用的是 Classic,插件和话题桥接方式完全不同。**看教程先确认它用的哪个版本,对不上就换教程。**

2. **视频教程版本错配:** B 站不少 ROS 2 课程基于 Humble 录制,99% 内容和 Jazzy 通用,但遇到命令报错时,第一反应应该是查 Jazzy 官方文档确认命令是否变了,而不是怀疑自己。

3. **官方文档是最好的教程:** docs.ros.org(选 Jazzy 版本)是逐步可执行的、随版本更新的。**从 ROS 阶段开始,建议以英文官方文档为第一手资料**,中文视频当辅助。

## 最后一句

这份笔记里的每个坑都是真踩过的。你也一定会踩到新的坑——**那不是你笨,那是学习本身。** 每修好一个报错,你就比只会复制粘贴命令的人多会一点东西。

祝顺利。

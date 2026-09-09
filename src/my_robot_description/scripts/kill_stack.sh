#!/bin/bash
# 彻底停掉本项目的仿真 + 导航栈, 并验证到零。
#
# 用法:  bash scripts/kill_stack.sh
#
# 为什么需要它, 以及为什么不能直接敲 pkill —— 两个坑都真踩过:
#
# 1. 模式串必须写在脚本文件里, 不能写在命令行上。
#    pgrep/pkill -f 匹配的是完整命令行, 而你敲的那条
#    `pkill -f "gz sim"` 自己的命令行里就含有 "gz sim",
#    于是它把执行清理的 shell 一起杀掉, 清理只做了一半,
#    而你看到的退出码像是成功的。
#
# 2. 模式不能写成 '/gz sim'。真正的仿真进程命令行是
#    `gz sim -s -r ...`, 没有前导斜杠; 而 'gz_tools_vendor'
#    只匹配得到外层的 sh/ruby 包装。杀了包装, gz sim 子进程
#    会活下来变成孤儿。
#
# 孤儿仿真的症状(全部实际遇到过, 当时都没读对):
#    /scan 频率是配置值的两倍甚至更多  -> 有多个发布者
#    /odom 是上一轮的陈旧值            -> 在跟旧仿真说话
#    gz model -m my_robot -p 返回空    -> 模型名在多个世界里歧义
#    日志刷 TF_OLD_DATA                -> 两个时钟源
#    参数改了不生效、lifecycle 状态自相矛盾 -> 同名节点随机应答
#
# 结论: 在这个环境里做的任何测量都不作数。测量之前先确认被测系统唯一。

SELF=$$
PAT='/opt/ros/jazzy/lib/nav2|ros_gz_bridge|robot_state_publisher|gz_tools_vendor|(^|[[:space:]])gz sim'

for _ in 1 2 3 4; do
  PIDS=$(pgrep -f "$PAT" | grep -v "^${SELF}$" | tr '\n' ' ')
  [ -z "$PIDS" ] && break
  kill -9 $PIDS 2>/dev/null
  sleep 2
done
sleep 1

LEFT=$(pgrep -f "$PAT" | grep -v "^${SELF}$" | wc -l)
echo "残留进程: $LEFT"
if [ "$LEFT" -ne 0 ]; then
  pgrep -af "$PAT" | grep -v "^${SELF} "
  exit 1
fi

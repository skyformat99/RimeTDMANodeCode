本文件夹包括了基于Contiki无线通信模块Node系统的软件代码：
Project      IAR环境下所有工程文档
Application  应用程序，以Contiki系统的protothread方式组织
Driver       有差异的驱动文件：CommPC.c/.h stm8L15x_it.c
main.c       基于C语言系统的入口函数
main.h       系统的全局配置文件
OS+Driver+RF 代码引用自"ShareCode/ContikiSTM8LSX1278"

软件版本命名：x.y.z
x=产品序号，如1=iNode1, 2=iNode2；
y=较大修正递增序号（增加功能模块），同时奇数是测试版本，偶数是发布版本；
z=Bug修复递增序号，即每修改一个Bug，该数字加一。
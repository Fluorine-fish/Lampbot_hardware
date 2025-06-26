# 写库规范 v0.0.1
___
## 0.前言
- 此文档用于维护RM项目中间层库的编写规范，主要包括命名规范、文件结构、FreeRTOS使用规范、头文件使用规范、日志系统等。
- CPP规范为 C++20
___
## 1.项目架构
- User
  - UserData 
  - UserLib 
  - UserTask
___
## 2.命名规范
### 1.文件命名
   - 小写字母+下划线
   - 前缀式命名
   - e.g. `task_debug.cpp`
### 2.函数命名
   - 一般函数：模仿HAL库命名，
   - 成员函数：纯小写字母+下划线（e.g. `can_filter_init()`）
___
## 3.FreeRTOS使用规范
### 1. 任务命名&入口函数：
   - 将`defaultTask`(默认生成的任务)重命名为`Task_Debug`用于测试调试（开发初期写shit用）
   - 任务命名模仿HAL库(e.g. `Task_Debug`)
   - 入口函数命名模仿HAL库命名（e.g.`AppTask_Debug()`）
### 2.Code Generation Option
   - 优先使用`As external`选项
___
## 4.头文件使用规范
### 1.头文件均使用`.h`文件
### 2.包含一个统一使用的头文件
   - `sys_public.h`：写在`/UserLib`下
   - 重定义一些操作，诸如Delay，new等
### 3.头文件编写&使用规范：
   - `#include`尽量放在`.c`文件中
   - 如果不需要对外开放接口的文件不配套头文件
___
## 5.日志系统(待完善)
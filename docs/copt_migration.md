# SANDO 使用 COPT

本项目的轨迹优化模型原来通过 Gurobi C++ API 构造。现在 CMake 默认使用杉数 COPT，规划模型本身不变：仍然包含连续轨迹变量、二次 jerk 目标、动力学二次约束以及安全走廊的指示约束。

## 1. 安装 COPT 后设置环境变量

在 Ubuntu 中假设 COPT 安装目录为 `/opt/copt`：

```bash
export COPT_HOME=/opt/copt
export LD_LIBRARY_PATH="$COPT_HOME/lib:$LD_LIBRARY_PATH"
export COPT_LICENSE_DIR="$HOME/.copt"
```

`COPT_HOME` 下应至少能找到：

```text
include/copt.h
include/coptcpp_inc/coptcpp_pch.h
lib/libcopt_cpp.so
```

许可证仍需按照 COPT 官方安装方式配置。许可证不是本项目代码的一部分。

## 2. 编译

在 ROS 2 工作空间中执行：

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select sando --cmake-args -DSANDO_USE_COPT=ON
source install/setup.bash
```

如果要临时恢复原 Gurobi 后端：

```bash
colcon build --packages-select sando --cmake-args -DSANDO_USE_COPT=OFF
```

## 3. 代码适配边界

- `FindCOPT.cmake` 查找 COPT 的头文件和 `libcopt_cpp` 动态库。
- `CMakeLists.txt` 通过 `SANDO_USE_COPT` 切换后端，默认值为 `ON`。
- `include/sando/copt_compat.hpp` 将原有求解器模型接口映射到 COPT 的 `Envr`、`Model`、`Var`、`Expr`、`QuadExpr`、`Constraint`、`QConstraint` 和 `GenConstr`。
- `src/sando/gurobi_solver.cpp` 中的规划算法、变量消元、Bézier/MINVO 控制点、动力学约束和安全走廊约束没有改动。
- `stopExecution()` 在 COPT 路径下使用 COPT 的 `Model::Interrupt()`；原 Gurobi 路径仍保留原回调中止逻辑。

为了避免一次迁移影响 SANDO 的其他模块，类名和部分参数名暂时保留 `SolverGurobi`、`max_gurobi_comp_time_sec` 等历史名称；它们在 COPT 构建中只是兼容性名称，不代表实际仍调用 Gurobi。

## 4. 当前验证范围

当前开发环境没有安装 ROS 2 和 COPT Linux SDK，因此本机只能完成源码和 CMake 静态检查，不能在这里替你完成最终链接。首次在 Ubuntu 编译时，如果 COPT 版本的 C++ 头文件目录不同，只需要调整 `FindCOPT.cmake` 中的 `COPT_CPP_INCLUDE_DIR` 搜索路径。

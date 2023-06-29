# 浮力调节机构数学建模

需要增大油囊体积时，无刷直流电机正转，液压泵开始吸排油，液压油从油箱流向油囊；
需要减小油囊体积时，无刷直流电机反转，液压油从油囊流向油箱。

## reference

[Modeling and control of a novel electro-hydrostatic actuator with adaptive pump displacement](https://www.sciencedirect.com/science/article/pii/S100093611830205X)

## 无刷直流电机

传统的无刷直流电机控制系统主要由电机主体、电子换向电路以及转子位置传感器三大部分组成，电子换向电路又主要由逆变电路和控制电路组成。

电机电压平衡方程与运动方程形式如下：

$$
\begin{cases}
    U=E+L\dot{i}+Ri\\
    T_e =T_l + J\dot{\omega} + B_v\omega
\end{cases}
$$

式中， $B_v$ 为电机粘滞摩擦系数。

忽略逆变器内部动态过程，通过传递函数来表达电动机电枢电压 $U_c$ 与控制信号电压 $U$ 之间关系。

$$
\frac{U_c(s)}{U(s)} = \frac{K_s}{T_ss+1}
$$

式中， $T_s$ 为时间常数， $K_s$ 为电压放大系数。 

采用三相六状态工作方式运行时，忽略无刷直流电机换相过程，在任意时刻无刷直流电机只有两相导通，无刷直流电机可看作直流电机，则无刷直流电机的电压平衡方程为（假设AB相导通）：

$$
U - e_A + e_B = Ri_A-Ri_B+(L-M)(\dot{i_A}-\dot{i_B})
$$

其中， A、B相反电动势 $e_A$、 $e_B$ 大小相等、符号相反，相电流 $i_A$ 、 $i_B$ 大小相等、符号相反，则可简化为

$$
U-e = R_sI+L_s\dot{I}
$$

式中， $R_s=2R$ ， $L_s=2(L-M)$ ， $e=K_e\omega$ 为无刷直流电机的反电动势。

则数学模型如下：

$$
\begin{cases}
U_c = E + L\dot{i}+Ri\\
E=K_c\omega\\
T_e = K_ti\\
T_e = J\dot{\omega}+T_f+T_l
\end{cases}
$$

其中， $U_c$ 为电枢电压， $E$ 为电枢反电动势， $R$ 为电枢绕组内阻， $L$ 为电枢绕组电感， $J$ 为折算到电机轴上的转动惯量， $\omega$ 为电机输出角速度， $T_e$ 为电磁转矩， $T_l$ 为负载转矩， $T_f$ 为摩擦转矩，取值与电机转速有关， $i$ 电枢绕组相电流。

## 液压泵

液压泵两端分别连通油箱和油囊。

$$
\begin{cases}
    Q = D_p\omega-K_{ip}(P_a - P_b) - K_{ep}(P_a-P_b)\\
T_l = D_p(P_a - P_b)
\end{cases}
$$

其中， $Q$ 液压泵至油囊的管路液压油流量， $D$ 为定量泵的排量 $D_p = \frac{D}{2\pi}$ ， $K_{ip}$ 为液压泵内泄露系数， $K_{ep}$ 为液压泵外泄露系数， $P_a$ 为液压泵至油囊的管路油液压力， $K_c$ 为电机转速常数， $K_t$ 为电机转矩常数。

## 油囊

视为液压缸的无杆腔，油囊体积变化视为规则形变的圆柱体

$$
\begin{cases}
    Q = A\dot{x}+\frac{V}{B}\dot{P_a}\\
    A(P_a - P_{sea}) = m\ddot{x}\\
    V = Ax
\end{cases}
$$

其中, $A$ 为油囊底面积， $x$ 为油囊形变位移， $P_{sea}$ 为海水压力， $V$ 为油囊体积， $P_a$ 为油囊内油液压力， $B$ 为油液体积弹性模量。

## 线性化

因为油囊流量方程中压缩性流量一项为变量 $x$ 与变量 $P_a$ 的耦合，为非线性化因素。在水下航行器浮力调节过程中，油囊油液压力近似为海水压力，视 $P_a = P_{sea}$ ，忽略压缩性流量，同时忽略泄露影响。取 $[i \quad \omega \quad x]$ 为状态变量 $[x_1 \quad x_2 \quad x_3 ]$，故得到状态空间方程为：

$$
\begin{bmatrix}
    \dot{x_1}\\
    \dot{x_2}\\
    \dot{x_3}
\end{bmatrix} =
\begin{bmatrix}
    -\frac{R}{L} & -\frac{K_e}{L} & 0\\
    \frac{K_t}{J} & 0 & 0\\
    \frac{D}{2\pi A} & 0 & 0
\end{bmatrix}\cdot
\begin{bmatrix}
    x_1\\
    x_2\\
    x_3
\end{bmatrix}+
\begin{bmatrix}
    \frac{1}{L} & 0\\
    0 & \frac{D}{2\pi J}\\
    0 & 0
\end{bmatrix}\cdot
\begin{bmatrix}
    U_c\\
    P_{sea}
\end{bmatrix}
$$

$$
\begin{split}
    \dot{X} = A\cdot X+B\cdot U\\
y  = V = Ax_3 = Ax
\end{split}
$$

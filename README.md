# LAB 2: uma_arm_control
This is the UMA arm control repo


## Launch the Dynamic model
ros2 launch uma_arm_control uma_arm_dynamics_launch.py

## Implementación del modelo dinámico

### Calcular la aceleración

La dinámica de un manipulador robótico de cadena cinemática abierta está dada por:


$$ \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{F}_b\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau} + \boldsymbol{\tau}_{ext} $$



donde:


* $\mathbf{P} \in \mathbb{R}^{n \times 1}$ es el vector de las posiciones articulares (). `joint_positions_`
* $\dot{\mathbf{q}} \in \mathbb{R}^{n \times 1}$ es el vector de velocidades conjuntas (). `joint_velocities_`
* $\ddot{\mathbf{q}} \in \mathbb{R}^{n \times 1}$ es el vector de aceleraciones de la articulación (). `joint_accelerations_`
* $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{n \times n}$ es la matriz de inercia.
* $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) \in \mathbb{R}^{n \times n}$ es la matriz de Coriolis y fuerzas centrífugas.
* $\mathbf{F}_b \in \mathbb{R}^{n \times n}$ es la matriz de fricción viscosa.
* $\mathbf{g} \in \mathbb{R}^{n \times 1}$ es el vector de gravedad.
* $\boldsymbol{\tau} \in \mathbb{R}^{n \times 1}$ es el vector de los torques articulares comandados (). `joint_torques_`
* $\boldsymbol{\tau}_{ext} \in \mathbb{R}^{n \times 1}$ es el vector de los torques de la articulación debidos a fuerzas externas.



En nuestro caso, la aceleración debida a los torques aplicados está dada por:

$$ \ddot{\mathbf{q}} = \mathbf{M}^{-1}(\mathbf{q})[ \boldsymbol{\tau} + \boldsymbol{\tau}_{ext} - \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} - \mathbf{F}_b\dot{\mathbf{q}} - \mathbf{g}(\mathbf{q}) ] $$



Para calcular las aceleraciones de las articulaciones, primero necesitamos calcular las matrices. Pueden calcularse aplicando las formulaciones de Lagrange o Newton-Euler. En nuestro caso, las matrices se definen por:


$$ 
\mathbf{M}(\mathbf{q}) = 
\begin{bmatrix} 
m_1 \cdot l_1^2 + m_2 \cdot (l_1^2 + 2 \cdot l_1 \cdot l_2 \cdot \cos(q_2) + l_2^2) & m_2 \cdot (l_1 \cdot l_2 \cdot \cos(q_2) + l_2^2) \\ 
m_2 \cdot (l_1 \cdot l_2 \cdot \cos(q_2) + l_2^2) & m_2 \cdot l_2^2 
\end{bmatrix} 
$$


$$ 
\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} = 
\begin{bmatrix} 
-m_2 \cdot l_1 \cdot l_2 \cdot \sin(q_2) \cdot (2 \cdot \dot{q}_1 \cdot \dot{q}_2 + \dot{q}_2^2) \\ 
m_2 \cdot l_1 \cdot l_2 \cdot \dot{q}_1^2 \cdot \sin(q_2) 
\end{bmatrix} 
$$


$$ 
\mathbf{F}_b = 
\begin{bmatrix} 
b_1 & 0 \\ 
0 & b_2 
\end{bmatrix} 
$$


$$ 
\mathbf{g}(\mathbf{q}) = 
\begin{bmatrix} 
(m_1 + m_2) \cdot l_1 \cdot g \cdot \cos(q_1) + m_2 \cdot g \cdot l_2 \cdot \cos(q_1 + q_2) \\ 
m_2 \cdot g \cdot l_2 \cdot \cos(q_1 + q_2) 
\end{bmatrix} 
$$


También necesitaremos calcular el jacobiano para incluir las llaves externas aplicadas en el EE en nuestro modelo:



$$ 
\mathbf{J}(\mathbf{q}) = 
\begin{bmatrix} 
-l_1 \cdot \sin(q_1) - l_2 \cdot \sin(q_1 + q_2) & -l_2 \cdot \sin(q_1 + q_2) \\ 
l_1 \cdot \cos(q_1) + l_2 \cdot \cos(q_1 + q_2) & l_2 \cdot \cos(q_1 + q_2) 
\end{bmatrix} 
$$




Entonces, podemos calcular τ_ext como:

$$ \boldsymbol{\tau}_{ext} = \mathbf{J}(\mathbf{q})^T \cdot \mathbf{F}_{ext} $$






### Integrar posición y velocidad

Como estamos implementando un sistema discreto:

$$ \ddot{\mathbf{q}}_{k+1} = \mathbf{M}^{-1}(\mathbf{q}_k)[ \boldsymbol{\tau}_k + \boldsymbol{\tau}_{ext_k} - \mathbf{C}(\mathbf{q}_k, \dot{\mathbf{q}}_k)\dot{\mathbf{q}}_k - \mathbf{F}_b\dot{\mathbf{q}}_k - \mathbf{g}(\mathbf{q}_k) ] $$



podemos obtener las velocidades y posición conjuntas mediante integración discreta a lo largo del tiempo como:


$$\dot{\mathbf{q}} = \int \ddot{\mathbf{q}} \, dt \implies \dot{\mathbf{q}}_{k+1} = \dot{\mathbf{q}}_k + \ddot{\mathbf{q}}_{k+1} \cdot \Delta t$$

$$ \mathbf{q} = \int \dot{\mathbf{q}} Dt \implies \mathbf{q}_{k+1} = \mathbf{q}_k + \dot{\mathbf{q}}_{k+1} \cdot \Delta t $$




## Lanzar el nodo del simulador dinámico

<div align="center">
  <img src="esquema.png" alt="esquema" width="70%" style="max-width:1100px;" />
  <br>
  <em>Figura: Esquema del sistema</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="grafo_rqt.jpg" alt="grafo_rqt" width="70%" style="max-width:1100px;" />
  <br>
  <em>Figura: Representación del sistema con rqt_graph</em>
</div>

## Representación gráfica

### Experimento 1

```yaml
uma_arm_dynamics:
  ros__parameters:
    frequency: 1000.0
    m1: 3.0
    m2: 2.0
    l1: 1.0
    l2: 0.6
    b1: 5.0
    b2: 5.0
    g: 9.81
    q0: [0.785398, -0.785398]
```

<div align="center">
  <img src="https://github.com/user-attachments/assets/45f14217-ce60-4168-8521-f8e476bc4129" alt="exp1_pos" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Posiciones del Experimento 1</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="https://github.com/user-attachments/assets/95a56708-a60c-4119-9ae8-0032caeb64f7" alt="exp1_vel" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Velocidades del Experimento 1</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="https://github.com/user-attachments/assets/4fee3646-6da8-4cde-bd19-4bf2ac8b5492" alt="exp1_acc" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Aceleraciones del Experimento 1</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="exp1.gif" alt="exp1_sim" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Simulación Experimento 1</em>
</div>

### Experimento 2

```yaml
uma_arm_dynamics:
  ros__parameters:
    frequency: 1000.0
    m1: 1.5
    m2: 1.0
    l1: 1.0
    l2: 0.6
    b1: 2.5
    b2: 2.5
    g: 9.81
    q0: [0.785398, -0.785398]
```

<div align="center">
  <img src="https://github.com/user-attachments/assets/6f05f320-1855-4bec-aba9-74e6cff444ae" alt="exp2_pos" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Posiciones del Experimento 2</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="velocidades2.png" alt="vel2" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Velocidades del Experimento 2</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="aceleraciones2.png" alt="acc2" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Aceleraciones del Experimento 2</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="exp2.gif" alt="exp2_sim" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Simulación Experimento 2</em>
</div>

### Experimento 3

```yaml
uma_arm_dynamics:
  ros__parameters:
    frequency: 1000.0
    m1: 7.0
    m2: 1.0
    l1: 1.0
    l2: 0.6
    b1: 4.0
    b2: 2.0
    g: 9.81
    q0: [0.785398, -0.785398]
```

<div align="center">
  <img src="posiciones4.png" alt="posiciones4" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Posiciones del Experimento 3</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="velocidades4.png" alt="velocidades4" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Velocidades del Experimento 3</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="aceleraciones4.png" alt="aceleraciones4" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Aceleraciones del Experimento 3</em>
</div>

<div align="center" style="margin-top:1rem;">
  <img src="exp3.gif" alt="exp2_sim" width="70%" style="max-width:1000px;" />
  <br>
  <em>Figura: Simulación Experimento 3</em>
</div>


## ¿Cuáles son los efectos de modificar los parámetros dinámicos?

Los parámetros de dinámica ($m_1$, $m_2$, $b_1$, $b_2$) tienen efectos significativos en el comportamiento del manipulador. **$m_1$ y $m_2$** controlan la inercia de cada eslabón, afectando la matriz de inercia $\mathbf{M}(\mathbf{q})$, los efectos de Coriolis en $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$, y el vector de gravedad; un aumento en $m_2$ amplifica el acoplamiento dinámico entre articulaciones y los efectos no lineales, mientras que $m_1$ principalmente afecta la inercia y el torque de gravedad de la primera articulación. **$b_1$ y $b_2$** representan el amortiguamiento viscoso en cada articulación a través de la matriz $\mathbf{F}_b$; valores mayores aumentan la disipación de energía produciendo movimientos más suaves y menos oscilatorios, pero reduciendo la velocidad de respuesta del sistema. Modificar estos parámetros permite ajustar la dinámica del brazo para aplicaciones específicas: masas mayores requieren más torque para acelerar, mientras que amortiguamientos mayores mejoran la estabilidad a costa de una respuesta más lenta.

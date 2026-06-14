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
<figure align="center" style="margin:2rem 0 1rem 0;">
  <img src="esquema.png" alt="esquema" width="75%" style="max-width:1100px;" />
  <figcaption><strong>Figura:</strong> Esquema del sistema</figcaption>
</figure>

Esquema como está en el enunciado:
<figure align="center" style="margin:1rem 0 2rem 0;">
  <img src="grafo_rqt.jpg" alt="grafo_rqt" width="75%" style="max-width:1100px;" />
  <figcaption><strong>Figura:</strong> Representación del sistema con rqt_graph</figcaption>
</figure>

## Representación gráfica

### Experimento 1
uma_arm_dynamics (parámetros):
```yaml
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

#### Posición
<figure align="center" style="margin:0.8rem 0;">
  <img src="https://github.com/user-attachments/assets/45f14217-ce60-4168-8521-f8e476bc4129" alt="exp1_pos" width="48%" style="max-width:800px; margin-right:2%;" />
  <img src="https://github.com/user-attachments/assets/95a56708-a60c-4119-9ae8-0032caeb64f7" alt="exp1_vel" width="48%" style="max-width:800px;" />
  <figcaption><strong>Figura:</strong> Posiciones del Experimento 1 (comparativa)</figcaption>
</figure>

#### Aceleración
<figure align="center" style="margin:0.8rem 0 1.2rem 0;">
  <img src="https://github.com/user-attachments/assets/4fee3646-6da8-4cde-bd19-4bf2ac8b5492" alt="exp1_acc" width="70%" style="max-width:1000px;" />
  <figcaption><strong>Figura:</strong> Aceleraciones del Experimento 1</figcaption>
</figure>

#### Simulación
<figure align="center" style="margin:1rem 0 1.5rem 0;">
  <img src="exp1.gif" alt="exp1_sim" width="75%" style="max-width:1000px;" />
  <figcaption><strong>Figura:</strong> GIF de la simulación (Experimento 1)</figcaption>
</figure>

### Experimento 2

uma_arm_dynamics (parámetros):
```yaml
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

#### Posición
<figure align="center" style="margin:0.8rem 0;">
<img width="1402" height="1045" alt="exp2_pos" src="https://github.com/user-attachments/assets/3487d03a-2f12-47cd-9d53-680252471445" />
  <figcaption><strong>Figura:</strong> Posiciones del Experimento 2</figcaption>
</figure>

#### Velocidad y Aceleración
<figure align="center" style="margin:0.8rem 0;">
  <img src="velocidades2.png" alt="vel2" width="48%" style="max-width:800px; margin-right:2%;" />
  <img src="aceleraciones2.png" alt="acc2" width="48%" style="max-width:800px;" />
  <figcaption><strong>Figura:</strong> Velocidades y aceleraciones del Experimento 2</figcaption>
</figure>

#### Simulación
<figure align="center" style="margin:1rem 0 1.5rem 0;">
  <img src="exp2.gif" alt="exp2_sim" width="75%" style="max-width:1000px;" />
  <figcaption><strong>Figura:</strong> GIF de la simulación (Experimento 2)</figcaption>
</figure>

### Experimento 3

uma_arm_dynamics (parámetros):
```yaml
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

#### Posición y Velocidad
<figure align="center" style="margin:0.8rem 0;">
  <img src="posiciones4.png" alt="pos3" width="48%" style="max-width:800px; margin-right:2%;" />
  <img src="velocidades4.png" alt="vel3" width="48%" style="max-width:800px;" />
  <figcaption><strong>Figura:</strong> Posición y velocidad (Experimento 3)</figcaption>
</figure>

#### Aceleración
<figure align="center" style="margin:1rem 0 1.5rem 0;">
  <img src="aceleraciones4.png" alt="acc3" width="70%" style="max-width:1000px;" />
  <figcaption><strong>Figura:</strong> Aceleraciones del Experimento 3</figcaption>
</figure>


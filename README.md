# Proyecto 2 — Planificación de Caminos Geométricos


**ISIS4826 Robótica Móvil — Universidad de los Andes, 2026-1**

## Autores

1. Johan Camilo Murcia
2. Esteban Alejandro Hernandez

---

## Descripción

Sistema de navegación autónoma para un robot **Hiwonder MentorPi** en ROS2 con Gazebo. El nodo planifica un camino desde `q0` hasta `qf` usando A* sobre un C-space en cuadrícula, ejecuta el camino alternando rotaciones y traslaciones, y al llegar usa el LiDAR para estimar la posición real del robot.

---

## Instalación

```bash
cd ~/proyecto2
colcon build --packages-select proyecto
source install/setup.bash
```

---

## Uso

### 1. Seleccionar la escena

Al inicio de `navigation_node.py`, cambiar el número de escena (1 al 6):

```python
NUMERO_ESCENA = 1
```

Recompilar después de cambiar:

```bash
colcon build --packages-select proyecto && source install/setup.bash
```

### 2. Lanzar la simulación

```bash
# Terminal 1 — Gazebo con la escena
gz sim escena<X>.sdf

# Terminal 2 — Bridge ROS2 ↔ Gazebo
ros2 run ros_gz_bridge parameter_bridge--ros-args-p config_file:=config_bridge.yaml
```

### 3. Correr el nodo

```bash
# Terminal 3
ros2 run proyecto navigation
```

### 4. Resultados

Al terminar, el nodo imprime en consola `qf`, `qf_est` y `q_act`, y guarda el camino en:

```
~/Camino-EscenaX.txt
```

> [!NOTE]
> Para usar el bridge es necesario tener el config_bridge.yaml que se encuentra en la maquina virtual.

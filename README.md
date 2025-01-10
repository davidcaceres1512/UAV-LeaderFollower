# **Esquema Líder-Seguidor para UAVs utilizando ROS 2 e Ignition Gazebo**

Este proyecto es presentado para el curso **Sistemas de Percepción en Robótica** de la **Universidad Miguel Hernández (UMH)**, como parte del **Máster en Robótica**.

Simula un sistema de control basado en el esquema líder-seguidor para drones utilizando **ROS 2**, **Ignition Gazebo** y **Rviz**. El sistema incluye nodos que permiten generar trayectorias, coordinar el movimiento entre los drones y visualizar la simulación en **Rviz**.

---

## **Índice**

1. [Descripción del Proyecto](#descripción-del-proyecto)
2. [Estructura del Directorio](#estructura-del-directorio)
3. [Nodos, Publicadores y Subscriptores](#nodos-publicadores-y-subscriptores)
4. [Guía de Instalación](#guía-de-instalación)
5. [Ejemplo de Ejecución](#ejemplo-de-ejecución)
6. [Referencias](#referencias)

---

## **Descripción del Proyecto**

El esquema líder-seguidor consiste en controlar dos UAVs:
- **Líder**: Sigue una trayectoria predefinida (hélice o lemniscata).
- **Seguidor**: Ajusta su posición y orientación para mantener una distancia relativa con el líder.

El proyecto incluye nodos en **C++ y Python** que controlan individualmente cada UAV, gestionan su posición, y publican las trayectorias y transformaciones necesarias para su visualización en **Rviz**.

> **Nota**: Los nodos en Python se ubican en la carpeta `scripts/`.

---

## **Estructura del Directorio**

```bash
├── launch/
│   ├── ign_world_launch.py            # Lanzador para un dron
│   ├── two_ign_world_launch.py        # Lanzador para esquema líder-seguidor
├── models/
│   ├── r1/                            # Modelos del dron líder
│   ├── r2/                            # Modelos del dron seguidor
├── my_uavs/
│   ├── __init__.py                    # Archivo inicializador del paquete
├── rviz/
│   ├── one_drone.rviz                 # Visualización para un dron
│   └── two_drones.rviz                # Visualización para dos drones
├── scripts/
│   ├── leader_controller.py           # Controlador en Python para el líder
│   ├── follower_controller.py         # Controlador en Python para el seguidor
├── src/
│   ├── quadrotor_controller.cpp       # Controlador del dron líder
│   ├── two_quadrotors_lf.cpp          # Controlador líder-seguidor
│   ├── two_quadrotors_paths.cpp       # Controlador con trayectorias visualizadas
├── urdf/
│   ├── r1.xacro                       # Definición URDF del líder
│   └── r2.xacro                       # Definición URDF del seguidor
├── worlds/
│   ├── my_custom_world.sdf            # Mundo personalizado
│   └── two_quadrotors_world.sdf       # Mundo líder-seguidor
├── package.xml                        # Configuración del paquete
└── CMakeLists.txt                     # Configuración de construcción
```

---

## **Nodos, Publicadores y Subscriptores**

### **Nodos**
1. **`two_quadrotors_lf.cpp`**
   - Controlador líder-seguidor.
   - Publica comandos de velocidad a ambos UAVs.
   - Subscripción a la odometría de los drones.

2. **`quadrotor_controller.cpp`**
   - Controlador para un UAV siguiendo una trayectoria predefinida.

3. **`two_quadrotors_paths.cpp`**
   - Agrega visualización de trayectorias en Rviz.

4. **Python Scripts (`scripts/`)**
   - **`leader_controller.py`**: Controlador del líder en Python.
   - **`follower_controller.py`**: Controlador del seguidor en Python.

### **Publicadores**
- **`/r1/cmd_vel` y `/r2/cmd_vel`**: Comandos de velocidad para líder y seguidor.
- **`/r1/path` y `/r2/path`**: Trayectorias en tiempo real.

### **Subscriptores**
- **`/r1/odom` y `/r2/odom`**: Odometría para posiciones y orientaciones.

---

## **Guía de Instalación**

### **Requisitos**
1. **ROS 2 Humble o Foxy**
   - Instalación desde: [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html).
2. **Ignition Gazebo**
   - Instalación desde: [Ignition Gazebo Documentation](https://ignitionrobotics.org/docs).

### **Pasos**
1. **Actualizar el sistema y dependencias necesarias**
   ```bash
   sudo apt-get update
   sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-xacro \
     ros-$ROS_DISTRO-tf2-* ros-$ROS_DISTRO-rviz-default-plugins ros-$ROS_DISTRO-ros-gz
   ```

2. **Clonar el repositorio**
   ```bash
   cd ~/ros2_ws/src/
   git clone <URL_REPOSITORIO>
   ```

3. **Construir el paquete**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_uavs --symlink-install
   source ~/ros2_ws/install/setup.bash
   ```

4. **Configurar variables de entorno**
   En el archivo `~/.bashrc`, agregar:
   ```bash
   export IGN_GAZEBO_RESOURCE_PATH=~/ros2_ws/src/my_uavs/models
   ```

5. **Verificar instalación**
   - Cargar un mundo vacío:
     ```bash
     ign gazebo empty.sdf
     ```
   - Verificar el listado de temas en Ignition:
     ```bash
     ign topic -l
     ```

   - Probar la conexión con ROS 2 mediante el puente de mensajes:
     ```bash
     ros2 run ros_gz_bridge parameter_bridge \
       /r1/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist \
       /model/r1/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry
     ```

---

## **Ejemplo de Ejecución**

1. **Ejecutar el lanzador líder-seguidor**
   ```bash
   ros2 launch my_uavs two_ign_world_launch.py
   ```

2. **Ejecutar el nodo líder-seguidor**
   ```bash
   ros2 run my_uavs two_quadrotors_lf r1 r2
   ```

3. **Enviar comandos manuales al dron (opcional)**
   ```bash
   ros2 topic pub /r1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
   ```

---

## **Referencias**

1. [Guía de Instalación de ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
2. [ROS y Gazebo: Comparación de Tipos de Mensajes](https://medium.com/@geetkal67/how-to-subscribe-to-ignition-gazebo-topics-using-ros2-8bcff7a0242e)
3. [Tutorial de ROS-Gazebo Bridge](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
4. [Documentación de Ignition Gazebo](https://gazebosim.org/docs)

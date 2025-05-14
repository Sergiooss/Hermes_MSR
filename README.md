# Hermes\_MSR

![Video demostración del robot Hermes cogiendo un cubo](https://drive.google.com/file/d/164FAdJ3IyVZkYIRyUjzqT3uezJm92M24/view?usp=sharing)

## Descripción general

Esta práctica consiste en la integración y simulación de un robot tipo SCARA móvil, llamado **Hermes**, en un entorno simulado con ROS 2 y Gazebo, incluyendo sensores, teleoperación, MoveIt y controladores. Parte de un modelo previo del robot creado con Blender.

El objetivo ha sido simular y controlar un escenario de *pick and place* en un mundo virtual, integrando sensores de cámara e IMU, realizando una buena organización del URDF mediante Xacro y lanzando todo el ecosistema ROS 2 necesario desde launch files.

---

## 1. Estructura del robot

El robot Hermes fue inicialmente diseñado en Blender, y desde ahí se generó un archivo URDF completo. Para mejorar su mantenibilidad y modularidad, se reorganizó todo el URDF dividiéndolo en distintos archivos `xacro` según las partes del robot:

* Base
* Brazo
* Gripper
* Cámaras
* IMU

Esto permitió una jerarquía de links y joints clara, y facilitó la posterior integración con MoveIt y Gazebo.

---

## 2. Sensores añadidos

Se añadieron los siguientes sensores:

* **Cámara frontal** en la base del robot, orientada hacia adelante.
* **Cámara de gripper**, colocada justo encima del efector final para ver objetos a manipular.
* **Sensor IMU**, en el centro del robot, para registrar aceleraciones durante el movimiento.

Estos dispositivos se integraron tanto en el modelo URDF como en los bridges ROS-Gazebo.

---

## 3. Integración con MoveIt

Se generó la configuración de MoveIt 2 utilizando el `setup assistant`, creando:

* El grupo de planificación **Scara** (brazo principal)
* El grupo **Gripper**
* Posiciones iniciales y planificadas del brazo

Posteriormente, se modificaron los archivos para permitir su uso en Gazebo y en simulación, incluyendo soporte para `ros2_controllers`.

---

## 4. Archivos de lanzamiento

### 🚀 Launcher principal: `robot_gazebo.launch.launch.py`

Este launch lanza el entorno completo de simulación:

* Gazebo con el mundo `urjc_excavation_world`
* 🚗 Spawnea el robot Hermes desde el `robot_description`
* 📊 RViz con el modelo ya cargado
* 🎮 Bridges de ROS-Gazebo, incluyendo:

  * Topics de imagen para ambas cámaras (`ros_gz_image`)
  * Bridges definidos en `rover_bridge.yaml`
  * Nodo `twist_stamper` para transformar comandos de velocidad

### 💡 Launcher de MoveIt: `move_group.launch.py`

Este launcher inicia los nodos necesarios de planificación de MoveIt para Hermes:

* Carga la configuración `hermes_moveit_config`
* Inicia el `move_group`

Esto permite controlar el brazo y el gripper desde RViz o desde scripts de planificación.

### 🛠️ Launcher de controladores: `robot_controllers.launch.py`

Carga todos los controladores ROS 2 necesarios para el robot:

* `joint_state_broadcaster`
* `hermes_base_control` → control diferencial de la base móvil
* `scara_controller` → control del brazo principal
* `gripper_controller` → control del efector final

Todos estos controladores se configuran desde los archivos `.yaml` situados en los paquetes `hermes_description` y `hermes_moveit_config`.

---

## 5. Video de demostración

Puedes ver el comportamiento final del robot Hermes realizando una tarea de recogida de un cubo en el siguiente video:

**[Video de Hermes recogiendo un cubo](https://drive.google.com/file/d/164FAdJ3IyVZkYIRyUjzqT3uezJm92M24/view?usp=sharing)**

---

## 6. Ejecución

```bash
# Lanzar simulación completa
ros2 launch hermes_description simulation.launch.py

# Lanzar controladores
ros2 launch hermes_description controllers.launch.py

# Lanzar moveit
ros2 launch hermes_moveit_config moveit.launch.py
```

---

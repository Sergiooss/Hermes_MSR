# Hermes\_MSR

![Video demostración del robot Hermes cogiendo un cubo](https://drive.google.com/file/d/164FAdJ3IyVZkYIRyUjzqT3uezJm92M24/view?usp=sharing)

## Descripción general

Esta práctica consiste en la integración y simulación de un robot tipo SCARA móvil, llamado **Hermes**, en un entorno simulado con ROS 2 y Gazebo, incluyendo sensores, teleoperación, MoveIt y controladores. Parte de un modelo previo del robot creado con Blender.

El objetivo ha sido simular y controlar un escenario de *pick and place* en un mundo virtual, integrando sensores de cámara e IMU, realizando una buena organización del URDF mediante Xacro y lanzando todo el ecosistema ROS 2 necesario desde launch files.

---

## 1. Estructura del robot

El robot **Hermes** fue inicialmente diseñado en Blender, y desde ahí se generó un archivo URDF completo. Para mejorar su mantenibilidad, modularidad y reutilización, el modelo se reestructuró utilizando archivos `xacro`, organizados jerárquicamente por componentes del robot. Esta modularización facilitó tanto la comprensión del modelo como su integración con **MoveIt** y **Gazebo**.

La estructura del directorio `urdf/` está dividida en las siguientes carpetas y archivos:

- **arm/**
  - `arm.urdf.xacro`: define el brazo principal del robot.
  - `gripper.urdf.xacro`: define la pinza final del manipulador.
  
- **base/**
  - `robot_base.urdf.xacro`: define la base central del robot.
  - `box_storage.urdf.xacro`: estructura superior para carga.
  
- **cushioning/**
  - `cushioning.urdf.xacro`: define el sistema de amortiguación.
  
- **radio/**
  - `radio.urdf.xacro`: modelo del módulo de radio principal.
  - `axel2axel.urdf.xacro`: define la estructura que une radios o ejes.

- **sensors/**
  - `camera.urdf.xacro`: incluye las cámaras montadas en el robot.
  - `imu.urdf.xacro`: sensor IMU ubicado en el centro del robot.

- **wheel/**
  - `wheel.urdf.xacro`: define las ruedas principales del robot.
  - `radio_wheel.urdf.xacro`: incluye los radios de las ruedas

Esta organización facilita el mantenimiento y extensión del modelo, permitiendo modificar partes específicas (como sensores o ruedas) sin alterar el resto del URDF.


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

## 6. Teleop_twist_keyboard
Gracias a este paquete movemos al robot y lo hacemos avanzar hasta los cubos.

---

## 6. Video de demostración

Puedes ver el comportamiento final del robot Hermes realizando una tarea de recogida de un cubo en el siguiente video:

**[Video de Hermes recogiendo un cubo](https://drive.google.com/file/d/164FAdJ3IyVZkYIRyUjzqT3uezJm92M24/view?usp=sharing)**

---

## 7. Ejecución

```bash
# Lanzar simulación completa
ros2 launch hermes_description simulation.launch.py

# Lanzar controladores
ros2 launch hermes_description controllers.launch.py

# Lanzar moveit
ros2 launch hermes_moveit_config moveit.launch.py
```

---

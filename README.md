# ros2-test-repo

Este repositorio presenta los primeros pasos para trabajar con ROS2 Jazzy. La explicación del paso a paso puedes encontrarla en la siguiente los siguientes videos de youtube.

## Preparativos
La instalación de ROS2 Jazzy requiere de Ubuntu 24.04 LTS, por lo cual comenzaremos con esta instalación.

Descargar Ubuntu 24.04 LTS Desktop desde este enlace: https://ubuntu.com/download/desktop

Para quienes comienzan a familiarizarse con Linux y ROS2, recomendamos utilizar una máquina virtual. En nuestro caso, utilizaremos VMWare (link de descarga: https://www.techspot.com/downloads/189-vmware-workstation-for-windows.html). 

El siguiente video muestra la instalación recomendada de Ubuntu 24.04: 

1. Crear la máquina. Asignar recursos.  
2. Instalar Ubuntu. Configuración recomendada user@ubuntu  

## 1. Instalar ROS2 Jazzy
El detalle de la instalación puede encontrarse en el tutorial oficial: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html  
1. Añadir el componente 'universe' a todos los repositorios 
`sudo add-apt-repository universe`  

2. Agregar la llave GPG de ROS2  
`sudo apt update && sudo apt install curl -y`  
`sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`

3. Agregar el repositorio al listado de fuentes de software  
`echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`

4. Instalar herramientas de desarrollo  
`sudo apt update && sudo apt install ros-dev-tools`

5. Instalar ROS  
`sudo apt upgrade`  
`sudo apt install ros-jazzy-desktop`  

6. Agregar ros-jazzy como fuente de software en `~/.bashrc`  
`source /opt/ros/jazzy/setup.bash`  

## 2. Otras instalaciones requeridas
Los siguientes aplicaciones y componentes serán requeridos para que todo funcione.  
    `sudo apt install mesa-utils`  
    `sudo apt install python3-pip`  
    `sudo apt install jstest-gtk
    `sudo apt install ros-joy*`  
    `sudo apt install ros-jazzy-joint-state-publisher`  
    `sudo apt install ros-jazzy-joint-state-publisher-gui`  
    `sudo apt install ros-jazzy-ros2-control `  
    `sudo apt install ros-jazzy-ros2-controllers`  
    `sudo apt install ros-jazzy-ros-gz`  
    `sudo apt install ros-jazzy-gz-ros2-control `  
    `sudo apt install ros-jazzy-navigation2`  
    `sudo apt install ros-jazzy-nav2-bringup`  

## 3. Configurar video para RVIZ y GazeboSim
Si las opciones de video por defecto en Ubuntu 24.04 no permiten la ejecución de RVIZ y/o GazeboSim, realizar las siguientes configuraciones:  
### Cambiar el controlador de video *wayland* por *X11* 
(https://robotics.stackexchange.com/questions/111436/rviz2-is-not-working-in-ros2-jazzy)  
1. Ejecutar `sudo apt-get install xorg openbox` si la salida es *wayland*, entonces instalar X11 con
     `sudo apt-get install xorg openbox`
2. Editar el archivo *custom.conf*, habilitando la línea "WaylandEnable=false" al quitar el comentario '#'.
     `sudo nano /etc/gdm3/custom.conf`  
   Guardar con `<control + S>` y salir con `<contrl + O>`.
3. Reiniciar

### Instalar la última versión de MESA driver
(https://itsfoss.com/install-mesa-ubuntu/)
1. Agregar el repositorio de MESA driver a las fuentes de software
   `sudo add-apt-repository ppa:kisak/kisak-mesa`
2. Actualizar la lista de repositorios y realizar una actualización de software
   `sudo apt get update`  
   `sudo apt get upgrade`  
3. Ejecutar `glxinfo | grep OpenGL` y verificar que no hay errores.

### Deshabilitar LibGL_DRI3
(https://gazebosim.org/docs/garden/troubleshooting#ubuntu)  
Agregar la siguiente instrucción al final del archivo `~/.bashrc` 
    `export LIBGL_DRI3_DISABLE=1`

## 4. Instalar Visual Studio Code
1. Descargar archivo .deb desde https://code.visualstudio.com/download  
2. Abrir terminal en carpeta descargas y ejecutar  
    `sudo apt install ./code<version>.deb`  
3. En code, instalar extensiones: *Python* (by Microsoft), *ROS* (by Microsoft)


## 5. Otras configuraciones útiles
Al compilar los repositorios de ROS2 mediante *colcon*, aparece un molesto aviso de librerías que van a quedar obsoletas. Este es sólo un mensaje de alerta (warning). Para no visualizar este mensaje, incluir las siguientes líneas al final del archivo `~/.bashrc`  
    `PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop`
    `export PYTHONWARNINGS`





RVIZ Issue 
sudo apt-get install xorg openbox
Run:
echo $XDG_SESSION_TYPE
If the output is wayland, then install X11 using:
sudo apt-get install xorg openbox
Then change the xdg session type and reboot:
sudo nano /etc/gdm3/custom.conf

Gazebo Issue
https://gazebosim.org/docs/garden/troubleshooting#ubuntu
export LIBGL_DRI3_DISABLE=1

Errores con MESA driver
glxinfo | grep OpenGL -->
MESA: error: ZINK: failed to choose pdev
glx: failed to create drisw screen
Solucion en: https://itsfoss.com/install-mesa-ubuntu/
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt update
sudo apt upgrade

de nuevo glxinfo | grep OpenGL --> sin errores








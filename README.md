# temp-repo

Primeros pasos con ROS2

# 0. Preparativos
En este tutorial trabajaremos con ROS2 Jazzy. La instalación requiere de Ubuntu 24.04 LTS, por lo cual comenzaremos con esta instalación.

Descargar Ubuntu 24.04 LTS Desktop desde este enlace: https://ubuntu.com/download/desktop

Para comenzar a familiarizarse con Linux y ROS2, recomendamos utilizar una máquina virtual. En nuestro caso, utilizaremos VirtualBox. En el computador que alojará la máquina virtual, la configuración de BIOS deberá permitir la virtualización y se deberá instalar VirtualBox junto con su correspondiente complemento VirtualBox Guest Additions.

El siguiente video muestra la instalación recomendada de Ubuntu utilizando VirtualBox: 

1. Crear la máquina. Asignar recursos.
2. Instalar Ubuntu. Configuración recomendada user@ubuntu
3. Instalar VirtualBox Guest Additions: ./autorun.sh (requiere bzip2, tar, build-essential)

# 1. Instalar ROS2 Jazzy (adaptado desde https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
Añadir el componente 'universe' a todos los repositorios
sudo add-apt-repository universe

Agregar la llave GPG de ROS2
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
Agregar el repositorio al listado de fuentes de software
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Instalar herramientas de desarrollo
sudo apt update && sudo apt install ros-dev-tools

Instalar ROS
sudo apt upgrade
sudo apt install ros-jazzy-desktop

Agregar al .bashrc
source /opt/ros/jazzy/setup.bash

Verificar con turtlesim

# 2. Otras instalaciones requeridas
Otras instalaciones requeridas:
sudo apt install mesa-utils
sudo apt install python3-pip
sudo apt install vscode
sudo apt install jstest-gtk
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros2-control 
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gz-ros2-control 
sudo apt install ros-jazzy-gz-ros2-navigation2
sudo apt install ros-jazzy-gz-ros2-nav2-bringup

# 3. Visual Studio Code
Descargar archivo .deb desde https://code.visualstudio.com/download
Abrir terminal en carpeta descargas y ejecutar
sudo apt install ./code<version>.deb
En code, instalar extensiones: Python (by Microsoft), ROS (by Microsoft)


# 4. Otras configuraciones útiles:
PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop
export PYTHONWARNINGS

RVIZ Issue https://robotics.stackexchange.com/questions/111436/rviz2-is-not-working-in-ros2-jazzy
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








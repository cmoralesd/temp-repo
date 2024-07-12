# ros2-test-repo

Este repositorio presenta los primeros pasos para trabajar con ROS2 Jazzy. La explicación del paso a paso puedes encontrarla en la siguiente los siguientes videos de youtube:   
1. Configurando el setup para que todo funcione
2. Primeros pasos en ROS2

## Preparativos
La instalación de ROS2 Jazzy requiere de Ubuntu 24.04 LTS, por lo cual comenzaremos con esta instalación.   
Existen 3 opciones para esto, pudiéndose utilizar cualquiera de ellas:

### 1. Instalar Ubuntu 24.04 LTS Desktop en forma nativa en el computador (recomendado para máximo rendimiento)
La instalación de Ubuntu puede compartir el disco duro con Windows y seleccionar el sistema operativo al momento del inicio.   
Es la configuración que aprovecha de mejor manera los recursos del computador, especialmente la tarjeta gráfica, y es la recomendada para desarrollo.
Este enlace describe en detalle cómo realizar el procedimiento: https://www.softzone.es/windows/como-se-hace/ubuntu-windows-dual-boot/

### 2. Instalar Ubuntu 24.04 en una máquina virtual (recomendado para el periodo de aprendizaje)
Una máquina virtual es un entorno seguro para familiarizarse con Linux y ROS2 sin tener que hacer cambios en el computador.   
Dependiendo de los recursos (RAM y número de procesadores destinados a la máquina virtual), el rendimiento puede ser suficientemente bueno, incluso para simulaciones complejas con GazeboSim.
El procedimiento en detalle sobre la creación de una máquina virtual en VMWare Workstation y la instalación de Ubuntu 24.04 puede encontrarse en este video:
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX   
Los archivos de una máquina virtual ya creada y lista para usarse, incluyendo ROS2 instalado, pueden encontrarse aquí:
XXXXXXXXXXXXXXXXXXXXX   

Para quienes comienzan a familiarizarse con Linux y ROS2, recomendamos utilizar una máquina virtual. En nuestro caso, utilizaremos VMWare (link de descarga: https://www.techspot.com/downloads/189-vmware-workstation-for-windows.html). 

### 3. En Windows, utilizando WSL (Windows Subsystem for Lunux)
Este método facilita la utilización de software de Linux en un entorno Windows. Su desempeño es bastante bueno, excepto para la utilización de simulaciones en GazeboSim. Es una configuración adecuada para las tareas de control y monitoreo utilizando ROS2, pero manteniendo la simulación en un software externo, como puede ser Webots.


## 1. Instalar ROS2 Jazzy
El detalle de la instalación puede encontrarse en el tutorial oficial: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html   
*El procedimiento siguiente es idéntico para instalar ROS en una instalación nativa de Ubuntu 24.04, en una máquina virtual o por medio de WSL*.   
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
    `sudo apt install jstest-gtk`   
    `sudo apt install ros-jazzy-joy*`  
    `sudo apt install ros-jazzy-joint-state-publisher`  
    `sudo apt install ros-jazzy-joint-state-publisher-gui`  
    `sudo apt install ros-jazzy-ros2-control`  
    `sudo apt install ros-jazzy-ros2-controllers`  
    `sudo apt install ros-jazzy-ros-gz`  
    `sudo apt install ros-jazzy-gz-ros2-control `  
    `sudo apt install ros-jazzy-navigation2`  
    `sudo apt install ros-jazzy-nav2-bringup`  

## 3. Configurar video para RVIZ y GazeboSim
Si las opciones de video por defecto en Ubuntu 24.04 no permiten la ejecución de RVIZ y/o GazeboSim, realizar las siguientes configuraciones:  
### Cambiar la interfaz gráfica *wayland* por *X11*  
(https://robotics.stackexchange.com/questions/111436/rviz2-is-not-working-in-ros2-jazzy)  
1. Verificar el sistema de gestión de ventanas gráficas, ejecutando:   
    `echo $XDG_SESSION_TYPE`   
   Si la salida es *wayland*, entonces instalar X11 con:   
     `sudo apt-get install xorg openbox`
3. Editar el archivo *custom.conf*, utilizando *nano*:   
     `sudo nano /etc/gdm3/custom.conf`   
   Habilitar la línea "WaylandEnable=false", quitando el caracter de comentario '#'.   
   Guardar con `<control + s>` y salir con `<control + o>`.
4. Reiniciar

### Instalar la última versión de MESA driver
(https://itsfoss.com/install-mesa-ubuntu/)
1. Agregar el repositorio de MESA driver a las fuentes de software:   
   `sudo add-apt-repository ppa:kisak/kisak-mesa`
2. Actualizar la lista de repositorios y realizar una actualización de software   
   `sudo apt get update`  
   `sudo apt get upgrade`  
3. Ejecutar `glxinfo | grep OpenGL` y verificar que no hay errores.

### Deshabilitar LibGL_DRI3
(https://gazebosim.org/docs/garden/troubleshooting#ubuntu)  
Agregar la siguiente instrucción al final del archivo *~/.bashrc*    
        `export LIBGL_DRI3_DISABLE=1`

## 4. Instalar Visual Studio Code
1. Descargar archivo .deb desde https://code.visualstudio.com/download  
2. Abrir terminal en carpeta descargas y ejecutar   
       `sudo apt install ./code<version>.deb`
4. En code, instalar extensiones: *Python* (by Microsoft), *ROS* (by Microsoft)

## 5. Otras configuraciones útiles
Al compilar los repositorios de ROS2 mediante `colcon build --symlink-install`, aparece un molesto aviso de librerías que van a quedar obsoletas (deprecation warning). Este es sólo un mensaje de alerta (warning) y no afecta la funcionalidad. Para no visualizar este mensaje, incluir las siguientes líneas al final del archivo `~/.bashrc`:  
    `PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop`   
    `export PYTHONWARNINGS`


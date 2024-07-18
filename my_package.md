# Tutorial - Primeros pasos con ROS2
Instrucciones básicas para crear un paquete de ROS2 con un nodo publicador y suscriptor, utilizando Ament Python.   
Para una explicación más detallada del proceso seguido en este tutorial, revisa este video:
## 1. Crear el espacio de trabajo
Abrir un terminal nuevo, desde la carpeta de usuario `~$` y crear una carpeta para contener el espacio de trabajo (workspace):   
      `mkdir my_ws`   
Entrar a la carpeta recién creada:   
      `cd my_ws` 
Dentro de la carpeta recién creada, crear una subcarpeta que alojará el código fuente (source code):   
      `mkdir src`   
Compilar el espacio de trabajo utilizando *colcon*:   
      `colcon build`  

## 2. Crear un paquete utilizando Ament Python
ROS2 ofrece instrucciones para crear paquetes rápidamente utilizando plantillas.   
Todos los paquetes se deben crear en el espacio de trabajo, dentro de la carpeta *src*. Abrimos un terminal en esa ubicación:      
      `cd ~/my_ws/src`
Crearemos un primer paquete con lenguaje Python, llamado *my_package*, conteniendo un nodo llamado *my_node*, utilizando las siguientes instrucciones:   
      `ros2 pkg create --build-type ament_python --node-name my_node my_package`   
Una vez creado el paquete, debe compilarse. Para ello, volver al espacio de trabajo:   
      `cd ..`  
Y compilar:
      `colcon build`  
Para ejecutar el código contenido en el paquete, es necesario agregarlo como fuente de software.   
Desde un terminal ubicado en la carpeta del espacio de trabajo, `~/my_ws$`, ejecutar:   
      `source install/setup.bash`
Ahora es posible ejecutar el código contenido en el nodo *my_node*:   
      `ros2 run my_package my_node`
El terminal mostrará un mensaje enviado por el nodo: "Hi from my_package."

## 3. Utilizando VSCode para editar el paquete
VSCode es una excelente herramienta para trabajar en la edición y deputación de código. Abriremos una ventana de VSCode directamente desde la carpeta *src* en el espacio de trabajo:
      `cd ~/my_ws/src`   
      `code .`   
Por medio de VSCode, revise la estructura de archivos que componen el paquete. Editaremos algunos de ellos.  
### Editando el archivo package.xml
Todo paquete de ROS contiene un archivo *package.xml*. Aquí se detalla información sobre el paquete y su desarrollador, para ser compartida con otros usuarios.   
Como una buena práctica, en este archivo editaremos información respecto de:
- versión
- descripción
- mantenedor
- licencia
### Editando el archivo setup.py
Todo paquete de ROS creado con Ament Python contiene un archivo *setup.py*. Este archivo configura las dependencias de librerías (dependencies), los archivos que van a ser compartidos con otros nodos (data files) y las instrucciones de entrada (entry points) que permitirán la ejecución de los nodos desde terminal.
Por ahora, en este archivo sólo actualizaremos la información del paquete para hacerla consistente con package.xml:   
### Editando el archivo my_node.py
El archivo my_node.py es el archivo ejecutable de este paquete. Por ahora su tarea es enviar un mensaje de saludo.   
Si editamos este archivo, los cambios no se verán reflejados en el ejecutable, sino hasta que el paquete sea nuevamente compilado. Utilizando la opción --symlink_install, es posible editar los archivos y hacer que los cambios sean inmediatemente ejecutables, sin necesidad de volver a compilar.   
      `cd ~/my_ws/`
      `colcon build --symlink_install`   
Y ejecutar nuevamente el nodo:
      `ros2 run my_package my_node`   
En caso que no pueda ser ejecutado:
1. Asegúrese que ha agregado el espacio de trabajo como fuente de software: `source install/setup.bash`
2. Elimine las carpetas *buils*, *install* y *log* desde el espacio de trabajo. Vuelva a compilar y vuelva a hacer *source*.
      


      

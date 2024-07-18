# Primeros pasos con ROS2 - Creando un paquete con Ament Python
Con las instrucciones de este tutorial se creará un primer paquete de ROS2, utilizando Ament Python.   
Para una explicación detallada del proceso seguido en este tutorial, revisa este video:  
## 1. Crea el espacio de trabajo
Abre un terminal nuevo, desde la carpeta de usuario `~$` y crea una carpeta para contener el espacio de trabajo (workspace):   
      `mkdir my_ws`   
Entra a la carpeta recién creada:   
      `cd my_ws`   
Crea allí una subcarpeta *src* que alojará el código fuente (source code):   
      `mkdir src`   
Compila el espacio de trabajo utilizando *colcon*:   
      `colcon build`  

## 2. Crea un paquete utilizando Ament Python
Un paquete de software es una estructura organizada de archivos de código. Cuando se desea instalar o compartir software en ROS2, éste debe estar organizado en un paquete.
ROS2 contiene instrucciones que permiten crear paquetes rápidamente utilizando plantillas.   
Todos los paquetes se deben crear en el espacio de trabajo, dentro de la carpeta *src*. Abre un terminal en esa ubicación:      
      `cd ~/my_ws/src`   
Crea tu primer paquete para alojar código en lenguaje Python. El nombre del paquete será *my_package* y contendrá un nodo llamado *my_node*. Esta es la instrucción para crear el paquete:   
      `ros2 pkg create --build-type ament_python --node-name my_node my_package`   
Una vez creado el paquete, debe compilarse. Para ello, vuelve al espacio de trabajo:   
      `cd ..`  
Y compila:
      `colcon build`  
Para ejecutar el código contenido en el paquete recién creado, primero es necesario agregar el espacio de trabajo como fuente de software.   
Desde un terminal, accede a la carpeta del espacio de trabajo, `~/my_ws$`, ejecuta:   
      `source install/setup.bash`   
Ahora es posible ejecutar el código contenido en el nodo *my_node*:   
      `ros2 run my_package my_node`   
El terminal mostrará un mensaje de saludo: *"Hi from my_package"*.   

## 3. Utiliza VSCode para editar el paquete
VSCode es una excelente herramienta para trabajar en la edición y deputación de código. Abriremos una ventana de VSCode directamente desde la carpeta *src* en el espacio de trabajo:   
      `cd ~/my_ws/src`   
      `code .`   
Por medio de VSCode, revisa la estructura de archivos que componen el paquete. Editaremos algunos de ellos.  
### Edita el archivo package.xml
Todo paquete de ROS contiene un archivo *package.xml*. Aquí se detalla información sobre el paquete y su desarrollador, para ser compartida con otros usuarios.   
Como una buena práctica, en este archivo editaremos la información de:
- versión
- descripción
- mantenedor
- licencia
### Edita el archivo setup.py
Todo paquete de ROS creado con Ament Python contiene un archivo *setup.py*. Este archivo configura las dependencias de librerías (dependencies), los archivos que van a ser compartidos con otros nodos (data files) y las instrucciones de entrada (entry points) que permitirán la ejecución de los nodos desde terminal.   
Por ahora, en este archivo sólo actualizaremos la información del paquete para hacerla consistente con package.xml:   
### Edita el archivo my_node.py
El archivo my_node.py es el archivo ejecutable de este paquete. Por ahora, su tarea es simplemente enviar un mensaje de saludo.   
Al editar este archivo, los cambios no se verán reflejados en el ejecutable, sino hasta que el paquete sea nuevamente compilado. Sin embargo, utilizando la opción `--symlink_install`, es posible editar los archivos y hacer que los cambios sean inmediatemente ejecutables al momento de guardar, sin necesidad de volver a compilar.   
      `cd ~/my_ws/`   
      `colcon build --symlink_install`   
Tras realizar los cambios, para iniciar nuevamente el nodo se debe ejecutar:   
      `ros2 run my_package my_node`   
En caso que el nodo no se inicie correctamente:
1. Asegúrate que has agregado el espacio de trabajo como fuente de software: `source install/setup.bash`
2. Elimina las carpetas *build*, *install* y *log* desde el espacio de trabajo. Vuelve a compilar y nuevamente `source install/setup.bash`

Sigue editando para dar a *my_node* la funcionalidad de un mensaje repetitivo, siguiendo la estructura típica de un nodo de Python. Esta estructura nos servirá como plantilla para los futuros nodos que vamos a crear.   
```
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        timer_period = 0.5 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        print('hello from ROS2!')

def main(args=None):
    try: 
        rclpy.init(args=args)
        my_node = MyNode()
        rclpy.spin(my_node)

    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
```

## 4. Crea un nodo publicador
El nodo de ejemplo *my_node* simplemente muestra un saludo por consola. Crearemos ahora un nodo que pueda transmitir un mensaje que podrá ser recibido por otros nodos de ROS2, sea que éste se aloje en el mismo equipo o en otro equipo conectado a la misma red.   
Tomando como ejemplo la estructura de nodo de *my_node*, agrega las líneas de código necesarias para incluir un publicador y un registo de evento en el log.   
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "message # %d" %self.i
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: '%s" % msg.data)
        self.i += 1

def main(args=None):
    try: 
        rclpy.init(args=args)
        pub = MyPublisher()
        rclpy.spin(pub)

    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
```

      

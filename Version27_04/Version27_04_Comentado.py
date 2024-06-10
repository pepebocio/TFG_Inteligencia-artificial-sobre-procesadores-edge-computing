
import time

"""librerias para imprimir en pantalla"""
import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtGui import QPainter, QColor, QPen, QPainterPath, QFontMetrics, QFont, QLinearGradient, QPainterPath
from PyQt5.QtCore import Qt, QRect, QTimer, QObject, pyqtSignal, QPoint
import threading
from queue import Queue

"""librerias servidores"""""
import socket
#import threading

"""libreria raton"""
import win32api
import win32con

"""Extraigo de win32con los eventos que voy a utilizar del ratón"""

clic_izquierdo = win32con.MOUSEEVENTF_LEFTDOWN
soltar_clic_izquierdo = win32con.MOUSEEVENTF_LEFTUP
clic_derecho = win32con.MOUSEEVENTF_RIGHTDOWN
soltar_clic_derecho = win32con.MOUSEEVENTF_RIGHTUP



class Communicate(QObject):
    close_signal = pyqtSignal()


"""
Creamos el canvas que usaremos para representar cosas en pantalla

la variable función será la encargada de seleccionar que se repesenta en pantalla en cada momento
"""
class Canvas(QWidget):
    def __init__(self):
        super().__init__()

        # Establecer las propiedades de la ventana
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setWindowState(Qt.WindowFullScreen)
        ##self.setGeometry(50, 50, 100, 100)  # Establecer el tamaño del canvas (x, y, ancho, largo)
        self.setAttribute(Qt.WA_TranslucentBackground) #hacer fondo transparente
        ##self.setWindowOpacity(0.1)  # Ajustar la opacidad de la ventana
        ##self.setStyleSheet("background-color: rgba(255, 255, 255, 128);")  #Dar color al fondo
       
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.actualizar_figuras)
        self.timer.start(10) 


        # Mostrar la ventana
        self.show()
        
        self.square_x = 50
        self.square_y = 50
        
        # Inicializar la lista de posiciones anteriores
        self.previous_positions = []
        
        
        # Comunicación entre hilos
        self.communication = Communicate()


    def paintEvent(self, event):
        painter = QPainter(self)
        
        if funcion==1:"""Modo para el calibrado"""
            # Dibujar un cuadrado en la posición actual
            painter.setBrush(QColor(255, 0, 0, 255))  # Rojo semi-transparente
            rect = QRect(self.square_x, self.square_y, dimensiones[0], dimensiones[1])
            #rect = QRect(posicion[0], posicion[1], dimensiones[0], dimensiones[1])
            painter.drawRect(rect)
            
        elif funcion==2:"""Modo para el puntero, durante el uso como ratón normal"""
            painter.setBrush(QColor.fromHsv(60,170,200))  # Rojo semi-transparente
            pen = QPen(QColor(255, 0, 0, 0))  # Crear un objeto QPen con el color deseado
            pen.setWidth(2)  # Establecer el grosor de la línea (por ejemplo, 2 píxeles)
            painter.setPen(pen)  # Establecer el objeto QPen en el QPainter
            # Dibujar líneas suaves entre puntos consecutivos
            painter.drawEllipse(posicion[0],posicion[1],70,70)
                        
        elif funcion==3:"""Modo para calibrar la extensión de la mano"""
            font = QFont('Helvetica', fuente)  # Tipo de fuente y tamaño
            font.setBold(True)
            fm = QFontMetrics(font)
            Twidth = fm.width(text)
            ##Theight = fm.height(self.text)
        
        
            painter.setBrush(QColor.fromHsv(color,170,200))  # Rojo semi-transparente
            rect = QRect(int(width/2-Twidth/2-margenes), int(height/2-fuente/2-margenes), int(Twidth+2*margenes), int(fuente+2*margenes))
            #rect = QRect(posicion[0], posicion[1], dimensiones[0], dimensiones[1])
            painter.drawRoundedRect(rect,margenes,margenes)
        

            painter.setPen(QColor(0, 0, 0))  # Color del texto
            painter.setFont(font)
            painter.drawText(QPoint(int(width/2-Twidth/2), int(height/2+fuente/2)), text)  # Posición del texto y el texto mismo
       
        
        elif funcion==4:  
            dimX=int(width/8)
            dimY=int(height/2)
            origenX=int(width/16)
            origenY=int(height/4)
        
            gradient = QLinearGradient(origenX, origenY,origenX+dimX, origenY+dimY)
            gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
            gradient.setColorAt(1,  QColor(140, 140, 140, 240))  # Opaco en el final
            painter.setBrush(gradient)# Gris medio  # Rojo semi-transparente
            
            MainRect = QRect(origenX,origenY, int(dimX), int(dimY))
            painter.drawRoundedRect(MainRect,margenes,margenes)
            
            px=0.9
            py=0.8

            
            y=int(10+(dimY-50)*(1-py)/5)
            x=int(20+(dimX-40)*(1-px)/2)
            altura=int((dimY-50)*(py)/4)
            anchura=int((dimX-40)*(px))
            TX=int(10+0.1*anchura)


            for i in range(4):
                painter.setPen(QColor(255, 255, 255,150))
                posX=origenX+x
                posY=origenY+(i+1)*y+i*altura
                gradient = QLinearGradient(posX, posY,posX+anchura, posY+altura)
                gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
                gradient.setColorAt(1,  QColor(170, 170, 170, 250))  # Opaco en el final

                
                if(i==(BotonMarcado-1)):
                    gradient.setColorAt(0, QColor(160, 230, 160, 220))  # Transparente en el inicio
                    gradient.setColorAt(1,  QColor(110, 170, 110, 250))  # Opaco en el final
                painter.setBrush(gradient)
                recti=QRect(posX,posY, anchura, altura)
                painter.drawRoundedRect(recti,10,10)
                
                painter.setPen(QColor(70, 70, 70))
                font = QFont('Helvetica', fuente)  # Tipo de fuente y tamaño
                font.setBold(True)
                
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])

                nuevafuente=int((anchura-TX)/Twidth*fuente)
                TY=int((altura-nuevafuente)/2)

                font = QFont('Helvetica', nuevafuente)
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])
                painter.setFont(font)
                painter.drawText(QPoint(int(posX+TX), int(posY+nuevafuente+TY)), MenuText[i])
                
                
            painter.setRenderHint(QPainter.Antialiasing)
            if modo==2:
                for i in range(numeroLineas):
                    pen = QPen(QColor.fromHsv(coloresUsados[i],170,200))  # Crear un objeto QPen con el color deseado
                    pen.setWidth(grosoresUsados[i])  # Establecer el grosor de la línea (por ejemplo, 2 píxeles)
                    painter.setPen(pen)
                    
                    puntos = [QPoint(x, y) for x, y in zip(lineasPintadasX[i], lineasPintadasY[i])]
                    painter.drawPolyline(*puntos)
                    
                    
            
        elif funcion==5:  # Nuevo caso para dibujar texto
            dimX=int(width/8)
            dimY=int(height/2)
            origenX=int(width/16)
            origenY=int(height/4)
        
            gradient = QLinearGradient(origenX, origenY,origenX+dimX, origenY+dimY)
            gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
            gradient.setColorAt(1,  QColor(140, 140, 140, 240))  # Opaco en el final
            painter.setBrush(gradient)# Gris medio  # Rojo semi-transparente
            
            MainRect = QRect(origenX,origenY, int(dimX), int(dimY))
            painter.drawRoundedRect(MainRect,margenes,margenes)
            
            px=0.9
            py=0.8

            
            y=int(10+(dimY-50)*(1-py)/5)
            x=int(20+(dimX-40)*(1-px)/2)
            altura=int((dimY-50)*(py)/4)
            anchura=int((dimX-40)*(px))
            TX=int(10+0.1*anchura)
            

            for i in range(4):
                painter.setPen(QColor(255, 255, 255,150))
                posX=origenX+x
                posY=origenY+(i+1)*y+i*altura
                gradient = QLinearGradient(posX, posY,posX+anchura, posY+altura)
                gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
                gradient.setColorAt(1,  QColor(170, 170, 170, 250))  # Opaco en el final
                painter.setBrush(gradient)
                
                if(i==(0)):
                    painter.setBrush(QColor.fromHsv(colorNuevo,170,200))  # Opaco en el final
                
                recti=QRect(posX,posY, anchura, altura)
                painter.drawRoundedRect(recti,10,10)
                
                painter.setPen(QColor(70, 70, 70))
                font = QFont('Helvetica', fuente)  # Tipo de fuente y tamaño
                font.setBold(True)
                
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])

                nuevafuente=int((anchura-TX)/Twidth*fuente)
                TY=int((altura-nuevafuente)/2)

                font = QFont('Helvetica', nuevafuente)
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])
                painter.setFont(font)
                painter.drawText(QPoint(int(posX+TX), int(posY+nuevafuente+TY)), MenuText[i])
                            
            painter.setRenderHint(QPainter.Antialiasing)
            if modo==2:
                for i in range(numeroLineas):
                    pen = QPen(QColor.fromHsv(coloresUsados[i],170,200))  # Crear un objeto QPen con el color deseado
                    pen.setWidth(grosoresUsados[i])  # Establecer el grosor de la línea (por ejemplo, 2 píxeles)
                    painter.setPen(pen)
                    
                    puntos = [QPoint(x, y) for x, y in zip(lineasPintadasX[i], lineasPintadasY[i])]
                    painter.drawPolyline(*puntos)
                    
                    
        elif funcion==6:  # Nuevo caso para dibujar texto
            dimX=int(width/8)
            dimY=int(height/2)
            origenX=int(width/16)
            origenY=int(height/4)
        
            gradient = QLinearGradient(origenX, origenY,origenX+dimX, origenY+dimY)
            gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
            gradient.setColorAt(1,  QColor(140, 140, 140, 240))  # Opaco en el final
            painter.setBrush(gradient)# Gris medio  # Rojo semi-transparente
            
            MainRect = QRect(origenX,origenY, int(dimX), int(dimY))
            painter.drawRoundedRect(MainRect,margenes,margenes)
            
            px=0.9
            py=0.8

            
            y=int(10+(dimY-50)*(1-py)/5)
            x=int(20+(dimX-40)*(1-px)/2)
            altura=int((dimY-50)*(py)/4)
            anchura=int((dimX-40)*(px))
            TX=int(10+0.1*anchura)
            

            for i in range(4):
                painter.setPen(QColor(255, 255, 255,150))
                posX=origenX+x
                posY=origenY+(i+1)*y+i*altura
                gradient = QLinearGradient(posX, posY,posX+anchura, posY+altura)
                gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
                gradient.setColorAt(1,  QColor(170, 170, 170, 250))  # Opaco en el final
                painter.setBrush(gradient)
                
                if(i==(1)):
                    painter.setBrush(QColor.fromHsv(55,170,200))  # Opaco en el final
                
                recti=QRect(posX,posY, anchura, altura)
                painter.drawRoundedRect(recti,10,10)
                
                painter.setPen(QColor(70, 70, 70))
                font = QFont('Helvetica', fuente)  # Tipo de fuente y tamaño
                font.setBold(True)
                
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])

                nuevafuente=int((anchura-TX)/Twidth*fuente)
                TY=int((altura-nuevafuente)/2)

                font = QFont('Helvetica', nuevafuente)
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])
                painter.setFont(font)
                painter.drawText(QPoint(int(posX+TX), int(posY+nuevafuente+TY)), MenuText[i])
                            
            painter.setRenderHint(QPainter.Antialiasing)
            if modo==2:
                for i in range(numeroLineas):
                    pen = QPen(QColor.fromHsv(coloresUsados[i],170,200))  # Crear un objeto QPen con el color deseado
                    pen.setWidth(grosoresUsados[i])  # Establecer el grosor de la línea (por ejemplo, 2 píxeles)
                    painter.setPen(pen)
                    
                    puntos = [QPoint(x, y) for x, y in zip(lineasPintadasX[i], lineasPintadasY[i])]
                    painter.drawPolyline(*puntos)
        elif funcion==7:  """Menu de ajuste de los promedios del ratón"""
            dimX=int(width/8)
            dimY=int(height/2)
            origenX=int(width/16)
            origenY=int(height/4)
        
            gradient = QLinearGradient(origenX, origenY,origenX+dimX, origenY+dimY)
            gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
            gradient.setColorAt(1,  QColor(140, 140, 140, 240))  # Opaco en el final
            painter.setBrush(gradient)# Gris medio  # Rojo semi-transparente
            
            MainRect = QRect(origenX,origenY, int(dimX), int(dimY))
            painter.drawRoundedRect(MainRect,margenes,margenes)
            
            px=0.9
            py=0.8

            
            y=int(10+(dimY-50)*(1-py)/5)
            x=int(20+(dimX-40)*(1-px)/2)
            altura=int((dimY-50)*(py)/4)
            anchura=int((dimX-40)*(px))
            TX=int(10+0.1*anchura)
            

            for i in range(4):
                painter.setPen(QColor(255, 255, 255,150))
                posX=origenX+x
                posY=origenY+(i+1)*y+i*altura
                gradient = QLinearGradient(posX, posY,posX+anchura, posY+altura)
                gradient.setColorAt(0, QColor(230, 230, 230, 220))  # Transparente en el inicio
                gradient.setColorAt(1,  QColor(170, 170, 170, 250))  # Opaco en el final
                painter.setBrush(gradient)
                
                if(i==(2)):
                    painter.setBrush(QColor.fromHsv(180,170,200))  # Opaco en el final
                
                recti=QRect(posX,posY, anchura, altura)
                painter.drawRoundedRect(recti,10,10)
                
                painter.setPen(QColor(70, 70, 70))
                font = QFont('Helvetica', fuente)  # Tipo de fuente y tamaño
                font.setBold(True)
                
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])

                nuevafuente=int((anchura-TX)/Twidth*fuente)
                TY=int((altura-nuevafuente)/2)

                font = QFont('Helvetica', nuevafuente)
                fm = QFontMetrics(font)
                Twidth = fm.width(MenuText[i])
                painter.setFont(font)
                painter.drawText(QPoint(int(posX+TX), int(posY+nuevafuente+TY)), MenuText[i])

    def actualizar_figuras(self):
        # Forzar un nuevo pintado en la ventana
        if funcion == 1:
            self.square_x = posicion[0]
            self.square_y = posicion[1]
                

        self.update()
        
    def close_canvas(self):
        # Cerrar la ventana del canvas
        self.close()


def run_gui(queue):
    app = QApplication(sys.argv)
    canvas = Canvas()
    # Manejar la señal de cierre
    canvas.communication.close_signal.connect(app.quit)
    # Poner la instancia de Canvas en la cola
    queue.put(canvas)
    sys.exit(app.exec_())
      
"""Variables para el envio de datos   (creo que se pueden borrar)"""    

red = 0
#lock = threading.Lock()
ordenbits=(10,10)

""""Obtenemos el tamaño de la pantalla"""
width = win32api.GetSystemMetrics(0)
height = win32api.GetSystemMetrics(1)

calibrado=[[0,0],[0,0],[0,0],[0,0]]

"""Funciones para eliminar el firewall para asegurar el envio de datos (sus nombres son explicativos)"""

import subprocess
import os

nombre_app = "AnularFirewall"

def obtener_ruta_actual():
    return os.path.dirname(os.path.abspath(__file__))

def obtener_ruta_temporal():
    return os.path.dirname(sys.executable)

def agregar_excepcion_firewall(nombre_app, ruta_exe):
    try:
        # Ejecutar el comando netsh para agregar una excepción al firewall de Windows
        subprocess.run(['netsh', 'advfirewall', 'firewall', 'add', 'rule',
                        'name="{}"'.format(nombre_app), 'dir=in', 'action=allow',
                        'program="{}"'.format(ruta_exe), 'enable=yes'], check=True)
        print(f"Excepción de firewall agregada para {nombre_app}")
    except subprocess.CalledProcessError as e:
        print(f"Error al agregar la excepción de firewall: {e}")

def eliminar_excepcion_firewall(nombre_app):
    try:
        # Ejecutar el comando netsh para eliminar la excepción del firewall de Windows
        subprocess.run(['netsh', 'advfirewall', 'firewall', 'delete', 'rule',
                        'name="{}"'.format(nombre_app)], check=True)
        print(f"Excepción de firewall eliminada para {nombre_app}")
    except subprocess.CalledProcessError as e:
        print(f"Error al eliminar la excepción de firewall: {e}")
        
    try:
        # Ejecutar el comando netsh para eliminar la excepción del firewall de Windows
        subprocess.run(['netsh', 'advfirewall', 'firewall', 'delete', 'rule',
                        'name="{}"'.format(nombre_app)], check=True)
        print(f"Excepción de firewall eliminada para {nombre_app}")
    except subprocess.CalledProcessError as e:
        print(f"Error al eliminar la excepción de firewall: {e}")


"""Variables para definir el tamaño de los cuadrados de calibración y sus posiciones en la pantalla según la resolución de esta"""
funcion=1
dimensiones=(200,200)
posiciones=((0,0),(width-dimensiones[0]-1,0),(width-dimensiones[0]-1,height-dimensiones[1]-1), (0,height-dimensiones[1]-1)) ##posicion de las 4 esquinas para colocar el cuadrado
posicion=posiciones[0]


"""Función que se utiliza al principio del codigo en el calibrado de la extensión de la mano"""

def calibrar_mano():
    global funcion,fuente,margenes,color,text
    funcion=3

    fuente=80

    margenes=70

    color=0

    text="ABRE LA MANO"

    data, address = server_socket.recvfrom(1024)

    color=120

    time.sleep(0.70)

    color=0

    text="CIERRA LA MANO"

    data, address = server_socket.recvfrom(1024)

    color=120

    time.sleep(0.70)
    
    return

"""funciones para el tratamiento de datos segun estemos calibrando o en el la parte principal del programa"""

def decode_data(data):
    ##data = data.decode('utf-8')
    
    if len(data) == 0:
        print("Paquete vacío. Ignorando...")
        return None, None  # Devolver None para indicar que se debe ignorar el paquete
    else:
        caso=data[0]-1
        ##print(caso)
        if caso==2 or caso==3 or caso==4 or caso==5:
            alpha = (data[1] & 0xFF) | ((data[2] & 0x03) << 8)  # Recuperar los 10 bits del primer valor
            beta = ((data[2] >> 2) & 0x3F) | ((data[3] & 0x0F) << 6)  # Recuperar los 10 bits del segundo valor
            return caso,beta,alpha
        else:
            return caso,None,None
        
        
def decode_data_calibrado(data):
    ##data = data.decode('utf-8')
    
    if len(data) < 3:
        print("Paquete vacío o semi-vacío. Ignorando...")
        return None, None  # Devolver None para indicar que se debe ignorar el paquete
    else:
        alpha = (data[0] & 0xFF) | ((data[1] & 0x03) << 8)  # Recuperar los 10 bits del primer valor
        beta = ((data[1] >> 2) & 0x3F) | ((data[2] & 0x0F) << 6)  # Recuperar los 10 bits del segundo valor
        return beta, alpha        
        
    

"""Función que convierte los angulos en posiciones dentro de la pantalla"""

def Ajustepantalla(beta,alpha):
    #acotamos entre nuestro máximo y minimo de calibracion
    beta=int((beta-anchuras[1])/(anchuras[0]-anchuras[1])*width)
    alpha=height-int((alpha-alturas[1])/(alturas[0]-alturas[1])*height)
    
    #aseguramos que no pasamos esos limites (demomento saturamos)
    if (alpha<0): alpha=0
    if(alpha>height): alpha=height
    if(beta<0): beta=0
    if(beta>width): beta=width
    
    ##divido entre 2^numerodebits para tener entre 0 y 1 (ya no por que controlamos con los margenes)
    #print(beta)
    #print(alpha)
    return beta, alpha

"""Función para usar teclas o botones del ratón"""

def press_key(key):
    win32api.keybd_event(key, 0, 0, 0)
    win32api.keybd_event(key, 0, win32con.KEYEVENTF_KEYUP, 0)

"""
Eliminamos el firewall con las funciones de antes
"""    

ruta_exe = os.path.join(obtener_ruta_temporal(), "RatonCompleto1.exe")
agregar_excepcion_firewall(nombre_app, ruta_exe)


"""
Abro el puerto para recibir datos del Arduino
"""
port = 1002

i=0
while(i==0):
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.bind(("0.0.0.0", port))
        i=1
    except OSError as e:
        print(f"Error al vincular el socket: {e}")
        server_socket.close()
        time.sleep(1)
        i=0
    

"""Recibo un byte que me confirma el incio de la comunicación"""

print("Esperando confirmacion de arduino")
data, address = server_socket.recvfrom(1024)
#print(data)
print("empieza el calibrado")

"""Inicio el canvas en un hilo secundario"""

#app = QApplication(sys.argv)

# Crear una cola para comunicarse entre los hilos
queue = Queue()

# Iniciar el hilo de la GUI
gui_thread = threading.Thread(target=run_gui, args=(queue,))
gui_thread.start()
# Obtener la instancia de Canvas de la cola
canvas = queue.get()

"""Recibo los 4 datos correspondientes a las 4 esquinas durante el calibrado de la pantalla"""

for i in range(4):
    posicion=posiciones[i]
    #canvas.actualizar_figuras()
    #print(data)
    data, address = server_socket.recvfrom(1024)#para recibir el dato
    #print(data)
    print(decode_data_calibrado(data))
    calibrado[i]=decode_data_calibrado(data)
    
print("fin calibracion")


"""Calibracion de la extension de la mano"""

calibrar_mano()

"""cierra el canvas"""
canvas.communication.close_signal.emit()

time.sleep(1)

"""De la calibracion obtenemos los limites de la pantalla"""
anchuras = [max([elemento[0] for elemento in calibrado]),min([elemento[0] for elemento in calibrado])]
alturas = [max([elemento[1] for elemento in calibrado]),min([elemento[1] for elemento in calibrado])]

anchuras = [512+(anchuras[0]-anchuras[1])/2,512-(anchuras[0]-anchuras[1])/2]

"""Cambio la función a 2, que es el modo para el ratón standard"""

funcion=2

"""Defino las distintas variables de control que me servirán para saber en que momento me encuentro haciendo cada gesto, o en que menu estoy. Los nombres son indicativos de que indica cada una"""

CanvasOn=False
ClicDerechoOn=False
ClicIzquierdoOn=False
DobleClic=False
GiroscopoOn=False
ClicCortoOn=False
Reposo=False
MenuDisponible=False
MenuOn=False
TeclaOn=False

"""Indica Cuanto dura el clic corto, y apartir de ahí se considerara clic mantenido. Esto es por que el ordenador considera clic corto cuando el ratón no se mueve durante ese tiempo, 
pero como es muy dificil mantener la mano completamente quieta, fijamos su posición de forma forzada en el codigo hasta que pase este tiempo de clic corto, apartir de ahí sigue a la mano
y se puede realizar un clic largo para arrastrar o seleccionar objetos"""

DuracionClicCorto=0.7

TiempoClic=0

"""Variable para promediar varias medidas de los angulos"""
alpha_media=[]
beta_media=[]
numero_promedios=20
numero_promedios_nuevo=numero_promedios

modo=0 
modoAnterior=0

"""Variables para el modo pintar"""
color=0
grosor=3
colorNuevo=color
grosorNuevo=grosor

coloresUsados=[]
lineasPintadasX=[]
lineasPintadasY=[]
grosoresUsados=[]

numeroLineas=0

Pintando=False
Borrando=False


"""Variables para conocer el ultimo boton pulsado, el menu en el que estamos, y que texto debe aparecer en este"""

MenuText=["","","",""]
Pestana=0 # 0 es decir ninguna
BotonMarcado=0


tiempoBucle=time.time()

tiempoBoton=0.7
"""
0=raton
1=teclado
2=pintar
3=off
"""

"""
Comentarios generales para las distintas funciones:
    Las variables e control sirven para saber si se ha entrado en una u otra función y no repetir dando lugar a error
    
    La variable caso se refiere a la posición de la mano recibida del arduino (del 0 al 9)
    
    La variable función apela a que tipo de menu estamos viendo en pantalla
    
    La variable modo se refiere a que modo de ratón tenemos activo
    
    la variable pestana (pestaña) indica en que submenu nos encontramos
"""


try:
    while(True):
        
        data, address = server_socket.recvfrom(1024)
        ##print(data)
        ##print(decode_data(data))
        BotonMarcado=0
        ##print((time.time()-tiempoBucle)*1000000)
        tiempoBucle=time.time()
        if(decode_data(data)!=(None, None)):
            caso=decode_data(data)[0]
            """Esta sección se encarga de desactivar los indicadores cuando deja de ocurrir el evento que se encargan de controlar"""
            if caso!=5 and CanvasOn and not MenuOn:
                canvas.communication.close_signal.emit()
                CanvasOn=False
            elif caso!=3 and ClicDerechoOn:
                ClicDerechoOn=False
            elif caso!=4 and ClicIzquierdoOn:
                ClicIzquierdoOn=False
                ClicCortoOn=False
                win32api.mouse_event(soltar_clic_izquierdo,posicion[0],posicion[1],0,0)
            elif ClicCortoOn and (time.time()-TiempoClic)>DuracionClicCorto:
                ClicCortoOn=False
            elif caso!=1 and DobleClic:
                DobleClic=False
            elif MenuDisponible and (caso!=0 and caso!=1):
                MenuDisponible=False
            elif TeclaOn and (caso!=6 and caso!=7 and caso!=8 and caso!=9):
                TeclaOn=False
            elif Pintando and caso!=4:
                Pintando=False
            elif Borrando and caso!=3:
                Borrando=False
               
            
            """Miramos si hay que usar el giroscopo"""
            if (caso==2 or caso==3 or caso==4 or caso==5) and (modo==0 or modo==1 or modo==2):
                
                if not GiroscopoOn:
                    print("giroscopo")
                    alpha_media=[]
                    beta_media=[]
                    GiroscopoOn=True
                    Reposo=False
                    
                beta_media.append(decode_data(data)[1])
                alpha_media.append(decode_data(data)[2])
                
                if (len(alpha_media)>numero_promedios):
                    beta_media.pop(0)
                    alpha_media.pop(0)
                    
                posicion=Ajustepantalla((sum(beta_media)/len(beta_media)),(sum(alpha_media)/len(alpha_media)))
                """Caso pintar pantalla"""
            
            
                if caso==5:
                    print("pintar")
                    if not CanvasOn:
                        funcion=2
                        # Crear una cola para comunicarse entre los hilos
                        queue = Queue()
                        # Iniciar el hilo de la GUI
                        gui_thread = threading.Thread(target=run_gui, args=(queue,))
                        gui_thread.start()
                        # Obtener la instancia de Canvas de la cola
                        canvas = queue.get()
                        CanvasOn=True
                        
                elif not ClicCortoOn and (modo==0 or modo==1):#Caso raton
                    win32api.SetCursorPos(posicion)
                    if(caso==3 and not ClicDerechoOn):
                        win32api.mouse_event(clic_derecho,posicion[0],posicion[1],0,0)
                        #time.sleep(0.1)
                        win32api.mouse_event(soltar_clic_derecho,posicion[0],posicion[1],0,0)
                        ClicDerechoOn=True
                    elif(caso==4 and not ClicIzquierdoOn):
                        win32api.mouse_event(clic_izquierdo,posicion[0],posicion[1],0,0)
                        #time.sleep(0.1)
    
                        ClicIzquierdoOn=True
                        ClicCortoOn=True
                        TiempoClic=time.time()
                
                elif modo==2:
                    win32api.SetCursorPos(posicion)
                    if caso==4:
                        
                        if not Pintando:
                            coloresUsados.append(color)
                            grosoresUsados.append(grosor)
                            Pintando=True
                            lineasPintadasX.append([posicion[0]])
                            lineasPintadasY.append([posicion[1]])
                            numeroLineas+=1
                        else:
                            lineasPintadasX[numeroLineas-1].append(posicion[0])
                            lineasPintadasY[numeroLineas-1].append(posicion[1])
                    elif caso==3 and not Borrando and numeroLineas>0:
                        Borrando=True
                        numeroLineas+=-1
                        del lineasPintadasX[numeroLineas]
                        del lineasPintadasY[numeroLineas]
                        del coloresUsados[numeroLineas]
                        del grosoresUsados[numeroLineas]
            
            """Apaga el giroscopo"""
            elif (GiroscopoOn and ((caso!=1 and modo!=0) or( caso ==0)) ):
                print("Apago giroscopo")
                GiroscopoOn=False
                
            """Eventos de clic durante el uso del raton"""
            elif (GiroscopoOn and caso==1 and not DobleClic):
                print("Doble Clic")
                win32api.mouse_event(clic_izquierdo,posicion[0],posicion[1],0,0)
                #time.sleep(0.1)
                win32api.mouse_event(soltar_clic_izquierdo,posicion[0],posicion[1],0,0)
                win32api.mouse_event(clic_izquierdo,posicion[0],posicion[1],0,0)
                #time.sleep(0.1)
                win32api.mouse_event(soltar_clic_izquierdo,posicion[0],posicion[1],0,0)
                DobleClic=True
            
            """Control de apertura del menu"""
            elif caso==1:
                if Reposo:
                    print("Menu disponible")
                    Reposo=False
                    MenuDisponible=True
                if MenuOn:
                    print("Cerrar menu")
                    MenuOn=False
                    Pestana=0
                    modo=modoAnterior
                    canvas.communication.close_signal.emit()
            elif caso==0:
                if MenuDisponible:
                    print("Modo menu")
                    modoAnterior=modo
                    MenuText=["Apps","Modo","Pintar","Ajustes"]
                    modo=3
                    funcion=4
                    queue = Queue()
                    # Iniciar el hilo de la GUI
                    gui_thread = threading.Thread(target=run_gui, args=(queue,))
                    gui_thread.start()
                    # Obtener la instancia de Canvas de la cola
                    canvas = queue.get()
                    CanvasOn=True
                    
                    MenuDisponible=False
                    MenuOn=True
                elif not (MenuOn or Reposo):
                    Reposo=True
                    print("Modo reposo")
            
            """Botones durante el modo giroscopo, con sus distintos modos"""
            elif (caso==6 or caso==7 or caso==8 or caso==9):
                if modo==0:
                    posicion=win32api.GetCursorPos()
                    if caso==6:
                        win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, posicion[0], posicion[1], 40, 0)

                    elif caso==7:
                        win32api.mouse_event(win32con.MOUSEEVENTF_WHEEL, posicion[0], posicion[1], -40, 0)

                    elif caso==8:
                        TeclaOn=True
                        
                    elif caso==9:
                        TeclaOn=True
                        
                elif modo==1 and not TeclaOn:
                    posicion=win32api.GetCursorPos()
                    if caso==6:
                        press_key(win32con.VK_LEFT)
                        TeclaOn=True
                    elif caso==7:
                        press_key(win32con.VK_RIGHT)
                        TeclaOn=True
                    elif caso==8:
                        press_key(win32con.VK_SPACE)
                        TeclaOn=True
                    elif caso==9:
                        TeclaOn=True
                
                """Apertura de los distintos submenus"""
                
                elif (modo==3) and MenuOn and not TeclaOn:
                    if Pestana==0:
                        if caso==6:
                            Pestana=1
                            BotonMarcado=1
                            time.sleep(tiempoBoton)
                            MenuText=["Calculadora","Recortes","Google","Atras"]
                            TeclaOn=True
                        elif caso==7:
                            Pestana=2
                            BotonMarcado=2
                            time.sleep(tiempoBoton)
                            MenuText=["Raton","Teclado","Apagado","Atras"]
                            TeclaOn=True
                        elif caso==8:
                            Pestana=3
                            BotonMarcado=3
                            time.sleep(tiempoBoton)
                            modo=2
                            MenuText=["Color","Grosor","Borrar Todo","Atras"]
                            TeclaOn=True  
                        elif caso==9:
                            Pestana=4
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            MenuText=["Pantalla","Mano","Promedios","Atras"]
                            TeclaOn=True
                    
                    """Submenu para abrir apps"""
                    elif Pestana==1:
                        if caso==6:
                            BotonMarcado=1
                            time.sleep(tiempoBoton)
                            subprocess.Popen("calc.exe")
                            TeclaOn=True
                        elif caso==7:
                            BotonMarcado=2
                            time.sleep(tiempoBoton)
                            subprocess.Popen("SnippingTool.exe")
                            TeclaOn=True
                        elif caso==8:
                            BotonMarcado=3
                            time.sleep(tiempoBoton)
                            subprocess.Popen("C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe")
                            TeclaOn=True
                        elif caso==9:
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            TeclaOn=True
                            MenuText=["Apps","Modo","Pintar","Ajustes"]
                            Pestana=0
                    
                    """Submenu para cambiar los modos"""
                    elif Pestana==2:
                        if caso==6:
                            BotonMarcado=1
                            time.sleep(tiempoBoton)
                            modoAnterior=0
                            TeclaOn=True
                        elif caso== 7:
                            BotonMarcado=2
                            time.sleep(tiempoBoton)
                            modoAnterior=1
                            TeclaOn=True
                        elif caso==8:
                            BotonMarcado=3
                            time.sleep(tiempoBoton)
                            modoAnterior=3
                            TeclaOn=True
                        elif caso==9:
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            TeclaOn=True
                            MenuText=["Apps","Modo","Pintar","Ajustes"]
                            Pestana=0
                            
                            
                        
                    """Submenu de ajustes"""
                    elif Pestana==4 and funcion!=7:
                        """Recalibrado de la apertura de la mano"""
                        if caso==7:
                            print("Enviar")
                            broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                            broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                            mensaje = "si".encode('utf-8')
                            broadcast_socket.sendto(mensaje,address)
                            broadcast_socket.close()

                            BotonMarcado=2
                            while True:
                                try:
                                    # Recibe el paquete UDP
                                    data, address = server_socket.recvfrom(1024)
                                    # Intenta decodificar el dato recibido
                                    try:
                                        flag = data.decode('utf-8')
                                        # Si la decodificación es exitosa, puedes usar el flag
                                        print("Flag recibido:", flag)
                                        # Haz algo aquí con el flag
                                        if flag=='a':
                                            break
                                    except UnicodeDecodeError:
                                        # Si no se puede decodificar el dato, maneja el error
                                        print("No se pudo decodificar el dato recibido")
                                        # Haz algo aquí para manejar esta situación, como ignorar el paquete o registrar el error
                                except socket.error as e:
                                    # Maneja cualquier error de socket que pueda ocurrir durante la recepción de datos
                                    print("Error de socket:", e)
                            calibrar_mano()
                            TeclaOn=True
                            funcion=4
                            
                        "Recalibrado de las esquinas"
                        elif caso==6:
                            BotonMarcado=1
                            time.sleep(tiempoBoton)
                            TeclaOn=True
                            funcion=1
                            for i in range(4):
                                TiempoEsquina=time.time()
                                posicion=posiciones[i]
                                #canvas.actualizar_figuras()
                                #print(data)
                                while (time.time()-TiempoEsquina)<3:
                                    data, address = server_socket.recvfrom(1024)#para recibir el dato
                                    #print(decode_data(data))
                                calibrado[i]=(decode_data(data)[1],decode_data(data)[2])
                                print(calibrado[i])
                            
                            anchuras = [max([elemento[0] for elemento in calibrado]),min([elemento[0] for elemento in calibrado])]
                            alturas = [max([elemento[1] for elemento in calibrado]),min([elemento[1] for elemento in calibrado])]
                            anchuras = [512+(anchuras[0]-anchuras[1])/2,512-(anchuras[0]-anchuras[1])/2]

                            funcion=4
                        
                        """Ajuste de los promedios del ratón"""
                        elif caso==8:
                            funcion=7
                            TeclaOn=True
                            MenuText=['  +    ','  -   ','Selecionar: '+str(numero_promedios),'Atras']
                        elif caso==9:
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            TeclaOn=True
                            MenuText=["Apps","Modo","Pintar","Ajustes"]
                            Pestana=0
                    elif funcion==7 and not TeclaOn:
                        if caso==6:
                            numero_promedios_nuevo+=1/25
                            MenuText=['  +    ','  -   ','Selecionar: '+str(int(numero_promedios_nuevo)),'Atras']
                        elif caso==8:
                            numero_promedios=int(numero_promedios_nuevo)
                            TeclaOn=True
                            funcion=4
                            BotonMarcado=3
                            time.sleep(tiempoBoton)
                            MenuText=["Pantalla","Mano","Promedios","Atras"]
                        elif caso==7:
                            if numero_promedios_nuevo>1:
                                numero_promedios_nuevo+=-1/25
                            MenuText=['  +    ','  -   ','Selecionar: '+str(int(numero_promedios_nuevo)),'Atras']
                        elif caso==9:
                            numero_promedios_nuevo=numero_promedios
                            TeclaOn=True
                            funcion=4
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            MenuText=["Pantalla","Mano","Promedios","Atras"]
                    
                """Submenu para pintar"""
                elif (modo==2) and MenuOn and not TeclaOn:
                    if funcion == 4:
                        if caso==6:
                            funcion=5
                            TeclaOn=True
                            MenuText=['Seleccionar','  +   ','  -   ','Atras']
                            print('modo 5')
                        elif caso==7:
                            funcion=6
                            TeclaOn=True
                            MenuText=['  +    ','Selecionar: '+str(grosor),'  -   ','Atras']
                            print('modo 6')
                        elif caso==8:
                            TeclaOn=True
                            BotonMarcado=3
                            time.sleep(tiempoBoton)
                            lineasPintadasX.clear()
                            lineasPintadasY.clear()
                            grosoresUsados.clear()
                            coloresUsados.clear()
                            numeroLineas=0
                        elif caso==9:
                            Pestana=0
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            MenuText=["Apps","Modo","Pintar","Ajustes"]
                            TeclaOn=True
                            modo=3
                                    
                    elif funcion==5:
                        if caso==6:
                            color=colorNuevo
                            TeclaOn=True
                            funcion=4
                            BotonMarcado=1
                            time.sleep(tiempoBoton)
                            MenuText=["Color","Grosor","Borrar Todo","Atras"]
                            
                        elif caso==7:
                            colorNuevo+=2
                            if colorNuevo>360:
                                colorNuevo=-30
                        elif caso==8:
                            colorNuevo+=-2
                            if colorNuevo<-30:
                                colorNuevo=360
                        elif caso==9:
                            colorNuevo=color
                            TeclaOn=True
                            funcion=4
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            MenuText=["Color","Grosor","Borrar Todo","Atras"]
                    elif funcion==6:
                        if caso==6:
                            grosorNuevo+=1/25
                            MenuText=['  +    ','Selecionar: '+str(int(grosorNuevo)),'  -   ','Atras']
                        elif caso==7:
                            grosor=int(grosorNuevo)
                            TeclaOn=True
                            funcion=4
                            BotonMarcado=2
                            time.sleep(tiempoBoton)
                            MenuText=["Color","Grosor","Borrar Todo","Atras"]
                        elif caso==8:
                            if grosorNuevo>1:
                                grosorNuevo+=-1/25
                            MenuText=['  +    ','Selecionar: '+str(int(grosorNuevo)),'  -   ','Atras']
                        elif caso==9:
                            grosorNuevo=grosor
                            TeclaOn=True
                            funcion=4
                            BotonMarcado=4
                            time.sleep(tiempoBoton)
                            MenuText=["Color","Grosor","Borrar Todo","Atras"]
                                
        
"""Cierre del programa"""

except KeyboardInterrupt:
    # Manejar una interrupción del teclado (Ctrl+C) para cerrar los sockets
    print("Cerrando sockets...")
    canvas.communication.close_signal.emit()
    server_socket.close()
    eliminar_excepcion_firewall(nombre_app)
    time.sleep(3)
    print("Sockets cerrados.")


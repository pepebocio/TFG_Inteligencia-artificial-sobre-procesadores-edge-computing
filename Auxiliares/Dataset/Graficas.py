# Lista para almacenar los datos
import numpy as np
datos = []

guarda_datos=open("DataSetOrdenado.txt","w")
guarda_datos.close


guarda_datos=open("DataSetOrdenado.txt","a")
# Abrir el archivo en modo lectura
with open('DataSet(1+2+3+4+5).txt', 'r') as archivo:
    # Leer cada línea del archivo
    for linea in archivo:
        # Dividir la línea en valores separados por espacios en blanco
        valores = linea.split()
        # Convertir los valores a enteros y agregarlos a la lista de datos
        datos.append([int(valor) for valor in valores])

# Ordenar los datos en base al quinto entero (grupo)
datos_ordenados = sorted(datos, key=lambda x: x[4])

# Imprimir los datos ordenados
for dato in datos_ordenados:
    print(dato)
    guarda_datos.write(str(dato[4])+'\t'+str(dato[0])+'\t'+str(dato[1])+'\t'+str(dato[2])+'\t'+str(dato[3])+'\n')
guarda_datos.close

import matplotlib.pyplot as plt

# Suponiendo que tienes los datos ordenados en una lista llamada datos_ordenados
# Extraer las posiciones de los dedos para cada grupo
grupos = [dato[4] for dato in datos_ordenados]
posiciones_dedo1 = [dato[0] for dato in datos_ordenados]
posiciones_dedo2 = [dato[1] for dato in datos_ordenados]
posiciones_dedo3 = [dato[2] for dato in datos_ordenados]
posiciones_dedo4 = [dato[3] for dato in datos_ordenados]

# Suponiendo que tienes los datos ordenados en una lista llamada datos_ordenados
# Extraer las posiciones de los dedos para cada posición y agruparlas por valor en la primera columna
posiciones_dedo1_por_grupo = {}
posiciones_dedo2_por_grupo = {}
posiciones_dedo3_por_grupo = {}
posiciones_dedo4_por_grupo = {}

for dato in datos_ordenados:
    grupo = dato[4]
    pos_dedo1 = dato[0]
    pos_dedo2 = dato[1]
    pos_dedo3 = dato[2]
    pos_dedo4 = dato[3]

    if grupo not in posiciones_dedo1_por_grupo:
        posiciones_dedo1_por_grupo[grupo] = []
        posiciones_dedo2_por_grupo[grupo] = []
        posiciones_dedo3_por_grupo[grupo] = []
        posiciones_dedo4_por_grupo[grupo] = []

    posiciones_dedo1_por_grupo[grupo].append(pos_dedo1)
    posiciones_dedo2_por_grupo[grupo].append(pos_dedo2)
    posiciones_dedo3_por_grupo[grupo].append(pos_dedo3)
    posiciones_dedo4_por_grupo[grupo].append(pos_dedo4)

# Graficar diagrama de caja para cada posición de los dedos en gráficos separados
plt.figure(figsize=(12, 8))

plt.subplot(221)
plt.boxplot(posiciones_dedo1_por_grupo.values(), labels=posiciones_dedo1_por_grupo.keys())
plt.title('Dedo 1')
plt.xlabel('Posición')
plt.ylabel('Medida del ADC')

plt.subplot(222)
plt.boxplot(posiciones_dedo2_por_grupo.values(), labels=posiciones_dedo2_por_grupo.keys())
plt.title('Dedo 2')
plt.xlabel('Posición')
plt.ylabel('Medida del ADC')


plt.subplot(223)
plt.boxplot(posiciones_dedo3_por_grupo.values(), labels=posiciones_dedo3_por_grupo.keys())
plt.title('Dedo 3')
plt.xlabel('Posición')
plt.ylabel('Medida del ADC')

plt.subplot(224)
plt.boxplot(posiciones_dedo4_por_grupo.values(), labels=posiciones_dedo4_por_grupo.keys())
plt.title('Dedo 4')
plt.xlabel('Posición')
plt.ylabel('Medida del ADC')


plt.tight_layout()
plt.show()

# Extraemos el último número de cada array para ver si las frecuencias
flags = [array[-1] for array in datos_ordenados]

plt.figure()

# Creamos un histograma
plt.hist(flags, bins=range(11), align='left', rwidth=0.8)

# Configuramos el título y las etiquetas de los ejes
plt.title('Distribución del data set')
plt.xlabel('Posición')
plt.ylabel('Frecuencia')

# Mostramos el histograma
plt.show()
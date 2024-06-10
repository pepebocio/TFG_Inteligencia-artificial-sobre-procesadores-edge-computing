# -*- coding: utf-8 -*-
"""
Created on Thu Mar 21 10:10:54 2024

@author: ELECTRONICA
"""
import numpy as np
datos = []

with open('DataSet(1+2+3+4+5).txt', 'r') as archivo:
    # Leer cada línea del archivo
    for linea in archivo:
        # Dividir la línea en valores separados por espacios en blanco
        valores = linea.split()
        # Convertir los valores a enteros y agregarlos a la lista de datos
        datos.append([int(valor) for valor in valores])

# Convertimos la lista en un array de NumPy para facilitar los cálculos

array = np.array(datos)

dedos= array[:, :4]

# Obtenemos el valor máximo y mínimo en cada columna
valores_maximos = np.max(dedos, axis=0)
valores_minimos = np.min(dedos, axis=0)

# Normalizamos los valores
dedos_normalizados = (dedos - valores_minimos) / (valores_maximos - valores_minimos)


datos_normalizados = np.column_stack((dedos_normalizados, array[:, 4].astype(int)))

guarda_datos=open("DataSetNormalizado.txt","w")
guarda_datos.close

guarda_datos=open("DataSetNormalizado.txt","a")
for dato in datos_normalizados:
    print(dato)
    guarda_datos.write(str(dato[0])+'\t'+str(dato[1])+'\t'+str(dato[2])+'\t'+str(dato[3])+'\t'+str(dato[4].astype(int))+'\n')
guarda_datos.close

guarda_datos=open("ValoresNormalizacion.txt","w")
guarda_datos.close

guarda_datos=open("ValoresNormalizacion.txt","a")
guarda_datos.write(str(valores_maximos[0])+'\t'+str(valores_maximos[1])+'\t'+str(valores_maximos[2])+'\t'+str(valores_maximos[3])+'\n')
guarda_datos.write(str(valores_minimos[0])+'\t'+str(valores_minimos[1])+'\t'+str(valores_minimos[2])+'\t'+str(valores_minimos[3])+'\n')
guarda_datos.close
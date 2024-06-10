import socket

dedo1 = 0
dedo2 = 0
dedo3 = 0
dedo4 =0

def manejar_mensaje(data, address):
    print(f"Datos brutos recibidos: {data}")
    global dedo1, dedo2, dedo3, dedo4
    mensaje = data.decode('utf-8')

    dedo1 = ord(mensaje[0])
    dedo1 += ord(mensaje[1])*128
    dedo2 = ord(mensaje[2])
    dedo2 += ord(mensaje[3])*128
    dedo3 = ord(mensaje[4])
    dedo3 += ord(mensaje[5])*128
    dedo4 = ord(mensaje[6])
    dedo4 += ord(mensaje[7])*128
    flag =ord(mensaje[8])
    print(f"dedo1: {dedo1}, dedo2: {dedo2}, dedo3: {dedo3}, dedo4: {dedo4} flag: {flag}")
    guarda_datos=open("DataSet.txt","a")
    guarda_datos.write(str(dedo1)+'\t'+str(dedo2)+'\t'+str(dedo3)+'\t'+str(dedo4)+'\t'+str(flag)+'\n')
    guarda_datos.close


def main():
    port = 1001
    
    guarda_datos=open("DataSet.txt","w")
    guarda_datos.close


    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server_socket.bind(("0.0.0.0", port))

        print(f"El servidor UDP está escuchando en el puerto {port}")

        while True:
            data, address = server_socket.recvfrom(1024)
            manejar_mensaje(data, address)
        
    except OSError as e:
        print(f"Error al vincular el socket: {e}")
        server_socket.close()
    except KeyboardInterrupt:
        # Manejar una interrupción del teclado (Ctrl+C) para cerrar el socket
        print("Cerrando el socket...")
        server_socket.close()
        print("Socket cerrado.")

if __name__ == "__main__":
    main()

import socket
import threading


class SocketServer:
    def __init__(self):
        self.altura_desejada = 0
        self.altura_medida = 0
        

    def returnDesiredHeight(self):
        print("Altura desejada: ", self.altura_desejada)
        valor = self.altura_desejada
        return valor
    
    def returnMesureHeight(self):
        return self.altura_medida

    def handle_client(self, client_socket, client_id):
        while True:
            # Lê dados do cliente
            data = client_socket.recv(1024).decode('utf-8')

            if not data:
                break

            # print(f'Mensagem: {client_id}): {data}')
            

            try:
                received = float(data)
                self.altura_medida = received
            #     # print(f'Altura medida (cliente: {client_id}): {received}')
            #     if(client_id==0):
            #         # print("Altura medida: ", self.altura_medida)
            #         self.altura_medida = received
            #     else:
            #         # print(f'Altura desejada (cliente: {client_id}): {received}')
            #         self.altura_desejada = received


            except ValueError:
                pass
                # print(f"Erro ao converter altura medida para float")


        print(f"Cliente {client_id} desconectado.")
        client_socket.close()

    def start_server(self,):

        # get the hostname
        host = '192.168.43.158' 
        port = 5002  # initiate port no above 1024

        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # get instance
        print("Server running")
        # look closely. The bind() function takes tuple as argument
        server_socket.bind((host, port))  # bind host address and port together

        # configure how many client the server can listen simultaneously
        server_socket.listen(2)

        client_id = 0

        while True:
            client_socket, addr = server_socket.accept()

            print(f"Conexão estabelecida com {addr}")

            # Inicia uma nova thread para lidar com o cliente
            threading.Thread(target=self.handle_client, args=(client_socket, client_id)).start()

            client_id += 1

if __name__ == "__main__":
    server = SocketServer()
    server.start_server()
    # pass
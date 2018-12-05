import socket,threading

host = ''
port = 23
clients = []

class ThreadedServer(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))

    def listen(self):
        self.sock.listen(5)
        while True:
            client, address = self.sock.accept()
            clients.append(client)
            print("{} connected".format(address))
            client.settimeout(60)
            threading.Thread(target = self.listenToClient,args = (clients,client,address)).start()

    def listenToClient(self,clients ,client, address):
        size = 1024
        while True:
            try:
                data = client.recv(size)
                if data:
                    # Set the response to echo back the recieved data 
                    response = data
                    print("Recived data is {}".format(data))
                    for x in clients:
                        x.send(response)
                    #client.send(response)
                else:
                    raise error('Client disconnected')
            except:
                print("{} CLOSE".format(address))
                clients.remove(client)
                client.close()
                return False

if __name__ == "__main__":
    ThreadedServer(host,port).listen()

'''
def listening(clients,client, address):
    size = 1024
    while True:
        try:
            data = client.recv(size)
            if data:
                # Set the response to echo back the recieved data 
                response = data
                print("Recived data is {}".format(data))
                #client.send(response)
                broadcast(clients, response)
                return response
            else:
                raise error('Client disconnected')
        except:
            client.close()
               
def broadcast(clients, response):
    for client in clients:
        client.send(response)           

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((host, port))
sock.listen(5)

while True:
    client, address = sock.accept()
    clients.append(client)
    print("{} connected".format(address))
    client.settimeout(60)
    response = listening(clients, client, address)
    #broadcast(clients, response)
'''
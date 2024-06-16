import socket
import threading

# Define the TCP server address and port
SERVER_ADDRESS = '192.168.0.100'
SERVER_PORT = 12345

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((SERVER_ADDRESS, SERVER_PORT))

# Listen for incoming connections
server_socket.listen(5)
print(f"Server listening on {SERVER_ADDRESS}:{SERVER_PORT}")

clients = []

# Client handling function
def handle_client(client_socket):
    clients.append(client_socket)
    while True:
        try:
            data = client_socket.recv(1024)
            if not data:
                break
        except:
            break
    clients.remove(client_socket)
    client_socket.close()

# Function to accept client connections
def accept_clients():
    while True:
        client_socket, client_address = server_socket.accept()
        print(f"Connection from {client_address}")
        threading.Thread(target=handle_client, args=(client_socket,), daemon=True).start()

# Start the thread to accept clients
threading.Thread(target=accept_clients, daemon=True).start()

try:
    while True:
        # Get user input and send it to all connected clients
        message = input("Enter message to send to client: ")
        if message:
            for client_socket in clients:
                try:
                    client_socket.sendall(message.encode())
                except:
                    clients.remove(client_socket)
except KeyboardInterrupt:
    print("Server is shutting down...")
    for client_socket in clients:
        client_socket.close()
    server_socket.close()

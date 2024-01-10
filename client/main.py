import requests
import socket
import time
import keyboard

def send_command(cmd):
    url = f"http://{esp_ip}/{cmd}"
    try:
        response = requests.get(url)
        if response.status_code != 200:
            print("Failed to send command")
        return response.text
    except Exception as e:
        print(f"Error: {e}")

esp_ip = '192.168.1.149'
user_choice = 5  # Changed variable name

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8080))
server_socket.listen(1)

while user_choice != 0:
    print("*****************************")
    print("Select command: ")
    print("1 - Stop Motors")
    print("2 - Heading Hold")
    print("3 - Start Flightplan")
    print("4 - Do Command 4")
    print("5 - logs")
    print("0 - Exit.")
    print("*****************************")
    user_choice = int(input())  # Now it should work correctly
    
    match user_choice:
        case 1:
            send_command("CMD1")
        case 2:
            send_command("CMD2")
        case 3:
            send_command("CMD3")
        case 4:
            send_command("CMD4")
        case 5:
            while not keyboard.is_pressed('esc'):
                result = send_command("CMD5");
                rpy = result.split(',')
                print("[Press 'esc' to exit] -- Roll = " + rpy[0] + "   Pitch = " + rpy[1] + "    Yaw = " + rpy[2].replace('\n',''))
                time.sleep(0.25)

        case 0:
            print("Exiting...")

user_choice = 0

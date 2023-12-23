import requests

def send_command(cmd):
    url = f"http://{esp_ip}/{cmd}"
    try:
        response = requests.get(url)
        if response.status_code == 200:
            print("Command sent successfully")
        else:
            print("Failed to send command")
    except Exception as e:
        print(f"Error: {e}")

esp_ip = '192.168.1.149'
user_choice = 5  # Changed variable name
while user_choice != 0:
    print("*****************************")
    print("Select command: ")
    print("1 - Do Command 1")
    print("2 - Do Command 2")
    print("3 - Do Command 3")
    print("4 - Do Command 4")
    print("5 - Do Command 5")
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
            send_command("CMD5")
        case 0:
            print("Exiting...")

user_choice = 0  # Reset the variable if needed

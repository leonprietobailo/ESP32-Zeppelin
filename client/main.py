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


input = "";
esp_ip = '192.168.1.149'
while input != 0:
    print("*****************************")
    print("Select command: ")
    print("1 - Do Command 1")
    print("2 - Do Command 2")
    print("3 - Do Command 3")
    print("4 - Do Command 4")
    print("5 - Do Command 5")
    print("0 - Exit.")

    input = input();
    
    match input:
        case "1":
            send_command("CMD1")
        case "2":
            send_command("CMD2")
        case "3":
            send_command("CMD3")
        case "4":
            send_command("CMD4")
        case "5":
            send_command("CMD5")
        case "0":
            print("Exitting...")

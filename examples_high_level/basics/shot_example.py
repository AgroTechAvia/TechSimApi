from agrotechsimapi import HighLevelSimClient
import time

ip = '127.0.0.1'
port = "1233"

color_red = [255, 0, 0]
color_green = [0, 255, 0]
color_blue = [0, 0, 255]

color_black = [0, 0, 0]

def main():
    client = HighLevelSimClient()

    client.connect(ip, port)
    time.sleep(2.0)
    # взлет
    client.takeoff()
    time.sleep(7.0)

    try:

        while True:
            print(client.setShoot(0.15))
            time.sleep(1)
    except Exception as err:
        print(f"[ERROR] {err}")

        client.disconnect()


if __name__ == "__main__":
    main()
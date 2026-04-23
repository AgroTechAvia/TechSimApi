from agrotechsimapi import HighLevelSimClient
import time

ip = '127.0.0.1'
port = "1233"


def main():
    client = HighLevelSimClient()

    client.connect(ip, port)

    time.sleep(5)

    try:

        while True:
            # client.setDiod(id, r, g, b)
            client.setDiod(0, 255, 0, 0)
            time.sleep(1)
            client.setDiod(0, 0, 255, 0)
            time.sleep(1)
            client.setDiod(0, 0, 0, 255)
            time.sleep(1)
            client.setDiod(0, 0, 0, 0)
            time.sleep(1)
    except Exception as err:
        print(f"[ERROR] {err}")

        client.disconnect()


if __name__ == "__main__":
    main()
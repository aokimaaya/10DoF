import os

from tool.dym_setup import Control
from utils.save_data import RecordData

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

else:
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def data_collection():
    DymControl = Control()
    recordData = RecordData(100, 5, DymControl)
    seq_num = 0

    try:
        while True:
            input_command = input(
                'First move to goal position then Press enter to start, "i" to initialize, "r" to record, "e" to finish. '
            )

            # Change Initial Position
            if input_command == "i":
                # Enable Servo Motors
                DymControl.Enable("xm540")

                # Initialize Position
                try:
                    goal = [2048, 2048, 2048]
                    DymControl.movej(goal, "xm540")
                    run = 1
                    while run:
                        run = DymControl.reach(goal, "xm540")

                except KeyboardInterrupt:
                    print("Interrupted")
                    DymControl.Disable("xm540")
                    input_command = ""

            # Start Data Collection
            elif input_command == "r":
                # Create Directory
                csv_path, image_path = recordData.create_datadir(seq_num)

                # Record Data
                # Disble Torque
                DymControl.Disable("xm540")

                # Record
                for i in range(100):
                    recordData.record(image_path, "xm540")
                    print(i)

                # Save Data
                recordData.save_data(csv_path)

                seq_num += 1

            elif input_command == "e":
                exit()
    except KeyboardInterrupt:
        print("Interrupted")
        DymControl.Disable("xm540")
        input_command = ""
    finally:
        print("Finish Data Collection.")


if __name__ == "__main__":
    data_collection()

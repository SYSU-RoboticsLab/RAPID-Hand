import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from control.rapid_hand_control.pcb_comm.serial_comm import PCBSerialInterface


class TactileVisualizer:
    """
    Realtime visualization for tactile sensor arrays across 5 fingers.
    Each sensor array is 12x8.
    """

    def __init__(self, hand: PCBSerialInterface):
        self.hand = hand
        self.fig, self.axes = plt.subplots(1, 5, figsize=(15, 3))
        self.images = []

        for i, ax in enumerate(self.axes):
            img = ax.imshow(np.zeros((12, 8)), cmap="viridis", vmin=0, vmax=200)
            ax.set_title(f"Finger {i + 1}")
            ax.axis("off")
            self.images.append(img)

    def update(self, frame):
        """
        Called periodically by FuncAnimation to update sensor data.
        """
        try:
            # Read tactile data and reshape to (5 fingers, 12 rows, 8 columns)
            data = self.hand.trigger_and_read().reshape(5, 12, 8)
            data = np.flip(data, axis=(1, 2))  # Flip for proper orientation

            for i in range(5):
                self.images[i].set_data(data[i])

        except Exception as e:
            print(f"[Warning] Failed to read tactile data: {e}")
        return self.images

    def run(self):
        """
        Start the real-time animation loop.
        """
        ani = animation.FuncAnimation(
            self.fig, self.update, interval=100, blit=False
        )
        plt.suptitle("Tactile Sensor Real-Time Visualization")
        plt.tight_layout()
        plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tactile Sensor Visualization Tool")
    parser.add_argument(
        "-p", "--pcb_port",
        type=str,
        default="/dev/ttyCH341USB0",
        help="Serial port for communicating with the tactile sensor PCB (e.g., /dev/ttyUSB0)"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        with PCBSerialInterface(args.pcb_port) as hand:
            visualizer = TactileVisualizer(hand)
            visualizer.run()
    except Exception as e:
        print(f"[Error] Failed to initialize PCB interface: {e}")


if __name__ == "__main__":
    main()

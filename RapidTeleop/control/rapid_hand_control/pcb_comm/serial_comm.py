import serial
import numpy as np


class PCBSerialInterface:
    """
    Pure Python-based tactile interface for a 5-finger robotic hand.
    
    - Serial configuration: 921600 bps, 8N1, no flow control
    - Send 0xFF to trigger MCU tactile sampling
    - Expected response: 5 blocks of 96 bytes each, 
      followed by 1 tail byte (0xFF) per block
    """
    SIGNAL = b'\xff'
    NUM_FINGERS = 5
    NUM_PER_FINGER = 96
    NUM_TACTILE = NUM_FINGERS * NUM_PER_FINGER

    def __init__(self, port: str, *, timeout: float = 1.0):
        self.serial = serial.Serial(
            port=port,
            baudrate=921600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            write_timeout=timeout,
        )

    def _read_exactly(self, size: int) -> bytes:
        """
        Read exactly `size` bytes from serial port.
        Raises IOError on timeout.
        """
        buffer = bytearray()
        while len(buffer) < size:
            chunk = self.serial.read(size - len(buffer))
            if not chunk:
                raise IOError("Serial timeout while reading data")
            buffer.extend(chunk)
        return bytes(buffer)

    def trigger_and_read(self) -> np.ndarray:
        """
        Trigger MCU to sample and read tactile data.

        Returns:
            np.ndarray: A (5, 96) uint8 tactile array.
        """
        # Trigger MCU
        self.serial.write(self.SIGNAL)
        self.serial.flush()

        # Read tactile data
        raw = bytearray()
        for finger_index in range(self.NUM_FINGERS):
            raw.extend(self._read_exactly(self.NUM_PER_FINGER))  # 96-byte tactile block
            tail = self._read_exactly(1)                         # 1-byte tail marker
            if tail != self.SIGNAL:
                raise ValueError(f"Invalid frame tail at finger {finger_index + 1}: got {tail!r}")

        # Convert to NumPy array
        tactile_data = np.frombuffer(raw, dtype=np.uint8)
        return tactile_data.reshape((self.NUM_FINGERS, self.NUM_PER_FINGER))

    def close(self):
        """Close serial port if open."""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit: close port."""
        self.close()


import wiringpi as wp
import time
import sys
import os


class Spin(object):
    def __init__(self, out):
        self.out = out
        wp.wiringPiSetup()
        wp.pinMode(self.out, wp.INPUT)
        return

    def read_Data(self):
        return wp.digitalRead(self.out)


if __name__ == "__main__":
    out1 = 25
    spin1 = Spin(out1)
    while(True):
        print(spin1.read_Data())
        # time.sleep(0.5)

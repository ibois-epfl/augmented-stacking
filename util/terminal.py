'''
Module containing utility functions for terminal interaction.
'''

import platform
import subprocess
import colorama as cr


CLR_USER_INPUT = cr.Fore.YELLOW
CLR_GENERAL_MSG = cr.Fore.GREEN
CLR_ERROR_MSG = cr.Fore.RED


def user_input(terminal_msg):
    """
    Ask for user input and return the input as a string

    param terminal_msg: message to be displayed in the terminal
    return: user input as a string
    """

    cr.init()
    print(CLR_USER_INPUT)
    user_input = input(f'{terminal_msg}')
    print(cr.Style.RESET_ALL)
    cr.deinit()

    return user_input

def error_print(terminal_msg):
    """
    Print an error message

    param terminal_msg: message to be displayed in the terminal
    """
    cr.init()
    print(CLR_ERROR_MSG)
    print(terminal_msg)
    print(cr.Style.RESET_ALL)
    cr.deinit()

def custom_print(terminal_msg):
    """
    Print a custom message

    param terminal_msg: message to be displayed in the terminal
    """
    
    cr.init()
    print(CLR_GENERAL_MSG)
    print(terminal_msg)
    print(cr.Style.RESET_ALL)
    cr.deinit()

def cat(path_file):
    """
    Used for display logo and credits.
    """
    if 'Linux' in platform.system():
        cmd = f'cat {path_file}'
        p = subprocess.Popen(cmd, shell=True)
        out, err = p.communicate()
    else:
        print("Augmented Stacking is not supported on this platform")
        os.exit(1)



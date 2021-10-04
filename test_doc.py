import random


def sign_of_num(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0


def auto(port_6=False, port_7=False):
    # Status of lower tower.
    if port_7:
        # Test this out and make changes if necessary. This is a CW rotation.
        zero_position = 1.25
    # Status of higher tower
    elif port_6:
        # Test this out and make changes if necessary. This is a CW rotation.
        zero_position = 1.75
    else:
        return "No port is on!"

    rand_position = round(random.uniform(-100., 100.), 3)

    remainder = abs(rand_position) % 2

    total_rotation = round(zero_position - remainder, 3)
    if total_rotation > 1 or total_rotation < -1:
        total_rotation = round(total_rotation - 2 * sign_of_num(total_rotation), 3)

    return rand_position, total_rotation


print(auto(port_6=True))

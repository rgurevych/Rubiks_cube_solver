from kociemba import solve
import serial
import time

def modify_input(cube):
    modified_colors = ["U", "R", "F", "D", "L", "B"]
    sides_colors = [cube[4], cube[13], cube[22], cube[31], cube[40], cube[49]]
    modified_cube = ""
    for i in range(0, 54):
        modified_cube += modified_colors[sides_colors.index(cube[i])]
    return modified_cube

def modify_solution(solution):
    solution_list = [step for step in solution.split(" ")]
    arduino_cube = ""
    possible_steps = ["U", "U'", "U2", "R", "R'", "R2", "L", "L'", "L2", "F", "F'", "F2", "D", "D'", "D2", "B", "B'", "B2"]
    modified_steps = ["U", "u", "UU", "R", "r", "RR", "L", "l", "ll", "F", "f", "FF", "D", "d", "DD", "B", "b", "BB"]
    for step in solution_list:
        arduino_cube += modified_steps[possible_steps.index(step)]
    return arduino_cube

def send_solution(arduino_cube):
    time.sleep(1)
    port.write(b'!')
    print("Ready signal was sent to Arduino")
    time.sleep(1)
    conf_received = False
    while conf_received == False:
        text = port.read(1).decode()
        if text == "!":
            conf_received = True
            print("confirmation received")
        else:
            port.reset_input_buffer()
    port.write(arduino_cube.encode())
    print("Solution was sent to Arduino")
    #while conf_received == False:
    response = port.read(2).decode()
    if int(response) == len(arduino_cube):
        port.write(b'+')
    else:
        port.write(b'x')
    time.sleep(1)
    #port.write(b'+')
    print("Data sent successfully")


def read_cube():
    #with port:
    request_received = False
    while request_received == False:
        text = port.read(1).decode()
        if text == "!":
            request_received = True
            print("request received")
        else:
            port.reset_input_buffer()
    print("connection established")
    time.sleep(1)
    port.write(b'!')
    cube = port.read(54).decode()
    #cube = raw_cube[2:]
    print("Cube received: " + cube)
    time.sleep(1)
    if len(cube) == 54:
        port.write(b'+')
    else:
        port.write(b'x')
    return(cube)



port = serial.Serial('COM5', 9600)

scanned_cube = read_cube()
#scanned_cube = "woybogwrybwbbwggygoyoogoryryowbrgyrwbybbyggwgrwrrbrowo"

modified_cube = modify_input(scanned_cube)
#print(modified_cube)
solution = solve(modified_cube)
print("Solution: " + solution)
arduino_cube = modify_solution(solution)
#arduino_cube = "rLUdBBUUllFFllUUllFF"
print("Modified solution: " + arduino_cube)
send_solution(arduino_cube)





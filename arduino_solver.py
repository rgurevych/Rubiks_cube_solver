from kociemba import solve
from cube_solver import solver
import serial


port = serial.Serial('COM5', 9600)
while True:
    print ("Ready for Arduino")
    scanned_cube = solver.read_cube(port)
    #scanned_cube = "woybogwrybwbbwggygoyoogoryryowbrgyrwbybbyggwgrwrrbrowo"
    modified_cube = solver.modify_input(scanned_cube)
    #print(modified_cube)
    solution = solve(modified_cube)
    print("Solution: " + solution)
    arduino_cube = solver.modify_solution(solution)
    #arduino_cube = "rLUdBBUUllFFllUUllFF"
    print("Modified solution: " + arduino_cube)
    solver.send_solution(arduino_cube, port)





from kociemba import solve


def modify_input(cube):
    modified_colors = ["U", "R", "F", "D", "L", "B"]
    sides_colors = [cube[4], cube[13], cube[22], cube[31], cube[40], cube[49]]
    modified_cube = ""
    for i in range(0, 54):
        modified_cube += modified_colors[sides_colors.index(cube[i])]
    return modified_cube

def modify_solution(solution):
    solution_list = [step for step in solution.split(" ")]
    print(solution_list)
    arduino_cube = ""
    possible_steps = ["U", "U'", "U2", "R", "R'", "R2", "L", "L'", "L2", "F", "F'", "F2", "D", "D'", "D2", "B", "B'", "B2"]
    modified_steps = ["U", "u", "UU", "R", "r", "RR", "L", "l", "ll", "F", "f", "FF", "D", "d", "DD", "B", "b", "BB"]
    for step in solution_list:
        arduino_cube += modified_steps[possible_steps.index(step)]
    return arduino_cube

scanned_cube = "woybogwrybwbbwggygoyoogoryryowbrgyrwbybbyggwgrwrrbrowo"
#scanned_cube = "DRLUUBFBRBLURRLRUBLRDDFDLFUFUFFDBRDUBRUFLLFDDBFLUBLRBD"
modified_cube = modify_input(scanned_cube)
print(modified_cube)
solution = solve(modified_cube)
print(solution)
arduino_cube = modify_solution(solution)
print(arduino_cube)





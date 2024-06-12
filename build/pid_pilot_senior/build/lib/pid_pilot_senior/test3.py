import numpy as np

pid_reference_counter = 0
pos = 0.5
pos2 = 0.9

if pid_reference_counter == 0:  
    orientation_reference_maximum = pos + 0.3
    orientation_reference_minimum = pos - 0.3
    if orientation_reference_maximum >= 2 * np.pi:
        orientation_reference_maximum = orientation_reference_maximum - (2 * np.pi)
    if orientation_reference_minimum < 0:
        orientation_reference_minimum = orientation_reference_minimum + (2 * np.pi)

    print(orientation_reference_maximum > pos2)
    print(pos2 > (orientation_reference_minimum))
    print(orientation_reference_maximum)
    print(orientation_reference_minimum)
    
    while(not (orientation_reference_maximum > pos2 or pos2 > (orientation_reference_minimum))):
        print(" running")
        # print("orientation_reference_minimum= " + str(orientation_reference_minimum))
        # print("pos = " + str(pos))
        # print("first loop correcting orientation")


import numpy as np
import matplotlib.pyplot as plt

with open("../build/feature_status.txt") as file:
    
    i = 0
    collect_velocity = False

    
    while True:
        content = file.readline()
        if content:
            if content[:10] == "Feature ID":
                if int(content[11:]) == 0:
                    collect_velocity = True
                    continue
            if (content[:4] == "Flow") and (collect_velocity == True):
                print(content[13:24])
                ux = float(content[13:24])
                
                print(content[25:])
                uy = float(content[25:])
                
                collect_velocity = False
                
            # print(content[:3])



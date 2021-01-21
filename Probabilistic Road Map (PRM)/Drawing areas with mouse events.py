## Usage hold down left mouse click and drag


import matplotlib.pyplot as plt
import numpy


#    _____                              _                   _   
#   |_   _|                            (_)                 | |  
#     | |  _ __ ___   __ _  __ _  ___   _ _ __  _ __  _   _| |_ 
#     | | | '_ ` _ \ / _` |/ _` |/ _ \ | | '_ \| '_ \| | | | __|
#    _| |_| | | | | | (_| | (_| |  __/ | | | | | |_) | |_| | |_ 
#   |_____|_| |_| |_|\__,_|\__, |\___| |_|_| |_| .__/ \__,_|\__|
#                           __/ |              | |              
#                          |___/               |_|              

image = plt.imread("Figure1.png");
plt.imshow(image)
ax  = plt.gca()
fig1 = plt.gcf()
ax.invert_yaxis()

#    ______               _     _       _                           _        _   _             
#   |  ____|             | |   (_)     | |                         | |      | | (_)            
#   | |____   _____ _ __ | |_   _ _ __ | |_ ___ _ __ _ __  _ __ ___| |_ __ _| |_ _  ___  _ __  
#   |  __\ \ / / _ \ '_ \| __| | | '_ \| __/ _ \ '__| '_ \| '__/ _ \ __/ _` | __| |/ _ \| '_ \ 
#   | |___\ V /  __/ | | | |_  | | | | | ||  __/ |  | |_) | | |  __/ || (_| | |_| | (_) | | | |
#   |______\_/ \___|_| |_|\__| |_|_| |_|\__\___|_|  | .__/|_|  \___|\__\__,_|\__|_|\___/|_| |_|
#                                                   | |                                        
#                                                   |_|                                        


coordinates = numpy.array([])
def mouse_drag(events):
    
    if events.button == 1: ## 
        global coordinates
        
        x = events.xdata
        y = events.ydata

        coordinates = numpy.append(coordinates, x)
        coordinates = numpy.append(coordinates, y)
        if numpy.size(coordinates) == 4:
            plt.plot((coordinates[0], coordinates[2]), (coordinates[1], coordinates[3]), "--r")
            a = coordinates
            coordinates = numpy.array([])
            plt.show()
            pass


#    ______               _     _                     _ _ _             
#   |  ____|             | |   | |                   | | (_)            
#   | |____   _____ _ __ | |_  | |__   __ _ _ __   __| | |_ _ __   __ _ 
#   |  __\ \ / / _ \ '_ \| __| | '_ \ / _` | '_ \ / _` | | | '_ \ / _` |
#   | |___\ V /  __/ | | | |_  | | | | (_| | | | | (_| | | | | | | (_| |
#   |______\_/ \___|_| |_|\__| |_| |_|\__,_|_| |_|\__,_|_|_|_| |_|\__, |
#                                                                  __/ |
#                                                                 |___/ 


#linetype = input("Enter suitable linetype. Selections: -, --, -., . ")
#linecolor = input("Enter suitable linecolors. Selection: b, g, m, y, r, k")
        
        
fig1.canvas.mpl_connect('button_press_event', mouse_drag)
#fig1.canvas.mpl_connect('motion_notify_event', onclick)
fig1.canvas.mpl_connect('button_release_event', mouse_drag)

plt.show()
    
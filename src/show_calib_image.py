
import sys
from PIL import Image
from PIL import ImageTk
if sys.version_info[0] == 2:  # the tkinter library changed it's name from Python 2 to 3.
    import Tkinter
    tkinter = Tkinter #I decided to use a library reference to avoid potential naming conflicts with people's programs.
else:
    import tkinter
    
# Calibration image path
CALIB_IMG = "calibration_grid.png"

root = tkinter.Tk()
w, h = root.winfo_screenwidth(), root.winfo_screenheight()
root.geometry("%dx%d+0+0" % (w, h))
root.attributes('-fullscreen', True)
root.bind('<Escape>', lambda e: root.quit())
lmain = tkinter.Label(root)
lmain.pack()
Calib_img = Image.open(CALIB_IMG)
lmain.imgtk = imgtk = ImageTk.PhotoImage(image=Calib_img)
lmain.configure(image=imgtk)
root.mainloop()

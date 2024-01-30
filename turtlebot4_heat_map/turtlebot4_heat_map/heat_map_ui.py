# Import Required Libraries
import os
import glob
from tkinter import *
from tkinter import ttk
from PIL import Image,ImageTk
from datetime import datetime
from tkinter import colorchooser
from tkcalendar import Calendar

import rclpy
import maps_over_time,multiple_maps
from ament_index_python import get_package_share_directory

class App(Tk):
    def __init__(self):
        super().__init__()        
        # Configure the main window
        self.title("Heat Map UI")
        self.geometry("1350x1200+350+350")
        self.date_dict = {}
        
        # Set the default colors for the multiple paths map
        self.color_dict = {
            "color_1"   : (0,0,255),        
            "color_2"   : (0,255,0),    
            "color_3"   : (255,0,0),    
            "color_4"   : (0,255,255),    
            "color_5"   : (255,0,255),    
            "color_6"   : (255,255,0)}

        self.initialize_ui()
        self.map_tabs()

    def initialize_ui(self):
        # Get a list of all entries in the specified directory
        heat_map_package = os.path.join(os.path.expanduser("~"), "cleaning_data")
        all_entries = os.listdir(heat_map_package)

        # Filter out subdirectories and files
        self.directories = [entry for entry in all_entries if entry != "saved_goals"]

        # Initialize and make changeable texts
        start_date     = StringVar(self,value= "Start Date is:")
        end_date       = StringVar(self,value= "End Date is:")
        self.map_name  = StringVar(self,value= "068_map")
        self.map_count = StringVar(self,value= "")

        # Create the UI elements in the main window
        Label       (self, text= "Map select",           anchor = "w", justify= "left").grid(sticky= W, column= 0, row= 2)
        Label       (self, textvariable= start_date,     anchor = "w", justify= "left").grid(sticky= W, column= 0, row= 0)
        Label       (self, textvariable= end_date,       anchor = "w", justify= "left").grid(sticky= W, column= 0, row= 1)
        Label       (self, textvariable= self.map_count, anchor = "w", justify= "left").grid(sticky= W, column= 3, row= 1)
        Button      (self, text= "Select date",          command= lambda: self.open_calendar(start_date, "start_date")).grid(sticky= W, column= 1, row= 0)
        Button      (self, text= "Select date",          command= lambda: self.open_calendar(end_date, "end_date"))    .grid(sticky= W, column= 1, row= 1)
        Button      (self, text= "Open maps",            command= self.maps_over_time, bg= 'red', activebackground= '#E33955', anchor= 'e', justify= "right").grid(sticky= E, column= 3, row= 0)
        ttk.Combobox(self, textvariable= self.map_name,  values = self.directories).grid(sticky= W, column= 1, row= 2)

    def map_tabs(self):
        live_map = os.path.join(os.path.expanduser('~'), 'cleaning_data', 'live_maps')
        check = glob.glob(os.path.join(live_map, str(datetime.now().date())+"*.png"))
        
        tab_states = ["disabled", "normal"]
        
        # Enable the live map if one is available currently or in the past, else disable the tab
        try:
            tab_state = tab_states[int(os.path.isfile(check[0]))]
        except IndexError:
            tab_state = tab_states[0]

        # Create the notebook tabs in the main window
        self.heat_maps = ttk.Notebook(name= "heat maps", width= 1200, height= 1000, padding= 5)

        self.tab1= Frame(self)
        self.tab2= Frame(self)
        self.tab3= Frame(self)
        self.tab4= Frame(self)
        self.heat_maps.add(self.tab1, text= "live map",           state= tab_state)
        self.heat_maps.add(self.tab2, text= "maps over time",     state= "disabled")
        self.heat_maps.add(self.tab3, text= "recent maps",        state= "disabled")
        self.heat_maps.add(self.tab4, text= "multiple heat maps", state= "normal")
        
        self.heat_maps.grid(column= 0, row= 3, columnspan= 6,rowspan= 8)        
        self.multiple_maps()
        self.update_live()

    def update_live(self):

        # Find the live map png
        live_map = os.path.join(os.path.expanduser('~'), 'cleaning_data', 'live_maps')
        map_file = glob.glob(os.path.join(live_map, str(datetime.now().date())+"*.png"))
        if map_file:

            # Show the png map in a canvas object in the tab
            self.live  = Image.open(map_file[0])
            self.live  = ImageTk.PhotoImage(self.live)   

            canvas_live = Canvas(self.tab1, width= 1250, height= 1100)
            canvas_live.pack(fill= "both")
            canvas_live.create_image(10, 10, anchor= NW, image= self.live)

    # Open the calendar and grab the selected date
    def open_calendar(self, text, key):
        # Set the date value for the date key
        def get_date():
            text.set(cal.get_date())
            full_datetime = cal.selection_get()
            self.date_dict[key] = full_datetime
            window.destroy()

        # Configure the calendar window
        window = Toplevel(self)
        window.title   ("Calendar")
        window.geometry("400x400+400+400")
        
        # Set how the datetime value is returned
        cal = Calendar(window, selectmode= 'day', locale= 'en_UK')
        cal.pack(pady= 20)
        
        # Add Button and Label
        Button(window, text= "Get Date",command= get_date).pack(pady= 20)

        # Color chooser window
    def choose_color(self, key):
        self.color_code = colorchooser.askcolor(title= "Choose color") 
        self.color_dict[key] = self.color_code[0]

    def open_maps(self):
        # Open the heat maps and expand them as large as possible in the available space ##Maps that are too large break the code here
        #path_file = os.path.join(os.path.expanduser('~'), "maps")
        #pgm = Image.open(os.path.join(path_file, self.map_name.get()+'.pgm'))
        #check = pgm.height if pgm.width > pgm.height else pgm.width
        #inflate = 790//(pgm.width*6)

        # Find and show the png maps of each heat map over time
        self.time     = Image.open          (os.path.join(os.path.expanduser('~'), 'cleaning_data', self.map_name.get(), 'heat_maps', str(self.date_dict["start_date"])+' to '+str(self.date_dict["end_date"])+' over time.png'))
        #self.time     = self.time.resize    ((self.time.width*inflate, self.time.height*inflate))
        #self.time     = self.time.rotate    (90, expand= True) if check == pgm.height else self.time
        self.time     = ImageTk.PhotoImage  (self.time)       
        
        self.recent   = Image.open          (os.path.join(os.path.expanduser('~'), 'cleaning_data', self.map_name.get(), 'heat_maps', str(self.date_dict["start_date"])+' to '+str(self.date_dict["end_date"])+' recent.png'))
        #self.recent   = self.recent.resize  ((self.recent.width*inflate, self.recent.height*inflate))
        #self.recent   = self.recent.rotate  (90, expand= True) if check == pgm.height else self.recent
        self.recent   = ImageTk.PhotoImage  (self.recent)

    def make_multiple_maps(self):
        # Get the list of dates and colors for the multiple paths map, and get the map name
        list_of_dates = list(self.date_dict.values())
        color_options = list(self.color_dict.values())
        map_name      = self.multiple_map_name.get()
        multiple_maps.main(list_of_dates, map_name, color_options)

        #pgm       = Image.open(os.path.join(os.path.expanduser('~'), "maps", self.map_name.get()+'.pgm'))
        #check     = pgm.height if pgm.width > pgm.height else pgm.width
        #inflate   = 790//(pgm.width*6)

        # Find and show the multiple paths map
        self.multiple = Image.open        (os.path.join(os.path.expanduser('~'), 'cleaning_data', map_name, 'heat_maps', str(list_of_dates)+ " multiple maps.png"))
        #self.multiple = self.multiple.resize((self.multiple.width*inflate, self.multiple.height*inflate))
        #self.multiple = self.multiple.rotate(90, expand= True) if check == pgm.height else self.multiple
        self.multiple = ImageTk.PhotoImage(self.multiple)

        canvas_multiple= Canvas(self.tab4, width= 950, height= 900)
        canvas_multiple.pack(fill= "both")
        canvas_multiple.create_image(10, 10, anchor= NW, image= self.multiple)

    def multiple_maps(self):
        
        # Set string vars that update with a change of the date
        date_1        = StringVar(self, value= "Date 1")
        date_2        = StringVar(self, value= "Date 2")
        date_3        = StringVar(self, value= "Date 3")
        date_4        = StringVar(self, value= "Date 4")
        date_5        = StringVar(self, value= "Date 5")
        date_6        = StringVar(self, value= "Date 5")
        self.multiple_map_name = StringVar(self,value= "068_map")

        # Set the UI elements for the multiple paths map window
        Button(self.tab4, text= "Choose Color", command= lambda: self.choose_color("color_1")).place(x= 50 , y= 800)
        Button(self.tab4, text= "Choose Color", command= lambda: self.choose_color("color_2")).place(x= 200, y= 800)
        Button(self.tab4, text= "Choose Color", command= lambda: self.choose_color("color_3")).place(x= 350, y= 800)
        Button(self.tab4, text= "Choose Color", command= lambda: self.choose_color("color_4")).place(x= 500, y= 800)
        Button(self.tab4, text= "Choose Color", command= lambda: self.choose_color("color_5")).place(x= 650, y= 800)
        Button(self.tab4, text= "Choose Color", command= lambda: self.choose_color("color_6")).place(x= 800, y= 800)

        Button(self.tab4, textvariable= date_1, command= lambda: self.open_calendar(date_1, "date_1")).place(x= 50 , y= 850)
        Button(self.tab4, textvariable= date_2, command= lambda: self.open_calendar(date_2, "date_2")).place(x= 200, y= 850)
        Button(self.tab4, textvariable= date_3, command= lambda: self.open_calendar(date_3, "date_3")).place(x= 350, y= 850)
        Button(self.tab4, textvariable= date_4, command= lambda: self.open_calendar(date_4, "date_4")).place(x= 500, y= 850)
        Button(self.tab4, textvariable= date_5, command= lambda: self.open_calendar(date_5, "date_5")).place(x= 650, y= 850)
        Button(self.tab4, textvariable= date_6, command= lambda: self.open_calendar(date_6, "date_6")).place(x= 800, y= 850)

        Button      (self.tab4, text= "Generate Map", command= self.make_multiple_maps, bg= 'red', activebackground= '#E33955').place(x=50, y=950)
        ttk.Combobox(self.tab4, textvariable= self.multiple_map_name, values= self.directories).place(x= 500, y= 950)
        
    # Call the maps over time making scripts
    def maps_over_time(self):
        if self.date_dict["start_date"] > self.date_dict ["end_date"]:
            self.date_dict["end_date"], self.date_dict["start_date"] = self.date_dict["start_date"], self.date_dict["end_date"]

        # Show the number of maps loaded
        map_count = maps_over_time.main(self.date_dict["start_date"], self.date_dict["end_date"], self.map_name.get())
        self.map_count.set("number of maps loaded: "+str(map_count))
        
        # Let the tabs be available to click on
        self.heat_maps.add(self.tab2,  state= "normal")
        self.heat_maps.add(self.tab3,  state= "normal")

        self.open_maps()
        
        # Show the png maps on the canvas objects for the heat maps over time
        canvas_time     = Canvas(self.tab2, width= 1000, height= 900)
        canvas_recent   = Canvas(self.tab3, width= 1000, height= 900)

        canvas_time  .pack(fill= "both")
        canvas_recent.pack(fill= "both")

        canvas_time  .create_image(10, 10, anchor= NW, image= self.time)
        canvas_recent.create_image(10, 10, anchor= NW, image= self.recent)

        # Show a color bar for reference in the tabs
        self.color_bar = Image.open(os.path.join(get_package_share_directory("turtlebot4_heat_map"), "resources", "colorbar.png"))
        self.color_bar = self.color_bar.resize((50, 250))
        self.color_bar = ImageTk.PhotoImage(self.color_bar)
        canvas_time.create_image  (990, 10, anchor= NE, image= self.color_bar)
        canvas_recent.create_image(990, 10, anchor= NE, image= self.color_bar)

def main():
    rclpy.init()

    app = App()
    app.mainloop()

# Execute Tkinter
if __name__ == "__main__":
    main()
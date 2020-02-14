import sys
import tkinter as tk
import tkinter.ttk as ttk
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter import messagebox

import numpy as np
import pandas as pd
import os

import serial
import serial.tools.list_ports
from serial.serialutil import SerialException

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk



class MainApplication:

    def __init__(self, parent):
        self.parent = parent

        _bgcolor = '#d9d9d9'  # X11 color: 'gray85'
        _fgcolor = '#000000'  # X11 color: 'black'
        _compcolor = '#d9d9d9' # X11 color: 'gray85'
        _ana1color = '#d9d9d9' # X11 color: 'gray85'
        _ana2color = '#ececec' # Closest X11 color: 'gray92'

        style.use('bmh')
        # style.use('ggplot')
        # style.use('fivethirtyeight')

        self.fig = Figure(dpi=75)
        self.a = self.fig.add_subplot(111)

        self.available_ports = []
        self.current_port = None
        self.baudrate = 115200
        self.bytesize = serial.EIGHTBITS
        self.timeout = None
        self.interval = 50
        self.xcount = 1000

        self.dt = np.dtype([
            ('GPS Breitengrad', np.float64), ('GPS Längengrad', np.float64), ('GPS Höhe (m)', np.float32), ('GPS Geschw. (km/h)', np.float32),
            ('GPS Jahr', np.uint16), ('GPS Monat', np.uint8), ('GPS Tag', np.uint8),
            ('GPS Stunde', np.uint8), ('GPS Minunte', np.uint8), ('GPS Sekunde', np.uint8),
            ('GPS Satelliten', np.uint8),
            ('Scheibentemp. Vorne (°C)', np.float64), ('Belagtemp. Vorne (°C)', np.float64), ('Umgebungstemp 1 (°C)', np.float64), ('Umgebungstemp 2 (°C)', np.float64),
            ('CanId0x101', np.uint64),
            ('Zeitstempel', np.uint32),
            ('Bremsdruck Vorne (bar)', np.uint16), ('Bremsdruck Hinten (bar)', np.uint16),
            ('Raddrehzahl Vorne (1/min)', np.uint32),
            ('Bremsstatus', np.uint8)
            ], align=True)

        self.data = pd.DataFrame([])
        self.default_columns = ['Bremsdruck Vorne (bar)', 'Raddrehzahl Vorne (1/min)', 'Belagtemp. Vorne (°C)']

        self.parent.geometry("1200x800+50+50")
        self.parent.minsize(1200, 800)
        self.parent.maxsize(3004, 1959)
        self.parent.resizable(1, 1)
        self.parent.title("Parser GUI")
        tk.Tk.iconbitmap(self.parent, default='static/logo.ico')
        self.parent.configure(background=_bgcolor)

        ###################################################
        ################# Plot - Optionen #################
        ###################################################

        self.plot_options = tk.LabelFrame(self.parent)
        self.plot_options.place(relx=0.01, rely=0.01, relheight=0.975, relwidth=0.2)
        self.plot_options.configure(relief='groove', text='Plot Optionen', background=_bgcolor)

        #################################################
        ################# Konvertierung #################
        #################################################

        self.conversion = tk.LabelFrame(self.parent)
        self.conversion.place(relx=0.25, rely=0.01, relheight=0.15, relwidth=0.35)
        self.conversion.configure(relief='groove', text='Konvertieren', background=_bgcolor)

        self.load_csv_button = ttk.Button(self.conversion)
        self.load_csv_button.place(relx=0.01, rely=0.025)
        self.load_csv_button.configure(text='von CSV laden', command=lambda: self.load_csv())

        self.load_bin_button = ttk.Button(self.conversion)
        self.load_bin_button.place(relx=0.25, rely=0.025)
        self.load_bin_button.configure(text='von BIN laden', command=lambda: self.load_bin())

        self.save_csv_button = ttk.Button(self.conversion)
        self.save_csv_button.place(relx=0.74, rely=0.025)
        self.save_csv_button.configure(state='disabled', text='als CSV speichern', command=lambda: self.save_csv())

        ##################################################
        ################# Live - Logging #################
        ##################################################

        self.live_logging = tk.LabelFrame(self.parent)
        self.live_logging.place(relx=0.625, rely=0.01, relheight=0.15, relwidth=0.365)
        self.live_logging.configure(relief='groove', text='Live-Aufzeichnung', background=_bgcolor)

        self.start_button = ttk.Button(self.live_logging)
        self.start_button.place(relx=0.01, rely=0.025)
        self.start_button.configure(state='disabled', text='Start', command=lambda: self.start_live_logging())

        self.stop_button = ttk.Button(self.live_logging)
        self.stop_button.place(relx=0.225, rely=0.025)
        self.stop_button.configure(state='disabled', text='Stop', command=lambda: self.stop_live_logging())

        self.connect_button = ttk.Button(self.live_logging)
        self.connect_button.place(relx=0.62, rely=0.025)
        self.connect_button.configure(text='Verbinden mit Datenlogger', command=lambda: self.connect_to_port())

        #################################################
        ################# Graph - Frame #################
        #################################################

        self.graph_frame = tk.Frame(self.parent)
        self.graph_frame.place(relx=0.15, rely=0.075, relheight=0.91, relwidth=0.84)
        self.graph_frame.configure(relief='groove', borderwidth="2", background=_bgcolor)

        self.canvas = FigureCanvasTkAgg(self.fig, self.graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget()

        toolbar = NavigationToolbar2Tk(self.canvas, self.graph_frame)
        toolbar.update()
        self.canvas._tkcanvas.place(relx=0, rely=0, relheight=1, relwidth=1)


    def load_csv(self):
        csv_file_types = (('csv Dateien', '*.csv'), ('alle Dateien', '*.*'))
        filepath = askopenfilename(filetypes=csv_file_types)
        if filepath:
            if filepath.endswith('.csv'):
                self.current_file = os.path.basename(filepath)
                self.data = pd.read_csv(filepath, sep=';', decimal=',')
                self.data.set_index(['Zeitstempel'], 1, inplace=True)
                [self.data.drop([col], 1, inplace=True) for col in list(self.data) if 'Unnamed' in col]

                self.updateOptions(self.data)
                self.updateGraph(self.data)
                self.save_csv_button.configure(state='disabled')
            else: messagebox.showinfo('Achtung!', 'Dateiendung muss ".csv" sein!')


    def load_bin(self):
        bin_file_types = ("bin Dateien","*.bin, *.BIN"),("alle Dateien","*.*")
        filepath = askopenfilename(filetypes=(bin_file_types))
        if filepath:
            if filepath.endswith('.bin') or filepath.endswith('.BIN'):
                self.current_file = os.path.basename(filepath)

                df = pd.DataFrame(np.fromfile(filepath, dtype=self.dt))
                df.set_index(['Zeitstempel'], 1, inplace=True)
                df = self.convert_can_msg(df)
                self.data = self.format_data(df)

                self.updateOptions(self.data)
                self.updateGraph(self.data)

                self.save_csv_button.configure(state='enabled')
            else: messagebox.showinfo('Achtung!', 'Dateiendung muss ".bin" oder ".BIN" sein!')


    def save_csv(self):
        csv_file_types = (('csv Dateien', '*.csv'), ('alle Dateien', '*.*'))
        filepath = asksaveasfilename(filetypes=csv_file_types)
        if filepath:
            if filepath.endswith('.csv'):
                self.data.to_csv(filepath, sep=';', decimal=',')
            else: messagebox.showinfo('Achtung!', 'Dateiendung muss ".csv" sein!')



    def connect_to_port(self):

        self.available_ports = [comport.device for comport in serial.tools.list_ports.comports()]

        dialog = tk.Toplevel(self.parent)
        dialog.resizable(0, 0)

        radio_frame = tk.Frame(dialog)
        radio_frame.grid(row=0, column=0, sticky='nsew')

        self.radio_variable = tk.IntVar()
        self.radio_variable.set(0)
        for i, port in enumerate(self.available_ports):
            radio_button = tk.Radiobutton(radio_frame, text=port, variable=self.radio_variable, value=i)
            radio_button.grid(row=i+1, column=0, sticky='nsew', padx=10, pady=1)
        
        button_frame = tk.Frame(dialog)
        button_frame.grid(row=2, column=0, sticky='nsew')
        confirm_Button = ttk.Button(button_frame, text='Bestätigen', command=lambda: self.update_port(dialog))
        confirm_Button.grid(row=0, column=0, sticky='nsew')


    def update_port(self, parent):
        parent.destroy()
        self.current_port = self.available_ports[self.radio_variable.get()]
        if self.current_port:
            self.start_button.configure(state='enabled')


    def start_live_logging(self):
        try:
            self.esp32 = serial.Serial(port=self.current_port,
                                    baudrate=self.baudrate,
                                    bytesize=self.bytesize,
                                    timeout=self.timeout)
            
            self.start_button.configure(state='disabled')
            self.stop_button.configure(state='enabled')

            self.after_process = self.parent.after(ms=self.interval, func=lambda: self.read_serial())
        except SerialException:
            messagebox.showerror('Fehler!', f'{self.current_port} konnte nicht geöffnet werden!\nZeitlimit überschritten.')

        

    def stop_live_logging(self):
        self.stop_button.configure(state='disabled')
        self.start_button.configure(state='enabled')

        self.parent.after_cancel(self.after_process)
        self.esp32.close()


    def read_serial(self):
        if self.esp32.in_waiting:
            incomming_line = self.esp32.read(self.esp32.in_waiting)
            df = pd.DataFrame(np.frombuffer(incomming_line, dtype=self.dt), columns=self.dt.names)
            df.set_index(['Zeitstempel'], 1, inplace=True)
            df = self.convert_can_msg(df)
            df = self.format_data(df)

            if hasattr(self, 'live_data'):
                self.live_data = self.live_data.append(df)
            else:
                self.live_data = df
                self.updateOptions(self.live_data)

            self.updateGraph(self.live_data.tail(self.xcount))

        self.after_process = self.parent.after(ms=self.interval, func=lambda: self.read_serial())


    def convert_can_msg(self, df):
        can_msg_list = [can_msg for can_msg in list(df) if 'CanId' in can_msg]

        for can_msg in can_msg_list:
            can_msg_length = 18
            hex_list = [hex(val) for val in df[can_msg]]

            for idx, hex_str in enumerate(hex_list):
                # hex-strings einheitlich auffüllen
                while len(hex_str) < can_msg_length:
                    hex_str = '0x' + '0' + hex_str.split('x')[1]
                hex_str = hex_str + '0'     # Wird für späteren Zugriff über index benötigt
                hex_list[idx] = hex_str

            if '0x101' in can_msg:
                msg_content = {
                    'Ladestatus': [[-3, -1], [-5, -3]],
                    'Batteriestrom (mA)': [[-7, -5], [-9, -7]],
                    'Batterieleistung (W * 1/10)': [[-11, -9], [-13, -11]],
                    'Batteriespannung (V)': [[-15, -13], [-17, -15]],
                    }
                for key in msg_content:
                    index = msg_content.get(key)
                    can_msg_0x101 = ['0x' + hex_str[index[0][0]:index[0][1]] + hex_str[index[1][0]:index[1][1]] for hex_str in hex_list]
                    can_msg_0x101 = [int(hex_str, 0) for hex_str in can_msg_0x101]
                    df[key] = can_msg_0x101

            df.drop([can_msg], 1, inplace=True)

        return df


    def format_data(self, df):
        pressure_rate = 78.0
        radius = 0.35
        circumference = 2 * np.pi * radius
        total_flanks = 84

        df['Bremsdruck Vorne (bar)'] = df['Bremsdruck Vorne (bar)'] * pressure_rate * 3.3 / 4095
        df['Bremsdruck Hinten (bar)'] = df['Bremsdruck Hinten (bar)'] * pressure_rate * 3.3 / 4095
        df['Raddrehzahl Vorne (1/min)'] = 1e6 / (df['Raddrehzahl Vorne (1/min)'] * total_flanks)
        df['Batteriespannung (V)'] = df['Batteriespannung (V)'] / 1000


        return df


    def updateOptions(self, data_list):
        self.show_columns = {name: tk.BooleanVar() for name in list(data_list)}
        [self.show_columns[key].set(1) if key in self.default_columns
            else self.show_columns[key].set(0) for key in self.show_columns]

        for i, key in enumerate(self.show_columns):
            check_box = tk.Checkbutton(self.plot_options)
            check_box.configure(text=key, variable=self.show_columns[key], background='#d9d9d9')
            check_box.configure(command=lambda: self.updateGraph(data_list))
            check_box.grid(row=i, column=0, sticky='w', padx=5, pady=1)


    def updateGraph(self, data_type):
        self.a.clear()

        show_this = [key for key in self.show_columns if self.show_columns[key].get() == 1]
        if show_this:
            self.a.plot(data_type.index, data_type[show_this])
        try:
            self.a.set_title(self.current_file)
        except AttributeError:
            self.a.set_title('Live Aufzeichnung')
        self.a.set_xlabel('Zeit in ms')
        self.a.legend(show_this)

        self.canvas.draw()



if __name__ == '__main__':
    root = tk.Tk()
    MainApplication(root)
    root.mainloop()
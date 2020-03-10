import sys
import tkinter as tk
import tkinter.ttk as ttk
from tkinter.filedialog import askopenfilename, asksaveasfilename
from tkinter import messagebox

import numpy as np
import pandas as pd
import os
import webbrowser
import pathlib



import serial
import serial.tools.list_ports as list_com_ports
from serial.serialutil import SerialException

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import style
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk



class DataloggerGUI:

    def __init__(self, parent):
        self.parent = parent

        self._bgcolor = '#d9d9d9'  # X11 color: 'gray85'
        self._fgcolor = '#000000'  # X11 color: 'black'
        self._compcolor = '#d9d9d9' # X11 color: 'gray85'
        self._ana1color = '#d9d9d9' # X11 color: 'gray85'
        self._ana2color = '#ececec' # Closest X11 color: 'gray92'

        # Styles aktivieren durch ein-/auskommentieren

        style.use('bmh')
        # style.use('ggplot')
        # style.use('fivethirtyeight')

        self.fig = Figure(dpi=75)
        self.a = self.fig.add_subplot(111)

        self.available_ports = []
        self.current_port = None
        self.baudrate = 115200              # Baudrate des ESP32
        self.bytesize = serial.EIGHTBITS
        self.timeout = None
        self.interval = 50                  # Refresh-Rate des Live-Loggings (sollte mit Bluetooth-Serial-Rate des ESP32 übereinstimmen)
        self.xcount = 100                   # maximale Anzahl der dargestellten Datenpunkte bei Live-Logging

        # Im Folgenden wird der Datentyp für die Konvertierung festgelegt. Dieser wird aus der datenstrultur.csv gebildet und MUSS mit 
        # der Datenstruktur des ESP32 identisch sein.
        self.current_path = pathlib.Path(__file__).parent.absolute()
        self.current_dt = 'static/datenstruktur.csv'
        self.get_data_type()
        self.data = pd.DataFrame([])

        # Folgede Liste legt die Startansicht fest. (Kann nach Bedarf geändert werden, die Strings müssen dabei übereinstimmen.)
        self.default_columns = ['Bremsdruck Vorne (bar)', 'Raddrehzahl Vorne (1/min)', 'Belagtemp. Vorne (°C)']
        # self.default_columns = ['Scheibentemp. Vorne (°C)', 'Belagtemp. Vorne (°C)', 'Umgebungstemp 1 (°C)', 'Umgebungstemp 2 (°C)']

        self.parent.geometry("1200x800+50+50")  # legt die Größe und Position beim Erzeugen des Fensters fest.
        self.parent.minsize(1200, 800)  # minimale Größe des Fensters (darunter nicht sinnvoll, da sonst die Darstellung nicht gut scaled)
        self.parent.maxsize(3004, 1959) # maximale Größe                             
        self.parent.resizable(1, 1)
        self.parent.title("Parser GUI")
        tk.Tk.iconbitmap(self.parent, default='static/logo.ico') # Für anderes Logo die logo.ico Datei ersetzen. (muss ICO sein)
        self.parent.configure(background=self._bgcolor)

        # Der Folgende Teil legt das Erscheinungsbild des GUI fest. Änderungen sind nicht empfohlen.
        # Wenn doch eine neue Funktion hinzukommen soll, sollte diese bestenfalls über ein Kaskardenmenü implementiert werden.

        ###################################################
        ################## Dropdownmenü ##################
        ###################################################
        
        self.menubar = tk.Menu(self.parent)

        self.file_m = tk.Menu(self.menubar, tearoff=0)
        self.file_m.add_command(label="Import CSV", command=lambda: self.load_csv())
        self.file_m.add_command(label="Import BIN", command=lambda: self.load_bin(repair=False))
        self.file_m.add_command(label="BIN reparieren", command=lambda: self.load_bin(repair=True))
        self.file_m.add_command(state='disabled', label="Export CSV", command=lambda: self.save_csv())
        self.file_m.add_separator()
        self.file_m.add_command(label="Exit", command=root.quit)
        self.menubar.add_cascade(label="File", menu=self.file_m)

        self.settings_m = tk.Menu(self.menubar, tearoff=0)
        self.settings_m.add_command(label="Datenstruktur auswählen", command=lambda: self.select_dt())
        self.settings_m.add_command(label="Optionen", command=lambda: self.show_options())
        self.menubar.add_cascade(label="Einstellungen", menu=self.settings_m)

        self.help_m = tk.Menu(self.menubar, tearoff=0)
        self.help_m.add_command(label="Dokumentation", command=lambda: self.open_dokumentation())
        self.menubar.add_cascade(label="Hilfe", menu=self.help_m)

        self.parent.config(menu=self.menubar)

        ###################################################
        ################# Plot - Optionen #################
        ###################################################

        self.plot_options = tk.LabelFrame(self.parent)
        self.plot_options.place(relx=0.01, rely=0.01, relheight=0.975, relwidth=0.2)
        self.plot_options.configure(relief='groove', text='Plot Optionen', background=self._bgcolor)

        #################################################
        ################# Konvertierung #################
        #################################################

        self.conversion = tk.LabelFrame(self.parent)
        self.conversion.place(relx=0.25, rely=0.01, relheight=0.15, relwidth=0.35)
        self.conversion.configure(relief='groove', text='Konvertieren', background=self._bgcolor)

        self.load_csv_button = ttk.Button(self.conversion)
        self.load_csv_button.place(relx=0.01, rely=0.025)
        self.load_csv_button.configure(text='von CSV laden', command=lambda: self.load_csv())

        self.load_bin_button = ttk.Button(self.conversion)
        self.load_bin_button.place(relx=0.25, rely=0.025)
        self.load_bin_button.configure(text='von BIN laden', command=lambda: self.load_bin(repair=False))

        self.repair_bin_button = ttk.Button(self.conversion)
        self.repair_bin_button.place(relx=0.5, rely=0.025)
        self.repair_bin_button.configure(text='BIN reparieren', command=lambda: self.load_bin(repair=True))

        self.save_csv_button = ttk.Button(self.conversion)
        self.save_csv_button.place(relx=0.74, rely=0.025)
        self.save_csv_button.configure(state='disabled', text='als CSV speichern', command=lambda: self.save_csv())

        ##################################################
        ################# Live - Logging #################
        ##################################################

        self.live_logging = tk.LabelFrame(self.parent)
        self.live_logging.place(relx=0.625, rely=0.01, relheight=0.15, relwidth=0.365)
        self.live_logging.configure(relief='groove', text='Live-Aufzeichnung', background=self._bgcolor)

        self.start_button = ttk.Button(self.live_logging)
        self.start_button.place(relx=0.01, rely=0.025)
        self.start_button.configure(state='disabled', text='Start', command=lambda: self.start_live_logging())

        self.stop_button = ttk.Button(self.live_logging)
        self.stop_button.place(relx=0.2, rely=0.025)
        self.stop_button.configure(state='disabled', text='Stop', command=lambda: self.stop_live_logging())

        self.reset_button = ttk.Button(self.live_logging)
        self.reset_button.place(relx=0.39, rely=0.025)
        self.reset_button.configure(state='disabled', text='Reset', command=lambda: self.reset_live_logging())

        self.connect_button = ttk.Button(self.live_logging)
        self.connect_button.place(relx=0.62, rely=0.025)
        self.connect_button.configure(text='Verbinden mit Datenlogger', command=lambda: self.connect_to_port())

        #################################################
        ################# Graph - Frame #################
        #################################################

        self.graph_frame = tk.Frame(self.parent)
        self.graph_frame.place(relx=0.15, rely=0.075, relheight=0.91, relwidth=0.84)
        self.graph_frame.configure(relief='groove', borderwidth="2", background=self._bgcolor)

        self.canvas = FigureCanvasTkAgg(self.fig, self.graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget()

        toolbar = NavigationToolbar2Tk(self.canvas, self.graph_frame)
        toolbar.update()
        self.canvas._tkcanvas.place(relx=0, rely=0, relheight=1, relwidth=1)


    # Ab hier beginnen die Methoden der DatenloggerGUI-Klasse. Diese stellen quasi das Backend dar und regeln den 
    # Ablauf der App. Hier nur Änderungen an den vorgesehenen Stellen vornehmen! (Siehe Dokumentation)
    # Bei Fragen gerne eine Mail an johannes.hoefler@outlook.com


    def get_data_type(self):
        '''Liest die vorgegebene Datenstruktur ein. Bei Änderungen müssen diese in der Excel-Liste analog übernommen werden.'''

        dtype_map = {
            'float64'   : np.float64,
            'float32'   : np.float32,
            'uint64'       : np.uint64,
            'uint32'    : np.uint32,
            'uint16'    : np.uint16,
            'uint8'     : np.uint8,
        }
        csv = pd.read_csv(self.current_dt, sep=';', encoding = 'ISO-8859-1')
        
        data_types = []
        for _, row in csv.iterrows():
            name = row['name']
            dtype = dtype_map.get(row['datatyp'], None)
            data_types.append((name, dtype))
        
        dt = np.dtype(data_types, align=True)
        self.dt = dt
    

    def select_dt(self):
        '''Ermöglicht die Auswahl verschiedener zuvor angelegter Datenstrukturen'''

        csv_file_types = (('csv Dateien', '*.csv'), ('alle Dateien', '*.*'))
        dt_path = askopenfilename(filetypes=csv_file_types, initialdir='static/')
        if dt_path:
            self.current_dt = dt_path
            self.get_data_type()


    def show_options(self):
        '''Öffnet Optionen-Fenster'''

        option_window = tk.Toplevel(self.parent)
        option_window.resizable(0, 0)

        settings = {}

        label_frame = tk.LabelFrame(option_window)
        label_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        label_frame.configure(relief='groove', text='Einstellungen') 

        setting1_text = ttk.Label(label_frame, text='Datenpunkte Live-Logging', anchor='w')
        setting1_text.grid(row=0, column=0, sticky='w', padx=10, pady=5)
        setting1_content = tk.StringVar()
        setting1_content.set(str(self.xcount))
        setting1_entry = ttk.Entry(label_frame, width=4, justify='right', textvariable=setting1_content)
        settings['1'] = setting1_content
        setting1_entry.grid(row=0, column=1, sticky='nsew', padx=10, pady=5)

        setting2_text = ttk.Label(label_frame, text='Refresh-Rate Live-Logging', anchor='w')
        setting2_text.grid(row=1, column=0, sticky='w', padx=10, pady=5)
        setting2_content = tk.StringVar()
        setting2_content.set(str(self.interval))
        setting2_entry = ttk.Entry(label_frame, width=4, justify='right', textvariable=setting2_content)
        settings['2'] = setting2_content
        setting2_entry.grid(row=1, column=1, sticky='nsew', padx=10, pady=5)

        button_frame = tk.Frame(option_window)
        button_frame.grid(row=2, column=0, sticky='nsew')
        confirm_Button = ttk.Button(button_frame, text='Bestätigen', command=lambda: self.validate_options(option_window, settings))
        confirm_Button.grid(row=0, column=0, sticky='nsew', padx=15, pady=5)
        cancle_Button = ttk.Button(button_frame, text='Abbrechen', command=lambda: option_window.destroy())
        cancle_Button.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)
    

    def validate_options(self, parent, settings_list):
        try:
            setting1 = int(settings_list['1'].get())
            if setting1 >= 1 and setting1 <= 999999:
                self.xcount = setting1
                parent.destroy()
            else:
                raise ValueError
        except ValueError:
            messagebox.showinfo('Achtung!', 'Bitte Wert von 1 bis 999999 eingeben')
            parent.attributes("-topmost", True)
            parent.attributes("-topmost", False)

        try:
            setting2 = int(settings_list['2'].get())
            if setting2 >= 50 and setting2 <= 10000:
                self.interval = setting2
                parent.destroy()
            else:
                raise ValueError
        except ValueError:
            messagebox.showinfo('Achtung!', 'Bitte Wert in ms von 50 bis 10000 eingeben')
            parent.attributes("-topmost", True)
            parent.attributes("-topmost", False)

        


    def open_dokumentation(self):
        '''Öffnet Dokumentation'''

        doc_path = str(self.current_path) + '/static/test.pdf'
        webbrowser.open_new(doc_path)


    def load_csv(self):
        '''Ermöglicht das plotten von CSV-Files, welche zuvor aus BIN-Files erzeugt wurden'''
        
        csv_file_types = (('csv Dateien', '*.csv'), ('alle Dateien', '*.*'))
        filepath = askopenfilename(filetypes=csv_file_types)
        if filepath:
            if filepath.endswith('.csv'):
                self.current_file = os.path.basename(filepath)
                self.data = pd.read_csv(filepath, sep=';', decimal=',')
                self.data.set_index(['Zeitstempel'], 1, inplace=True)
                [self.data.drop([col], 1, inplace=True) for col in list(self.data) if 'Unnamed' in col]

                self.update_options(self.data)
                self.update_graph(self.data)
                self.save_csv_button.configure(state='disabled')
            else: messagebox.showinfo('Achtung!', 'Dateiendung muss ".csv" sein!')


    def load_bin(self, repair):
        '''BIN-Files, welche vom Datenlogger erzeugt wurden, werden automatisch konvertiert
        und anschließend geplottet'''

        bin_file_types = ("bin Dateien","*.bin, *.BIN"),("alle Dateien","*.*")
        filepath = askopenfilename(filetypes=(bin_file_types))
        if filepath:
            if filepath.endswith('.bin') or filepath.endswith('.BIN'):
                self.current_file = os.path.basename(filepath)

                df = pd.DataFrame(np.fromfile(filepath, dtype=self.dt))
                if repair:
                    df = self.repair_file(df)

                df.set_index(['Zeitstempel'], 1, inplace=True)

                df = self.convert_can_msg(df)
                self.data = self.format_data(df)

                self.update_options(self.data)
                self.update_graph(self.data)

                self.save_csv_button.configure(state='enabled')
                self.file_m.entryconfig(2, state='active')
            else: messagebox.showinfo('Achtung!', 'Dateiendung muss ".bin" oder ".BIN" sein!')

    
    def repair_file(self, df):
        '''Ermöglicht die Reparatur von beschädigten BIN-Dateien. Plausibilisierung anhand des Zeitstempels'''

        broken_rows = []
        for idx, row in df.iterrows():
            if idx < len(df) - 1:
                delta_t = df['Zeitstempel'][idx+1] - df['Zeitstempel'][idx]
                if delta_t > 500 or delta_t < 0:
                    broken_rows.append(idx)

        if broken_rows:
            df.drop(df.index[broken_rows], inplace=True)

        return df


    def save_csv(self):
        '''Geladene BIN-Files (oder CSV-Files) werden ins CSV-Format knovertiert und können somit
        auch von anderen Programmen (Excel, Matlab...) gelesen werden'''

        csv_file_types = (('csv Dateien', '*.csv'), ('alle Dateien', '*.*'))
        filepath = asksaveasfilename(filetypes=csv_file_types)

        if filepath:
            if filepath.endswith('.csv'):
                self.data.to_csv(filepath, sep=';', decimal=',')
            else: messagebox.showinfo('Achtung!', 'Dateiendung muss ".csv" sein!')



    def connect_to_port(self):
        '''Ermöglicht die Verbindung zum Datenlogger, wenn dieser zuvor mit der Bluetooth-Schnittstelle
        des PCs verbunden wurde. Das Suchen nach verfügbaren Ports kann ein paar Sekunden in Anspruch nehmen.'''

        # Liste aller verfügbaren COM-Ports:
        self.available_ports = [comport.device for comport in list_com_ports.comports()]

        # Popup mit Auswahlmöglichkeiten
        dialog = tk.Toplevel(self.parent)
        dialog.resizable(0, 0)

        label_frame = tk.Frame(dialog)
        label_frame.grid(row=0, column=0, sticky='nsew')
        label = ttk.Label(label_frame, text='Auswahl COM-Port:')
        label.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        radio_frame = tk.Frame(dialog)
        radio_frame.grid(row=1, column=0, sticky='nsew')

        self.radio_variable = tk.IntVar()
        self.radio_variable.set(0)
        for i, port in enumerate(self.available_ports):
            radio_button = tk.Radiobutton(radio_frame, text=port, variable=self.radio_variable, value=i)
            radio_button.grid(row=i+1, column=0, sticky='nsew', padx=5, pady=1)
        
        button_frame = tk.Frame(dialog)
        button_frame.grid(row=2, column=0, sticky='nsew')
        confirm_Button = ttk.Button(button_frame, text='Bestätigen', command=lambda: self.update_port(dialog))
        confirm_Button.grid(row=0, column=0, sticky='nsew', padx=15, pady=5)
        cancle_Button = ttk.Button(button_frame, text='Abbrechen', command=lambda: dialog.destroy())
        cancle_Button.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        label_frame2 = tk.Frame(dialog)
        label_frame2.grid(row=3, column=0, sticky='nsew')
        label = ttk.Label(label_frame2, text='Datenlogger muss zuvor per Bluetooth\nmit PC verbunden werden!')
        label.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)


    def update_port(self, parent):
        '''gehört zur Funktionalität von connect_to_port'''

        parent.destroy()
        self.current_port = self.available_ports[self.radio_variable.get()]
        if self.current_port:
            self.start_button.configure(state='enabled')


    def start_live_logging(self):
        '''Verfügbar nachdem mit COM-Port verbunden wurde. Verbindung zu gewähltem COM-Port 
        wird hergestellt und der read_serial-Loop wird initiiert.'''

        try:
            self.esp32 = serial.Serial(port=self.current_port,
                                    baudrate=self.baudrate,
                                    bytesize=self.bytesize,
                                    timeout=self.timeout)
            
            self.start_button.configure(state='disabled')
            self.stop_button.configure(state='enabled')
            self.reset_button.configure(state='enabled')

            self.after_process = self.parent.after(ms=self.interval, func=lambda: self.read_serial())
        except SerialException:
            messagebox.showerror('Fehler!', f'{self.current_port} konnte nicht geöffnet werden!\nZeitlimit überschritten.')


    def stop_live_logging(self):
        ''''Beendet die Live-Logging Messung. Der read_serial-Loop wird gestoppt und die Verbindung
        zum COM-Port geschlossen.'''

        self.stop_button.configure(state='disabled')
        self.start_button.configure(state='enabled')
        self.reset_button.configure(state='disabled')

        self.parent.after_cancel(self.after_process)
        self.esp32.close()


    def reset_live_logging(self):
        '''Resettet den Live-Logging-Datensatz und löscht die bisher geschriebenen Daten.'''

        if hasattr(self, 'live_data'):
            self.live_data = self.live_data[0:0]


    def read_serial(self):
        '''Kern des Live-Loggings. Liest die vom Datenlogger via Serial-BT geschickten Datensätze und konvertiert diese
        Zeile für Zeile analog zur load_bin Funktion. Der Graph wird bei jedem Durchlauf geplottet'''

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
                self.update_options(self.live_data)

            self.update_graph(self.live_data.tail(self.xcount))

        self.after_process = self.parent.after(ms=self.interval, func=lambda: self.read_serial())


    def convert_can_msg(self, df):
        '''Convertiert die CAN-Nachrichten. Neue CAN-IDs müssen hier analog zur ID 0x101 hinzugefügt werden.
        Für detailierte Informationen lohnt ein Blick in die Dokumentation.'''

        can_msg_list = [can_msg for can_msg in list(df) if 'CanId' in can_msg]  # Identifizierung der CAN-IDs aus dem logData-Struct
        # Unbedingt die Nomenklatur beachten! Die CAN-Nachrichten, welche maximal 8 byte lang sind werden als uint64-Integer 
        # interpretiert, und anschließend in Hex-Strings umgewandelt. Dies ermöglicht die schnellste und universellste Konvertierung
        # unabhängig von der Läng der IDs. Einziger Nachteil ist, dass die Reihenfolge der Bytes umgedreht wird. Dies ist zu beachten!

        can_msg_length = 18

        for can_msg in can_msg_list:
            hex_list = [hex(val) for val in df[can_msg]] # Konvertieren aller Einträge in Hex-Strings
            for idx, hex_str in enumerate(hex_list):
                # Hex-Strings einheitlich auffüllen
                while len(hex_str) < can_msg_length:
                    hex_str = '0x' + '0' + hex_str.split('x')[1]
                hex_str = hex_str + '0'     # Wird für späteren Zugriff über index benötigt
                hex_list[idx] = hex_str
            
            # Im Folgenden werden die IDs decodiert. Weitere IDs können hier über eine if-Abfrage analog zur ID 0x101
            # hinzugefügt werden. Der Inhalt der ID muss dabei allerdings bekannt sei.
            if '0x101' in can_msg:
                # Aufschlüsselung der CAN-Nachricht (Name : Position im Hex-String)
                content_0x101 = {
                    'Ladestatus': [[-3, -1], [-5, -3]],
                    'Batteriestrom (mA)': [[-7, -5], [-9, -7]],
                    'Batterieleistung (W * 1/10)': [[-11, -9], [-13, -11]],
                    'Batteriespannung (mV)': [[-15, -13], [-17, -15]],
                    }
                # Die CAN-Nachricht wird hier mit Hilfe des zuvor angelegten Schlüssels dekodiert.
                for key in content_0x101:
                    index = content_0x101.get(key)
                    # Zusammenbau mittels Schlüssel zu Hex-Zahl
                    can_msg_0x101 = ['0x' + hex_str[index[0][0]:index[0][1]] + hex_str[index[1][0]:index[1][1]] for hex_str in hex_list]
                    # Konvertierung zu Integer und ablegen in Daten-Farme
                    can_msg_0x101 = [int(hex_str, 0) for hex_str in can_msg_0x101]
                    df[key] = can_msg_0x101

            # Unkonvertierte CAN-Nachricht wird aus Daten-Frame gelöscht.
            # Für Debugging der Hardware kann diese auch ausgegeben werden, dazu einfach auskommentieren.
            df.drop([can_msg], 1, inplace=True)

        return df


    def format_data(self, df):
        '''Postprocess-Formatierung von einigen Daten. Einheiten sind zu berücksichtigen'''

        pressure_rate = 78.0                    # Sensibilität der Drucksensoren
        radius = 0.35                           # geschätzter Abrollradius, für Umrechnung Raddrehz. -> Geschw.
        circumference = 2 * np.pi * radius
        total_flanks = 84                       # Anzahl Flanken in Bremsscheibe vorne

        df['Bremsdruck Vorne (bar)'] = df['Bremsdruck Vorne (bar)'] * pressure_rate * 3.3 / 4095        # Drucksensorsignal -> bar
        df['Bremsdruck Hinten (bar)'] = df['Bremsdruck Hinten (bar)'] * pressure_rate * 3.3 / 4095      # Drucksensorsignal -> bar
        df['Raddrehzahl Vorne (1/min)'] = 1e6 * 60 / (df['Raddrehzahl Vorne (1/min)'] * total_flanks)   # Hallsignal -> Umdrehungen/min
        df['Geschw. - Rad (km/h)'] = df['Raddrehzahl Vorne (1/min)'] * circumference * 60 / 1000        # Umdrehungen/min -> km/h (Rad)

        return df


    def update_options(self, df):
        self.show_columns = {name: tk.BooleanVar() for name in list(df)}
        [self.show_columns[key].set(1) if key in self.default_columns
            else self.show_columns[key].set(0) for key in self.show_columns]

        for i, key in enumerate(self.show_columns):
            check_box = tk.Checkbutton(self.plot_options)
            check_box.configure(text=key, variable=self.show_columns[key], background='#d9d9d9')
            check_box.configure(command=lambda: self.update_graph(df))
            check_box.grid(row=i, column=0, sticky='w', padx=5, pady=1)


    def update_graph(self, df):
        self.a.clear()

        show_this = [key for key in self.show_columns if self.show_columns[key].get() == 1]
        if show_this:
            self.a.plot(df.index, df[show_this])
        try:
            self.a.set_title(self.current_file)
        except AttributeError:
            self.a.set_title('Live Aufzeichnung')
        self.a.set_xlabel('Zeit in ms')
        self.a.legend(show_this)

        self.canvas.draw()



if __name__ == '__main__':
    root = tk.Tk()
    DataloggerGUI(root)
    root.mainloop()
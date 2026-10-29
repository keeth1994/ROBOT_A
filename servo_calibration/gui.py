"""Desktop calibration UI. Run: python servo_calibration/gui.py [--file PATH]."""
import argparse
import copy
import json
import queue
import threading
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

from calibrate import Connection, JOINTS, angles_for, export_table

CHANNELS = (0, 1, 2, 3, 8, 9, 10, 11)
DEFAULT_CHANNELS = dict(zip(JOINTS, (8, 3, 1, 2, 0, 11, 9, 10)))
LABELS = ('Foran venstre – sving', 'Foran venstre – løft',
          'Bak venstre – sving', 'Bak venstre – løft',
          'Foran høyre – sving', 'Foran høyre – løft',
          'Bak høyre – sving', 'Bak høyre – løft')


class Session:
    """Serial operations and measurements; used only by the background worker."""

    def __init__(self, path, connection_factory=Connection):
        self.path = path
        self.data = json.loads(path.read_text()) if path.exists() else {}
        self.factory = connection_factory
        self.connection = None
        self.active = None

    def connect(self, port):
        if self.connection is not None:
            raise ValueError('Allerede tilkoblet')
        self.connection = self.factory(port)
        self.off()
        self.status()

    def disconnect(self):
        connection, self.connection = self.connection, None
        self.active = None
        if connection is not None:
            connection.close()

    def off(self):
        self.active = None
        return self.connection.request('OFF')

    def status(self):
        reply = self.connection.request('STATUS')
        if not reply.startswith('OK STATUS '):
            raise ValueError('Uventet svar fra kontrolleren')
        fields = dict(item.split('=') for item in reply.split()[2:])
        if fields['ready'] != '1':
            raise ValueError('Kontrolleren er ikke klar. Sjekk I2C og start ESP32 på nytt.')
        if self.active and (int(fields['channel']), int(fields['pulse'])) != self.active[1:]:
            self.active = None
            raise ValueError('Servotilstanden er endret. Aktiver leddet på nytt.')
        return reply

    def select(self, joint, channel):
        self.active = None
        if joint not in JOINTS or channel not in CHANNELS:
            raise ValueError('Ugyldig ledd eller kanal')
        if joint in self.data and self.data[joint]['channel'] != channel:
            raise ValueError('Leddet har målinger på en annen kanal. Slett målingene først.')
        if any(name != joint and entry['channel'] == channel
               for name, entry in self.data.items()):
            raise ValueError('Kanalen har allerede målinger for et annet ledd.')
        reply = self.connection.request(f'SELECT {channel}')
        if reply != f'OK PULSE {channel} 1500':
            raise ValueError('Uventet svar ved aktivering')
        self.active = (joint, channel, 1500)

    def jog(self, delta):
        if self.active is None or delta not in (-10, -5, -1, 1, 5, 10):
            raise ValueError('Aktiver et ledd først')
        joint, channel, pulse = self.active
        # Check that the previous output is still active before moving it.
        self.status()
        reply = self.connection.request(f'JOG {delta}')
        if reply != f'OK PULSE {channel} {pulse + delta}':
            raise ValueError('Uventet svar ved justering')
        self.active = (joint, channel, pulse + delta)

    def record(self, angle):
        if self.active is None or angle not in angles_for(self.active[0]):
            raise ValueError('Aktiver et ledd først')
        self.status()
        joint, channel, pulse = self.active
        data = copy.deepcopy(self.data)
        data.setdefault(joint, {'channel': channel, 'points': {}})['points'][str(angle)] = pulse
        self.save(data)

    def save(self, data):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, indent=2) + '\n')
        temporary.replace(self.path)
        self.data = data

    def forget(self, joint):
        data = copy.deepcopy(self.data)
        data.pop(joint, None)
        self.save(data)

    def export(self):
        try:
            table = export_table(self.data)
        except KeyError as exc:
            raise ValueError('Mål tre vinkler for yaw og fem for pitch (også −90° og +90°) før eksport.') from exc
        output = self.path.with_name('servo_table.h')
        output.write_text(table)
        return f'Eksportert til {output}'


class App:
    def __init__(self, root, session):
        self.root, self.session = root, session
        self.jobs, self.results = queue.Queue(), queue.Queue()
        self.busy = self.closing = False
        self.stopping = False
        self.connected = False
        self.active = None
        self.data = copy.deepcopy(session.data)
        self.last_poll = 0
        self.port = tk.StringVar()
        self.channel = tk.StringVar(value='0')
        self.status_text = tk.StringVar(value='Ikke tilkoblet')
        self.pulse_text = tk.StringVar(value='Ingen servo aktiv')
        root.title('Servokalibrering – ROBOT A')
        root.geometry('1100x730')
        root.minsize(1050, 700)
        root.protocol('WM_DELETE_WINDOW', self.close)
        outer = ttk.Frame(root, padding=18)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='Servokalibrering', font=('', 20, 'bold')).pack(anchor='w')
        ttk.Label(outer, text='Støtt roboten. Aktivering flytter valgt servo til 1500 µs. '
                  'Ha strømbryteren tilgjengelig.', wraplength=800).pack(anchor='w', pady=(5, 12))
        row = ttk.Frame(outer)
        row.pack(fill='x')
        ttk.Label(row, text='USB-port:').pack(side='left')
        self.ports = ttk.Combobox(row, textvariable=self.port, width=26)
        self.ports.pack(side='left', padx=8)
        self.refresh_button = ttk.Button(row, text='Finn porter', command=self.refresh_ports)
        self.refresh_button.pack(side='left')
        self.connect_button = ttk.Button(row, text='Koble til', command=self.toggle_connection)
        self.connect_button.pack(side='left', padx=8)
        self.stop_button = tk.Button(row, text='STOPP / PWM AV', bg='#b42318', fg='white',
                                     font=('', 11, 'bold'), command=self.stop)
        self.stop_button.pack(side='right')
        ttk.Label(outer, textvariable=self.status_text, wraplength=800).pack(anchor='w', pady=10)
        self.table = ttk.Treeview(outer, columns=('name', 'channel', '-90', '-45', '0', '45', '90'),
                                  show='headings', height=8, selectmode='browse')
        for col, title, width in [('name', 'Ledd', 310), ('channel', 'Kanal', 65),
                                  ('-90', '−90° pitch (µs)', 125),
                                  ('-45', '−45° (µs)', 110), ('0', '0° (µs)', 110),
                                  ('45', '+45° (µs)', 110), ('90', '+90° pitch (µs)', 125)]:
            self.table.heading(col, text=title)
            self.table.column(col, width=width, anchor='w' if col == 'name' else 'center')
        self.table.pack(fill='x')
        self.table.bind('<<TreeviewSelect>>', self.choose)
        row = ttk.Frame(outer)
        row.pack(fill='x', pady=12)
        ttk.Label(row, text='Kanal for valgt ledd:').pack(side='left')
        self.channels = ttk.Combobox(row, textvariable=self.channel, values=CHANNELS,
                                     state='readonly', width=5)
        self.channels.pack(side='left', padx=8)
        self.activate = ttk.Button(row, text='Aktiver valgt ledd → 1500 µs', command=self.select)
        self.activate.pack(side='left')
        self.forget_button = ttk.Button(row, text='Slett leddets målinger', command=self.forget)
        self.forget_button.pack(side='right')
        ttk.Label(outer, textvariable=self.pulse_text, font=('', 15, 'bold')).pack(anchor='w')
        row = ttk.Frame(outer)
        row.pack(anchor='w', pady=10)
        self.motion_buttons = []
        for step in (-10, -5, -1, 1, 5, 10):
            button = ttk.Button(row, text=f'{step:+d} µs', width=10,
                                command=lambda d=step: self.submit('jog', d))
            button.pack(side='left', padx=(0, 6))
            self.motion_buttons.append(button)
        ttk.Label(outer, text='Mål vinkelen fysisk. Knappene nedenfor lagrer dagens puls; '
                  'de flytter ikke servoen.\nPitch: −90° = 45° over horisontalt, '
                  '−45° = horisontalt ut, 0° = halvveis opp, '
                  '+45° = fot ned (CAD), +90° = gammel nedre grense.\n'
                  'Yaw: 0° = CAD-nøytralstilling. Mål etter eventuell giring.',
                  wraplength=1020).pack(anchor='w')
        row = ttk.Frame(outer)
        row.pack(anchor='w', pady=10)
        self.endpoint_buttons = []
        for angle in (-90, -45, 0, 45, 90):
            button = ttk.Button(row, text=f'Lagre {angle:+d}°',
                                command=lambda a=angle: self.submit('record', a))
            button.pack(side='left', padx=(0, 8))
            self.motion_buttons.append(button)
            if abs(angle) == 90:
                self.endpoint_buttons.append(button)
        self.export_button = ttk.Button(row, text='Eksporter servo_table.h',
                                        command=lambda: self.submit('export'))
        self.export_button.pack(side='left', padx=15)
        ttk.Label(outer, text=f'Målinger lagres automatisk: {session.path}',
                  wraplength=800).pack(anchor='w', pady=(8, 0))
        ttk.Label(outer, text='Ved mekanisk stopp eller belastet servo: slå av strømmen. '
                  'PWM AV kan ikke garantere stopp ved I2C-feil.', wraplength=800).pack(anchor='w', pady=5)
        self.render()
        self.table.selection_set(self.table.get_children()[0])
        self.refresh_ports()
        threading.Thread(target=self.worker, daemon=True).start()
        root.after(80, self.poll)

    def refresh_ports(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
            self.ports['values'] = ports
            if ports and not self.port.get():
                self.port.set(ports[0])
        except ImportError:
            self.status_text.set('Installer avhengigheter: python -m pip install -r servo_calibration/requirements.txt')

    def selected(self):
        selected = self.table.selection()
        return selected[0] if selected else JOINTS[0]

    def choose(self, _event=None):
        joint = self.selected()
        self.channel.set(str(self.data.get(joint, {}).get('channel', DEFAULT_CHANNELS[joint])))

    def render(self):
        rows = sorted(zip(JOINTS, LABELS), key=lambda row:
                      self.data.get(row[0], {}).get('channel', DEFAULT_CHANNELS[row[0]]))
        for index, (joint, label) in enumerate(rows):
            entry = self.data.get(joint, {})
            channel = entry.get('channel', DEFAULT_CHANNELS[joint])
            values = (f'{joint} · {label}', f'{channel:02d}',
                      *(entry.get('points', {}).get(str(a), '—') if a in angles_for(joint)
                        else '' for a in (-90, -45, 0, 45, 90)))
            if self.table.exists(joint):
                self.table.item(joint, values=values)
            else:
                self.table.insert('', 'end', iid=joint, values=values)
            self.table.move(joint, '', index)
        self.connect_button.configure(text='Koble fra' if self.connected else 'Koble til',
                                      state='disabled' if self.busy else 'normal')
        for widget in (self.ports, self.refresh_button):
            widget.configure(state='disabled' if self.connected or self.busy else 'normal')
        self.activate.configure(state='normal' if self.connected and not self.busy else 'disabled')
        for widget in self.motion_buttons:
            widget.configure(state='normal' if self.active and not self.busy else 'disabled')
        if self.active and not self.active[0].endswith('_pitch'):
            for widget in self.endpoint_buttons:
                widget.configure(state='disabled')
        for widget in (self.forget_button, self.export_button):
            widget.configure(state='disabled' if self.busy else 'normal')
        self.stop_button.configure(state='normal' if self.connected else 'disabled')
        self.pulse_text.set(f'{self.active[0]} · kanal {self.active[1]} · {self.active[2]} µs'
                            if self.active else 'Ingen servo aktiv')

    def submit(self, action, *args):
        if self.busy or self.closing:
            return
        self.busy = True
        self.jobs.put((action, args))
        self.render()

    def toggle_connection(self):
        if self.connected:
            self.submit('disconnect')
        elif self.port.get().strip():
            self.status_text.set('Kobler til …')
            self.submit('connect', self.port.get().strip())

    def select(self):
        self.submit('select', self.selected(), int(self.channel.get()))

    def forget(self):
        joint = self.selected()
        if messagebox.askyesno('Slett målinger', f'Slette alle lagrede målinger for {joint}?'):
            self.submit('forget', joint)

    def stop(self):
        if self.stopping:
            return
        # Do not wait for a pending serial request. The write lock serializes bytes.
        self.active = None
        self.stopping = True
        try:
            if self.session.connection:
                self.session.connection.send('OFF')
            self.status_text.set('PWM AV forespurt. Bruk strømbryteren hvis servoen ikke stopper.')
        except Exception as exc:
            self.status_text.set(f'Kunne ikke sende PWM AV: {exc}. Slå av servostrømmen.')
        # Close the connection after any in-flight operation, discarding its pose.
        self.busy = True
        self.jobs.put(('disconnect', ()))
        self.render()

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop()
        self.status_text.set('Slår av PWM og kobler fra …')

    def worker(self):
        while True:
            action, args = self.jobs.get()
            result, error = None, None
            try:
                result = getattr(self.session, action)(*args)
            except Exception as exc:
                error = str(exc) or type(exc).__name__
                # An unknown command outcome invalidates the connection and active pose.
                try:
                    self.session.disconnect()
                except Exception:
                    pass
            self.results.put((action, result, error, self.session.connection is not None,
                              self.session.active, copy.deepcopy(self.session.data)))

    def poll(self):
        try:
            while True:
                action, result, error, self.connected, self.active, self.data = self.results.get_nowait()
                if self.stopping and action != 'disconnect':
                    self.active = None
                    self.render()
                    continue
                self.busy = False
                if action == 'disconnect':
                    self.stopping = False
                if self.closing and action == 'disconnect':
                    self.root.destroy()
                    return
                if error:
                    self.status_text.set(f'Feil: {error}. Frakoblet; sjekk oppsettet før ny tilkobling.')
                elif action != 'status':
                    self.status_text.set(str(result) if result else {
                        'connect': 'Tilkoblet. Velg ledd og kanal, og aktiver når roboten er støttet.',
                        'disconnect': 'Frakoblet. PWM AV forespurt.',
                        'select': 'Servo aktiv. Juster i små steg og mål vinkelen fysisk.',
                        'record': 'Måling lagret.', 'forget': 'Målinger slettet.',
                    }.get(action, 'Klar'))
                self.render()
        except queue.Empty:
            pass
        self.last_poll += 1
        if self.last_poll >= 12:
            self.last_poll = 0
            if self.connected and not self.busy and not self.closing:
                self.submit('status')
        self.root.after(80, self.poll)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path, default=Path(__file__).with_name('measurements.json'))
    args = parser.parse_args()
    root = tk.Tk()
    try:
        session = Session(args.file)
    except (OSError, ValueError) as exc:
        messagebox.showerror('Kunne ikke lese målinger', str(exc))
        root.destroy()
        return
    App(root, session)
    root.mainloop()


if __name__ == '__main__':
    main()

"""Visible MuJoCo servo bench. Values are hypotheses, not measured Parallax data."""
from __future__ import annotations
import argparse
import csv
import json
import math
import time
from collections import deque
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent


class Bench:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'models/servo_bench.xml'))
        self.data = mujoco.MjData(self.model)
        self.frequency = 0.5
        self.amplitude = math.radians(30)
        self.stall_torque = 0.27  # Exploratory assumption; not a continuous rating.
        self.free_speed = math.radians(230)  # Exploratory assumption; calibrate on the actual servo.
        self.phase = 0.0
        self.target = 0.0
        self.torque = 0.0
        self.payload_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'payload')
        self.samples = deque(maxlen=100000)
        mujoco.mj_forward(self.model, self.data)

    def set_payload(self, kg):
        kg = max(0.001, float(kg))
        q, dq, t = self.data.qpos.copy(), self.data.qvel.copy(), self.data.time
        self.model.body_mass[self.payload_id] = kg
        self.model.body_inertia[self.payload_id] = 0.4 * kg * 0.018**2
        mujoco.mj_setConst(self.model, self.data)
        self.data.qpos[:] = q
        self.data.qvel[:] = dq
        self.data.time = t
        mujoco.mj_forward(self.model, self.data)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.phase = self.target = self.torque = 0.0
        self.samples.clear()
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        dt = self.model.opt.timestep
        self.phase += 2 * math.pi * self.frequency * dt
        self.target = self.amplitude * math.sin(self.phase)
        # Generic PD servo plus a linear motoring torque-speed envelope.
        # Braking torque is limited separately. No kinematic teleport or forced velocity.
        requested = 2.2 * (self.target - self.data.qpos[0]) - 0.065 * self.data.qvel[0]
        motoring = requested * self.data.qvel[0] > 0
        available = self.stall_torque
        if motoring:
            available *= max(0.0, 1 - abs(self.data.qvel[0]) / self.free_speed)
        self.torque = float(np.clip(requested, -available, available))
        self.data.ctrl[0] = self.torque
        self.data.mocap_quat[0] = [math.cos(self.target / 2), 0, math.sin(self.target / 2), 0]
        mujoco.mj_step(self.model, self.data)
        self.samples.append((self.data.time, self.target, float(self.data.qpos[0]),
                             float(self.data.qvel[0]), self.torque, self.frequency,
                             float(self.model.body_mass[self.payload_id]), self.stall_torque, self.free_speed))

    def export(self):
        folder = ROOT / 'results'
        folder.mkdir(exist_ok=True)
        dest = folder / time.strftime('servo_%Y%m%d_%H%M%S.csv')
        with dest.open('w', newline='', encoding='utf8') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'target_rad', 'actual_rad', 'velocity_rad_s', 'torque_Nm',
                        'frequency_Hz', 'payload_kg', 'stall_torque_assumed_Nm', 'free_speed_assumed_rad_s'])
            w.writerows(self.samples)
        return dest


def check():
    metrics = {}
    for name, frequency in [('slow', 0.3), ('fast', 1.5)]:
        b = Bench()
        b.frequency = frequency
        for _ in range(2500):
            b.step()
        a = np.asarray(b.samples)
        assert np.isfinite(a).all()
        assert np.max(np.abs(a[:, 4])) <= b.stall_torque + 1e-9
        assert abs(b.data.time - 5) < 1e-8
        metrics[name + '_rms_error_deg'] = math.degrees(float(np.sqrt(np.mean((a[500:, 1] - a[500:, 2])**2))))
    assert metrics['fast_rms_error_deg'] > metrics['slow_rms_error_deg']
    b.set_payload(0.15)
    b.step()
    assert abs(b.model.body_mass[b.payload_id] - 0.15) < 1e-12
    metrics.update(mujoco_version=mujoco.__version__, status='PASS', actual_servo_validated=False)
    (ROOT / 'results').mkdir(exist_ok=True)
    (ROOT / 'results/check.json').write_text(json.dumps(metrics, indent=2), encoding='utf8')
    print(json.dumps(metrics, indent=2))


def run():
    import tkinter as tk
    from tkinter import ttk
    import mujoco.viewer

    bench = Bench()
    panel = tk.Tk()
    panel.title('ROBOT_A | MuJoCo servotest')
    panel.geometry('510x730+20+60')
    panel.configure(bg='#17212d')
    style = ttk.Style()
    style.theme_use('clam')
    wrap = ttk.Frame(panel, padding=18)
    wrap.pack(fill='both', expand=True)
    ttk.Label(wrap, text='MuJoCo · Servotest', font=('Segoe UI', 20, 'bold')).pack(anchor='w')
    ttk.Label(wrap, text='Testscene – ikke A7-roboten. Verdiene er foreløpige.', wraplength=460).pack(anchor='w', pady=(5, 12))
    ttk.Label(wrap, text='Blå: ønsket vinkel   |   Oransje: simulert vinkel').pack(anchor='w')
    ttk.Label(wrap, text='Arm: 150 mm / 40 g · Bevegelse: ±30°').pack(anchor='w', pady=(3, 10))
    values = {}
    for key, label, low, high, initial, resolution in [
        ('frequency', 'Tempo (Hz)', 0.1, 2.0, 0.5, 0.1),
        ('payload', 'Belastning på armenden (g)', 10, 250, 50, 10),
        ('speed', 'Antatt tomgangshastighet (grader/s)', 60, 500, 230, 10),
        ('torque', 'Antatt stoppmoment (Nm) – ikke kontinuerlig', 0.05, 0.8, 0.27, 0.01),
    ]:
        var = tk.DoubleVar(value=initial)
        values[key] = var
        tk.Scale(wrap, label=label, variable=var, from_=low, to=high, resolution=resolution,
                 orient='horizontal', length=460, highlightthickness=0, font=('Segoe UI', 10)).pack(fill='x', pady=3)
    status = tk.StringVar()
    ttk.Label(wrap, textvariable=status, font=('Consolas', 11), justify='left').pack(anchor='w', pady=10)
    chart = tk.Canvas(wrap, width=460, height=125, bg='#17212d', highlightthickness=0)
    chart.pack(fill='x')
    buttons = ttk.Frame(wrap)
    buttons.pack(fill='x', pady=12)
    state = {'running': True, 'paused': False, 'last': time.perf_counter(), 'accum': 0.0, 'payload': 50.0}
    pause_label = tk.StringVar(value='Pause')
    def pause():
        state['paused'] = not state['paused']
        pause_label.set('Fortsett' if state['paused'] else 'Pause')
    def reset():
        with viewer.lock():
            bench.reset()
        state['accum'] = 0
    feedback = tk.StringVar(value='Endre tempo og se når armen begynner å henge etter.')
    def export():
        feedback.set('Lagret: results/' + bench.export().name)
    ttk.Button(buttons, textvariable=pause_label, command=pause).pack(side='left', padx=(0, 8))
    ttk.Button(buttons, text='Nullstill', command=reset).pack(side='left', padx=(0, 8))
    ttk.Button(buttons, text='Lagre CSV', command=export).pack(side='left')
    ttk.Label(wrap, textvariable=feedback, wraplength=460).pack(anchor='w')
    viewer = mujoco.viewer.launch_passive(bench.model, bench.data, show_left_ui=False, show_right_ui=False)
    with viewer.lock():
        viewer.cam.lookat[:] = [0, 0, 0.27]
        viewer.cam.distance = 0.82
        viewer.cam.azimuth = 100
        viewer.cam.elevation = -12
    def close():
        state['running'] = False
        viewer.close()
        panel.destroy()
    panel.protocol('WM_DELETE_WINDOW', close)
    def tick():
        if not state['running']:
            return
        if not viewer.is_running():
            close()
            return
        now = time.perf_counter()
        elapsed = min(now - state['last'], 0.1)
        state['last'] = now
        bench.frequency = values['frequency'].get()
        bench.free_speed = math.radians(values['speed'].get())
        bench.stall_torque = values['torque'].get()
        with viewer.lock():
            if state['payload'] != values['payload'].get():
                state['payload'] = values['payload'].get()
                bench.set_payload(state['payload'] / 1000)
            if not state['paused']:
                state['accum'] += elapsed
                while state['accum'] >= bench.model.opt.timestep:
                    bench.step()
                    state['accum'] -= bench.model.opt.timestep
        viewer.sync()
        goal, actual = math.degrees(bench.target), math.degrees(bench.data.qpos[0])
        status.set(f'Ønsket: {goal:6.1f}°   Faktisk: {actual:6.1f}°\nAvvik:  {goal-actual:6.1f}°   Moment: {bench.torque:+.3f} Nm')
        chart.delete('all')
        chart.create_line(0, 62, 460, 62, fill='#506070')
        samples = list(bench.samples)[-2000::10]
        if len(samples) > 1:
            t0 = max(0, bench.data.time - 4)
            for column, color in [(1, '#26b8ff'), (2, '#ff941f')]:
                points = []
                for row in samples:
                    points += [(row[0]-t0)/4*460, 62-math.degrees(row[column])*1.6]
                chart.create_line(*points, fill=color, width=2)
        panel.after(20, tick)
    panel.after(20, tick)
    panel.mainloop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='Run a deterministic headless environment check.')
    if parser.parse_args().check:
        check()
    else:
        run()

"""Interactive A16 CAD robot: kinematic inspection and torque-limited physics."""
from __future__ import annotations
import argparse, csv, json, math, struct, time, zlib
from pathlib import Path
from collections import deque
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
LEGS=['FL','RL','FR','RR']
PARTS=['hip_pitch','yaw','distal_pitch','steering']

class Robot:
    def __init__(self):
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/'models/robot_a16.xml'))
        self.data=mujoco.MjData(self.model)
        self.manifest=json.loads((ROOT/'reference/robot_a16_manifest.json').read_text())
        self.cfg=self.manifest['assumptions']
        self.names=[l+'_'+p for l in LEGS for p in PARTS]
        self.jids=np.array([self.model.joint(n).id for n in self.names])
        self.qadr=self.model.jnt_qposadr[self.jids]
        self.dadr=self.model.jnt_dofadr[self.jids]
        self.aids=np.array([self.model.actuator(n).id for n in self.names])
        self.wheels=[self.model.joint(l+'_wheel_roll').id for l in LEGS]
        self.pinion=[self.model.joint(l+'_pinion').id for l in LEGS]
        self.target=np.zeros(16);self.brakes=np.zeros(4)
        self.torque_limit=float(self.cfg['servo_stall_torque_Nm'])
        self.multipliers=np.array([2. if n.endswith('_hip_pitch') else 1. for n in self.names])
        self.free_speed=math.radians(self.cfg['servo_free_speed_deg_s'])
        self.mode='kinematic';self.samples=deque(maxlen=60000)
        self.steps=0;self.demo=False;self.demo_joint=0;self.frequency=.3;self.amplitude=math.radians(10)
        self.demo_center=0.;self.demo_start=0.;self.last_torque=np.zeros(16);self.last_available=np.zeros(16)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        self.target[:]=0;self.brakes[:]=0;self.steps=0;self.demo=False
        self.samples.clear();self.last_torque[:]=0
        mujoco.mj_forward(self.model,self.data)

    def enable_demo(self,index):
        self.demo_joint=index;self.demo_start=self.data.time
        lo,hi=self.model.jnt_range[self.jids[index]]
        self.demo_center=float(np.clip(self.target[index],lo+self.amplitude,hi-self.amplitude))
        self.demo=True

    def set_mode(self,mode):
        self.mode=mode;self.data.qvel[:]=0
        if mode=='kinematic':
            self.data.qpos[:7]=self.model.qpos0[:7]
        mujoco.mj_forward(self.model,self.data)

    def step(self):
        if self.demo:
            i=self.demo_joint;lo,hi=self.model.jnt_range[self.jids[i]]
            self.target[i]=np.clip(self.demo_center+self.amplitude*math.sin(2*math.pi*self.frequency*(self.data.time-self.demo_start)),lo,hi)
        if self.mode=='kinematic':
            self.data.qpos[:7]=self.model.qpos0[:7]
            self.data.qpos[self.qadr]=self.target
            self.data.qvel[:]=0;self.data.ctrl[:]=0
            for k,j in enumerate(self.pinion):self.data.qpos[self.model.jnt_qposadr[j]]=-self.target[k*4+3]
            self.data.time+=self.model.opt.timestep
            mujoco.mj_forward(self.model,self.data)
            self.last_torque[:]=0
        else:
            speed=self.data.qvel[self.dadr]
            requested=(2.2*(self.target-self.data.qpos[self.qadr])-.065*speed)*self.multipliers
            available=self.torque_limit*self.multipliers
            motoring=requested*speed>0
            available[motoring]*=np.maximum(0,1-np.abs(speed[motoring])/self.free_speed)
            self.last_available=available
            self.last_torque=np.clip(requested,-available,available)
            # The panel's torque assumption also updates the compiled actuator limits.
            self.model.actuator_ctrlrange[self.aids]=np.column_stack((-self.torque_limit*self.multipliers,self.torque_limit*self.multipliers))
            self.data.ctrl[self.aids]=self.last_torque
            for k,j in enumerate(self.wheels):
                self.model.dof_frictionloss[self.model.jnt_dofadr[j]]=.0002+self.brakes[k]*self.cfg['brake_torque_Nm']
            mujoco.mj_step(self.model,self.data)
        self.steps+=1
        if self.steps%10==0:
            self.samples.append([self.data.time,0 if self.mode=='kinematic' else 1,self.torque_limit,math.degrees(self.free_speed),*self.brakes,*self.target,*self.data.qpos[self.qadr],*self.last_torque,*self.wheel_forces(),*self.data.qpos[:3]])

    def wheel_forces(self):
        forces=np.zeros(4);geom_to_wheel={self.model.geom(l+'_wheel_contact').id:i for i,l in enumerate(LEGS)}
        floor=self.model.geom('floor').id
        for i in range(self.data.ncon):
            contact=self.data.contact[i]
            if floor not in (contact.geom1,contact.geom2):continue
            gid=contact.geom2 if contact.geom1==floor else contact.geom1
            if gid in geom_to_wheel:
                wrench=np.zeros(6);mujoco.mj_contactForce(self.model,self.data,i,wrench)
                forces[geom_to_wheel[gid]]+=max(0,wrench[0])
        return forces

    def export(self):
        path=ROOT/'results'/time.strftime('robot_%Y%m%d_%H%M%S.csv')
        path.parent.mkdir(exist_ok=True)
        with path.open('w',newline='',encoding='utf8') as f:
            w=csv.writer(f);w.writerow(['time_s','physics_mode','stall_torque_assumed_Nm','free_speed_assumed_deg_s']+[l+'_brake_fraction' for l in LEGS]+[n+'_target_rad' for n in self.names]+[n+'_actual_rad' for n in self.names]+[n+'_torque_Nm' for n in self.names]+[l+'_wheel_floor_normal_N' for l in LEGS]+['base_x_m','base_y_m','base_z_m']);w.writerows(self.samples)
        path.with_suffix('.json').write_text(json.dumps({'mode':self.mode,'stall_torque_assumed_Nm':self.torque_limit,'free_speed_assumed_deg_s':math.degrees(self.free_speed),'mass_assumed_kg':self.manifest['total_mass_assumed_kg'],'limitations':self.manifest['limitations']},indent=2),encoding='utf8')
        return path

def write_png(path,rgb):
    h,w,_=rgb.shape
    def chunk(kind,payload):return struct.pack('>I',len(payload))+kind+payload+struct.pack('>I',zlib.crc32(kind+payload)&0xffffffff)
    raw=b''.join(b'\x00'+row.tobytes() for row in rgb.astype(np.uint8))
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

def render(path):
    r=Robot();camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
    camera.lookat[:]=[0,0,.17];camera.distance=1.25;camera.azimuth=135;camera.elevation=-25
    opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.sitegroup[4]=0
    with mujoco.Renderer(r.model,height=1000,width=1440) as renderer:
        renderer.update_scene(r.data,camera=camera,scene_option=opt);write_png(path,renderer.render())

def check():
    r=Robot();m,d=r.model,r.data
    assert m.nu==16 and m.njnt==25 and m.nq==31,(m.nu,m.njnt,m.nq)
    assert r.manifest['servo_envelopes']==24
    assert np.isfinite(m.body_inertia).all() and (m.body_mass[1:]>0).all()
    checks={'cad_bodies':r.manifest['cad_body_count'],'actuated_joints':m.nu,'wheel_hinges':4,'coupled_pinion_hinges':4,'mass_assumed_kg':float(m.body_mass.sum())}
    # Independent forward-kinematics oracle for each of the 16 commanded axes.
    errors=[];orientation_errors=[]
    axes={x['name']:x for x in r.manifest['joints']}
    for i,n in enumerate(r.names):
        leg=n[:2];point0=d.site_xpos[m.site(leg+'_wheel_center').id].copy()
        wheel_body=m.body(leg+'_wheel_roll').id
        orientation0=d.xmat[wheel_body].reshape(3,3).copy()
        a=axes[n];axis=np.array(a['axis']);origin=np.array(a['origin_cad_m'])+m.qpos0[:3]
        q=-.35 if n.endswith('_yaw') else .35
        r.target[i]=q;r.step()
        delta=point0-origin
        expected=origin+delta*math.cos(q)+np.cross(axis,delta)*math.sin(q)+axis*(axis@delta)*(1-math.cos(q))
        errors.append(float(np.linalg.norm(d.site_xpos[m.site(leg+'_wheel_center').id]-expected)))
        skew=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
        rotation=math.cos(q)*np.eye(3)+math.sin(q)*skew+(1-math.cos(q))*np.outer(axis,axis)
        orientation_errors.append(float(np.max(np.abs(d.xmat[wheel_body].reshape(3,3)-rotation@orientation0))))
        r.reset()
    assert max(errors)<1e-7,errors
    checks['max_axis_fk_error_m']=max(errors)
    assert max(orientation_errors)<1e-10,orientation_errors
    checks['max_axis_rotation_matrix_error']=max(orientation_errors)
    # CAD visual placement: recomputed compiled mesh bounds against exported body vertices.
    with gzip_import() as exported:
        grouped={}
        for a in r.manifest['assignments']:
            rr=exported['bodies'][a['index']];grouped.setdefault(a['link'],[]).extend(rr['vertices'])
        bounds_errors=[]
        for link,vs in grouped.items():
            bid=m.body(link).id;points=[]
            for gid in range(m.ngeom):
                if m.geom_bodyid[gid]!=bid or m.geom_type[gid]!=mujoco.mjtGeom.mjGEOM_MESH:continue
                mesh=m.geom_dataid[gid];start=m.mesh_vertadr[mesh];count=m.mesh_vertnum[mesh]
                points.extend(m.mesh_vert[start:start+count]@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid])
            actual=np.array(points);expected=np.array(vs)+m.qpos0[:3]
            bounds_errors.append(float(np.max(np.abs(np.r_[actual.min(0)-expected.min(0),actual.max(0)-expected.max(0)]))))
    assert max(bounds_errors)<.0005,bounds_errors
    checks['max_cad_visual_bounds_error_m']=max(bounds_errors)
    # Full endpoint commands: 90 degrees for pitch; 180 total for yaw and wheel steering.
    for endpoint in [0,1]:
        r.target[:]=m.jnt_range[r.jids,endpoint];r.step();assert np.isfinite(d.qpos).all()
        np.testing.assert_allclose(d.qpos[r.qadr],r.target,atol=1e-12)
    r.reset();r.set_mode('physics')
    max_torque=0.;max_penetration=0.;floor_contacts=0
    for k in range(1500):
        r.step();max_torque=max(max_torque,float(np.max(np.abs(r.last_torque))))
        assert np.all(np.abs(r.last_torque)<=r.last_available+1e-12)
        assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
        floor_contacts+=int(any(m.geom('floor').id in (d.contact[i].geom1,d.contact[i].geom2) for i in range(d.ncon)))
        if d.ncon:max_penetration=max(max_penetration,float(-np.min(d.contact.dist[:d.ncon])))
    assert d.time>2.99 and floor_contacts>0 and max_torque<=2*r.torque_limit+1e-10
    assert not np.any(d.warning.number),d.warning.number
    checks.update(physics_seconds=float(d.time),max_servo_torque_Nm=max_torque,floor_contact_steps=floor_contacts,max_contact_penetration_m=max_penetration,physics_warnings=d.warning.number.tolist(),base_height_after_test_m=float(d.qpos[2]))
    # Passive wheel with and without brake in free fall (same initial conditions).
    speeds=[]
    for brake in [0,1]:
        r.reset();r.set_mode('physics');d.qpos[2]=5
        dof=m.jnt_dofadr[r.wheels[0]];d.qvel[dof]=12;r.brakes[0]=brake
        for _ in range(100):r.step()
        speeds.append(float(abs(d.qvel[dof])))
    assert speeds[0]>1 and speeds[1]<speeds[0]*.2,speeds
    checks['wheel_speed_after_0_2s_free_vs_braked_rad_s']=speeds
    dest=ROOT/'results/robot_a16_check.json';dest.parent.mkdir(exist_ok=True);dest.write_text(json.dumps(checks,indent=2))
    print(json.dumps(checks,indent=2))

def gzip_import():
    import gzip
    from contextlib import contextmanager
    @contextmanager
    def read():
        from cad_source_a16 import load_cad_source
        yield load_cad_source()
    return read()

def gui(start_physics=False):
    import tkinter as tk
    from tkinter import ttk
    import mujoco.viewer
    r=Robot();r.set_mode('physics' if start_physics else 'kinematic');root=tk.Tk();root.title('ROBOT A16 | MuJoCo kontrollpanel');root.geometry('490x820+15+35')
    root.configure(bg='#17212e');style=ttk.Style();style.theme_use('clam')
    style.configure('TFrame',background='#17212e');style.configure('TLabel',background='#17212e',foreground='#e7edf5',font=('Segoe UI',10));style.configure('Title.TLabel',font=('Segoe UI',17,'bold'))
    style.configure('TCheckbutton',background='#17212e',foreground='#e7edf5')
    main=ttk.Frame(root,padding=16);main.pack(fill='both',expand=True)
    ttk.Label(main,text='ROBOT A16',style='Title.TLabel').pack(anchor='w')
    ttk.Label(main,text='4 bein · 16 styrte ledd · 4 passive hjul').pack(anchor='w')
    ttk.Label(main,text=f"Antatt masse: {r.manifest['total_mass_assumed_kg']:.2f} kg · ikke veid",wraplength=440).pack(anchor='w',pady=(4,10))
    mode=tk.StringVar(value='Fysikk – begrenset servo' if start_physics else 'Visning – uten fysikk')
    dropdown=ttk.Combobox(main,textvariable=mode,values=['Visning – uten fysikk','Fysikk – begrenset servo'],state='readonly');dropdown.pack(fill='x')
    dropdown.bind('<<ComboboxSelected>>',lambda e:r.set_mode('physics' if mode.get().startswith('Fysikk') else 'kinematic'))
    ttk.Label(main,text='Visning flytter ledd direkte og tester ikke servoytelse.\nFysikk kan få roboten til å synke eller velte.',wraplength=440).pack(anchor='w',pady=8)
    controls=ttk.Frame(main);controls.pack(fill='x');paused=tk.BooleanVar(value=False)
    ttk.Checkbutton(controls,text='Pause',variable=paused).pack(side='left')
    vars=[];brakevars=[];labels=['Pitch ved plattform','Yaw','Pitch ved hjul','Hjulstyring']
    notebook=ttk.Notebook(main);notebook.pack(fill='x',pady=8)
    for li,leg in enumerate(LEGS):
        tab=ttk.Frame(notebook,padding=10);notebook.add(tab,text=leg)
        for pi,label in enumerate(labels):
            index=li*4+pi;v=tk.DoubleVar(value=0);vars.append(v)
            lo,hi=np.degrees(r.model.jnt_range[r.jids[index]])
            def changed(value,i=index):
                r.target[i]=math.radians(float(value))
            tk.Scale(tab,from_=float(lo),to=float(hi),orient='horizontal',variable=v,label=label+' (°)',resolution=1,command=changed,bg='#17212e',fg='#e7edf5',highlightthickness=0,troughcolor='#34455b',length=385).pack(fill='x')
        bv=tk.DoubleVar(value=0);brakevars.append(bv)
        tk.Scale(tab,from_=0,to=100,orient='horizontal',variable=bv,label='Bremse (%) – idealisert',command=lambda v,k=li:r.brakes.__setitem__(k,float(v)/100),bg='#17212e',fg='#e7edf5',highlightthickness=0,troughcolor='#34455b').pack(fill='x')
    def reset():
        r.reset()
        for v in vars+brakevars:v.set(0)
        demo.set(False)
    ttk.Button(controls,text='Nullstill',command=reset).pack(side='left',padx=8)
    status=tk.StringVar(value='Klar. Dra sliderne for å prøve leddene.')
    def save():status.set('Lagret: '+r.export().name)
    ttk.Button(controls,text='Lagre CSV',command=save).pack(side='right')
    demo=tk.BooleanVar(value=False);demo_row=ttk.Frame(main);demo_row.pack(fill='x')
    def demo_change():
        if demo.get():r.enable_demo(notebook.index(notebook.select())*4)
        else:r.demo=False
    ttk.Checkbutton(demo_row,text='Svingtest: pitch på valgt bein',variable=demo,command=demo_change).pack(side='left')
    freq=tk.DoubleVar(value=.3)
    tk.Scale(main,from_=.1,to=2,resolution=.1,orient='horizontal',variable=freq,label='Svingfrekvens (Hz), amplitude 10°',command=lambda v:setattr(r,'frequency',float(v)),bg='#17212e',fg='#e7edf5',highlightthickness=0,troughcolor='#34455b').pack(fill='x')
    tuning=ttk.Frame(main);tuning.pack(fill='x',pady=6)
    torque=tk.StringVar(value=str(r.torque_limit));speed=tk.StringVar(value=str(round(math.degrees(r.free_speed))))
    ttk.Label(tuning,text='Nm per servo (hofte x2):').pack(side='left');ttk.Entry(tuning,textvariable=torque,width=6).pack(side='left',padx=4)
    ttk.Label(tuning,text='°/s:').pack(side='left');ttk.Entry(tuning,textvariable=speed,width=6).pack(side='left',padx=4)
    def apply():
        try:
            t,s=float(torque.get()),float(speed.get())
            if not (0<t<=10 and 1<s<=2000):raise ValueError()
            r.torque_limit=t;r.free_speed=math.radians(s)
        except ValueError:status.set('Bruk 0–10 Nm og 1–2000 °/s.')
    ttk.Button(tuning,text='Bruk',command=apply).pack(side='left',padx=6)
    show_collision=tk.BooleanVar(value=False)
    ttk.Checkbutton(main,text='Vis forenklede kontaktformer',variable=show_collision).pack(anchor='w')
    telemetry=tk.StringVar();ttk.Label(main,textvariable=telemetry,wraplength=440).pack(anchor='w',pady=8)
    ttk.Label(main,textvariable=status,wraplength=440).pack(anchor='w')
    viewer=mujoco.viewer.launch_passive(r.model,r.data,show_left_ui=False,show_right_ui=False)
    viewer.cam.lookat[:]=[0,0,.17];viewer.cam.distance=1.25;viewer.cam.azimuth=135;viewer.cam.elevation=-25
    viewer.opt.geomgroup[3]=0;viewer.opt.sitegroup[4]=0
    closed=False;last=time.perf_counter();accumulator=0.;last_update=0.
    def close():
        nonlocal closed
        if closed:return
        closed=True;viewer.close();root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    def tick():
        nonlocal last,accumulator,last_update
        if closed:return
        if not viewer.is_running():close();return
        now=time.perf_counter();elapsed=min(.08,now-last);last=now
        if not paused.get():
            accumulator+=elapsed
            with viewer.lock():
                while accumulator>=r.model.opt.timestep:r.step();accumulator-=r.model.opt.timestep
        else:accumulator=0
        viewer.opt.geomgroup[3]=int(show_collision.get());viewer.sync()
        if now-last_update>.15:
            last_update=now
            err=np.degrees(np.abs(r.target-r.data.qpos[r.qadr]))
            forces=r.wheel_forces()
            telemetry.set(f"Tid {r.data.time:.1f} s · største vinkelavvik {err.max():.1f}°\nHjul mot gulv (N): "+' / '.join(f'{x:.1f}' for x in forces))
        root.after(16,tick)
    root.after(50,tick);root.mainloop()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--physics',action='store_true');parser.add_argument('--check',action='store_true');parser.add_argument('--render',type=Path);args=parser.parse_args()
    if args.check:check()
    elif args.render:render(args.render)
    else:gui(args.physics)

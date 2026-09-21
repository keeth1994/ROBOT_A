"""Reproducible A7.2 physics trials. Scripted attempts, never kinematic evidence."""
from pathlib import Path
import argparse,csv,json,math,hashlib,time
import numpy as np
import mujoco
from run_robot import Robot,LEGS

ROOT=Path(__file__).resolve().parent
DEST=ROOT/'results/capabilities'
RATED_TORQUE=38*0.00706155183333
RATED_SPEED=60/.19
CASES={'stand':5,'wave':9,'walk':12,'run':10,'crawl':12,'skate':12,'push':7,'recovery_side':10,'recovery_back':10,'dance':9}

def smooth(t):
    t=np.clip(t,0,1);return t*t*(3-2*t)

def command(case,t):
    q=np.zeros((4,4));brakes=np.ones(4)
    u=max(0,t-1.5);a=smooth(u)
    if case=='wave':
        q[0,0]=20*a;q[0,1]=18*a*math.sin(2*math.pi*.6*u)
    elif case in ['walk','crawl','run']:
        freq={'walk':.3,'crawl':.18,'run':1.2}[case]
        offsets=[0,.5,.75,.25] if case!='run' else [0,.5,.5,0]
        for k,offset in enumerate(offsets):
            phase=2*math.pi*(freq*u+offset);lift=max(0,math.sin(phase))
            q[k,0]=a*((18 if case=='crawl' else 3)+12*lift)
            q[k,2]=a*(8 if case=='crawl' else 3)*lift
            q[k,1]=a*12*math.cos(phase)*([1,1,-1,-1][k])
    elif case=='skate':
        brakes[:]=0
        for k in range(4):
            phase=2*math.pi*.4*u+(k%2)*math.pi
            q[k,1]=a*22*math.sin(phase)
            q[k,3]=a*25*math.cos(phase)
            brakes[k]=float(math.sin(phase)>.7)
    elif case.startswith('recovery'):
        for k in range(4):
            q[k,0]=a*(35+30*math.sin(2*math.pi*.25*u+k*math.pi/2))
            q[k,2]=a*25*(1+math.sin(2*math.pi*.25*u+k*math.pi/2))
    elif case=='dance':
        q[:,0]=a*(8+6*math.sin(2*math.pi*.6*u))
        q[:,1]=a*15*math.sin(2*math.pi*.3*u)*np.array([1,-1,-1,1])
    return np.deg2rad(q.reshape(-1)),brakes

def sample(r,target,tau,available):
    m,d=r.model,r.data;pid=m.body('platform').id
    forces=r.wheel_forces();floor=m.geom('floor').id
    wheelids={m.geom(l+'_wheel_contact').id for l in LEGS}
    bodyforce=0;penetration=0
    for j in range(d.ncon):
        c=d.contact[j];penetration=max(penetration,-c.dist)
        if floor in [c.geom1,c.geom2]:
            other=c.geom2 if c.geom1==floor else c.geom1
            if other not in wheelids:
                wrench=np.zeros(6);mujoco.mj_contactForce(m,d,j,wrench);bodyforce+=max(0,wrench[0])
    tilt=math.degrees(math.acos(np.clip(d.xmat[pid].reshape(3,3)[2,2],-1,1)))
    centers=np.array([d.site_xpos[m.site(l+'_wheel_center').id] for l in LEGS])
    return [d.time,*d.xpos[pid],tilt,float(np.rad2deg(np.sqrt(np.mean((target-d.qpos[r.qadr])**2)))),float(np.max(np.abs(tau))),float(np.mean((np.abs(tau)>=available*.98)&(available>1e-6))),bodyforce,penetration,*forces,*centers[:,2]]

COLS=['time_s','base_x_m','base_y_m','base_z_m','tilt_deg','joint_rms_error_deg','peak_torque_Nm','saturated_joint_fraction','nonwheel_ground_normal_N','max_penetration_m']+[l+'_wheel_N' for l in LEGS]+[l+'_wheel_z_m' for l in LEGS]

def trial(case,torque=RATED_TORQUE,speed=RATED_SPEED,mu=.8,mass_scale=1,kp=12,kd=.25,label=None,save=True):
    r=Robot();m,d=r.model,r.data;r.set_mode('physics');m.body_mass[1:]*=mass_scale;m.body_inertia[1:]*=mass_scale;mujoco.mj_setConst(m,d);r.reset()
    # Match the two contacting surfaces, avoiding max-mixing with the old floor friction.
    m.geom_friction[m.geom_contype>0,0]=mu
    m.actuator_ctrlrange[r.aids]=[-torque,torque]
    if case.startswith('recovery'):
        angle=math.pi/2 if case=='recovery_side' else math.pi
        d.qpos[3:7]=[math.cos(angle/2),math.sin(angle/2),0,0];d.qpos[2]=.5
        mujoco.mj_forward(m,d)
        # Start above floor, without teleporting during the trial. Gravity supplies the fall.
    target=np.zeros(16);brakes=np.ones(4);logs=[];qposes=[];targets=[];torques=[]
    for step in range(round(CASES[case]/m.opt.timestep)):
        if step%10==0:target,brakes=command(case,d.time)
        target=np.clip(target,m.jnt_range[r.jids,0],m.jnt_range[r.jids,1])
        vel=d.qvel[r.dadr];requested=kp*(target-d.qpos[r.qadr])-kd*vel
        available=np.full(16,torque);motoring=requested*vel>0
        available[motoring]*=np.maximum(0,1-np.abs(vel[motoring])/math.radians(speed))
        tau=np.clip(requested,-available,available);d.ctrl[r.aids]=tau
        for k,j in enumerate(r.wheels):m.dof_frictionloss[m.jnt_dofadr[j]]=.0002+brakes[k]*.15
        d.xfrc_applied[:]=0
        if case=='push' and 2<=d.time<2.2:d.xfrc_applied[m.body('platform').id,1]=5
        mujoco.mj_step(m,d)
        assert np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel))
        assert np.max(np.abs(tau))<=torque+1e-12
        if step%10==0:
            mujoco.mj_forward(m,d);logs.append(sample(r,target,tau,available));qposes.append(d.qpos.copy());targets.append(target.copy());torques.append(tau.copy())
    a=np.array(logs);late=a[:,0]>=1.5;final=a[:,0]>=CASES[case]-1
    support_ok=bool(np.all(a[late,8]<1) and np.all(a[late,4]<25) and np.min(a[late,3])>.10)
    goal_window=(a[:,0]>3)&(a[:,0]<7)
    wave_ok=case=='wave' and support_ok and bool(np.mean((a[goal_window,14]>.065)&(a[goal_window,10]<.5)&np.all(a[goal_window,11:14]>.5,axis=1))>.8)
    xtravel=float(a[-1,1]-a[0,1]);speed_x=xtravel/CASES[case]
    lift_counts=[int(np.count_nonzero(np.diff((a[:,10+k]>.5).astype(int))==-1)) for k in range(4)]
    if case=='stand':passed=support_ok and float(np.max(a[late,5]))<10
    elif case=='wave':passed=wave_ok
    elif case=='push':passed=support_ok and float(np.max(a[final,4]))<10
    elif case.startswith('recovery'):passed=bool(np.all(a[final,4]<15)&np.all(a[final,3]>.10)&np.all(a[final,8]<1))
    elif case=='dance':passed=support_ok and float(np.max(a[late,5]))<10
    elif case=='walk':passed=support_ok and xtravel>.25 and min(lift_counts)>=2 and np.max(a[late,5])<10
    elif case=='run':passed=support_ok and speed_x>.25 and min(lift_counts)>=4 and np.max(a[late,5])<10
    elif case=='crawl':
        # Low-stance progression screen only; an obstacle test is separately required.
        passed=bool(xtravel>.25 and np.all(a[late,4]<25) and np.all(a[late,8]<1) and .06<float(np.median(a[late,3]))<.10)
    elif case=='skate':
        rolls=np.array(qposes)[:,[m.jnt_qposadr[j] for j in r.wheels]]
        passed=support_ok and xtravel>.25 and float(np.mean(np.abs(rolls[-1]-rolls[0])))>2
    else:raise ValueError(case)
    result={'case':case,'label':label or case,'status':'screen_pass' if passed else 'not_demonstrated','mass_kg':float(m.body_mass.sum()),'mass_scale':mass_scale,'torque_limit_Nm':torque,'free_speed_deg_s':speed,'mu':mu,'kp':kp,'kd':kd,'duration_s':float(d.time),'support_screen_pass':support_ok,'max_tilt_deg':float(np.max(a[:,4])),'min_base_height_m':float(np.min(a[:,3])),'max_nonwheel_ground_normal_N':float(np.max(a[:,8])),'max_joint_rms_error_deg':float(np.max(a[:,5])),'saturation_fraction_after_1_5s':float(np.mean(a[late,7])),'displacement_x_m':xtravel,'mean_x_speed_m_s':speed_x,'wheel_liftoff_counts':lift_counts,'max_torque_Nm':float(np.max(a[:,6])),'max_penetration_m':float(np.max(a[:,9])),'warnings':d.warning.number.tolist(),'controller':'Untrained joint-space script, assumed internal servo PD; 50 Hz target updates and 500 Hz physics. No base fixing, no trajectory forcing.','scope':'Locomotion attempts are diagnostic only: no closed-loop gait controller, obstacle traversal or robust recovery policy has been validated.'}
    if save:
        DEST.mkdir(exist_ok=True,parents=True);name=label or case
        with (DEST/(name+'.csv')).open('w',newline='',encoding='utf8') as f:w=csv.writer(f);w.writerow(COLS);w.writerows(logs)
        np.savez_compressed(DEST/(name+'.npz'),qpos=qposes,target=targets,torque=torques,metrics=a)
        (DEST/(name+'.json')).write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({k:result[k] for k in ['label','status','support_screen_pass','min_base_height_m','max_joint_rms_error_deg']}),flush=True)
    return result

def suite():
    results=[trial(c) for c in CASES]
    sensitivity=[]
    for mu in [.4,.8,1.2]:
        for mass in [.75,1,1.25]:
            sensitivity.append(trial('stand',mu=mu,mass_scale=mass,label=f'stand_mu{mu}_mass{mass}'))
    sensitivity.append(trial('stand',kp=2.2,kd=.065,label='stand_original_PD'))
    sensitivity.append(trial('stand',speed=230,label='stand_original_speed'))
    for factor in [2,4,8]:sensitivity.append(trial('stand',torque=RATED_TORQUE*factor,label=f'stand_hypothetical_torque_x{factor}'))
    result={'revision':'A7.2','model_sha256':hashlib.sha256((ROOT/'models/robot_a7_2.xml').read_bytes()).hexdigest(),'servo_source':'https://www.parallax.com/package/parallax-standard-servo-downloads/','servo_basis':'Digital v3.0: 38 oz-in at 6 V, 0.19 s/60 degrees. Treat rated torque as an optimistic instantaneous ceiling, not validated continuous torque. Actual analog/digital unit unknown.','cases':results,'sensitivity':sensitivity}
    (DEST/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf8')

def replay(case):
    import mujoco.viewer
    r=Robot();z=np.load(DEST/(case+'.npz'));q=z['qpos'];metrics=z['metrics']
    with mujoco.viewer.launch_passive(r.model,r.data) as viewer:
        viewer.cam.lookat[:]=[0,0,.15];viewer.cam.distance=1.3;viewer.cam.azimuth=135;viewer.cam.elevation=-25;viewer.opt.geomgroup[3]=0;viewer.opt.sitegroup[4]=0
        t0=time.monotonic()
        while viewer.is_running():
            i=int((time.monotonic()-t0)*50)%len(q);r.data.qpos[:]=q[i];r.data.time=metrics[i,0];mujoco.mj_forward(r.model,r.data);viewer.sync();time.sleep(.01)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--case',choices=list(CASES));p.add_argument('--replay');a=p.parse_args()
    if a.replay:replay(a.replay)
    elif a.case:trial(a.case)
    else:suite()

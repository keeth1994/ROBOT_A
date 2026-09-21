"""Optimistic static equilibrium screen. Not a dynamic gait or a strength calculation."""
import json, math
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import linprog
from run_robot import Robot,LEGS

def minimum_torque(r,mu=.8,support=(0,1,2,3)):
    m,d=r.model,r.data
    d.qvel[:]=0;mujoco.mj_forward(m,d)
    # 12 contact force components, 16 servo torques, 4 brakes, 4 gear reactions, peak torque.
    A=np.zeros((m.nv,37));bounds=[(None,None)]*37;ub=[];rhs=[]
    points=[]
    for k,leg in enumerate(LEGS):
        gid=m.geom(leg+'_wheel_contact').id;axis=d.geom_xmat[gid].reshape(3,3)[:,2]
        az=axis[2];radial=np.array([0.,0.,1.])-az*axis
        pt=d.geom_xpos[gid]-.0325*radial/max(np.linalg.norm(radial),1e-12)
        if abs(az)>1e-8:pt-=.015*np.sign(az)*axis
        points.append(pt)
        jp=np.zeros((3,m.nv));mujoco.mj_jac(m,d,jp,None,pt,m.geom_bodyid[gid]);A[:,3*k:3*k+3]=jp.T
        bounds[3*k+2]=(0,None)
        if k not in support:
            for j in range(3):bounds[3*k+j]=(0,0)
        for axisindex in [0,1]:
            for sign in [-1,1]:
                row=np.zeros(37);row[3*k+axisindex]=sign;row[3*k+2]=-mu;ub.append(row);rhs.append(0)
        A[m.jnt_dofadr[r.wheels[k]],28+k]=1;bounds[28+k]=(-.15,.15)
        A[m.jnt_dofadr[r.pinion[k]],32+k]=1;A[r.dadr[k*4+3],32+k]=1
    for j,dof in enumerate(r.dadr):
        A[dof,12+j]=1
        for sign in [-1,1]:
            row=np.zeros(37);row[12+j]=sign;row[-1]=-1;ub.append(row);rhs.append(0)
    bounds[-1]=(0,None);c=np.zeros(37);c[-1]=1
    res=linprog(c,A_ub=ub,b_ub=rhs,A_eq=A,b_eq=d.qfrc_bias,bounds=bounds,method='highs')
    ans={'feasible':bool(res.success),'support_height_spread_m':float(np.ptp(np.array(points)[list(support),2]))}
    if res.success:
        ans.update(minimum_peak_servo_Nm=float(res.x[-1]),torques_Nm=dict(zip(r.names,res.x[12:28].tolist())),residual_max=float(np.max(np.abs(A@res.x-d.qfrc_bias))))
    return ans

def run():
    r=Robot();rows=[]
    for hip in [0,10,20,30,40,50,60]:
        for distal in [0,15,30]:
            r.reset();r.data.qpos[r.qadr[::4]]=math.radians(hip);r.data.qpos[r.qadr[2::4]]=math.radians(distal)
            row=minimum_torque(r);row.update(hip_deg=hip,distal_deg=distal);rows.append(row)
    r.reset();r.data.qpos[r.qadr[0]]=math.radians(15)
    wave=minimum_torque(r,support=(1,2,3))
    result={'scope':'Optimistic rigid static equilibrium at 21 symmetric pitch poses, yaw zero. Point wheel contacts, square friction pyramid, ideal 0.15 Nm brakes. No joint-stop support, no structural/body support, no dynamics. Pose height spread reported; this is not a proof over all configurations.','mass_kg':float(r.model.body_mass.sum()),'mu':.8,'poses':rows,'FL_lifted_15deg_three_supports':wave}
    dest=Path(__file__).resolve().parent/'results/capabilities';dest.mkdir(exist_ok=True,parents=True)
    (dest/'static_capacity.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps({'neutral':rows[0],'best_sampled':min((x for x in rows if x['feasible']),key=lambda x:x['minimum_peak_servo_Nm']),'wave':wave},indent=2))
if __name__=='__main__':run()

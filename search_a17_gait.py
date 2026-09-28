from run_robot_a17 import evaluate,ROOT
from concurrent.futures import ProcessPoolExecutor
import numpy as np,json

def trial(p):
 try:return evaluate(p,10)
 except Exception as e:return dict(parameters=p,error=str(e))
def score(x):
 if 'error' in x:return -100
 return x['forward_m']-2*abs(x['sideways_m'])-.001*abs(x['final_heading_deg'])-2*x['nonfoot_floor_fraction']-(2 if x['fallen'] else 0)
if __name__=='__main__':
 rng=np.random.default_rng(17);ps=[]
 for i in range(96):
  pattern='crawl' if i%2 else 'trot'
  ps.append(dict(period=float(rng.uniform(.7,3.8)),duty=float(rng.uniform(.5,.85)),yaw_amplitude=float(rng.uniform(.06,.4)),pitch_lift=float(rng.uniform(.15,.8)),pitch_stance=float(rng.choice([0,.08,.16,.24,.32])),pattern=pattern,kp=4.,kd=.08))
 results=[]
 with ProcessPoolExecutor(max_workers=4) as pool:
  for i,x in enumerate(pool.map(trial,ps)):
   results.append(x)
   if (i+1)%16==0:print(i+1,'best',score(max(results,key=score)),flush=True)
 results.sort(key=score,reverse=True)
 (ROOT/'results/a17_search.json').write_text(json.dumps(results,indent=2))
 (ROOT/'reference/a17_gait.json').write_text(json.dumps(results[0]['parameters'],indent=2))
 print(json.dumps(results[:3],indent=2))

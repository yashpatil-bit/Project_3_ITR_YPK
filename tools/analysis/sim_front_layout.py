# Feasibility of the FRONT layout (other group: wire facing the robot, handle straight out),
# 250 IK seeds per variant. Run from the repo root:  python3 tools/analysis/sim_front_layout.py 0.65
# (argument = distance of the wire centre from the base axis, m)
import os, sys, numpy as np, types, importlib.util
sys.path.insert(0,'.'); sys.path.insert(0,'scripts')
for m in ["rclpy","rclpy.node"]: sys.modules.setdefault(m, types.ModuleType(m))
sys.modules["rclpy.node"].Node=object
spec=importlib.util.spec_from_file_location("fp","scripts/find_pose.py"); fp=importlib.util.module_from_spec(spec); spec.loader.exec_module(fp)
from wire_trace.urdf_chain import ChainKinematics
kin=ChainKinematics(tool_xyz=(0,0,0.08)); lo,hi=kin.lower,kin.upper
rng=np.random.default_rng(1)
seeds=[rng.uniform(lo,hi) for _ in range(250)]
rows=[]
for d in (float(sys.argv[1]),):
  for h in (0.22,0.30):
    for pitch in (0,15,30):
      for mode,rolls in (("alt",(45,135,225,315)),):
        for r0 in rolls:
          poses=fp.wire_poses(np.array([0.0,d,h]),0.0,0.30,0.15,4,np.radians(r0),0.01,mode=mode,pitch=np.radians(pitch),plane=0.0,tol_deg=40)
          sols=[]
          for s in seeds:
              q0,ok=fp.ik_point(kin,poses[0][0],poses[0][1],s.copy(),iters=120,lo=lo,hi=hi)
              if ok and not any(np.max(np.abs(q0-q1))<0.1 for q1 in sols): sols.append(q0)
          best=None
          for q0 in sols:
              sc,info=fp.evaluate(kin,poses,q0,lo,hi,0.0,0.5)
              if sc is not None and (best is None or sc>best[0]): best=(sc,info)
          if best: rows.append((best[0],d,h,pitch,r0,len(sols),best[1]))
        print(f"d={d} h={h} pitch={pitch}: {sum(1 for r in rows if r[1]==d and r[2]==h and r[3]==pitch)} feasible", flush=True)
rows.sort(key=lambda r:-r[0])
print("BEST")
for sc,d,h,pitch,r0,ns,info in rows[:8]:
    qs=info["qs"]
    print(f"dist={d:.2f} h={h:.2f} pitch={pitch:2d} roll0={r0:3d} margin {np.degrees(info['margin']):4.0f}deg sigma {info['smin']:.3f}  "+" ".join(f"j{j+1}[{np.degrees(qs[:,j].min()):.0f},{np.degrees(qs[:,j].max()):.0f}]" for j in range(6)))
    os.makedirs("data/sim", exist_ok=True); np.save(f"data/sim/front_q0_{d}_{h}_{pitch}_{r0}.npy", qs[0])
print("DONE")

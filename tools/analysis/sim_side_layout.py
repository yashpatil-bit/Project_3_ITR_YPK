# Feasibility of the SIDE layout on HEAL's real kinematics (config/heal_chain.json).
# Run from the repo root:  python3 tools/analysis/sim_side_layout.py side
# Prints the best wire placements (margin to joint limits, min singular value, joint ranges).
import sys, numpy as np, types, importlib.util
sys.path.insert(0,'.'); sys.path.insert(0,'scripts')
for m in ["rclpy","rclpy.node"]: sys.modules.setdefault(m, types.ModuleType(m))
sys.modules["rclpy.node"].Node=object
spec=importlib.util.spec_from_file_location("fp","scripts/find_pose.py"); fp=importlib.util.module_from_spec(spec); spec.loader.exec_module(fp)
from wire_trace.urdf_chain import ChainKinematics
OFF=0.08
lay = sys.argv[1]
results=[]
if lay=="side":
    # NEW: axial grip (handle straight out of the gripper), gripper pointing SIDEWAYS (+x),
    # wire plane parallel to the robot's reach direction, offset sideways so the wrist sits on the y axis
    kin=ChainKinematics(tool_xyz=(0,0,OFF))
    lo,hi=kin.lower,kin.upper
    seeds=[np.zeros(6)]+[np.random.default_rng(s).uniform(lo*0.9,hi*0.9) for s in range(30)]
    for xo in (0.30, 0.34, 0.38, 0.42):
      for y0 in (0.30,0.35,0.40,0.45,0.50):
        for h in (0.22,):
          for mode,rolls in (("roll",(0,90)),("alt",(45,135))):
            for r0 in rolls:
              c=np.array([xo,y0,h])
              poses=fp.wire_poses(c,0.0,0.30,0.15,4,np.radians(r0),0.01,mode=mode,plane=np.radians(90),tol_deg=40)
              best=None
              for s in seeds:
                  q0,ok=fp.ik_point(kin,poses[0][0],poses[0][1],s.copy(),iters=120,lo=lo,hi=hi)
                  if not ok: continue
                  sc,info=fp.evaluate(kin,poses,q0,lo,hi,0.0,0.5)
                  if sc is not None and (best is None or sc>best[0]): best=(sc,info)
              if best: results.append((best[0],xo,y0,h,mode,r0,best[1]))
results.sort(key=lambda r:-r[0])
for sc,xo,y0,h,mode,r0,info in results[:8]:
    qs=info["qs"]
    print(f"x_wire={xo:.2f} y_centre={y0:.2f} h={h:.2f} {mode:4s} roll0={r0:3d}  margin {np.degrees(info['margin']):5.1f}deg  sigma_min {info['smin']:.3f}  "
          f"joint ranges(deg): " + " ".join(f"j{j+1}[{np.degrees(qs[:,j].min()):.0f},{np.degrees(qs[:,j].max()):.0f}]" for j in range(6)))
print(len(results),"feasible")

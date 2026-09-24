"""Whole-window local costmap occupancy vs commanded speed.

Run as:  ros2 run marsim_nav nav_occupancy -- <gx> <gy> [秒]


Sampling only ALONG THE PLAN understates saturation: the plan threads the gaps,
while MPPI scores trajectories that DEVIATE from it -- and those deviate into
the 1.0 m inflation bands. So measure the whole 10x10 m window.

Reminder: on /local_costmap/costmap the publisher writes LETHAL(254) as 100 and
INSCRIBED(253) as 99, so 100 means blocked. Counting ==254 is always zero.
"""
import math, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

class P(Node):
    def __init__(s):
        super().__init__('occ')
        s.g=None; s.pos=(0.,0.); s.cmd=0.
        s.create_subscription(OccupancyGrid,'/local_costmap/costmap',lambda m: setattr(s,'g',m),1)
        s.create_subscription(Odometry,'/odom',lambda m: setattr(s,'pos',(m.pose.pose.position.x,m.pose.pose.position.y)),50)
        s.create_subscription(Twist,'/cmd_vel_nav',lambda m: setattr(s,'cmd',math.hypot(m.linear.x,m.linear.y)),50)

def main():
    gx,gy,secs=float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3])
    rclpy.init(); n=P(); ac=ActionClient(n,NavigateToPose,'navigate_to_pose')
    if not ac.wait_for_server(timeout_sec=60): print('NO_ACTION_SERVER',flush=True); return 1
    g=NavigateToPose.Goal(); g.pose=PoseStamped(); g.pose.header.frame_id='map'
    g.pose.header.stamp=n.get_clock().now().to_msg()
    g.pose.pose.position.x, g.pose.pose.position.y = gx, gy
    g.pose.pose.orientation.w=1.0
    h=None
    for a in range(12):
        f=ac.send_goal_async(g); rclpy.spin_until_future_complete(n,f,timeout_sec=15); h=f.result()
        if h and h.accepted: break
        time.sleep(2)
    if not h or not h.accepted: print('GOAL_ALWAYS_REJECTED',flush=True); return 1
    print(f'目标 ({gx},{gy}) 已接受',flush=True)
    print(f'{"t":>6} {"|cmd|":>6} {"自由%":>6} {"高代价%":>7} {"致命%":>6} {"非零%":>6} {"按格计数"}',flush=True)
    res=h.get_result_async(); t0=time.time(); nxt=t0
    try:
        while time.time()-t0<secs:
            rclpy.spin_once(n,timeout_sec=0.05)
            now=time.time()
            if now>=nxt:
                nxt=now+2.0
                g_=n.g
                if g_ is None: print(f'{now-t0:6.1f}  无代价图',flush=True); continue
                a=np.array(g_.data,dtype=np.int16).reshape(g_.info.height,g_.info.width)&0xFF
                free=(a==0).mean()*100; hi=(a>50).mean()*100
                leth=(a==100).mean()*100; nz=(a>0).mean()*100
                print(f'{now-t0:6.1f} {n.cmd:6.2f} {free:6.1f} {hi:7.1f} {leth:6.1f} {nz:6.1f} '
                      f'{int((a>0).sum())}/{a.size}',flush=True)
            if res.done():
                print(f'*** 结束 {time.time()-t0:.1f}s ***',flush=True); break
        else:
            print(f'*** {secs:.0f}s 未结束 ***',flush=True)
    except KeyboardInterrupt:
        print('*** 中断 ***',flush=True)
    finally:
        if rclpy.ok(): rclpy.shutdown()
    return 0
if __name__=='__main__': sys.exit(main())

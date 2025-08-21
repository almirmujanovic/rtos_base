from __future__ import annotations

def decide_command(dets, img_w, img_h, sensors):
    # Sensor override
    front = sensors.get('vl53')
    if front is not None and 0 < front < 20:
        return {"steer":0.0,"throttle":0.0,"action":"stop_range"}
    if not dets:
        return {"steer":0.0,"throttle":0.5,"action":"forward_clear"}
    cls, conf, x1,y1,x2,y2 = max(dets, key=lambda d:(d[4]-d[2])*(d[5]-d[3]))
    cx=(x1+x2)/2; cy=(y1+y2)/2
    nx=(cx/img_w)*2-1; ny=(cy/img_h)*2-1
    area=(x2-x1)*(y2-y1)/(img_w*img_h)
    steer = max(-1,min(1,-0.8*nx))
    if area>0.20 or ny>0.4:
        throttle, action = 0.0, 'stop_obstacle_close'
    elif area>0.10:
        throttle, action = 0.2, 'slow_caution'
    else:
        throttle, action = 0.4, 'forward_avoid'
    return {"steer":float(steer),"throttle":float(throttle),"action":action,"target":cls,"conf":round(conf,2)}

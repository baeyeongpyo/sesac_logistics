"""Paint shared test_y perception results on a GUI frame; no GUI/ROS imports."""
import cv2
import numpy as np


def render_floor_overlay(frame,debug,source_shape):
    out=frame.copy()
    sx,sy=out.shape[1]/source_shape[1],out.shape[0]/source_shape[0]
    def point(p):return (int(round(p[0]*sx)),int(round(p[1]*sy)))
    def box(identity,color):
        x,y,w,h=identity['roi_px']
        cv2.rectangle(out,point([x,y]),point([x+w,y+h]),color,1)
    for identity in debug.get('rejected',[]):box(identity,(120,120,120))
    selected=debug.get('selected')
    if selected:box(selected,(0,180,255))
    d=debug.get('detection')
    if d:
        top=d['top_edge_px']
        cv2.line(out,point(top[0]),point(top[1]),(0,255,0),3)
        cv2.circle(out,point(d['x_identity']['center_px']),5,(0,0,255),-1)
        bottom=min(source_shape[0]-1,d['x_identity']['center_px'][1]+30)
        for side in d['side_bands']:
            cv2.line(out,point(side['point']),point([side['k']*bottom+side['q'],bottom]),(255,120,0),2)
    # This is the same detector on the latest raw frame, not the trial's frozen
    # command decision. Do not present a live preview as an issued motion plan.
    text='TEST_Y LIVE: '+debug['reason']
    cv2.rectangle(out,(0,out.shape[0]-25),(out.shape[1]-1,out.shape[0]-1),(0,0,0),-1)
    cv2.putText(out,text,(5,out.shape[0]-7),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
    return out

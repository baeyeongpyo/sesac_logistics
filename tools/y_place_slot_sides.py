"""Offline side-band evidence and intersections for a red-adjoining Y slot."""
import math
import cv2
import numpy as np


def side_intersections(frame, detection):
    """Locate both striped sides enclosing X; never use X ROI as endpoints."""
    height, width = frame.shape[:2]
    identity = detection['x_identity']
    cx, cy = identity['center_px']
    _, _, w, h = identity['roi_px']
    top = np.asarray(detection['top_edge_px'], float)
    slope = (top[1,1]-top[0,1])/(top[1,0]-top[0,0])
    intercept = top[0,1]-slope*top[0,0]
    top_y = slope*cx+intercept
    if cy <= top_y:
        return None
    low = max(0, int(top_y+8))
    high = min(height-1, int(cy+.45*h))
    if high-low < 20:
        return None
    x0, x1 = max(0,int(cx-w)), min(width,int(cx+w))
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,45,130)
    lines = cv2.HoughLinesP(edges[low:high,x0:x1],1,np.pi/720,15,
                           minLineLength=max(20,(high-low)*.35),maxLineGap=20)
    if lines is None:
        return None
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv,(15,40,65),(42,255,255))>0
    # Dark yellow can also have V<105: count each pixel once, never as both
    # colours. Otherwise support can exceed 1 and favour the wrong stripe.
    black = (hsv[:,:,2]<105) & ~yellow
    ys = np.linspace(low,high,60)
    radius = max(24,int(w*.3))
    offsets = np.arange(-radius,radius+1)
    groups = [[],[]]
    for a,b,c,d in lines.reshape(-1,4).astype(float):
        a,c = a+x0,c+x0
        b,d = b+low,d+low
        if abs(d-b)<8:
            continue
        k = (c-a)/(d-b)
        q = a-k*b
        # Side bands must cross the top band, rather than run along the
        # opposite horizontal border. Use their relative image directions.
        crossing = abs(1-k*slope)/(math.hypot(1,k)*math.hypot(1,slope))
        if crossing < .4:
            continue
        at_x = k*cy+q
        if abs(at_x-cx)<w*.18:
            continue
        sample_x = np.rint(k*ys[:,None]+q+offsets).astype(int)
        sample_y = np.broadcast_to(np.rint(ys).astype(int)[:,None],sample_x.shape)
        valid = (sample_x>=0)&(sample_x<width)
        sx = np.clip(sample_x,0,width-1)
        py = (yellow[sample_y,sx]&valid).mean(axis=0)
        pb = (black[sample_y,sx]&valid).mean(axis=0)
        marked = (py+pb)>.65
        bounds = np.flatnonzero(np.diff(np.r_[False,marked,False]))
        for start,end in zip(bounds[::2],bounds[1::2]):
            if end-start<4 or start==0 or end==len(offsets):
                continue
            y_support = float(py[start:end].mean())
            b_support = float(pb[start:end].mean())
            if y_support<.08 or b_support<.12:
                continue
            # A short dark/yellow fragment must not stand in for a full side.
            # Check support along the band, not just its whole-area average.
            longitudinal = ((yellow[sample_y,sx] | black[sample_y,sx]) & valid)[:,start:end].mean(axis=1)
            if sum(float(block.mean()) >= .5 for block in np.array_split(longitudinal,6)) < 5:
                continue
            centre_q = q+float(offsets[start:end].mean())
            x_at_centre = k*cy+centre_q
            side = 0 if x_at_centre<cx else 1
            # A striped band has a plain strip immediately on either side.
            outside = np.r_[py[max(0,start-4):start]+pb[max(0,start-4):start],
                            py[end:end+4]+pb[end:end+4]]
            score = y_support+b_support-float(outside.mean())
            if score < .25:
                continue
            denominator = 1-k*slope
            if abs(denominator)<1e-6:
                continue
            ix = (k*intercept+centre_q)/denominator
            iy = slope*ix+intercept
            # The horizontal band can continue across adjacent slots. Its
            # global endpoints must not veto the nearer sides enclosing X.
            band=detection.get('warning_tape')
            if band is not None and not band['x_min_px']-12<=ix<=band['x_max_px']+12:
                continue
            groups[side].append(dict(score=score,point=[float(ix),float(iy)],
                                     k=float(k),q=float(centre_q),
                                     yellow=y_support,black=b_support,width_px=float(end-start)))
    if not all(groups):
        return None
    reduced=[]
    for group in groups:
        kept=[]
        for row in sorted(group,key=lambda r:(abs(r['k']*cy+r['q']-cx),-r['score'])):
            if any(abs(row['point'][0]-other['point'][0])<4 and
                   abs(row['k']-other['k'])<.12 for other in kept):
                continue
            kept.append(row)
            if len(kept)==8:break
        reduced.append(kept)
    choices=[]
    for left in reduced[0]:
        for right in reduced[1]:
            if not left['point'][0]<cx<right['point'][0]:continue
            x_left=left['k']*cy+left['q']
            x_right=right['k']*cy+right['q']
            if not x_left<cx<x_right:continue
            support=interior_x_evidence(frame,left,right,cx,cy,top_y)
            if support is None:continue
            imbalance=abs((cx-x_left)-(x_right-cx))/(x_right-x_left)
            score=min(left['score'],right['score'])+min(support)-imbalance
            choices.append((score,left,right,support))
    if not choices:return None
    def span(row):return (row[2]['k']*cy+row[2]['q'])-(row[1]['k']*cy+row[1]['q'])
    nearest=min(choices,key=span)
    # Different fitted edges within the same physical tape are one boundary.
    # Prefer their pixel support, but never jump across the inter-slot floor.
    same_band=max(nearest[1]['width_px'],nearest[2]['width_px'])
    _,left,right,support=max((row for row in choices if span(row)<=span(nearest)+same_band),key=lambda row:row[0])
    return dict(detection,top_edge_px=[left['point'],right['point']],
                side_bands=[left,right],interior_x_support=support,
                border_source='warning_band_intersections')


def interior_x_evidence(frame,left,right,cx,cy,top_y):
    """Verify crossing yellow strokes in original pixels, without a guessed warp.

    Exclude the striped border, require two strokes to cross at the selected
    X, and require yellow support along both strokes. Border geometry cannot
    manufacture an X by stretching a different yellow region.
    """
    bottom_y = min(frame.shape[0]-1, 2*cy-top_y)
    yy,xx=np.mgrid[:frame.shape[0],:frame.shape[1]]
    lx=left['k']*yy+left['q']
    rx=right['k']*yy+right['q']
    span=right['k']*cy+right['q']-(left['k']*cy+left['q'])
    depth=cy-top_y
    if span <= 0 or depth <= 0:
        return None
    inside=(xx>lx+.08*span)&(xx<rx-.08*span)&(yy>top_y+3)&(yy<bottom_y)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow=(cv2.inRange(hsv,(15,90,65),(42,255,255))>0)&inside
    lines=cv2.HoughLinesP(np.uint8(yellow)*255,1,np.pi/360,15,
                          minLineLength=max(20,span*.30),maxLineGap=8)
    if lines is None:
        return None
    top_slope=(right['point'][1]-left['point'][1])/(right['point'][0]-left['point'][0])
    groups=[[],[]]
    for row in lines.reshape(-1,4):
        a,b=row.reshape(2,2).astype(float)
        if a[0]>b[0]:a,b=b,a
        v=b-a
        if v[0]<2:
            continue
        relative=v[1]-top_slope*v[0]
        if abs(relative)<max(4,depth*.3):
            continue
        groups[int(relative>0)].append((a,b))
    sample_y,sample_x=np.nonzero(inside)
    sample_y,sample_x=sample_y[::3],sample_x[::3]
    neutral_samples=(hsv[sample_y,sample_x,1]<65)&(hsv[sample_y,sample_x,2]>110)
    if len(sample_x)<30 or float(neutral_samples.mean())<.15:
        return None
    best=None
    for a,b in groups[0]:
        for c,d in groups[1]:
            matrix=np.column_stack((b-a,c-d))
            if abs(np.linalg.det(matrix))<1e-6:continue
            t,u=np.linalg.solve(matrix,c-a)
            cross=a+t*(b-a)
            if not (.15<t<.85 and .15<u<.85):continue
            if abs(cross[0]-cx)>span*.12 or abs(cross[1]-cy)>max(5,depth*.25):continue
            # X diagonals must lead back to this pair of top corners.
            # Check in original coordinates, so perspective convergence is allowed.
            top_intercept=left['point'][1]-top_slope*left['point'][0]
            corner_x=[]
            for start,end in [(a,b),(c,d)]:
                v=end-start
                denom=v[1]-top_slope*v[0]
                t_top=(top_slope*start[0]+top_intercept-start[1])/denom
                corner_x.append(float(start[0]+t_top*v[0]))
            corner_x.sort()
            top_span=right['point'][0]-left['point'][0]
            if any(abs(actual-expected)>.18*top_span for actual,expected in
                   zip(corner_x,[left['point'][0],right['point'][0]])):continue
            supports=[]
            for start,end in [(a,b),(c,d)]:
                samples=np.rint(np.linspace(start,end,60)).astype(int)
                supports.append(float(yellow[samples[:,1],samples[:,0]].mean()))
            if min(supports)<.75:continue
            # Solid yellow patches also contain Hough diagonals. An actual X
            # must leave neutral floor visible between its two strokes.
            off_strokes=np.ones(len(sample_x),dtype=bool)
            for start,end in [(a,b),(c,d)]:
                v=end-start
                distance=np.abs(v[0]*(sample_y-start[1])-v[1]*(sample_x-start[0]))/np.linalg.norm(v)
                off_strokes &= distance > max(8,span*.08)
            if off_strokes.sum()<10 or float(neutral_samples[off_strokes].mean())<.45:continue
            if best is None or min(supports)>min(best):best=supports
    return best

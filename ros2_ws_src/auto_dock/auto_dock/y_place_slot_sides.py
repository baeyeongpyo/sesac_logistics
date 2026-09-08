"""Offline side-band evidence and intersections for a red-adjoining Y slot."""
import math
import cv2
import numpy as np


def side_intersections(frame, detection, diagnostics=None):
    """Locate both striped sides enclosing X; never use X ROI as endpoints."""
    if diagnostics is not None:
        diagnostics.update(missing_side="unknown")
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
    if diagnostics is not None:
        diagnostics["missing_side"] = ("both" if not any(groups) else
            "left" if not groups[0] else "right" if not groups[1] else "none")
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


def interior_yellow_evidence(frame, left, right, cx, cy, top_y):
    """Require a yellow patch in the central interior, excluding all borders.

    Test each row against the actual side lines. The middle 30% of the slot
    rejects an empty inter-slot gap even when two outer sides span both slots.
    """
    height, width = frame.shape[:2]
    depth = cy - top_y
    if depth <= 0:
        return None
    yy, xx = np.mgrid[:height, :width]
    lx = left['k'] * yy + left['q']
    rx = right['k'] * yy + right['q']
    span = rx - lx
    slope = ((right['point'][1] - left['point'][1]) /
             (right['point'][0] - left['point'][0]))
    top = top_y + slope * (xx - cx)
    inside = ((span > 0) & (xx > lx + .35 * span) &
              (xx < rx - .35 * span) & (yy > top + .4 * depth) &
              (yy < top + 1.6 * depth))
    area = int(inside.sum())
    if area < 100:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = (cv2.inRange(hsv, (15, 90, 65), (42, 255, 255)) > 0) & inside
    count = int(yellow.sum())
    ratio = count / area
    if count < 30 or ratio < .05:
        return None
    _, _, stats, _ = cv2.connectedComponentsWithStats(yellow.astype(np.uint8))
    if len(stats) < 2 or int(stats[1:, cv2.CC_STAT_AREA].max()) < 20:
        return None
    return [ratio, ratio]


def interior_x_evidence(frame, left, right, cx, cy, top_y):
    """Compatibility name: interior yellow presence, no complete-X requirement."""
    return interior_yellow_evidence(frame, left, right, cx, cy, top_y)

"""Offline Y-slot geometry. No ROS imports and no command publication.

Pixel corners come from a closed physical border contour, never X endpoints.
Floor coordinates are (right, forward) cm; positive heading turns left.
"""
import math

import cv2
import numpy as np
from .y_place_warning_tape import detect_warning_tape
from .y_place_slot_sides import side_intersections, interior_yellow_evidence


def yellow_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (15, 40, 65), (42, 255, 255))


def x_candidates(frame):
    """Use crossing yellow strokes only to identify a slot/search region."""
    yellow = yellow_mask(frame)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(yellow)
    found = []
    for label in range(1, count):
        x, y, w, h, area = stats[label]
        if area < 250 or w < 45 or h < 25:
            continue
        mask = np.uint8(labels[y:y+h, x:x+w] == label) * 255
        lines = cv2.HoughLinesP(mask, 1, np.pi/180, 20,
                               minLineLength=max(25, w*.35), maxLineGap=12)
        if lines is None:
            continue
        positive, negative = [], []
        for line in lines.reshape(-1, 4).astype(float):
            a, b = line.reshape(2, 2)
            if b[0] < a[0]:
                a, b = b, a
            angle = math.degrees(math.atan2(*(b-a)[::-1]))
            if 0 <= angle < 89:
                positive.append((a, b))
            if -89 < angle < 0:
                negative.append((a, b))
        crosses = []
        for a, b in positive:
            for c, d in negative:
                matrix = np.column_stack((b-a, c-d))
                if abs(np.linalg.det(matrix)) < 1e-6:
                    continue
                t, u = np.linalg.solve(matrix, c-a)
                p = a+t*(b-a)
                if .15 < t < .85 and .15 < u < .85 and .2*w < p[0] < .8*w and .2*h < p[1] < .8*h:
                    crosses.append(p + (x, y))
        if crosses:
            found.append(dict(center_px=np.median(crosses, axis=0).tolist(),
                              roi_px=[int(x), int(y), int(w), int(h)]))
    return found


def ordered_quad(points):
    """TL, TR, BR, BL in image order (visible far/near edges)."""
    points = np.asarray(points, dtype=np.float64).reshape(4, 2)
    centre = points.mean(axis=0)
    points = points[np.argsort(np.arctan2(points[:, 1]-centre[1], points[:, 0]-centre[0]))]
    # Choose the opposite-edge pair most transverse in the image.
    edges = np.roll(points, -1, axis=0)-points
    horizontal = np.abs(edges[:, 0]) / np.linalg.norm(edges, axis=1)
    pair = 0 if horizontal[0]+horizontal[2] >= horizontal[1]+horizontal[3] else 1
    top = min((pair, pair+2), key=lambda i: (points[i, 1]+points[(i+1)%4, 1])/2)
    points = np.roll(points, -top, axis=0)
    if points[0, 0] > points[1, 0]:
        points = points[[1, 0, 3, 2]]
    return points


def outer_candidates(frame):
    """Closed contours of the black/yellow physical border and image edges."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    yellow = yellow_mask(frame)
    dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, 105))
    border = cv2.bitwise_or(yellow, dark)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 45, 130)
    proposals = []
    for source, mask in [('black_yellow_border', border), ('image_edges', edges)]:
        for kernel in (3, 5):
            connected = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((kernel, kernel), np.uint8))
            contours, _ = cv2.findContours(connected, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = abs(cv2.contourArea(contour))
                if area < 1800 or area > frame.shape[0]*frame.shape[1]*.5:
                    continue
                perimeter = cv2.arcLength(contour, True)
                for epsilon in (.012, .02, .035):
                    polygon = cv2.approxPolyDP(contour, epsilon*perimeter, True)
                    if len(polygon) != 4 or not cv2.isContourConvex(polygon):
                        continue
                    quad = ordered_quad(polygon[:, 0])
                    if np.min(np.linalg.norm(np.roll(quad, -1, axis=0)-quad, axis=1)) < 25:
                        continue
                    proposals.append((quad, source, area))
                    break
    return proposals


def border_line_quads(frame, identity):
    """Intersect four independently observed boundary lines around selected X."""
    cx, cy = identity['center_px']
    x, y, w, h = identity['roi_px']
    x0, x1 = max(0, int(cx-w*.85)), min(frame.shape[1], int(cx+w*.85))
    y0, y1 = max(0, int(cy-h*.95)), min(frame.shape[0], int(cy+h*.95))
    edges = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 45, 130)
    lines = cv2.HoughLinesP(edges[y0:y1,x0:x1], 1, np.pi/720, 20,
                           minLineLength=max(20, min(w,h)*.35),
                           maxLineGap=max(14,int(min(w,h)*.3)))
    if lines is None:
        return []
    groups = [[], [], [], []]  # top, right, bottom, left
    for points in lines.reshape(-1,2,2).astype(float):
        points += (x0,y0)
        a,b = points
        v = b-a
        length = np.linalg.norm(v)
        line = np.cross([*a,1.], [*b,1.])
        line /= np.linalg.norm(line[:2])
        if abs(v[0]) > abs(v[1])*1.4:
            intercept = -(line[0]*cx+line[2])/line[1]
            if cy-h*.95 < intercept < cy-h*.22:
                groups[0].append((length,line,intercept))
            elif cy+h*.22 < intercept < cy+h*.95:
                groups[2].append((length,line,intercept))
        else:
            intercept = -(line[1]*cy+line[2])/line[0]
            if cx-w*.8 < intercept < cx-w*.23:
                groups[3].append((length,line,intercept))
            elif cx+w*.23 < intercept < cx+w*.8:
                groups[1].append((length,line,intercept))
    reduced = []
    for group in groups:
        kept = []
        for row in sorted(group, key=lambda r:-r[0]):
            if any(abs(row[2]-k[2]) < 3 and abs(np.dot(row[1][:2],k[1][:2])) > .998 for k in kept):
                continue
            kept.append(row)
            if len(kept) == 12:
                break
        reduced.append(kept)
    from itertools import product
    proposals = []
    for combination in product(*reduced):
        lines = [row[1] for row in combination]
        corners = []
        for i,j in [(3,0),(0,1),(1,2),(2,3)]:
            p = np.cross(lines[i], lines[j])
            if abs(p[2]) < 1e-8:
                break
            corners.append(p[:2]/p[2])
        if len(corners) != 4:
            continue
        quad = np.asarray(corners)
        if np.any(quad < (x0-3,y0-3)) or np.any(quad > (x1+3,y1+3)):
            continue
        contour = quad.astype(np.float32)
        if not cv2.isContourConvex(contour) or cv2.pointPolygonTest(contour, (cx,cy), False) <= 0:
            continue
        lengths = np.linalg.norm(np.roll(quad,-1,axis=0)-quad,axis=1)
        if min(lengths)<25 or min(lengths[0],lengths[2])/max(lengths[0],lengths[2])<.35:
            continue
        proposals.append((quad,'four_boundary_hough_lines',abs(cv2.contourArea(contour))))
    return proposals


def detect_square(frame, anchor_px=None):
    """Choose an X once, then find a separate enclosing physical quadrilateral."""
    xs = x_candidates(frame)
    if not xs:
        return None
    target = np.asarray(anchor_px if anchor_px is not None else [frame.shape[1]/2, frame.shape[0]/2])
    ordered = sorted(xs, key=lambda x: np.linalg.norm(np.asarray(x['center_px'])-target))
    # During initial acquisition, reject striped-corner crossings that are not
    # an X inside a physical square. A supplied identity never falls through.
    for selected in ordered[:1] if anchor_px is not None else ordered:
        result = _detect_selected_square(frame, selected, xs)
        if result is not None:
            return result
    return None


def _detect_selected_square(frame, selected, xs, prefer_top=False):
    centre = np.asarray(selected['center_px'])
    choices = []
    for quad, source, area in outer_candidates(frame) + border_line_quads(frame, selected):
        if cv2.pointPolygonTest(quad.astype(np.float32), tuple(centre), False) <= 0:
            continue
        # Verify the physical striped perimeter; X strokes alone have no closed
        # four-sided black/yellow border. Measure all four bands independently.
        transform = cv2.getPerspectiveTransform(quad.astype(np.float32),
                       np.float32([[0,0],[159,0],[159,159],[0,159]]))
        crossing = transform@np.array([*centre,1.])
        crossing = crossing[:2]/crossing[2]
        if np.any(crossing<45) or np.any(crossing>114):
            continue
        warped = cv2.warpPerspective(frame, transform, (160,160))
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        dark = hsv[:, :, 2] < 110
        yellow = yellow_mask(warped) > 0
        yy, xx = np.mgrid[:160,:160]
        inside = (xx>25)&(xx<135)&(yy>25)&(yy<135)
        diagonal_support = [float(yellow[inside & (np.abs(v)<6)].mean())
                            for v in (xx-yy,xx+yy-159)]
        if min(diagonal_support) < .5:
            continue
        off_x = inside & (np.abs(xx-yy)>24) & (np.abs(xx+yy-159)>24)
        neutral = (hsv[:,:,1]<65) & (hsv[:,:,2]>110)
        if float(neutral[off_x].mean()) < .6:
            continue
        bands = [(slice(1,14),slice(8,152)), (slice(8,152),slice(146,159)),
                 (slice(146,159),slice(8,152)), (slice(8,152),slice(1,14))]
        support = [dict(dark=float(dark[b].mean()), yellow=float(yellow[b].mean()),
                        combined=float((dark|yellow)[b].mean())) for b in bands]
        if any(s['dark'] < .08 or s['yellow'] < .08 or s['combined'] < .55 for s in support):
            continue
        # Stripe junctions at the perimeter are not additional slot identities.
        # The normalized two-diagonal test above establishes the interior X.
        # A line inside the striped band also has yellow/black on its outside.
        # Prefer the boundary where the side/bottom exterior becomes plain floor.
        expanded = cv2.warpPerspective(frame, np.array([[1,0,16],[0,1,16],[0,0,1.]])@transform, (192,192))
        ehsv = cv2.cvtColor(expanded, cv2.COLOR_BGR2HSV)
        marked = (ehsv[:,:,2]<110) | (yellow_mask(expanded)>0)
        exterior = [marked[32:160,7:14].mean(),marked[32:160,178:185].mean(),marked[178:185,32:160].mean()]
        score = min(s['combined'] for s in support) + .3*min(diagonal_support) - .6*float(np.mean(exterior)) + area/frame.size
        if prefer_top:
            # Rank the actual edge above the X ahead of contours extending onto
            # a neighbouring pallet. This is a score, not an association veto.
            x, y, w, h = selected['roi_px']
            score -= 2*abs(float(quad[:2,1].mean())-y)/h
            score -= 2*(abs(quad[0,0]-x)+abs(quad[1,0]-(x+w)))/w
        choices.append((score, quad, source, support))
    if not choices:
        return None
    _, quad, source, support = max(choices, key=lambda row: row[0])
    return dict(x_identity=selected, outer_corners_px=quad.tolist(),
                top_edge_px=quad[:2].tolist(), bottom_edge_px=quad[[3,2]].tolist(),
                border_source=source, edge_support=support,
                image_size=[frame.shape[1], frame.shape[0]])


def detect_nearest_topline(frame, anchor_px=None, *, prefer_left=False):
    return inspect_nearest_topline(frame,anchor_px,prefer_left=prefer_left)['detection']


def inspect_nearest_topline(frame, anchor_px=None, *, prefer_left=False):
    """Reacquire independently; no old anchor, distance or angle association gate.

    Associate X with its adjoining red/warning band before locking a slot.
    Once locked, missing sides never redirect placement to a neighbouring slot.
    """
    xs = x_candidates(frame)
    if not xs:
        return dict(detection=None,selected=None,reason='no_x_candidate',rejected=[])
    centre = np.asarray(anchor_px if anchor_px is not None else [frame.shape[1]/2, frame.shape[0]/2])
    rejected=[]
    # Initial PLACE chooses the leftmost candidate; subsequent frames retain
    # the existing anchor/reacquisition behavior.
    key = ((lambda d: d["center_px"][0]) if prefer_left else
           (lambda d: np.linalg.norm(np.asarray(d["center_px"])-centre)))
    for identity in sorted(xs, key=key):
        x, y, w, h = identity['roi_px']
        observation = dict(x_identity=identity, image_size=[frame.shape[1],frame.shape[0]],
                           top_edge_px=[[x,y],[x+w-1,y]])
        band = warning_centerline(frame, observation)
        if band is not None:
            diagnostics = {}
            detection=side_intersections(frame, band, diagnostics=diagnostics)
            return dict(detection=detection,selected=identity,
                        reason='accepted' if detection is not None else 'side_or_interior_x_missing',
                        rejected=rejected, **diagnostics)
        rejected.append(identity)
    return dict(detection=None,selected=None,reason='no_red_warning_band',rejected=rejected)


def warning_centerline(frame, detection):
    """Use the warning-band centre next to red tape, in the selected slot span."""
    height, width = frame.shape[:2]
    edge = np.asarray(detection['top_edge_px'])
    x0 = max(0, int(edge[:, 0].min()))
    x1 = min(width, int(edge[:, 0].max()) + 1)
    pad = max(20, int(height * .08))
    y0 = max(0, int(edge[:, 1].min()) - pad)
    y1 = min(height, int(edge[:, 1].max()) + pad)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Same wraparound red HSV defaults as detect_dock_end_markers.
    red = cv2.inRange(hsv, (0,120,65), (12,255,255)) | cv2.inRange(hsv, (168,120,65), (179,255,255))
    mask = np.zeros_like(red)
    mask[y0:y1, x0:x1] = red[y0:y1, x0:x1]
    count, _, stats, centres = cv2.connectedComponentsWithStats(mask)
    points = np.asarray([centres[i] for i in range(1,count)
                         if stats[i,cv2.CC_STAT_AREA]>=30], np.float32)
    if len(points)<2:
        return None
    vx, vy, cx, cy = cv2.fitLine(points, cv2.DIST_L2, 0, .01, .01).reshape(-1)
    if abs(vx)<1e-6:
        return None
    slope = float(vy/vx)
    intercept = float(cy-slope*cx)
    yy, xx = np.mgrid[:height,:width]
    # Keep the warning band on the X side of the adjoining red tape.
    depth = yy-(slope*xx+intercept)
    region = (xx>=x0)&(xx<x1)&(depth>=0)&(depth<=max(12,height*.035))
    selected = np.full_like(frame, 180)
    selected[region] = frame[region]
    band = detect_warning_tape(selected, minimum_yellow_pixels=100,
                               filter_config={'min_component_pixels':30, 's_min':40,
                                              'reference_width_px':x1-x0})
    if band is None:
        return None
    slope = math.tan(math.radians(band['angle_deg']))
    intercept = band['center_y_ratio']*height-slope*width*.5
    line = [[float(x), float(slope*x+intercept)] for x in edge[:,0]]
    return dict(detection, top_edge_px=line, border_source='warning_tape_centerline',
                warning_tape=band)


def topline_pose(observation, profile, staging_cm=40.):
    """Fresh top edge alone defines the centre and inward normal at staging."""
    if observation['image_size'] != profile['image_size']:
        raise ValueError('floor_calibration_image_mismatch')
    points = np.asarray(observation['top_edge_px'], dtype=float)
    h = np.asarray(profile['image_to_floor_h'], dtype=float)
    projected = np.column_stack((points, np.ones(2))) @ h.T
    floor = projected[:, :2]/projected[:, 2, None]
    tangent = floor[1]-floor[0]
    tangent /= np.linalg.norm(tangent)
    inward = np.array([-tangent[1], tangent[0]])
    if inward[1] < 0:
        inward *= -1
    centre = floor.mean(axis=0)
    return dict(top_center_cm=centre.tolist(), top_edge_floor_cm=floor.tolist(),
                inward_normal=inward.tolist(), staging_cm=staging_cm,
                stage_right_forward_cm=(centre-staging_cm*inward).tolist(),
                heading_left_deg=math.degrees(math.atan2(-inward[0], inward[1])))


def square_pose(observation, profile, staging_cm=40.):
    if observation['image_size'] != profile['image_size']:
        raise ValueError('floor_calibration_image_mismatch')
    quad = np.asarray(observation['outer_corners_px'], dtype=float)
    h = np.asarray(profile['image_to_floor_h'], dtype=float)
    projected = np.column_stack((quad, np.ones(4))) @ h.T
    if not np.isfinite(projected).all() or np.any(np.abs(projected[:, 2]) < 1e-8):
        raise ValueError('invalid_floor_projection')
    floor = projected[:, :2]/projected[:, 2, None]
    top, bottom = floor[:2], floor[[3,2]]
    top_c, bottom_c = top.mean(axis=0), bottom.mean(axis=0)
    def unit(vector):
        length = np.linalg.norm(vector)
        if length < 1e-8:
            raise ValueError('degenerate_square')
        return vector/length
    top_t, bottom_t = unit(top[1]-top[0]), unit(bottom[1]-bottom[0])
    if np.dot(top_t, bottom_t) < 0:
        bottom_t *= -1
    tangent = unit(top_t+bottom_t)
    edge_normal = np.array([-tangent[1], tangent[0]])
    centre_axis = unit(top_c-bottom_c)
    if np.dot(edge_normal, centre_axis) < 0:
        edge_normal *= -1
    # Both parallel edge tangents AND the centre-to-centre direction vote.
    inward = unit(2*edge_normal+centre_axis)
    heading = math.degrees(math.atan2(-inward[0], inward[1]))
    stage = top_c-staging_cm*inward
    hull = np.asarray(profile.get('calibration_pixel_hull', []), np.float32)
    return dict(top_edge_floor_cm=top.tolist(), bottom_edge_floor_cm=bottom.tolist(),
                top_center_cm=top_c.tolist(), bottom_center_cm=bottom_c.tolist(),
                edge_normal=edge_normal.tolist(), centre_axis=centre_axis.tolist(),
                inward_normal=inward.tolist(), heading_left_deg=heading,
                stage_right_forward_cm=stage.tolist(), staging_cm=staging_cm,
                edge_heading_difference_deg=math.degrees(math.acos(float(np.clip(np.dot(top_t,bottom_t),-1,1)))),
                axis_disagreement_deg=math.degrees(math.acos(float(np.clip(np.dot(edge_normal,centre_axis),-1,1)))),
                extrapolated=bool(len(hull) and any(cv2.pointPolygonTest(hull, tuple(p), False)<0 for p in quad)))


def draw_evidence(frame, detection, pose=None):
    out = frame.copy()
    if detection is None:
        cv2.putText(out, 'NO PHYSICAL QUAD', (10, 40), 0, .7, (0,0,255), 2)
        return out
    if 'outer_corners_px' in detection:
        q = np.rint(detection['outer_corners_px']).astype(int)
        cv2.polylines(out, [q], True, (255,0,255), 2)
        cv2.line(out, tuple(q[3]),tuple(q[2]), (255,255,0), 3)
    top = np.rint(detection['top_edge_px']).astype(int)
    cv2.line(out, tuple(top[0]),tuple(top[1]), (0,255,0), 3)
    for side in detection.get('side_bands',[]):
        y0 = int(round(side['point'][1]))
        y1 = min(frame.shape[0]-1,int(detection['x_identity']['center_px'][1]+40))
        cv2.line(out,(int(side['k']*y0+side['q']),y0),
                 (int(side['k']*y1+side['q']),y1),(255,100,0),2)
    cv2.circle(out, tuple(np.rint(detection['x_identity']['center_px']).astype(int)), 5, (0,0,255), -1)
    if pose:
        cv2.putText(out, f"yaw {pose['heading_left_deg']:+.2f} stage {np.round(pose['stage_right_forward_cm'],1)}", (8,40),0,.55,(0,0,255),2)
    return out


def red_centerline_for_locked_slot(frame, hint):
    """Approximate locked-slot target from visible red tape when warning fails."""
    height,width=frame.shape[:2]
    edge=np.asarray(hint['top_edge_px'],float)
    x0=max(0,int(edge[:,0].min())); x1=min(width,int(edge[:,0].max())+1)
    pad=max(20,int(height*.08))
    y0=max(0,int(edge[:,1].min())-pad); y1=min(height,int(edge[:,1].max())+pad)
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    red=cv2.inRange(hsv,(0,120,65),(12,255,255))|cv2.inRange(hsv,(168,120,65),(179,255,255))
    mask=np.zeros_like(red);mask[y0:y1,x0:x1]=red[y0:y1,x0:x1]
    count,_,stats,centres=cv2.connectedComponentsWithStats(mask)
    points=np.asarray([centres[i] for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=30],np.float32)
    if len(points)<2:return None
    vx,vy,cx,cy=cv2.fitLine(points,cv2.DIST_L2,0,.01,.01).reshape(-1)
    if abs(vx)<1e-6:return None
    slope=float(vy/vx);intercept=float(cy-slope*cx)
    return dict(hint,top_edge_px=[[float(x),float(slope*x+intercept)] for x in edge[:,0]],
                border_source='red_tape_approximate',position_source='visible_red_tape_with_confirmed_slot',
                approximate=True,red_components=len(points))


def detect_locked_topline(frame, reference_frame, reference):
    """Track the confirmed floor patch, then refit visible tapes without X."""
    if reference_frame is None or reference is None:
        return None
    gray0=cv2.cvtColor(reference_frame,cv2.COLOR_BGR2GRAY)
    gray1=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    mask0=np.zeros_like(gray0); mask1=np.zeros_like(gray1)
    edge=np.asarray(reference['top_edge_px'],float)
    mask0[max(0,int(edge[:,1].min())-12):,:]=255
    mask1[frame.shape[0]//2:,:]=255
    sift=cv2.SIFT_create()
    k0,d0=sift.detectAndCompute(gray0,mask0); k1,d1=sift.detectAndCompute(gray1,mask1)
    if d0 is None or d1 is None:return None
    pairs=cv2.BFMatcher().knnMatch(d0,d1,k=2)
    matches=[a for pair in pairs if len(pair)==2 for a,b in [pair] if a.distance<.75*b.distance]
    if len(matches)<4:return None
    src=np.float32([k0[m.queryIdx].pt for m in matches]); dst=np.float32([k1[m.trainIdx].pt for m in matches])
    transform,inliers=cv2.findHomography(src,dst,cv2.RANSAC,4.)
    if transform is None or inliers is None or int(inliers.sum())<4:return None
    tracked=cv2.perspectiveTransform(np.float32([edge]),transform)[0]
    if not np.isfinite(tracked).all() or tracked[1,0]<=tracked[0,0]:return None
    height,width=frame.shape[:2]
    cx=float(tracked[:,0].mean()); top_y=float(tracked[:,1].mean())
    cy=(max(0.,top_y)+height-1)/2
    span=float(tracked[1,0]-tracked[0,0])
    hint=dict(image_size=[width,height],top_edge_px=tracked.tolist(),
              x_identity=dict(center_px=[cx,cy],roi_px=[cx-span/2,top_y,span,2*(height-top_y)]),
              identity_source='tracked_confirmed_floor',tracking_inliers=int(inliers.sum()))
    band=warning_centerline(frame,hint)
    red_only=band is None
    if red_only:band=red_centerline_for_locked_slot(frame,hint)
    if band is None:return None
    # Keep the confirmed side identities through the floor homography. A close
    # crop can merge an X stroke with the right border, so do not reselect sides.
    line=np.asarray(band['top_edge_px'],float)
    slope=(line[1,1]-line[0,1])/(line[1,0]-line[0,0]); intercept=line[0,1]-slope*line[0,0]
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    yellow=cv2.inRange(hsv,(15,40,65),(42,255,255))>0
    black=(hsv[:,:,2]<105)&~yellow
    sides=[]
    for old in reference.get('side_bands',[]):
        y0=old['point'][1]; y1=min(reference_frame.shape[0]-1,y0+80)
        source=np.float32([[[old['k']*y0+old['q'],y0],[old['k']*y1+old['q'],y1]]])
        a,b=cv2.perspectiveTransform(source,transform)[0]
        if abs(b[1]-a[1])<1:return None
        k=float((b[0]-a[0])/(b[1]-a[1])); q=float(a[0]-k*a[1])
        if abs(1-k*slope)<1e-6:return None
        ix=(k*intercept+q)/(1-k*slope); iy=slope*ix+intercept
        ys=np.arange(max(0,int(iy)+4),height)
        if red_only:
            sides.append(dict(point=[float(ix),float(iy)],k=k,q=q,source='confirmed_side_projection'))
            continue
        if len(ys)<8:return None
        xs=np.rint(k*ys[:,None]+q+np.arange(-4,5)).astype(int)
        yy=np.broadcast_to(ys[:,None],xs.shape)
        valid=(xs>=0)&(xs<width)
        if valid.sum()<30:return None
        yp=float(yellow[yy[valid],xs[valid]].mean()); bp=float(black[yy[valid],xs[valid]].mean())
        if yp<.08 or bp<.12:return None
        sides.append(dict(point=[float(ix),float(iy)],k=k,q=q,yellow=yp,black=bp))
    if len(sides)!=2:return None
    support = interior_yellow_evidence(frame, sides[0], sides[1], cx, cy, top_y)
    if support is None:return None
    return dict(band,interior_yellow_support=support,top_edge_px=[s['point'] for s in sides],side_bands=sides,
                border_source='red_tape_approximate' if red_only else 'tracked_sides_fresh_warning_centerline')

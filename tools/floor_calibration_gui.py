"""Manual top-line, two-distance floor calibration dialog."""
import json
import time
from pathlib import Path

import cv2
from python_qt_binding.QtCore import Qt, QRect
from python_qt_binding.QtGui import QImage, QPainter, QPixmap, QColor
from python_qt_binding.QtWidgets import (
    QDialog, QLabel, QPushButton, QDoubleSpinBox, QHBoxLayout, QVBoxLayout,
)
from floor_calibration import fit_floor_calibration, floor_point, save_floor_profile, validate_top_line
from floor_calibration_depth import (
    build_depth_calibration_evidence, fit_depth_extrinsics,
)


class CornerImage(QLabel):
    def __init__(self, clicked):
        super().__init__('샘플의 실제 거리를 입력하고 화면 고정 버튼을 누르세요')
        self.clicked = clicked
        self.frame = None
        self.points = []
        self.draw_rect = QRect()
        self.setMinimumSize(480, 320)
        self.setAlignment(Qt.AlignCenter)

    def paintEvent(self, event):
        if self.frame is None:
            return super().paintEvent(event)
        rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        pixmap = QPixmap.fromImage(QImage(rgb.data, w, h, rgb.strides[0], QImage.Format_RGB888).copy())
        size = pixmap.size().scaled(self.size(), Qt.KeepAspectRatio)
        self.draw_rect = QRect((self.width()-size.width())//2, (self.height()-size.height())//2,
                               size.width(), size.height())
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#111111'))
        painter.drawPixmap(self.draw_rect, pixmap)
        painter.setPen(QColor('#00ffff'))
        for i, (x, y) in enumerate(self.points):
            px = round(self.draw_rect.x()+x/w*self.draw_rect.width())
            py = round(self.draw_rect.y()+y/h*self.draw_rect.height())
            painter.drawEllipse(px-5, py-5, 10, 10)
            painter.drawText(px+7, py-7, str(i+1))

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton or self.frame is None or not self.draw_rect.contains(event.pos()):
            return
        h, w = self.frame.shape[:2]
        point = [(event.x()-self.draw_rect.x())*w/self.draw_rect.width(),
                 (event.y()-self.draw_rect.y())*h/self.draw_rect.height()]
        self.clicked(point)


class FloorCalibrationDialog(QDialog):
    def __init__(self, parent, saved):
        super().__init__(parent)
        self.window = parent
        self.saved = saved
        self.samples = {}
        self.distance = None
        self.sample_id = None
        self.session_dir = None
        self.profile = None
        self.frozen_depth = None
        self.frozen_camera_info = None
        self.frozen_extrinsics = None
        self.rgb_source_stamp_ns = None
        self.rgb_frame_id = None
        self.check_mode = False
        self.setWindowTitle('Y존 Depth / 적재 바닥 2점 캘리브레이션')
        self.resize(840, 780)
        layout = QVBoxLayout(self)
        instructions = QLabel(
            '물건을 실은 상태로 포크 중앙을 같은 주의선 사각형 중앙에 맞추고, 양옆 선과 평행하게 세우세요.\n'
            '거리: 포크 끝 → 먼 쪽 주의선 바깥 경계까지 실측해 각 샘플에 입력하세요.\n'
            '각 화면에서 top line 바깥 경계의 ① 왼쪽 끝 → ② 오른쪽 끝만 찍으세요.\n'
            '화면 고정 시 같은 source stamp의 registered depth와 CameraInfo도 함께 고정됩니다.\n'
            '두 위치에서 카메라 높이·기울기를 유지하세요. 이동할 때는 주행 화면으로 돌아가세요.')
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        dimensions = QHBoxLayout()
        self.width_cm = QDoubleSpinBox()
        self.width_cm.setRange(0, 500)
        self.width_cm.setDecimals(1)
        self.width_cm.setSuffix(' cm')
        self.width_cm.setSpecialValueText('실측 입력')
        self.width_cm.valueChanged.connect(self.dimensions_changed)
        dimensions.addWidget(QLabel('top line 실제 바깥 폭(좌우)'))
        dimensions.addWidget(self.width_cm)
        layout.addLayout(dimensions)
        captures = QHBoxLayout()
        self.sample_distances = {}
        for sample_id in (1, 2):
            field = QDoubleSpinBox()
            field.setRange(0, 10000)
            field.setDecimals(1)
            field.setSuffix(' cm')
            field.setSpecialValueText('실측 거리')
            field.valueChanged.connect(lambda value, i=sample_id: self.distance_changed(i))
            self.sample_distances[sample_id] = field
            captures.addWidget(QLabel(f'샘플 {sample_id}'))
            captures.addWidget(field)
            button = QPushButton('화면 고정')
            button.clicked.connect(lambda checked=False, i=sample_id: self.capture(i))
            captures.addWidget(button)
        back = QPushButton('주행 화면으로')
        back.clicked.connect(self.hide)
        captures.addWidget(back)
        layout.addLayout(captures)
        self.canvas = CornerImage(self.on_point)
        layout.addWidget(self.canvas, 1)
        self.status = QLabel('샘플 1·2 대기')
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        actions = QHBoxLayout()
        undo = QPushButton('마지막 점 취소'); undo.clicked.connect(self.undo)
        self.store = QPushButton('현재 양 끝점 저장'); self.store.clicked.connect(self.store_sample)
        self.apply_depth = QPushButton('Y Depth 보정값만 저장')
        self.apply_depth.clicked.connect(self.finish_depth_only)
        self.apply = QPushButton('Homography + Y Depth 함께 저장')
        self.apply.clicked.connect(self.finish)
        actions.addWidget(undo); actions.addWidget(self.store)
        actions.addWidget(self.apply_depth); actions.addWidget(self.apply)
        layout.addLayout(actions)
        check = QPushButton('저장된 보정으로 현재 바닥 좌표 확인 (화면 클릭)')
        check.clicked.connect(self.capture_check)
        layout.addWidget(check)
        self.load_profile()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.hide()
        # Do not propagate calibration-window keys into vehicle controls.
        event.accept()

    def keyReleaseEvent(self, event):
        event.accept()

    def load_profile(self):
        try:
            data = json.loads(Path(self.window.args.pose_config).read_text())
            self.profile = data.get('floor_calibration_presets', {}).get('loaded')
            if self.profile:
                self.width_cm.setValue(self.profile['width_cm'])
                self.status.setText('저장된 적재 바닥 보정 있음 · 새 샘플을 찍거나 좌표 확인을 누르세요')
        except (OSError, ValueError, KeyError, TypeError):
            self.profile = None

    def dimensions_changed(self):
        if self.samples:
            self.samples.clear()
            self.status.setText('치수가 바뀌어 두 샘플을 초기화했습니다')

    def distance_changed(self, sample_id):
        self.samples.pop(sample_id, None)
        if self.sample_id == sample_id:
            self.distance = None
            self.canvas.points = []
            self.canvas.update()
        self.status.setText(f'샘플 {sample_id} 거리 변경 · 화면을 다시 고정하세요')

    def fresh_frame(self, require_registered_depth=False):
        node = self.window.node
        if require_registered_depth:
            snapshot, depth, camera_info = node.calibration_registered_pair_snapshot()
        else:
            snapshot = node.calibration_rgb_snapshot()
            depth = camera_info = None
        if (snapshot is None
                or time.monotonic()-snapshot["received_monotonic"] > .35):
            raise ValueError('최신 원본 카메라 화면이 없습니다')
        if time.monotonic()-node.last_motor_active_monotonic < .5:
            raise ValueError('정지 후 0.5초 이상 기다린 뒤 화면을 고정하세요')
        cmd = node.latest_cmd_vel
        if cmd and any(abs(cmd[k]) > 1e-5 for k in ('linear_x', 'linear_y', 'angular_z')):
            raise ValueError('차량을 정지한 뒤 화면을 고정하세요')
        return snapshot, depth, camera_info

    def capture(self, sample_id):
        try:
            distance = self.sample_distances[sample_id].value()
            if distance <= 0:
                raise ValueError('이 위치의 실제 거리를 먼저 입력하세요')
            snapshot, self.frozen_depth, self.frozen_camera_info = self.fresh_frame(
                require_registered_depth=True
            )
            frame = snapshot["frame"]
            self.distance = distance
            self.sample_id = sample_id
            self.check_mode = False
            self.canvas.frame = frame
            self.canvas.points = []
            self.frame_at = snapshot["received_monotonic"]
            self.rgb_source_stamp_ns = int(snapshot["source_stamp_ns"])
            self.rgb_frame_id = str(snapshot["frame_id"])
            config = json.loads(Path(self.window.args.pose_config).read_text())
            self.frozen_extrinsics = {
                "camera_pitch_deg": config.get(
                    "y_slot_depth_camera_pitch_deg", config.get("camera_pitch_deg")
                ),
                "camera_yaw_deg": config.get("camera_yaw_deg", 0.0),
                "depth_camera_to_fork_tip_offset_cm": config.get(
                    "y_slot_depth_camera_to_fork_tip_offset_cm",
                    config.get("depth_camera_to_fork_tip_offset_cm"),
                ),
                "centerline_offset_cm": config.get(
                    "y_slot_centerline_offset_cm",
                    config.get("centerline_offset_cm", 0.0),
                ),
            }
            self.canvas.update()
            self.status.setText(f'{distance}cm 고정 · top line 왼쪽 끝 → 오른쪽 끝을 클릭하세요')
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.status.setText(str(exc))

    def on_point(self, point):
        if self.check_mode:
            try:
                p = floor_point(self.profile, point, list(self.canvas.frame.shape[1::-1]),
                                vehicle=self.window.args.vehicle, source_topic=self.window.args.tape_image_topic)
                self.status.setText(f"오른쪽 {p['right_cm']:+.1f}cm · 포크 앞 {p['forward_cm']:.1f}cm · "
                                    f"이 점을 향할 각도 {p['bearing_left_deg']:+.1f}° (좌회전 +)")
                self.canvas.points = [point]
            except (ValueError, KeyError, TypeError) as exc:
                self.status.setText(str(exc))
        elif len(self.canvas.points) < 2:
            self.canvas.points.append(point)
            self.status.setText(f'{self.distance}cm · {len(self.canvas.points)}/2점 선택')
        self.canvas.update()

    def undo(self):
        if self.canvas.points:
            self.canvas.points.pop(); self.canvas.update()

    def store_sample(self):
        try:
            if self.check_mode or self.distance is None:
                raise ValueError('샘플의 거리를 입력하고 화면을 먼저 고정하세요')
            size = list(self.canvas.frame.shape[1::-1])
            points = validate_top_line(self.canvas.points, size)
            evidence = build_depth_calibration_evidence(
                points, self.frozen_depth, self.frozen_camera_info,
                self.frozen_extrinsics, self.distance,
                rgb_stamp_ns=self.rgb_source_stamp_ns,
                rgb_frame_id=self.rgb_frame_id,
                rgb_topic=self.window.args.tape_image_topic,
                depth_topic=self.window.args.registered_depth_topic,
            )
            if self.session_dir is None:
                self.session_dir = Path(self.window.args.output_dir)/'floor_calibration'/str(time.time_ns())
                self.session_dir.mkdir(parents=True)
            image = self.session_dir/f'sample_{self.sample_id}.png'
            if not cv2.imwrite(str(image), self.canvas.frame):
                raise ValueError('샘플 이미지 저장 실패')
            sample = {'sample_id': self.sample_id, 'far_line_cm': self.distance, 'image_size': size, 'top_line_px': points.tolist(),
                      'image': str(image), 'captured_monotonic': self.frame_at,
                      'rgb_source_stamp_ns': self.rgb_source_stamp_ns,
                      'rgb_frame_id': self.rgb_frame_id,
                      'line_definition': 'selected_square_tl_to_tr',
                      'load_state': 'loaded', 'alignment_source': 'operator_aligned',
                      'registered_depth_evidence': evidence}
            (self.session_dir/f'sample_{self.sample_id}.json').write_text(json.dumps(sample, indent=2)+'\n')
            self.samples[self.sample_id] = sample
            self.status.setText(
                '저장된 샘플: '
                + ', '.join(f"{i}번 {s['far_line_cm']:g}cm" for i, s in sorted(self.samples.items()))
                + f" · depth {evidence['fork_frame_line_pose']['forward_cm']:.1f}cm"
                + f" (입력 대비 {evidence['forward_error_cm']:+.1f}cm)"
            )
        except (ValueError, OSError) as exc:
            self.status.setText(str(exc))

    def finish(self):
        try:
            profile = fit_floor_calibration(list(self.samples.values()), self.width_cm.value(),
                vehicle=self.window.args.vehicle, source_topic=self.window.args.tape_image_topic)
            depth_fit = fit_depth_extrinsics(list(self.samples.values()))
            profile['depth_extrinsic_calibration'] = depth_fit
            fitted = depth_fit['fitted_extrinsics']
            save_floor_profile(
                self.window.args.pose_config, profile,
                config_updates={
                    'y_slot_depth_camera_pitch_deg': fitted['camera_pitch_deg'],
                    'y_slot_depth_camera_to_fork_tip_offset_cm': fitted[
                        'depth_camera_to_fork_tip_offset_cm'
                    ],
                    'y_slot_depth_extrinsic_calibration': depth_fit,
                },
            )
            self.profile = profile
            self.saved(profile)
            self.status.setText(
                '저장 완료 · depth pitch '
                f"{fitted['camera_pitch_deg']:+.3f}° · 렌즈→포크 "
                f"{fitted['depth_camera_to_fork_tip_offset_cm']:.2f}cm · "
                f"fit RMSE {depth_fit['residual_rmse_cm']:.2f}cm"
            )
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.status.setText(str(exc))

    def finish_depth_only(self):
        try:
            depth_fit = fit_depth_extrinsics(list(self.samples.values()))
            fitted = depth_fit['fitted_extrinsics']
            save_floor_profile(
                self.window.args.pose_config, None,
                config_updates={
                    'y_slot_depth_camera_pitch_deg': fitted['camera_pitch_deg'],
                    'y_slot_depth_camera_to_fork_tip_offset_cm': fitted[
                        'depth_camera_to_fork_tip_offset_cm'
                    ],
                    'y_slot_depth_extrinsic_calibration': depth_fit,
                },
            )
            self.status.setText(
                'Y Depth만 저장 완료 · DOCK/Homography 변경 없음 · pitch '
                f"{fitted['camera_pitch_deg']:+.3f}° · 렌즈→포크 "
                f"{fitted['depth_camera_to_fork_tip_offset_cm']:.2f}cm · "
                f"fit RMSE {depth_fit['residual_rmse_cm']:.2f}cm"
            )
        except (ValueError, OSError, KeyError, TypeError) as exc:
            self.status.setText(str(exc))

    def capture_check(self):
        try:
            if not self.profile:
                raise ValueError('두 샘플로 보정값을 먼저 저장하세요')
            self.canvas.frame = self.fresh_frame()[0]["frame"]
            self.canvas.points = []
            self.check_mode = True
            self.canvas.update()
            self.status.setText('고정된 화면의 X 중심 등을 클릭하면 포크 기준 좌우·전방 거리와 방위각을 표시합니다')
        except ValueError as exc:
            self.status.setText(str(exc))

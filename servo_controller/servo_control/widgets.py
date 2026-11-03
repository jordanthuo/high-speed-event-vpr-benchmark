"""Tk canvas presentation components for telemetry and profile previews.

Widgets render state supplied by ui.py and should not acquire hardware or send
commands. Angles passed to the compass are session-relative; trend angles and
profile origins follow their respective caller contracts. Keep drawing work
bounded/throttled in the caller so acquisition queues are not starved.
Canvas y increases downward; the compass shifts degrees by -90 for North.
"""

import math
import tkinter as tk
from collections.abc import Iterable

from .telemetry import TelemetrySample


NAVY = "#0B2E52"
BLUE = "#0B67E8"
TEAL = "#0E9F86"
GREEN = "#159447"
ORANGE = "#EA7A12"
RED = "#CC2E3B"
INK = "#14233A"
MUTED = "#637083"
GRID = "#DCE3EA"
SURFACE = "#FFFFFF"


class MotionCompass(tk.Canvas):
    """Session-relative platform and motor orientation display."""

    def __init__(self, master: tk.Misc, **kwargs: object) -> None:
        kwargs.setdefault("height", 235)
        super().__init__(master, background=SURFACE, borderwidth=0, highlightthickness=0, **kwargs)
        self.motor_angle: float | None = None
        self.platform_angle: float | None = None
        self.target_platform_angle: float | None = None
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_values(
        self,
        motor_angle: float | None,
        platform_angle: float | None,
        target_platform_angle: float | None = None,
    ) -> None:
        self.motor_angle = motor_angle
        self.platform_angle = platform_angle
        self.target_platform_angle = target_platform_angle
        self.redraw()

    @staticmethod
    def _point(cx: float, cy: float, radius: float, angle_deg: float) -> tuple[float, float]:
        angle = math.radians(angle_deg - 90.0)
        return cx + radius * math.cos(angle), cy + radius * math.sin(angle)

    def redraw(self) -> None:
        self.delete("all")
        width, height = max(260, self.winfo_width()), max(230, self.winfo_height())
        cx, cy = width / 2.0, height * 0.50
        radius = min(width * 0.28, height * 0.30, 72)
        self.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#AEB9C6", width=3)
        for angle in range(0, 360, 15):
            outer = self._point(cx, cy, radius, angle)
            inner = self._point(cx, cy, radius - (12 if angle % 90 == 0 else 6), angle)
            self.create_line(*inner, *outer, fill=INK if angle % 90 == 0 else "#AEB9C6", width=2 if angle % 90 == 0 else 1)
        for angle, label in ((0, "N  0°"), (90, "E  90°"), (180, "S  180°"), (270, "W  270°")):
            x, y = self._point(cx, cy, radius + 32, angle)
            self.create_text(x, y, text=label, fill=INK, font=("Segoe UI", 8, "bold"))

        if self.platform_angle is not None:
            front = self._point(cx, cy, radius - 14, self.platform_angle)
            rear = self._point(cx, cy, radius - 14, self.platform_angle + 180)
            self.create_line(*rear, *front, fill="#41556F", width=9, capstyle="round")
            self.create_oval(front[0] - 8, front[1] - 8, front[0] + 8, front[1] + 8, fill=TEAL, outline=SURFACE, width=2)
            angle_x, angle_y = self._point(cx, cy, radius + 18, self.platform_angle)
            self.create_text(angle_x, angle_y, text=f"{self.platform_angle % 360:.1f}°", fill=TEAL, font=("Segoe UI", 8, "bold"))

        if self.motor_angle is not None:
            tip = self._point(cx, cy, radius * 0.52, self.motor_angle)
            self.create_line(cx, cy, *tip, fill=ORANGE, width=5, arrow="last", arrowshape=(12, 14, 5))
        self.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill=INK, outline=SURFACE)

        # The only rotating numeric annotation is the wrapped platform angle.


class ProfilePreview(tk.Canvas):
    """Requested profile preview with motor/platform traces."""

    def __init__(self, master: tk.Misc, **kwargs: object) -> None:
        kwargs.setdefault("height", 330)
        super().__init__(master, background=SURFACE, borderwidth=0, highlightthickness=0, **kwargs)
        self.profile = None
        self.cursor_s: float | None = None
        self.samples = []
        self.channels = {
            "command": True,
            "estimated_motor": True,
            "estimated_platform": True,
            "recorded_motor": True,
            "recorded_platform": True,
        }
        self.acceleration_s = 2.0
        self.deceleration_s = 2.0
        self.speed_limit_rpm = 100.0
        self.position_origin_motor_deg = 0.0
        self.position_target_history: list[tuple[float, float]] = []
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_profile(self, profile: object | None, cursor_s: float | None = None) -> None:
        self.profile, self.cursor_s = profile, cursor_s
        self.redraw()

    def set_recorded_samples(self, samples: Iterable[object]) -> None:
        self.samples = list(samples)
        self.redraw()

    def set_channels(self, **channels: bool) -> None:
        self.channels.update(channels)
        self.redraw()

    def set_ramps(self, acceleration_s: float, deceleration_s: float, speed_limit_rpm: float) -> None:
        self.acceleration_s = max(0.001, acceleration_s)
        self.deceleration_s = max(0.001, deceleration_s)
        self.speed_limit_rpm = max(0.001, speed_limit_rpm)
        self.redraw()

    def set_position_origin(self, motor_degrees: float) -> None:
        self.position_origin_motor_deg = motor_degrees
        self.redraw()

    def set_position_targets(self, targets: Iterable[tuple[float, float]]) -> None:
        self.position_target_history = list(targets)
        self.redraw()

    def _speed_series(self, profile: object) -> tuple[list[float], list[float], list[float]]:
        duration = max(profile.duration_s, 0.001)
        count = max(2, min(2401, int(duration * 20) + 1))
        times = [duration * index / (count - 1) for index in range(count)]
        command: list[float] = []
        point_index = 0
        for time_s in times:
            while point_index + 1 < len(profile.points) and time_s >= profile.points[point_index + 1].time_s:
                point_index += 1
            command.append(profile.points[point_index].value)
        estimated = [command[0]]
        accel_rate = self.speed_limit_rpm / self.acceleration_s
        decel_rate = self.speed_limit_rpm / self.deceleration_s
        for index in range(1, len(times)):
            current, target = estimated[-1], command[index]
            dt = times[index] - times[index - 1]
            next_value = current
            remaining = dt
            if current * target < 0:
                time_to_zero = abs(current) / decel_rate
                if remaining <= time_to_zero:
                    next_value = current - math.copysign(decel_rate * remaining, current)
                    remaining = 0.0
                else:
                    next_value = 0.0
                    remaining -= time_to_zero
            if remaining > 0:
                accelerating = abs(target) > abs(next_value)
                rate = accel_rate if accelerating else decel_rate
                delta = target - next_value
                next_value += math.copysign(min(abs(delta), rate * remaining), delta) if delta else 0.0
            estimated.append(next_value)
        return times, command, estimated

    def redraw(self) -> None:
        self.delete("all")
        width, height = max(460, self.winfo_width()), max(260, self.winfo_height())
        left, right, top, bottom = 64, width - 22, 72, height - 40
        self.create_line(left, top, left, bottom, fill="#AAB5C2")
        self.create_line(left, bottom, right, bottom, fill="#AAB5C2")
        if self.profile is None:
            self.create_text((left + right) / 2, (top + bottom) / 2, text="Import a CSV to preview the requested motion", fill=MUTED, font=("Segoe UI", 11))
            return
        profile = self.profile
        series: list[tuple[str, list[float], list[float], str, tuple[int, int] | None, int]] = []
        if profile.kind == "speed":
            times, command, estimated = self._speed_series(profile)
            if self.channels["command"]:
                command_times = [profile.points[0].time_s]
                command_steps = [profile.points[0].value]
                for previous, point in zip(profile.points, profile.points[1:]):
                    command_times.extend((point.time_s, point.time_s))
                    command_steps.extend((previous.value, point.value))
                series.append(("Motor speed command", command_times, command_steps, BLUE, None, 2))
            if self.channels["estimated_motor"]:
                series.append(("Estimated Motor Speed Profile", times, estimated, ORANGE, None, 3))
            if self.channels["estimated_platform"]:
                series.append(("Estimated Platform Speed Profile", times, [value / profile.ratio for value in estimated], "#7447D6", (7, 4), 2))
            recorded_times = [sample.elapsed_s for sample in self.samples if sample.encoder_feedback_rpm is not None and sample.elapsed_s <= profile.duration_s]
            if self.channels["recorded_motor"]:
                series.append(("Recorded Motor Speed Feedback", recorded_times, [sample.encoder_feedback_rpm for sample in self.samples if sample.encoder_feedback_rpm is not None and sample.elapsed_s <= profile.duration_s], TEAL, (4, 3), 2))
            if self.channels["recorded_platform"]:
                series.append(("Recorded Platform Speed Feedback", recorded_times, [sample.platform_speed_rpm for sample in self.samples if sample.encoder_feedback_rpm is not None and sample.elapsed_s <= profile.duration_s], "#B04BC4", (3, 3), 2))
            ylabel, title = "rpm", "COMMAND / ESTIMATED PROFILE / RECORDED FEEDBACK"
        else:
            times = [point.time_s for point in profile.points]
            platform_command = [point.value for point in profile.points]
            estimated_motor = [value * profile.ratio for value in platform_command]
            if self.channels["command"]:
                if self.position_target_history:
                    target_times = [self.position_target_history[0][0]]
                    target_values = [self.position_target_history[0][1]]
                    for previous, current in zip(
                        self.position_target_history,
                        self.position_target_history[1:],
                    ):
                        target_times.extend((current[0], current[0]))
                        target_values.extend((previous[1], current[1]))
                    series.append(("Active Platform Target", target_times, target_values, BLUE, None, 2))
                else:
                    series.append(("Nominal Platform Trajectory", times, platform_command, BLUE, None, 2))
            if self.channels["estimated_motor"]:
                series.append(("Nominal Motor Trajectory", times, estimated_motor, ORANGE, None, 3))
            if self.channels["estimated_platform"]:
                series.append(("Nominal Platform Trajectory", times, platform_command, "#7447D6", (7, 4), 2))
            recorded = list(self.samples)
            recorded_times = [sample.elapsed_s for sample in recorded]
            recorded_motor = [None if sample.motor_position_deg is None else sample.motor_position_deg - self.position_origin_motor_deg for sample in recorded]
            recorded_platform = [None if value is None else value / profile.ratio for value in recorded_motor]
            if self.channels["recorded_motor"]:
                series.append(("Recorded Motor Position Feedback", recorded_times, recorded_motor, TEAL, (4, 3), 2))
            if self.channels["recorded_platform"]:
                series.append(("Recorded Platform Position Feedback", recorded_times, recorded_platform, "#B04BC4", (3, 3), 2))
            ylabel, title = "deg", "ACTIVE TARGET / NOMINAL TRAJECTORY / RECORDED FEEDBACK"
        all_values = [value for _name, _times, values, _color, _dash, _width in series for value in values if value is not None]
        if not all_values:
            all_values = [0.0]
        low, high = min(all_values), max(all_values)
        span = max(1.0, high - low)
        low, high = low - span * 0.1, high + span * 0.1
        self.create_text(left, 16, anchor="w", text=title, fill=INK, font=("Segoe UI", 10, "bold"))
        for index in range(5):
            y = top + index * (bottom - top) / 4
            value = high - index * (high - low) / 4
            self.create_line(left, y, right, y, fill="#DCE3EA", dash=(3, 4))
            self.create_text(left - 8, y, anchor="e", text=f"{value:+.1f}", fill=MUTED, font=("Segoe UI", 8))
        self.create_text(12, top - 12, anchor="w", text=ylabel, fill=MUTED, font=("Segoe UI", 8))
        duration = max(
            profile.duration_s,
            max((sample.elapsed_s for sample in self.samples), default=0.0),
            max((item[0] for item in self.position_target_history), default=0.0),
            0.001,
        )
        def points(times: list[float], values: list[float]) -> list[float]:
            output: list[float] = []
            for time_s, value in zip(times, values):
                if value is not None:
                    output.extend((left + time_s / duration * (right - left), top + (high - value) / (high - low) * (bottom - top)))
            return output
        legend_x, legend_y = left, 34
        for name, times, values, color, dash, line_width in series:
            plotted = points(times, values)
            if len(plotted) >= 4:
                self.create_line(*plotted, fill=color, width=line_width, dash=dash)
            item_width = max(116, len(name) * 4.6 + 38)
            if legend_x + item_width > right:
                legend_x = left
                legend_y += 16
            self.create_line(legend_x, legend_y, legend_x + 20, legend_y, fill=color, width=line_width, dash=dash)
            self.create_text(legend_x + 26, legend_y, anchor="w", text=name, fill=INK, font=("Segoe UI", 7))
            legend_x += item_width
        for index in range(5):
            x = left + index * (right - left) / 4
            self.create_text(x, bottom + 17, text=f"{duration * index / 4:g}s", fill=MUTED, font=("Segoe UI", 8))
        if self.cursor_s is not None:
            x = left + max(0.0, min(duration, self.cursor_s)) / duration * (right - left)
            self.create_line(x, top, x, bottom, fill=RED, width=2)


class SceneDisplacementChart(tk.Canvas):
    """Time-aligned LabJack AIN0 scene vertical displacement for one profile run."""

    def __init__(self, master: tk.Misc, **kwargs: object) -> None:
        kwargs.setdefault("height", 190)
        super().__init__(master, background=SURFACE, borderwidth=0, highlightthickness=0, **kwargs)
        self.samples: list[TelemetrySample] = []
        self.nominal_duration_s = 0.0
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_samples(self, samples: Iterable[TelemetrySample], nominal_duration_s: float = 0.0) -> None:
        self.samples = list(samples)
        self.nominal_duration_s = max(0.0, nominal_duration_s)
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width, height = max(460, self.winfo_width()), max(170, self.winfo_height())
        left, right, top, bottom = 64, width - 22, 38, height - 34
        self.create_text(left, 16, anchor="w", text="SCENE VERTICAL DISPLACEMENT — LABJACK AIN0", fill=INK, font=("Segoe UI", 10, "bold"))
        self.create_line(left, top, left, bottom, fill="#AAB5C2")
        self.create_line(left, bottom, right, bottom, fill="#AAB5C2")
        valid = [sample for sample in self.samples if sample.drawstring_extension_mm is not None]
        duration = max(
            self.nominal_duration_s,
            max((sample.elapsed_s for sample in self.samples), default=0.0),
            0.001,
        )
        if not valid:
            self.create_text(
                (left + right) / 2,
                (top + bottom) / 2,
                text="Waiting for profile-run LabJack AIN0 data",
                fill=MUTED,
                font=("Segoe UI", 10),
            )
            return
        values = [float(sample.drawstring_extension_mm) for sample in valid if sample.drawstring_extension_mm is not None]
        low, high = min(values), max(values)
        span = max(1.0, high - low)
        low, high = low - span * 0.1, high + span * 0.1
        for index in range(5):
            y = top + index * (bottom - top) / 4
            value = high - index * (high - low) / 4
            self.create_line(left, y, right, y, fill=GRID, dash=(3, 4))
            self.create_text(left - 8, y, text=f"{value:.1f}", anchor="e", fill=MUTED, font=("Segoe UI", 8))
            x = left + index * (right - left) / 4
            self.create_text(x, bottom + 16, text=f"{duration * index / 4:g}s", fill=MUTED, font=("Segoe UI", 8))
        self.create_text(12, top - 10, anchor="w", text="mm", fill=MUTED, font=("Segoe UI", 8))
        points: list[float] = []
        for sample in valid:
            x = left + min(duration, sample.elapsed_s) / duration * (right - left)
            y = top + (high - float(sample.drawstring_extension_mm)) / (high - low) * (bottom - top)
            points.extend((x, y))
        if len(points) >= 4:
            self.create_line(*points, fill="#7447D6", width=2)


class SpeedGauge(tk.Canvas):
    def __init__(self, master: tk.Misc, minimum: float = -100, maximum: float = 100, **kwargs: object) -> None:
        self.compact = bool(kwargs.pop("compact", False))
        kwargs.setdefault("height", 215 if self.compact else 285)
        super().__init__(
            master,
            background=SURFACE,
            borderwidth=0,
            highlightthickness=0,
            **kwargs,
        )
        self.minimum = minimum
        self.maximum = maximum
        self.feedback: float | None = None
        self.setpoint = 0.0
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_values(self, feedback: float | None, setpoint: float) -> None:
        self.feedback = feedback
        self.setpoint = max(self.minimum, min(self.maximum, setpoint))
        self.redraw()

    def set_range(self, minimum: float, maximum: float) -> None:
        if minimum >= maximum:
            raise ValueError("Gauge minimum must be below maximum")
        self.minimum = minimum
        self.maximum = maximum
        self.setpoint = max(self.minimum, min(self.maximum, self.setpoint))
        self.redraw()

    def _angle(self, value: float) -> float:
        fraction = (value - self.minimum) / (self.maximum - self.minimum)
        return 180.0 * (1.0 - fraction)

    def redraw(self) -> None:
        self.delete("all")
        width = max(320, self.winfo_width())
        height = max(205 if self.compact else 250, self.winfo_height())
        cx = width / 2
        cy = height * 0.72
        # Reserve space outside the arc for the setpoint value and scale labels.
        radius = min(width * (0.34 if self.compact else 0.40), height * (0.48 if self.compact else 0.54), cx - 52, cy - 48)
        box = (cx - radius, cy - radius, cx + radius, cy + radius)
        self.create_arc(*box, start=0, extent=180, style="arc", width=5, outline="#AEB9C6")

        for index in range(21):
            value = self.minimum + index * (self.maximum - self.minimum) / 20
            angle = math.radians(self._angle(value))
            outer = radius
            inner = radius - (15 if index % 5 == 0 else 8)
            x1, y1 = cx + inner * math.cos(angle), cy - inner * math.sin(angle)
            x2, y2 = cx + outer * math.cos(angle), cy - outer * math.sin(angle)
            self.create_line(x1, y1, x2, y2, fill=INK if index % 5 == 0 else "#8B98A8", width=2 if index % 5 == 0 else 1)
            if index % 5 == 0:
                lx = cx + (radius + 42) * math.cos(angle)
                ly = cy - (radius + 42) * math.sin(angle)
                self.create_text(lx, ly, text=f"{value:+.0f}" if value else "0", fill=INK, font=("Segoe UI", 9, "bold"))

        marker_angle = math.radians(self._angle(self.setpoint))
        marker_radius = radius + 4
        mx = cx + marker_radius * math.cos(marker_angle)
        my = cy - marker_radius * math.sin(marker_angle)
        tangent_x, tangent_y = -math.sin(marker_angle), -math.cos(marker_angle)
        inward_x, inward_y = -math.cos(marker_angle), math.sin(marker_angle)
        points = [
            mx + inward_x * 16,
            my + inward_y * 16,
            mx + tangent_x * 8,
            my + tangent_y * 8,
            mx - tangent_x * 8,
            my - tangent_y * 8,
        ]
        self.create_polygon(points, fill=BLUE, outline=BLUE)

        # The value sits just outside the triangle base. Major scale labels are
        # on a separate, more distant radius so the two label layers stay clear.
        setpoint_label_radius = radius + 23
        label_x = cx + setpoint_label_radius * math.cos(marker_angle)
        label_y = cy - setpoint_label_radius * math.sin(marker_angle)
        midpoint = (self.minimum + self.maximum) / 2.0
        label_side = 1.0 if self.setpoint >= midpoint else -1.0
        label_x += label_side * math.sin(marker_angle) * 14
        label_y += label_side * math.cos(marker_angle) * 14

        if self.feedback is not None:
            needle_angle = math.radians(self._angle(max(self.minimum, min(self.maximum, self.feedback))))
            needle_inner_radius = radius * 0.64
            needle_outer_radius = radius - 25
            inner_x = cx + needle_inner_radius * math.cos(needle_angle)
            inner_y = cy - needle_inner_radius * math.sin(needle_angle)
            outer_x = cx + needle_outer_radius * math.cos(needle_angle)
            outer_y = cy - needle_outer_radius * math.sin(needle_angle)
            self.create_line(
                inner_x,
                inner_y,
                outer_x,
                outer_y,
                fill=INK,
                width=6,
                capstyle="round",
            )

        self.create_text(
            label_x,
            label_y,
            text=f"{self.setpoint:+.1f}",
            fill=BLUE,
            font=("Segoe UI", 9, "bold"),
        )

        value_text = "--" if self.feedback is None else f"{self.feedback:+.1f}"
        self.create_text(cx, cy - 42, text=value_text, fill=INK, font=("Segoe UI", 25 if self.compact else 31, "bold"))
        self.create_text(cx, cy - 12, text="rpm feedback", fill=MUTED, font=("Segoe UI", 9 if self.compact else 10))
        direction_value = self.feedback if self.feedback is not None else 0
        direction = "FORWARD" if direction_value > 0.05 else "REVERSE" if direction_value < -0.05 else "STOPPED"
        self.create_text(cx, cy + 22, text=direction, fill=TEAL if direction != "STOPPED" else MUTED, font=("Segoe UI", 10 if self.compact else 12, "bold"))
        self.create_line(cx - 115, height - 22, cx - 90, height - 22, fill=INK, width=4)
        self.create_text(cx - 82, height - 22, text="Feedback", anchor="w", fill=MUTED, font=("Segoe UI", 8))
        self.create_polygon(cx + 35, height - 29, cx + 45, height - 15, cx + 25, height - 15, fill=BLUE)
        self.create_text(cx + 54, height - 22, text="Setpoint", anchor="w", fill=MUTED, font=("Segoe UI", 8))


class TrendChart(tk.Canvas):
    def __init__(self, master: tk.Misc, compact: bool = False, **kwargs: object) -> None:
        super().__init__(
            master,
            background=SURFACE,
            borderwidth=0,
            highlightthickness=0,
            height=210 if compact else 460,
            **kwargs,
        )
        self.compact = compact
        self.samples: list[TelemetrySample] = []
        self.show_command = True
        self.show_feedback = True
        self.show_error = not compact
        self.show_platform = False
        self.platform_label = "Platform speed 2:1"
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_samples(self, samples: Iterable[TelemetrySample]) -> None:
        self.samples = list(samples)
        self.redraw()

    def set_channels(self, command: bool, feedback: bool, error: bool, platform: bool) -> None:
        self.show_command = command
        self.show_feedback = feedback
        self.show_error = error
        self.show_platform = platform
        self.redraw()

    def set_platform_ratio(self, ratio: float) -> None:
        self.platform_label = f"Platform speed {ratio:g}:1"
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(400, self.winfo_width())
        height = max(170, self.winfo_height())
        left, right, top, bottom = 52, width - 18, 52 if self.compact else 34, height - 32
        plot_width, plot_height = right - left, bottom - top

        values: list[float] = []
        for sample in self.samples:
            for value in (
                sample.integrated_command_rpm if self.show_command else None,
                sample.encoder_feedback_rpm if self.show_feedback else None,
                sample.tracking_error_rpm if self.show_error else None,
                sample.platform_speed_rpm if self.show_platform else None,
            ):
                if value is not None:
                    values.append(value)
        maximum = max(100.0, max((abs(value) for value in values), default=0.0) * 1.15)
        maximum = math.ceil(maximum / 20.0) * 20.0

        for index in range(5):
            y = top + index * plot_height / 4
            value = maximum - index * maximum * 2 / 4
            self.create_line(left, y, right, y, fill=GRID, dash=(3, 4))
            self.create_text(left - 8, y, text=f"{value:+.0f}", anchor="e", fill=MUTED, font=("Segoe UI", 8))
        self.create_line(left, top, left, bottom, fill="#AAB5C2")
        self.create_line(left, bottom, right, bottom, fill="#AAB5C2")

        if self.samples:
            start_time = self.samples[0].elapsed_s
            end_time = max(start_time + 1.0, self.samples[-1].elapsed_s)

            def point(sample: TelemetrySample, value: float) -> tuple[float, float]:
                x = left + (sample.elapsed_s - start_time) / (end_time - start_time) * plot_width
                y = top + (maximum - value) / (2 * maximum) * plot_height
                return x, y

            series = [
                (self.show_command, lambda item: item.integrated_command_rpm, BLUE, None, 3),
                (self.show_feedback, lambda item: item.encoder_feedback_rpm, TEAL, (7, 4), 2),
                (self.show_error, lambda item: item.tracking_error_rpm, ORANGE, (5, 4), 2),
                (self.show_platform, lambda item: item.platform_speed_rpm, "#7447D6", None, 2),
            ]
            for visible, accessor, color, dash, line_width in series:
                if not visible:
                    continue
                points: list[float] = []
                for sample in self.samples:
                    value = accessor(sample)
                    if value is not None:
                        points.extend(point(sample, value))
                if len(points) >= 4:
                    self.create_line(*points, fill=color, width=line_width, smooth=False, dash=dash)

            duration = end_time - start_time
            for index in range(5):
                x = left + index * plot_width / 4
                relative = -duration + index * duration / 4
                self.create_text(x, bottom + 15, text=f"{relative:.0f}s", fill=MUTED, font=("Segoe UI", 8))
        else:
            self.create_text((left + right) / 2, (top + bottom) / 2, text="Waiting for telemetry", fill=MUTED, font=("Segoe UI", 11))

        legend_x = left
        legends: list[tuple[str, str, tuple[int, int] | None]] = []
        if self.show_command:
            legends.append(("Requested motor speed", BLUE, None))
        if self.show_feedback:
            legends.append(("Encoder feedback", TEAL, (7, 4)))
        if self.show_error:
            legends.append(("Tracking error", ORANGE, (5, 4)))
        if self.show_platform:
            legends.append((self.platform_label, "#7447D6", None))
        if self.compact:
            for index, (text, color, dash) in enumerate(legends):
                legend_y = 13 + index * 16
                self.create_line(legend_x, legend_y, legend_x + 20, legend_y, fill=color, width=3, dash=dash)
                self.create_text(legend_x + 27, legend_y, text=text, anchor="w", fill=INK, font=("Segoe UI", 8))
        else:
            for text, color, dash in legends:
                self.create_line(legend_x, 17, legend_x + 20, 17, fill=color, width=3, dash=dash)
                self.create_text(legend_x + 27, 17, text=text, anchor="w", fill=INK, font=("Segoe UI", 8))
                legend_x += 145


class PositionChart(tk.Canvas):
    """Periodic motor/platform orientation trend using sin(theta)."""

    def __init__(self, master: tk.Misc, **kwargs: object) -> None:
        super().__init__(
            master,
            background=SURFACE,
            borderwidth=0,
            highlightthickness=0,
            height=230,
            **kwargs,
        )
        self.samples: list[TelemetrySample] = []
        self.show_motor = True
        self.show_platform = True
        self.platform_label = "Platform sin(θ) 2:1"
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_samples(self, samples: Iterable[TelemetrySample]) -> None:
        self.samples = list(samples)
        self.redraw()

    def set_platform_ratio(self, ratio: float) -> None:
        self.platform_label = f"Platform sin(θ) {ratio:g}:1"
        self.redraw()

    def set_channels(self, motor: bool, platform: bool) -> None:
        self.show_motor = motor
        self.show_platform = platform
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(400, self.winfo_width())
        height = max(180, self.winfo_height())
        left, right, top, bottom = 60, width - 18, 34, height - 32
        plot_width, plot_height = right - left, bottom - top
        minimum, maximum = -1.0, 1.0

        for index in range(5):
            y = top + index * plot_height / 4
            value = maximum - index * (maximum - minimum) / 4
            self.create_line(left, y, right, y, fill=GRID, dash=(3, 4))
            self.create_text(left - 8, y, text=f"{value:+.2f}", anchor="e", fill=MUTED, font=("Segoe UI", 8))
        self.create_line(left, top, left, bottom, fill="#AAB5C2")
        self.create_line(left, bottom, right, bottom, fill="#AAB5C2")

        if self.samples:
            start_time = self.samples[0].elapsed_s
            end_time = max(start_time + 1.0, self.samples[-1].elapsed_s)

            def draw_series(accessor: object, color: str, dash: tuple[int, int] | None = None) -> None:
                points: list[float] = []
                for sample in self.samples:
                    value = accessor(sample)  # type: ignore[operator]
                    if value is None:
                        continue
                    x = left + (sample.elapsed_s - start_time) / (end_time - start_time) * plot_width
                    y = top + (maximum - value) / (maximum - minimum) * plot_height
                    points.extend((x, y))
                if len(points) >= 4:
                    self.create_line(*points, fill=color, width=2, dash=dash)

            if self.show_motor:
                draw_series(
                    lambda item: None if item.motor_position_deg is None else math.sin(math.radians(item.motor_position_deg)),
                    BLUE,
                )
            if self.show_platform:
                draw_series(
                    lambda item: None if item.platform_position_deg is None else math.sin(math.radians(item.platform_position_deg)),
                    "#7447D6",
                    (7, 4),
                )
            duration = end_time - start_time
            for index in range(5):
                x = left + index * plot_width / 4
                relative = -duration + index * duration / 4
                self.create_text(x, bottom + 15, text=f"{relative:.0f}s", fill=MUTED, font=("Segoe UI", 8))
        else:
            self.create_text((left + right) / 2, (top + bottom) / 2, text="Waiting for P0.011 orientation telemetry", fill=MUTED, font=("Segoe UI", 10))

        legends: list[tuple[str, str, tuple[int, int] | None]] = []
        if self.show_motor:
            legends.append(("Motor sin(θ)", BLUE, None))
        if self.show_platform:
            legends.append((self.platform_label, "#7447D6", (7, 4)))
        legend_x = left
        for text, color, dash in legends:
            self.create_line(legend_x, 17, legend_x + 20, 17, fill=color, width=3, dash=dash)
            self.create_text(legend_x + 27, 17, text=text, anchor="w", fill=INK, font=("Segoe UI", 8))
            legend_x += 165

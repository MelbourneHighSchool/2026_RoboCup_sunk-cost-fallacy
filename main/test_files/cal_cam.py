"""Calibrate HSV values for ball and goals."""
from lib.Vision import Vision
from lib.interface import WSServer
from lib import term

import cv2

def main(config):
    vision = Vision()
    vision.start()

    server = WSServer()
    server.run()
    # load hsv values from config
    hsv = config.get_value("hsv")
    if hsv is None:
        hsv = {
            "ball": {"low": (0, 0, 0), "high": (0, 0, 0)},
            "bgoal": {"low": (0, 0, 0), "high": (0, 0, 0)},
            "ygoal": {"low": (0, 0, 0), "high": (0, 0, 0)}
        }
    ball_low, ball_high = hsv["ball"].values()
    bgoal_low, bgoal_high = hsv["bgoal"].values()
    ygoal_low, ygoal_high = hsv["ygoal"].values()

    # run hsv input for ball and goals
    ball_hsv = term.HSVInput("Ball HSV", ball_low, ball_high, "1/3")
    while ball_hsv.is_running():
        # Update ball bounds in vision
        vision.ball_bounds_v[:] = ball_hsv.low + ball_hsv.high
        vision.wait_next_frame()
        # Draw on frame
        frame = vision.camera.latest_frame
        center = (frame.shape[1] // 2 - 20, frame.shape[0] // 2 + 55)

        angle, dist, x, y, r = vision.ball_info
        frame = cv2.line(frame, (center[0], center[1]), (x, y), (50, 50, 255), 2, cv2.LINE_AA)
        frame = cv2.circle(frame, (x, y), r, (50, 50, 255), 2, cv2.LINE_AA)

        frame = cv2.putText(frame, f"Ball angle: {angle}", (15, 25), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"Ball dist: {dist}", (15, 55), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"Low: {ball_hsv.low}", (15, 85), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"High: {ball_hsv.high}", (15, 115), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        # Send frame
        server.send_frame(frame)
    if ball_hsv.quit:
        return

    bgoal_hsv = term.HSVInput("Blue Goal HSV", bgoal_low, bgoal_high, "2/3")
    while bgoal_hsv.is_running():
        # Update goal bounds in vision
        vision.goal_bounds_v[:] = list(bgoal_hsv.low) + list(bgoal_hsv.high) + list(ygoal_low) + list(ygoal_high)
        vision.wait_next_frame()
        # Draw on frame
        frame = vision.camera.latest_frame
        center = (frame.shape[1] // 2 - 20, frame.shape[0] // 2 + 55)

        raw_angle, ang_width, distance, x, y, w, h, rot = vision.bgoal_info
        angle = -(raw_angle * 180 / 32767 + 90) % 360
        frame = cv2.line(frame, (center[0], center[1]), (x, y), (50, 50, 255), 2, cv2.LINE_AA)
        frame = cv2.rectangle(frame, cv2.RotatedRect((float(x), float(y)), (float(w), float(h)), float(rot)).boundingRect(), (50, 50, 255), 2, cv2.LINE_AA)

        frame = cv2.putText(frame, f"Goal angle: {angle}", (15, 25), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"Low: {bgoal_hsv.low}", (15, 55), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"High: {bgoal_hsv.high}", (15, 85), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        # Send frame
        server.send_frame(frame)
    if bgoal_hsv.quit:
        return
    
    ygoal_hsv = term.HSVInput("Yellow Goal HSV", ygoal_low, ygoal_high, "3/3")
    while ygoal_hsv.is_running():
        # Update goal bounds in vision
        vision.goal_bounds_v[:] = list(bgoal_hsv.low) + list(bgoal_hsv.high) + list(ygoal_hsv.low) + list(ygoal_hsv.high)
        vision.wait_next_frame()
        # Draw on frame
        frame = vision.camera.latest_frame
        center = (frame.shape[1] // 2 - 20, frame.shape[0] // 2 + 55)

        raw_angle, ang_width, distance, x, y, w, h, rot = vision.ygoal_info
        angle = -(raw_angle * 180 / 32767 + 90) % 360
        frame = cv2.line(frame, (center[0], center[1]), (x, y), (50, 50, 255), 2, cv2.LINE_AA)
        frame = cv2.rectangle(frame, cv2.RotatedRect((float(x), float(y)), (float(w), float(h)), float(rot)).boundingRect(), (50, 50, 255), 2, cv2.LINE_AA)

        frame = cv2.putText(frame, f"Goal angle: {angle}", (15, 25), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"Low: {ygoal_hsv.low}", (15, 55), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        frame = cv2.putText(frame, f"High: {ygoal_hsv.high}", (15, 85), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
        # Send frame
        server.send_frame(frame)
    if ygoal_hsv.quit:
        return
    # save all values
    config.set_value("hsv", {
        "ball": {"low": ball_hsv.low, "high": ball_hsv.high},
        "bgoal": {"low": bgoal_hsv.low, "high": bgoal_hsv.high},
        "ygoal": {"low": ygoal_hsv.low, "high": ygoal_hsv.high}
    })
    config.save_config()
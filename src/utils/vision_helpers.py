# -*- coding: utf-8 -*-
"""
Hjælpefunktioner til vision systemet
"""

from shapely.geometry import LineString, Point
from ..config.settings import *

def is_cross_blocking_path(robot_head, robot_tail, ball_pos, cross_pos):
    """Tjek om kors blokerer stien fra robot til bold"""
    if not cross_pos:
        return False
    robot_x = (robot_head["pos"][0] + robot_tail["pos"][0]) // 2
    robot_y = (robot_head["pos"][1] + robot_tail["pos"][1]) // 2
    path = LineString([(robot_x, robot_y), ball_pos])
    return path.distance(Point(cross_pos)) < CROSS_AVOID_RADIUS

def choose_unblocked_ball(robot_head, robot_tail, balls, cross_pos):
    """Vælg bold der ikke er blokeret af kors"""
    for ball in balls:
        if not is_cross_blocking_path(robot_head, robot_tail, ball, cross_pos):
            return ball
    return balls[0]  # fallback 
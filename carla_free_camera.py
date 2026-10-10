#!/usr/bin/env python3

import math
import sys
import time

import carla
import pygame


# ==========================================
# CARLA CONNECTION
# ==========================================

HOST = "127.0.0.1"
PORT = 2000

# ==========================================
# CAMERA SETTINGS
# ==========================================

WINDOW_WIDTH = 520
WINDOW_HEIGHT = 240

NORMAL_SPEED = 20.0
FAST_MULTIPLIER = 4.0

MOUSE_SENSITIVITY = 0.18

MIN_SPEED = 1.0
MAX_SPEED = 300.0

FPS = 60


def main():

    # ======================================
    # CONNECT CARLA
    # ======================================

    print("Connecting to CARLA...")

    client = carla.Client(HOST, PORT)
    client.set_timeout(10.0)

    world = client.get_world()
    spectator = world.get_spectator()

    spawn_points = world.get_map().get_spawn_points()

    if spawn_points:
        spawn = spawn_points[0]

        initial_location = carla.Location(
            x=spawn.location.x,
            y=spawn.location.y,
            z=spawn.location.z + 30.0
        )

        initial_yaw = spawn.rotation.yaw
    else:
        initial_location = carla.Location(
            x=0.0,
            y=0.0,
            z=50.0
        )

        initial_yaw = 0.0

    initial_pitch = -30.0

    location = carla.Location(
        x=initial_location.x,
        y=initial_location.y,
        z=initial_location.z
    )

    yaw = initial_yaw
    pitch = initial_pitch

    move_speed = NORMAL_SPEED

    # ======================================
    # INITIALIZE PYGAME
    # ======================================

    pygame.init()

    screen = pygame.display.set_mode(
        (WINDOW_WIDTH, WINDOW_HEIGHT)
    )

    pygame.display.set_caption(
        "CARLA Free Camera - 360 Degree"
    )

    clock = pygame.time.Clock()

    font = pygame.font.Font(None, 25)
    small_font = pygame.font.Font(None, 21)

    center_x = WINDOW_WIDTH // 2
    center_y = WINDOW_HEIGHT // 2

    mouse_captured = False

    def capture_mouse(enabled):

        nonlocal mouse_captured

        mouse_captured = enabled

        pygame.event.set_grab(enabled)
        pygame.mouse.set_visible(not enabled)

        if enabled:
            pygame.mouse.set_pos(
                (center_x, center_y)
            )

        pygame.mouse.get_rel()

    capture_mouse(True)

    print()
    print("================================")
    print("CARLA FREE CAMERA")
    print("================================")
    print("WASD       Move")
    print("Q / E      Down / Up")
    print("Mouse      Rotate camera")
    print("Shift      Fast movement")
    print("Wheel      Change speed")
    print("F          Capture / Release mouse")
    print("R          Reset camera")
    print("ESC        Exit")
    print("================================")
    print()

    # ======================================
    # MAIN LOOP
    # ======================================

    running = True

    try:

        while running:

            dt = min(
                clock.tick(FPS) / 1000.0,
                0.1
            )

            # ==================================
            # EVENTS
            # ==================================

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

                elif event.type == pygame.KEYDOWN:

                    if event.key == pygame.K_ESCAPE:
                        running = False

                    elif event.key == pygame.K_f:

                        capture_mouse(
                            not mouse_captured
                        )

                    elif event.key == pygame.K_r:

                        location = carla.Location(
                            x=initial_location.x,
                            y=initial_location.y,
                            z=initial_location.z
                        )

                        yaw = initial_yaw
                        pitch = initial_pitch

                elif event.type == pygame.MOUSEWHEEL:

                    move_speed *= 1.2 ** event.y

                    move_speed = max(
                        MIN_SPEED,
                        min(MAX_SPEED, move_speed)
                    )

                elif event.type == pygame.WINDOWFOCUSLOST:

                    if mouse_captured:
                        capture_mouse(False)

            # ==================================
            # MOUSE ROTATION
            # ==================================

            if mouse_captured:

                # Read relative movement.
                dx, dy = pygame.mouse.get_rel()

                yaw = (
                    yaw + dx * MOUSE_SENSITIVITY
                ) % 360.0

                pitch -= dy * MOUSE_SENSITIVITY

                pitch = max(
                    -89.0,
                    min(89.0, pitch)
                )

                # Recenter cursor to avoid
                # hitting the window boundary.
                pygame.mouse.set_pos(
                    (center_x, center_y)
                )

                # Discard relative movement
                # generated by cursor recentering.
                pygame.mouse.get_rel()

            # ==================================
            # KEYBOARD MOVEMENT
            # ==================================

            keys = pygame.key.get_pressed()

            speed = move_speed

            if (
                keys[pygame.K_LSHIFT]
                or keys[pygame.K_RSHIFT]
            ):
                speed *= FAST_MULTIPLIER

            yaw_rad = math.radians(yaw)

            forward_x = math.cos(yaw_rad)
            forward_y = math.sin(yaw_rad)

            right_x = -math.sin(yaw_rad)
            right_y = math.cos(yaw_rad)

            move_x = 0.0
            move_y = 0.0
            move_z = 0.0

            if keys[pygame.K_w]:
                move_x += forward_x
                move_y += forward_y

            if keys[pygame.K_s]:
                move_x -= forward_x
                move_y -= forward_y

            if keys[pygame.K_d]:
                move_x += right_x
                move_y += right_y

            if keys[pygame.K_a]:
                move_x -= right_x
                move_y -= right_y

            if keys[pygame.K_e]:
                move_z += 1.0

            if keys[pygame.K_q]:
                move_z -= 1.0

            magnitude = math.sqrt(
                move_x * move_x
                + move_y * move_y
                + move_z * move_z
            )

            if magnitude > 0:

                location.x += (
                    move_x / magnitude
                    * speed * dt
                )

                location.y += (
                    move_y / magnitude
                    * speed * dt
                )

                location.z += (
                    move_z / magnitude
                    * speed * dt
                )

            # ==================================
            # UPDATE CARLA SPECTATOR
            # ==================================

            transform = carla.Transform(
                location,
                carla.Rotation(
                    pitch=pitch,
                    yaw=yaw,
                    roll=0.0
                )
            )

            spectator.set_transform(transform)

            # ==================================
            # DRAW CONTROL WINDOW
            # ==================================

            screen.fill((24, 29, 39))

            title = font.render(
                "CARLA FREE CAMERA - 360 DEGREE",
                True,
                (240, 240, 240)
            )

            screen.blit(title, (20, 15))

            lines = [
                "WASD: Move     Q/E: Down/Up",
                "Mouse: Look    Shift: Fast",
                "Wheel: Speed   R: Reset",
                "F: Release/Capture Mouse",
                "ESC: Exit",
            ]

            for i, line in enumerate(lines):

                text = small_font.render(
                    line,
                    True,
                    (210, 220, 230)
                )

                screen.blit(
                    text,
                    (20, 50 + i * 25)
                )

            status = (
                f"Speed: {move_speed:.1f} m/s"
                f"    Yaw: {yaw:.1f}"
                f"    Pitch: {pitch:.1f}"
            )

            status_text = small_font.render(
                status,
                True,
                (120, 220, 170)
            )

            screen.blit(
                status_text,
                (20, 190)
            )

            mouse_status = (
                "Mouse: CAPTURED"
                if mouse_captured
                else "Mouse: RELEASED"
            )

            mouse_text = small_font.render(
                mouse_status,
                True,
                (240, 190, 100)
            )

            screen.blit(
                mouse_text,
                (20, 215)
            )

            pygame.display.flip()

    finally:

        pygame.event.set_grab(False)
        pygame.mouse.set_visible(True)
        pygame.quit()


if __name__ == "__main__":

    try:
        main()
        print("新加载地图后需要重启本程序")

    except KeyboardInterrupt:
        pygame.quit()

    except Exception as e:
        pygame.quit()
        print("ERROR:", e)
        sys.exit(1)


import glfw
import OpenGL.GL as gl
import OpenGL.GLU as glu
import numpy as np

def pid_controller(ball_pos, prev_error, integral, dt=0.01, Kp=0.1, Ki=0.01, Kd=0.05):
    error = -ball_pos
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, error, integral

def initialize_glfw():
    if not glfw.init():
        raise Exception("GLFW can't be initialized")
    window = glfw.create_window(800, 600, "3DOF Platform Simulation", None, None)
    if not window:
        glfw.terminate()
        raise Exception("GLFW window can't be created")
    glfw.make_context_current(window)
    return window

def window_resize(window, width, height):
    gl.glViewport(0, 0, width, height)
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    glu.gluPerspective(45, width / float(height) if height > 0 else 1, 0.1, 100.0)
    gl.glMatrixMode(gl.GL_MODELVIEW)

def draw_platform(tilt_x, tilt_y, radius=1.0, num_segments=32):
    gl.glPushMatrix()
    gl.glColor3f(0.0, 0.0, 0.0)
    gl.glRotatef(np.degrees(tilt_x), 1, 0, 0)
    gl.glRotatef(np.degrees(tilt_y), 0, 1, 0)
    gl.glBegin(gl.GL_TRIANGLE_FAN)
    gl.glVertex3f(0, 0, 0)
    for i in range(num_segments + 1):
        angle = 2 * np.pi * i / num_segments
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        gl.glVertex3f(x, 0, y)
    gl.glEnd()
    gl.glPopMatrix()

def draw_ball(x, y, z):
    gl.glPushMatrix()
    gl.glTranslatef(x, y, z)
    gl.glColor3f(0.0, 0.0, 1.0)
    quadric = glu.gluNewQuadric()
    glu.gluSphere(quadric, 0.1, 32, 32)
    glu.gluDeleteQuadric(quadric)
    gl.glPopMatrix()

def setup_viewport(window):
    width, height = glfw.get_framebuffer_size(window)
    gl.glViewport(0, 0, width, height)
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    glu.gluPerspective(60, width / float(height) if height > 0 else 1, 0.1, 100.0)
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    glu.gluLookAt(0, 1, 5, 0, 0, 0, 0, 1, 0)

def main():
    window = initialize_glfw()
    setup_viewport(window)
    gl.glClearColor(1.0, 1.0, 1.0, 1.0)
    glfw.set_window_size_callback(window, window_resize)

    # Initialize PID control variables
    ball_x = 0
    ball_y = 0.1
    ball_z = 0.4
    prev_error_x = prev_error_z = 0
    integral_x = integral_z = 0

    while not glfw.window_should_close(window):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Calculate tilt using PID controller
        tilt_x, prev_error_x, integral_x = pid_controller(ball_x, prev_error_x, integral_x)
        tilt_y, prev_error_z, integral_z = pid_controller(ball_z, prev_error_z, integral_z)

        draw_platform(tilt_x, tilt_y)
        draw_ball(ball_x, ball_y, ball_z)

        # Update simulation state
        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()

if __name__ == "__main__":
    main()
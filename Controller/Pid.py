def pid_controller(ball_pos, prev_error, integral, dt=0.01, Kp=0.1, Ki=0.01, Kd=0.05):
    error = -ball_pos
    derivative = (error - prev_error) / dt
    integral += error * dt
    output = Kp * error + Ki * integral + Kd * derivative
    return output, error, integral
import os

import torch

if os.name == "nt":
    import msvcrt

    def getch() -> bytes:
        return msvcrt.getch().decode()

else:
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch() -> str:
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def get_loss(angles, model, device, criterion):
    loss_coefficients = [1.0, 0.1, 0.001, 0.0001]
    angle_loss, current_loss = 0, 0
    rnn_hidden = None
    angles = angles.to(device)

    # print("angles.shape",angles.shape)#.shape,images.shape,"ang_len=len(angles[0,step,:])",len(angles[0,0,:]))
    # print("angles.shape", angles.shape)
    # print("len(angles[2])", len(angles[0]))
    ang_len = angles.shape[2]  # len(angles[2])
    for step in range(len(angles[0]) - 1):

        t_angle = angles[:, step, :]
        # print("rnn_hidden.shape",rnn_hidden.shape)
        t1_angle, rnn_hidden = model(t_angle, rnn_hidden)
        # print(angles[:,step+1].shape, t1_angle.shape)
        angle_loss += criterion(angles[:, step + 1, :3], t1_angle[:, :3])
        if ang_len > 3:
            current_loss += criterion(angles[:, step + 1, 3:], t1_angle[:, 3:])

    angle_loss /= len(angles[0] - 1)

    angle_loss *= loss_coefficients[0]
    if ang_len > 3:
        current_loss /= len(angles[0] - 1)
        current_loss *= loss_coefficients[0]

    if ang_len > 3:
        loss_sum = angle_loss + current_loss

        return loss_sum, angle_loss, current_loss, rnn_hidden
    else:
        loss_sum = angle_loss

        return loss_sum, angle_loss, rnn_hidden

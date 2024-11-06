import os

import numpy as np

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

else:
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def get_loss(angles, images, model, device, criterion):
    loss_coefficients = [1.0, 0.1, 0.001, 0.0001]
    angle_loss, image_loss, predimage_loss, current_loss = 0, 0, 0, 0
    rnn_hidden = None
    angles = angles.to(device)

    images = np.transpose(images, (0, 1, 4, 2, 3))
    images = images.to(device)
    # print("angles.shape",angles.shape,images.shape,"ang_len=len(angles[0,step,:])",len(angles[0,0,:]))

    for step in range(len(angles[0]) - 1):
        ang_len = angles.shape[2]  # len(angles[0,step,:])
        t_image = images[:, step, :]
        t_angle = angles[:, step, :]

        t_pred_image, t1_pred_image, t1_angle, rnn_hidden = model(
            t_image, t_angle, rnn_hidden
        )
        # print(angles[:,step+1].shape, t1_angle.shape)
        angle_loss += criterion(angles[:, step + 1, :3], t1_angle[:, :3])
        if ang_len > 3:
            current_loss += criterion(angles[:, step + 1, 3:], t1_angle[:, 3:])

        image_loss += criterion(images[:, step], t_pred_image)
        predimage_loss += criterion(images[:, step + 1], t1_pred_image)

    angle_loss /= len(angles[0] - 1)

    image_loss /= len(angles[0] - 1)
    predimage_loss /= len(angles[0] - 1)

    angle_loss *= loss_coefficients[0]
    if ang_len > 3:
        current_loss /= len(angles[0] - 1)
        current_loss *= loss_coefficients[0]
    image_loss *= loss_coefficients[1]

    predimage_loss *= loss_coefficients[1]

    if ang_len > 3:
        loss_sum = angle_loss + image_loss + predimage_loss + current_loss

        return loss_sum, angle_loss, current_loss, image_loss, predimage_loss
    else:
        loss_sum = angle_loss + image_loss + predimage_loss

        return loss_sum, angle_loss, image_loss, predimage_loss

import argparse
import datetime
import json
import os

import torch
from torch.utils.data import TensorDataset

# from models.LSTM_6 import BASE
from models.LSTM_6_working_incline import BASE
from utils.edit_data import make_dataset, normalize_all, save_configuration
from utils.get_losses_lstm import get_loss

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


# Syntax: python -m src.learning_LSTM data/wiping/2023-09-17_21-12-17.670129_LF_Data_ --batchsize 16 --epoch 10000 --img_dir  /images/
# 
# 2022-11-19_08-24-38.120429-auto  --batchsize 8 --epoch 10
# python -m src.learning_LSTM data/wiping/
#
# python -m src.learning_LSTM ~/Inflatable_Data/data/wiping/2023-09-17_21-12-17.670129_LF_Data_ --batchsize 16 --epoch 10000 --img_dir  /images/
# 2023-01-08_22-11-06.071465-S_newx5x5x7 --batchsize 16 --epoch 10000 --img_dir  /images/
#
# 2022-12-07_13-53-10.861204-Inc_5x5x7a  --batchsize 2 --epoch 10
# _small_board
# For remote server:
# python -m src.learning_LSTM ~/Inflatable_Data/data/wiping/B_Inc_ang-65-25x5x5x10 --batchsize 8 --epoch 10000 --img_dir  /images/

# python -m src.learning_LSTM ~/Inflatable_Data/data/wiping/B_65_20x5x5x10 --batchsize 32 --epoch 10000 --img_dir  /images/ --c_num 2


# python -m src.learning_LSTM ~/Inflatable_Data/data/wiping/B_50-20x5x10x7_256 --batchsize 16 --epoch 10000 --img_dir  /images/
# 2023-01-08_22-11-06.071465-S_newx5x5x7 --batchsize 16 --epoch 10000 --img_dir  /images/
# B_55-20x3x6x8 --batchsize 16 --epoch 10000 --img_dir  /images/ --limit_seq 256
# 2023-01-08_22-11-06.071465-S_newx5x5x7 --batchsize 16 --epoch 10000 --img_dir  /images/
#
# B_45-2025qx5x5x6 --batchsize 16 --epoch 10000 --img_dir  /images/
#
# 2023-01-04_23-16-08.814402-B_65-20x5x10x10 --batchsize 16 --epoch 10000 --img_dir  /images/
# B_55-20x3x6x8 --batchsize 16 --epoch 10000 --img_dir  /images/ --limit_seq 256
# 2023-01-04_23-16-08.814402-B_65-20x5x10x10 --batchsize 16 --epoch 10000 --img_dir  /images/
# B_65-20x5x10x10 --batchsize 32 --epoch 10000 --img_dir  /images/
# 2023-01-04_23-16-08.814402-B_65-20x5x10x10 --batchsize 32 --epoch 10000 --img_dir  /images/
# B_55-20x3x6x8 --batchsize 32 --epoch 10000 --img_dir  /images/
# B_65-30x5x5x8_tt --batchsize 32 --epoch 10000 --img_dir  /images/
# B_65_20x5x5x10 --batchsize 32 --epoch 10000 --img_dir  /images/ --c_num 2
# B_65-30x5x5x8_tt --batchsize 32 --epoch 10000 --img_dir  /images/


# python -m src.learning_LSTM ~/Inflatable_Data/data/wiping/2023-01-04_23-16-08.814402-B_65-20x5x10x10 --batchsize 16 --epoch 10000 --img_dir  /images/ --limit_seq 128


def Learning(data_dir, epoch, batchsize, num_workers, limit_seq, img_dir, c_num):
    # Load Data
    dataset = make_dataset(data_dir, img_dir, 64, limit_seq)
    angles_list = list(range(0, 70, 5))
    # Normaliza Data
    normalized_tarin_csv, normalized_tarin_image = normalize_all(dataset, "train")
    normalized_test_csv, normalized_test_image = normalize_all(dataset, "test")

    # Define Training Logs
    training_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if not os.path.exists("logs/learning_logs/wiping/{}".format(training_id)):
        os.mkdir("logs/learning_logs/wiping/{}".format(training_id))
        os.mkdir("logs/learning_logs/wiping/{}/models".format(training_id))
        print("Make dircetory at logs/learning_logs/wiping/{}.".format(training_id))

    # Make Data Loader
    train_csv_img = TensorDataset(normalized_tarin_csv, normalized_tarin_image)
    train_dataloader = torch.utils.data.DataLoader(
        train_csv_img, batch_size=batchsize, shuffle=True, num_workers=num_workers
    )
    test_csv_img = TensorDataset(normalized_test_csv, normalized_test_image)
    test_dataloader = torch.utils.data.DataLoader(
        test_csv_img, batch_size=batchsize, shuffle=True, num_workers=num_workers
    )
    print("Established data loaders.")

    # Setup Learning Setting
    device = torch.device("cuda:" + str(c_num) if torch.cuda.is_available() else "cpu")
    print(
        "device name:",
        device,
        "len(dataset[train_minmax])",
        len(dataset["train_minmax"]),
    )
    model = BASE(len(dataset["train_minmax"]))
    model = model.to(device)
    criterion = torch.nn.MSELoss()
    optimizer = torch.optim.AdamW(model.parameters())

    # Start Learning
    print("Start Training.")
    start_time = datetime.datetime.now()
    for e in range(epoch):
        # Caluculate Training Loss
        for train_angles, train_image in train_dataloader:
            # print("train_angles.shape",train_angles.shape)
            # start_time = datetime.datetime.now()
            # if e % 100 == 0:
            #     start_time = datetime.datetime.now()
            model.train()

            train_loss, train_angle_loss, train_current_loss, rnn_hidden_ = get_loss(
                train_angles, model, device, criterion
            )

            optimizer.zero_grad()
            train_loss.backward()
            optimizer.step()
            end_time = datetime.datetime.now()
            elapsed_time = end_time - start_time

        # Caluculate Test Data
        if e % 100 == 0:
            with torch.no_grad():
                for test_angles, test_image in test_dataloader:
                    model.eval()

                    (
                        test_loss,
                        test_angle_loss,
                        test_current_loss,
                        rnn_hidden_,
                    ) = get_loss(test_angles, model, device, criterion)

                # Save Losses
                line = "{} time:{:.7f}s {:.7f} {:.7f} {:.7f} {:.7f}".format(  # {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f}
                    e,
                    elapsed_time.total_seconds(),
                    train_angle_loss.item(),
                    train_current_loss.item(),
                    test_angle_loss.item(),
                    test_current_loss.item(),
                    # rnn_hidden_[0][0],
                    # rnn_hidden_[0][1],
                    # rnn_hidden_[0][2],
                    # rnn_hidden_[0][3],
                    # rnn_hidden_[0][4],
                    # rnn_hidden_[0][5],
                    # rnn_hidden_[0][6],
                    # rnn_hidden_[0][7],
                    # rnn_hidden_[0][8],
                    # rnn_hidden_[0][9],
                    # rnn_hidden_[0][10],
                    # rnn_hidden_[0][11]
                )
                print("{} epoch:".format(epoch), line)
                with open(
                    "logs/learning_logs/wiping/{}/progress.csv".format(training_id), "a"
                ) as f:
                    line += "\n"
                    f.write(line)

        if e % 1000 == 0:
            print("Saving model_{}".format(e))
            torch.save(
                model.state_dict(),
                "logs/learning_logs/wiping/{}/models/model_{}.pth".format(
                    training_id, e
                ),
            )

    # Save Final Model
    torch.save(
        model.state_dict(),
        "./logs/learning_logs/wiping/{}/models/model_final.pth".format(training_id),
    )
    print("Finished training: {}".format(training_id))

    # Save the configuration
    config = save_configuration(
        data_dir, epoch, batchsize, dataset["train_minmax"], dataset["test_minmax"]
    )
    with open(
        "logs/learning_logs/wiping/{}/dataset.json".format(training_id), "w"
    ) as f:
        json.dump(config, f)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("data_dir", type=str, default="data/normalized_screwing/train")
    parser.add_argument("--epoch", type=int, default=2000)
    parser.add_argument("--batchsize", type=int, default=16)
    parser.add_argument("--img_dir", type=str, default="/images/")
    parser.add_argument("--num_workers", type=int, default=0)
    parser.add_argument("--limit_seq", type=int, default=0)
    parser.add_argument("--c_num", type=int, default=0)
    args = parser.parse_args()
    print(args)
    Learning(
        data_dir=args.data_dir,
        epoch=args.epoch,
        img_dir=args.img_dir,
        batchsize=args.batchsize,
        num_workers=args.num_workers,
        limit_seq=args.limit_seq,
        c_num=args.c_num,
    )

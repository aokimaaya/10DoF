import argparse
import datetime
import json
import os

import torch
from torch.utils.data import TensorDataset

from extra.models.LSTM_with_CAE_new import BASE
from utils.edit_data import make_dataset, normalize_all, save_configuration
from extra.utils.get_losses import get_loss

# Syntax: python -m src.learning_new data/wiping/straight_200_mb  --batchsize 8 --epoch 10 --img_dir /images_side/
# _small_board
# For remote server:
# python -m src.learning_new ~/Inflatable_Data/data/wiping/straight_200_mb --batchsize 16 --epoch 10000 --img_dir /images_side/


def Learning(data_dir, epoch, img_dir, num_workers, cuda_num, batchsize):
    # Load Data
    dataset = make_dataset(data_dir, img_dir, 128)

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
    device = torch.device(
        "cuda:" + str(cuda_num) if torch.cuda.is_available() else "cpu"
    )
    print(
        "device name:",
        device,
        "len(dataset[train_minmax])",
        len(dataset["train_minmax"]),
    )
    model = BASE(len(dataset["train_minmax"]), 15)
    model = model.to(device)
    criterion = torch.nn.MSELoss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.0001)

    # Start Learning
    print("Start Training.")
    start_time = datetime.datetime.now()
    for e in range(epoch):
        # Caluculate Training Loss
        for train_angles, train_images in train_dataloader:
            # start_time = datetime.datetime.now()
            # if e % 100 == 0:
            #     start_time = datetime.datetime.now()
            model.train()

            (
                train_loss,
                train_angle_loss,
                train_current_loss,
                train_image_loss,
                train_predimage_loss,
            ) = get_loss(train_angles, train_images, model, device, criterion)

            optimizer.zero_grad()
            train_loss.backward()
            optimizer.step()
            end_time = datetime.datetime.now()
            elapsed_time = end_time - start_time

        # Caluculate Test Data
        if e % 100 == 0:
            with torch.no_grad():
                for test_angles, test_images in test_dataloader:
                    model.eval()

                    (
                        test_loss,
                        test_angle_loss,
                        test_current_loss,
                        test_image_loss,
                        test_predimage_loss,
                    ) = get_loss(test_angles, test_images, model, device, criterion)

                # Save Losses
                line = "{} time:{:.7f}s {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f} {:.7f}".format(
                    e,
                    elapsed_time.total_seconds(),
                    train_angle_loss.item(),
                    train_current_loss.item(),
                    train_image_loss.item(),
                    train_predimage_loss.item(),
                    test_angle_loss.item(),
                    test_current_loss.item(),
                    test_image_loss.item(),
                    test_predimage_loss.item(),
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
    parser.add_argument("--img_dir", type=str, default="/images_side/")
    parser.add_argument("--batchsize", type=int, default=16)
    parser.add_argument("--num_workers", type=int, default=0)
    parser.add_argument("--cuda_num", type=int, default=0)

    args = parser.parse_args()
    print(args)
    Learning(
        data_dir=args.data_dir,
        epoch=args.epoch,
        img_dir=args.img_dir,
        batchsize=args.batchsize,
        num_workers=args.num_workers,
        cuda_num=args.cuda_num,
    )

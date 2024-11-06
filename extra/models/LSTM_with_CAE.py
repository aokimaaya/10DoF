import torch
from torch import nn


class BASE(nn.Module):
    def __init__(self, input_len, out_img):
        super(BASE, self).__init__()
        self.ch = [3, 8, 16, 32]
        kw = [4, 3, 3]
        sd = [1, 2]

        self.out_img = out_img
        self.encoder = nn.Sequential(
            nn.Conv2d(self.ch[0], self.ch[1], kw[0], sd[1]),
            nn.ReLU(),
            nn.BatchNorm2d(self.ch[1]),
            nn.Conv2d(self.ch[1], self.ch[2], kw[1], sd[1]),
            nn.ReLU(),
            nn.BatchNorm2d(self.ch[2]),
            nn.Conv2d(self.ch[2], self.ch[3], kw[2], sd[1]),
            nn.ReLU(),
            nn.BatchNorm2d(self.ch[3]),
        )
        self.enc2lstm = nn.Sequential(
            nn.Linear(self.out_img * self.out_img * self.ch[3], 128),
            nn.ReLU(),
            nn.Linear(128, 16),
            nn.ReLU(),
        )
        self.lstm2dec = nn.Sequential(
            nn.Linear(16, 128),
            nn.ReLU(),
            nn.Linear(128, self.out_img * self.out_img * self.ch[3]),
            nn.ReLU(),
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(self.ch[3], self.ch[2], kw[2], sd[1]),
            nn.ReLU(),
            nn.BatchNorm2d(self.ch[2]),
            nn.ConvTranspose2d(self.ch[2], self.ch[1], kw[1], sd[1]),
            nn.ReLU(),
            nn.BatchNorm2d(self.ch[1]),
            nn.ConvTranspose2d(self.ch[1], self.ch[0], kw[0], sd[1]),
            nn.ReLU(),
            nn.BatchNorm2d(self.ch[0]),
        )
        self.lnr2lstm = nn.Sequential(
            nn.Linear(input_len, 12),
            nn.ReLU(),
            nn.Linear(12, 12),
            nn.ReLU(),
            nn.Dropout(0.5),
        )
        self.lstm = nn.LSTMCell(12 + 16, 12 + 16)
        self.lstm2lnr = nn.Sequential(
            nn.Linear(12, 12), nn.ReLU(), nn.Dropout(0.5), nn.Linear(12, input_len)
        )

    def forward(self, current_img, current_angles, feat_hc):
        # Encode img
        enc_img = self.encoder(current_img)
        enc_img = enc_img.reshape(-1, self.out_img * self.out_img * self.ch[3])
        enc_img = self.enc2lstm(enc_img)

        # Decode img (Auto Encoder)
        dec_img = self.lstm2dec(enc_img)
        dec_img = dec_img.reshape(-1, self.ch[3], self.out_img, self.out_img)
        cur_img = self.decoder(dec_img)

        # Linear Transformation
        angles_feat = self.lnr2lstm(current_angles)

        # LSTM
        feats = torch.cat([enc_img, angles_feat], dim=-1)
        feat_hid, feat_state = self.lstm(feats, feat_hc)

        # Predict angles at next step
        angles_pred = self.lstm2lnr(feat_hid[:, :12])

        # Predict image at next step
        img_pred = self.lstm2dec(feat_hid[:, 12:])
        img_pred = img_pred.reshape(-1, self.ch[3], self.out_img, self.out_img)
        img_pred = self.decoder(img_pred)

        return cur_img, img_pred, angles_pred, (feat_hid, feat_state)

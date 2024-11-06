from torch import nn


class BASE(nn.Module):
    def __init__(self, input_len: int) -> None:
        super(BASE, self).__init__()
        drop = 0.5
        ch = [12, 24, 12]
        self.lnr2lstm = nn.Sequential(
            nn.Linear(input_len, ch[0]),
            nn.ReLU(),  # nn.Dropout(drop),
            nn.Linear(ch[0], ch[2]),
            nn.ReLU(),
            nn.Dropout(drop),
            # nn.Linear(ch[1], ch[0]),
            # nn.ReLU(), nn.Dropout(drop),
        )
        self.lstm = nn.LSTMCell(ch[0], ch[0])
        self.lstm2lnr = nn.Sequential(
            # nn.Linear(ch[0], ch[1]), nn.ReLU(), nn.Dropout(drop),
            nn.Linear(ch[2], ch[0]),
            nn.ReLU(),
            nn.Dropout(drop),
            nn.Linear(12, input_len),
        )

    def forward(
        self, current_angles: list[float], feat_hc: list[float]
    ) -> tuple[float, tuple[float, float]]:

        # Linear Transformation
        angles_feat = self.lnr2lstm(current_angles)

        # LSTM
        # feats = torch.cat([enc_img, angles_feat], dim=-1)
        feat_hid, feat_state = self.lstm(angles_feat, feat_hc)

        # Predict angles at next step
        angles_pred = self.lstm2lnr(feat_hid[:, :12])

        return angles_pred, (feat_hid, feat_state)

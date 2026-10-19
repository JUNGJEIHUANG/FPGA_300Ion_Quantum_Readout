import logging
import math

import torch
import torch.nn as nn
from torch.nn import functional as F
from nets.DWNetV2 import DWNetV2, InvertedResidual


class DWNetV2_unet(nn.Module):
    """Original DW-UNet (without PSF Attention). Use PSFANet for the newer model."""

    def __init__(self, pre_trained, mode="train", width_mult=1.0):
        super(DWNetV2_unet, self).__init__()

        self.mode = mode
        self.backbone = DWNetV2(width_mult=width_mult)

        c1 = int(16 * width_mult)
        c2 = int(24 * width_mult)
        c3 = int(32 * width_mult)
        c4 = int(96 * width_mult)
        c5 = self.backbone.last_channel

        self.dconv1 = nn.ConvTranspose2d(c5, c4, 4, padding=1, stride=2)
        self.invres1 = InvertedResidual(2 * c4, c4, 1, 6)

        self.dconv2 = nn.ConvTranspose2d(c4, c3, 4, padding=1, stride=2)
        self.invres2 = InvertedResidual(2 * c3, c3, 1, 6)

        self.dconv3 = nn.ConvTranspose2d(c3, c2, 4, padding=1, stride=2)
        self.invres3 = InvertedResidual(2 * c2, c2, 1, 6)

        self.dconv4 = nn.ConvTranspose2d(c2, c1, 4, padding=1, stride=2)
        self.invres4 = InvertedResidual(2 * c1, c1, 1, 6)

        self.conv_last = nn.Conv2d(c1, 3, 1)
        self.conv_score = nn.Conv2d(3, 1, 1)

        self._init_weights()

        if pre_trained is not None:
            full_state_dict = torch.load(
                pre_trained, map_mode="cuda" if torch.cuda.is_available() else "cpu"
            )
            backbone_state_dict = {}
            for key, value in full_state_dict.items():
                if key.startswith("backbone."):
                    new_key = key.replace("backbone.", "", 1)
                    backbone_state_dict[new_key] = value
            self.backbone.load_state_dict(backbone_state_dict)

    def forward(self, x):
        for n in range(0, 2):
            x = self.backbone.features[n](x)
        x1 = x

        for n in range(2, 4):
            x = self.backbone.features[n](x)
        x2 = x

        for n in range(4, 7):
            x = self.backbone.features[n](x)
        x3 = x

        for n in range(7, 14):
            x = self.backbone.features[n](x)
        x4 = x

        for n in range(14, 19):
            x = self.backbone.features[n](x)
        x5 = x

        decoder_feature0 = self.dconv1(x)
        decoder_feature0 = F.interpolate(
            decoder_feature0, size=x4.shape[2:], mode="bilinear", align_corners=True
        )
        up1 = torch.cat([x4, decoder_feature0], dim=1)
        up1 = self.invres1(up1)

        decoder_feature1 = self.dconv2(up1)
        decoder_feature1 = F.interpolate(
            decoder_feature1, size=x3.shape[2:], mode="bilinear", align_corners=True
        )
        up2 = torch.cat([x3, decoder_feature1], dim=1)
        up2 = self.invres2(up2)

        decoder_feature2 = self.dconv3(up2)
        decoder_feature2 = F.interpolate(
            decoder_feature2, size=x2.shape[2:], mode="bilinear", align_corners=True
        )
        up3 = torch.cat([x2, decoder_feature2], dim=1)
        up3 = self.invres3(up3)

        decoder_feature3 = self.dconv4(up3)
        decoder_feature3 = F.interpolate(
            decoder_feature3, size=x1.shape[2:], mode="bilinear", align_corners=True
        )
        up4 = torch.cat([x1, decoder_feature3], dim=1)
        up4 = self.invres4(up4)

        x = self.conv_last(up4)
        x = self.conv_score(x)
        return x

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d) or isinstance(m, nn.ConvTranspose2d):
                n = m.kernel_size[0] * m.kernel_size[1] * m.out_channels
                m.weight.data.normal_(0, math.sqrt(2.0 / n))
                if m.bias is not None:
                    m.bias.data.zero_()
            elif isinstance(m, nn.BatchNorm2d):
                m.weight.data.fill_(1)
                m.bias.data.zero_()
            elif isinstance(m, nn.Linear):
                m.weight.data.normal_(0, 0.01)
                m.bias.data.zero_()

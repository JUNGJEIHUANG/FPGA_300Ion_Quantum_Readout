import torch
import torch.nn as nn
import torch.nn.functional as F


class DoubleConv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.net(x)


class ViTBottleneck(nn.Module):
    """Transformer bottleneck operating on spatial feature maps (B, C, H, W)."""

    def __init__(self, embed_dim, spatial_size, depth=4, num_heads=4,
                 mlp_ratio=2, dropout=0.1):
        super().__init__()
        H, W = spatial_size
        num_patches = H * W
        self.pos_embed = nn.Parameter(torch.zeros(1, num_patches, embed_dim))
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=embed_dim,
            nhead=num_heads,
            dim_feedforward=embed_dim * mlp_ratio,
            dropout=dropout,
            activation="gelu",
            batch_first=True,
            norm_first=True,
        )
        self.transformer = nn.TransformerEncoder(
            encoder_layer, num_layers=depth, norm=nn.LayerNorm(embed_dim)
        )
        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, x):
        B, C, H, W = x.shape
        tokens = x.flatten(2).transpose(1, 2)          # (B, H*W, C)
        tokens = self.transformer(tokens + self.pos_embed)
        return tokens.transpose(1, 2).reshape(B, C, H, W)


class ViTUNet(nn.Module):
    """U-Net with ViT bottleneck for 88x456 grayscale ion-fluorescence frames.

    Encoder (enc1-enc4) and decoder (up4/dec4 … up2/dec2) are structurally
    identical to StandardUNet.  The standard conv bottleneck is replaced by a
    Transformer that attends globally over the 11x57 = 627 spatial tokens at
    1/8 resolution, giving the model long-range context without downsampling
    further.
    """

    def __init__(
        self,
        in_channels: int = 1,
        out_channels: int = 1,
        base_channels: int = 32,
        vit_depth: int = 4,
        vit_heads: int = 4,
        vit_mlp_ratio: int = 2,
        dropout: float = 0.1,
        image_size: tuple = (88, 456),
    ):
        super().__init__()
        c = base_channels
        H, W = image_size

        # ── Encoder (identical to StandardUNet) ──────────────────────────────
        self.enc1 = DoubleConv(in_channels, c)          # (c,  H,   W  )
        self.enc2 = DoubleConv(c,     c * 2)            # (2c, H/2, W/2)
        self.enc3 = DoubleConv(c * 2, c * 4)            # (4c, H/4, W/4)
        self.enc4 = DoubleConv(c * 4, c * 8)            # (8c, H/8, W/8)
        self.pool = nn.MaxPool2d(2)

        # ── ViT bottleneck (replaces pool + DoubleConv bottleneck) ───────────
        # enc4 output is (8c, H/8, W/8).  For (88,456): H/8=11, W/8=57.
        vit_h, vit_w = H // 8, W // 8
        self.vit = ViTBottleneck(
            embed_dim=c * 8,
            spatial_size=(vit_h, vit_w),
            depth=vit_depth,
            num_heads=vit_heads,
            mlp_ratio=vit_mlp_ratio,
            dropout=dropout,
        )

        # ── Decoder (same pattern as StandardUNet) ───────────────────────────
        self.up4  = nn.ConvTranspose2d(c * 8, c * 4, 2, stride=2)
        self.dec4 = DoubleConv(c * 8, c * 4)   # cat(e3: 4c) + up4(4c)

        self.up3  = nn.ConvTranspose2d(c * 4, c * 2, 2, stride=2)
        self.dec3 = DoubleConv(c * 4, c * 2)   # cat(e2: 2c) + up3(2c)

        self.up2  = nn.ConvTranspose2d(c * 2, c, 2, stride=2)
        self.dec2 = DoubleConv(c * 2, c)        # cat(e1:  c) + up2( c)

        self.out  = nn.Conv2d(c, out_channels, 1)

    @staticmethod
    def _align(x: torch.Tensor, ref: torch.Tensor) -> torch.Tensor:
        if x.shape[2:] != ref.shape[2:]:
            x = F.interpolate(x, size=ref.shape[2:], mode="bilinear",
                              align_corners=False)
        return x

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Encoder
        e1 = self.enc1(x)                    # (B, c,   H,   W  )
        e2 = self.enc2(self.pool(e1))         # (B, 2c,  H/2, W/2)
        e3 = self.enc3(self.pool(e2))         # (B, 4c,  H/4, W/4)
        b  = self.enc4(self.pool(e3))         # (B, 8c,  H/8, W/8)

        # ViT global attention at bottleneck resolution
        b  = self.vit(b)                      # (B, 8c,  H/8, W/8)

        # Decoder with skip connections
        d4 = self._align(self.up4(b),  e3)
        d4 = self.dec4(torch.cat([e3, d4], dim=1))   # (B, 4c, H/4, W/4)

        d3 = self._align(self.up3(d4), e2)
        d3 = self.dec3(torch.cat([e2, d3], dim=1))   # (B, 2c, H/2, W/2)

        d2 = self._align(self.up2(d3), e1)
        d2 = self.dec2(torch.cat([e1, d2], dim=1))   # (B,  c, H,   W  )

        return self.out(d2)

import torch
import torch.nn as nn


class ViTSeg(nn.Module):
    """Compact ViT segmentation baseline for fixed 88x456 grayscale frames."""

    def __init__(
        self,
        image_size=(88, 456),
        patch_size=8,
        embed_dim=128,
        depth=4,
        num_heads=4,
        mlp_ratio=4,
        dropout=0.1,
    ):
        super().__init__()
        height, width = image_size
        if height % patch_size or width % patch_size:
            raise ValueError("image dimensions must be divisible by patch_size")

        self.grid_size = (height // patch_size, width // patch_size)
        num_patches = self.grid_size[0] * self.grid_size[1]
        self.patch_embed = nn.Conv2d(
            1, embed_dim, kernel_size=patch_size, stride=patch_size
        )
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
        self.encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=depth, norm=nn.LayerNorm(embed_dim)
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(
                embed_dim, 64, kernel_size=patch_size, stride=patch_size
            ),
            nn.GELU(),
            nn.Conv2d(64, 32, kernel_size=3, padding=1),
            nn.GELU(),
            nn.Conv2d(32, 1, kernel_size=1),
        )
        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, x):
        x = self.patch_embed(x)
        batch, channels, grid_h, grid_w = x.shape
        x = x.flatten(2).transpose(1, 2)
        x = self.encoder(x + self.pos_embed)
        x = x.transpose(1, 2).reshape(batch, channels, grid_h, grid_w)
        return self.decoder(x)


class ViTSeg_DWDecoder(nn.Module):
    """ViT segmentation model with a decoder equivalent to DW-UNet."""

    def __init__(
        self,
        image_size=(88, 456),
        patch_size=8,
        embed_dim=128,
        depth=4,
        num_heads=4,
        mlp_ratio=4,
        dropout=0.1,
    ):
        super().__init__()
        from nets.DWNetV2 import InvertedResidual
        height, width = image_size
        if height % patch_size or width % patch_size:
            raise ValueError("image dimensions must be divisible by patch_size")

        self.grid_size = (height // patch_size, width // patch_size)
        num_patches = self.grid_size[0] * self.grid_size[1]
        self.patch_embed = nn.Conv2d(
            1, embed_dim, kernel_size=patch_size, stride=patch_size
        )
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
        self.encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=depth, norm=nn.LayerNorm(embed_dim)
        )

        # Equivalent decoder to DW-UNet:
        # Stage 1: channel projection (embed_dim -> 96) + InvertedResidual
        self.dconv1 = nn.Conv2d(embed_dim, 96, 3, padding=1)
        self.invres1 = InvertedResidual(96, 96, 1, 6)

        # Stage 2: upsample to 22x114 (stride 2) from 96 to 32
        self.dconv2 = nn.ConvTranspose2d(96, 32, 4, padding=1, stride=2)
        self.invres2 = InvertedResidual(32, 32, 1, 6)

        # Stage 3: upsample to 44x228 (stride 2) from 32 to 24
        self.dconv3 = nn.ConvTranspose2d(32, 24, 4, padding=1, stride=2)
        self.invres3 = InvertedResidual(24, 24, 1, 6)

        # Stage 4: upsample to 88x456 (stride 2) from 24 to 16
        self.dconv4 = nn.ConvTranspose2d(24, 16, 4, padding=1, stride=2)
        self.invres4 = InvertedResidual(16, 16, 1, 6)

        self.conv_last = nn.Conv2d(16, 3, 1)
        self.conv_score = nn.Conv2d(3, 1, 1)

        nn.init.trunc_normal_(self.pos_embed, std=0.02)

    def forward(self, x):
        x = self.patch_embed(x)
        batch, channels, grid_h, grid_w = x.shape
        x = x.flatten(2).transpose(1, 2)
        x = self.encoder(x + self.pos_embed)
        x = x.transpose(1, 2).reshape(batch, channels, grid_h, grid_w)

        # Decoder forward pass (equivalent to DW-UNet decoder blocks)
        dec0 = self.invres1(self.dconv1(x))
        dec1 = self.invres2(self.dconv2(dec0))
        dec2 = self.invres3(self.dconv3(dec1))
        dec3 = self.invres4(self.dconv4(dec2))

        out = self.conv_last(dec3)
        out = self.conv_score(out)
        return out

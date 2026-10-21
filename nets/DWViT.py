"""DW-ViT: checkpoint-compatible FPGA-oriented dilated depthwise-separable model.
Default selected model: dilation=2, 376321 parameters, input N x 1 x 88 x 456.
The Transformer/decoder parameter names intentionally match the frozen experiment.
"""
import torch
from torch import nn

class _PatchTransformer(nn.Module):
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

class DWViT(_PatchTransformer):
    def __init__(self, dilation=2):
        if dilation not in (1, 2):
            raise ValueError("Supported controlled decoder dilations: 1 or 2")
        super().__init__(image_size=(88,456),patch_size=8,embed_dim=64,
                         depth=2,num_heads=2,mlp_ratio=2,dropout=0.1)
        self.decoder[2] = nn.Sequential(
            nn.Conv2d(64,64,kernel_size=3,stride=1,padding=dilation,
                      dilation=dilation,groups=64,bias=True),
            nn.Conv2d(64,32,kernel_size=1,stride=1,bias=True),
        )

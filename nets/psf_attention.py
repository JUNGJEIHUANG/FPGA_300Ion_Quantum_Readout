import torch
import torch.nn as nn
import torch.nn.functional as F


class PSFAttention(nn.Module):
    """
    Learnable PSF matched-filter spatial attention.

    Each call receives the shared log_sigma_base (nn.Parameter owned by the
    parent model).  The effective sigma at this decoder level is:

        sigma_eff = exp(log_sigma_base) / scale_factor

    so all levels automatically stay consistent with a single physical PSF
    width.  After training, exp(log_sigma_base) reports the PSF width in
    full-resolution pixels -- a physically interpretable measurement of the
    optical system.
    """

    def __init__(self, channels: int, kernel_size: int, scale_factor: float):
        super().__init__()
        assert kernel_size % 2 == 1, "kernel_size must be odd"
        self.kernel_size = kernel_size
        self.scale_factor = scale_factor

        # SENet-style channel gate: selects which channels benefit from PSF
        # matching.  Reduction ratio 4 keeps it lightweight.
        r = max(1, channels // 4)
        self.gate = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Flatten(),
            nn.Linear(channels, r),
            nn.ReLU(inplace=True),
            nn.Linear(r, channels),
            nn.Sigmoid(),
        )

    def _build_kernel(self, log_sigma_base: torch.Tensor, device, dtype):
        sigma = (torch.exp(log_sigma_base) / self.scale_factor).clamp(0.5, 8.0)
        k = self.kernel_size
        r = k // 2
        coords = torch.arange(-r, r + 1, dtype=dtype, device=device)
        yy, xx = torch.meshgrid(coords, coords, indexing="ij")
        kernel = torch.exp(-(xx ** 2 + yy ** 2) / (2 * sigma ** 2))
        return kernel / kernel.sum()  # (k, k), normalised

    def forward(self, x: torch.Tensor, log_sigma_base: torch.Tensor) -> torch.Tensor:
        B, C, H, W = x.shape
        k = self.kernel_size
        kernel = self._build_kernel(log_sigma_base, x.device, x.dtype)
        # Expand to (C, 1, k, k) for depthwise convolution
        kernel = kernel.view(1, 1, k, k).expand(C, 1, k, k)

        # Matched filter: response is strongest where features look like the PSF
        psf_resp = F.conv2d(x, kernel, padding=k // 2, groups=C)

        spatial_attn = torch.sigmoid(psf_resp)                      # (B, C, H, W)
        ch_gate = self.gate(x).view(B, C, 1, 1)                     # (B, C, 1, 1)
        return x * spatial_attn * ch_gate

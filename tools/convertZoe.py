import torch
from transformers import ZoeDepthForDepthEstimation

# Load model
model = ZoeDepthForDepthEstimation.from_pretrained("Intel/zoedepth-nyu-kitti")
model.eval()

# Use a fixed dummy input
H, W = 384, 384  # Must match your C++ inference size
dummy_input = torch.randn(1, 3, H, W)

# Export with static shapes (no dynamic_axes)
torch.onnx.export(
    model,
    dummy_input,
    "zoedepth_fixed384.onnx",
    input_names=['input'],
    output_names=['depth'],
    opset_version=14,
    do_constant_folding=True,
    export_params=True
)

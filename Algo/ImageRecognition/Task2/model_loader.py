"""Load the pinned arrow-only checkpoint without unrestricted pickle execution."""
from pathlib import Path

def load_model(path: Path):
    import torch
    from torch import nn
    from ultralytics import YOLO
    from ultralytics.nn import tasks
    from ultralytics.nn.modules import conv, block, head

    allowed = {}
    for module in (nn, tasks, conv, block, head):
        for obj in vars(module).values():
            if isinstance(obj, type) and issubclass(obj, nn.Module):
                allowed[obj.__module__ + '.' + obj.__qualname__] = obj
    needed = torch.serialization.get_unsafe_globals_in_checkpoint(path)
    unknown = [name for name in needed if name not in allowed]
    if unknown:
        raise RuntimeError(f'Checkpoint contains unsupported globals: {unknown}')
    with torch.serialization.safe_globals([allowed[name] for name in needed]):
        checkpoint = torch.load(path, map_location='cpu', weights_only=True)
    net = checkpoint.get('ema') or checkpoint['model']
    net = net.float().eval()
    if sorted(net.names.values()) != ['left', 'right']:
        raise RuntimeError(f'Expected arrow-only model; found {net.names}')
    model = YOLO('yolo11m.yaml', task='detect')
    model.model = net
    model.overrides = {'task': 'detect', 'imgsz': 640, 'conf': .1, 'device': 'cpu'}
    return model

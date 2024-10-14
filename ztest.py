import torch

t = torch.Tensor([1, 0, 3])
if((t == 2).nonzero(as_tuple=True)[0].shape[0] > 0):
    print((t == 2).nonzero(as_tuple=True)[0].item())
else:
    print("fail")
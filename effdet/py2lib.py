import torch
from src.test_model import EfficientDet

model_path = "./trained_models/best_75.pth"

model = EfficientDet(num_classes=1)

checkpoint = torch.load(model_path).module

model.load_state_dict(checkpoint.state_dict())

model.cuda()

model.eval()

example = torch.rand(1, 3, 512, 512).cuda()

traced_net = torch.jit.trace(model, example)
traced_net.save("./libmodel/model_best_75_lib.pt")


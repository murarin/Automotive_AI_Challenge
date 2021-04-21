import os
import argparse
import torch
from torchvision import transforms
from src.dataset import CocoDataset, Resizer, Normalizer, LGSVLDataset
from src.config import COCO_CLASSES, colors
import cv2
import shutil
import time
from src.test_model import EfficientDet
from torchvision.ops.boxes import nms as nms_torch

def nms(dets, thresh):
    #print("a", dets[:, :4].size())
    #print("b", dets[:, 4].size())
    return nms_torch(dets[:, :4], dets[:, 4], thresh)

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_size", type=int, default=512, help="The common width and height for all images")
    parser.add_argument("--data_path", type=str, default="data/COCO", help="the root folder of dataset")
    parser.add_argument("--cls_threshold", type=float, default=0.5)
    parser.add_argument("--nms_threshold", type=float, default=0.5)
    parser.add_argument("--pretrained_model", type=str, default="trained_models/signatrix_efficientdet_coco.pth")
    parser.add_argument("--output", type=str, default="predictions")
    args = parser.parse_args()
    return args

def test(opt):
    #model = torch.load(opt.pretrained_model).module
    model_path = "./trained_models/best_75.pth"

    model = EfficientDet(num_classes=1)

    checkpoint = torch.load(model_path).module

    model.load_state_dict(checkpoint.state_dict())
    model.cuda()

    dataset = LGSVLDataset(opt.data_path, image_sets='test', transform=transforms.Compose([Normalizer(), Resizer()]))

    if os.path.isdir(opt.output):
        shutil.rmtree(opt.output)
    os.makedirs(opt.output)

    total_time = 0.0
    #for index in range(len(dataset)):
    for index in range(1000):
        data = dataset[index]
        scale = data['scale']

        #start = time.time()
        #with torch.no_grad():
            #scores, labels, boxes = model(data['img'].cuda().permute(2, 0, 1).float().unsqueeze(dim=0))
        #end = time.time()
        #total_time += end - start

        #print("Total time", total_time)
        #print("Mean time", total_time / 1000.0)

        #if dataset.ids[index] != "1585019926_4100":
        #    continue

        with torch.no_grad():
            #scores, labels, boxes = model(data['img'].cuda().permute(2, 0, 1).float().unsqueeze(dim=0))
            classification, transformed_anchors, scores_over_thresh, scores = model(data['img'].cuda().permute(2, 0, 1).float().unsqueeze(dim=0))

            #print(scores_over_thresh.sum())

            if scores_over_thresh.sum() == 0:
                continue

            classification = classification[:, scores_over_thresh, :]
            #print(classification.size())
            transformed_anchors = transformed_anchors[:, scores_over_thresh, :]
            scores = scores[:, scores_over_thresh, :]

            anchors_nms_idx = nms(torch.cat([transformed_anchors, scores], dim=2)[0, :, :], 0.5)

            #print("adsgsadgsadg", classification[0, anchors_nms_idx, :])

            #print(classification[0, anchors_nms_idx, :].size())
            scores, labels = classification[0, anchors_nms_idx, :].max(dim=1)
            #print(scores.size())
            #print(labels.size())

            #print("klklklkllkgdlj", classification[0, anchors_nms_idx, :].max(dim=1))

            boxes = transformed_anchors[0, anchors_nms_idx, :]

            boxes /= scale

        #print(boxes.shape)
        if boxes.shape[0] > 0:
            #image_info = dataset.coco.loadImgs(dataset.image_ids[index])[0]
            #path = os.path.join(dataset.root_dir, 'images', dataset.set_name, image_info['file_name'])
            path = os.path.join(dataset.root, 'rgb', dataset.ids[index] + '.png')
            output_image = cv2.imread(path)
            print(path)



            for box_id in range(boxes.shape[0]):
                pred_prob = float(scores[box_id])
                if pred_prob < opt.cls_threshold:
                    break
                pred_label = int(labels[box_id])
                xmin, ymin, xmax, ymax = boxes[box_id, :]
                color = colors[pred_label]
                cv2.rectangle(output_image, (xmin, ymin), (xmax, ymax), color, 2)
                text_size = cv2.getTextSize(COCO_CLASSES[pred_label] + ' : %.2f' % pred_prob, cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]

                cv2.rectangle(output_image, (xmin, ymin), (xmin + text_size[0] + 3, ymin + text_size[1] + 4), color, -1)
                cv2.putText(
                    output_image, COCO_CLASSES[pred_label] + ' : %.2f' % pred_prob,
                    (xmin, ymin + text_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1,
                    (255, 255, 255), 1)

            cv2.imwrite("{}/{}_prediction.png".format(opt.output, dataset.ids[index]), output_image)






if __name__ == "__main__":
    opt = get_args()
    test(opt)

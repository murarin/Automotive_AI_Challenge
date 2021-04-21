python3 efficient_net_train.py --lr 0.001 \
    -d ../traffic_light_data/train_test \
    -a efficientnet-b4 \
    -j 4 \
    --epochs 200 \
    --gpu 1 \
    -b 128 \
    --model ./model

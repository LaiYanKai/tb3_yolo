from ultralytics import YOLO

# Load a model
model = YOLO("yolo11n.pt")

# Train the model
train_results = model.train(
    data="datasets.yaml",  # path to dataset YAML
    epochs=100,  # number of training epochs
    imgsz=320,  # All images are automatically resized before training and predicting.
    device="cpu",  # device to run on, i.e. device=0 or device=0,1,2,3 or device=cpu
)

# Evaluate model performance on the validation set
metrics = model.val()

# Export the model
path = model.export()  # return path to exported model

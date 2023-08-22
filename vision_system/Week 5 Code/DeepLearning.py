import cv2
import torch
import torchvision.transforms as transforms
from midas.midas_net import MidasNet

# Load the pre-trained MiDaS model
model = MidasNet("model-small-70d6b9c8.pt")
model.eval()

# Set up the camera
cap = cv2.VideoCapture(0)  # Use the default camera (change to a different number for external cameras)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Preprocess the image for the MiDaS model
    input_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    input_image = transforms.ToTensor()(input_image).unsqueeze(0)
    
    # Get the depth prediction from the model
    with torch.no_grad():
        prediction = model(input_image)
        
    depth_map = prediction.squeeze().cpu().numpy()
    
    # Normalize the depth map for visualization
    normalized_depth = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
    
    # Display the original image and depth map
    cv2.imshow("Original Image", frame)
    cv2.imshow("Depth Map", normalized_depth)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

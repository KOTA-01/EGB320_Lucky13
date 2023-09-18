import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import transforms
from torch.utils.data import DataLoader
from sklearn.model_selection import train_test_split
from depth_dataset import DepthDataset  # Define your dataset class
from depth_estimation_model import DepthEstimationModel  # Define your model architecture

# Set your hyperparameters
batch_size = 16
initial_learning_rate = 0.001
num_epochs = 20

# Create the dataset and data loaders for training and validation
transform = transforms.Compose([
    transforms.Resize((256, 256)),  # Resize images to a common size
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])
dataset = DepthDataset(data_dir='path_to_data', transform=transform)
train_data, val_data = train_test_split(dataset, test_size=0.2, random_state=42)
train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True)
val_loader = DataLoader(val_data, batch_size=batch_size, shuffle=False)

# Initialize the model
model = DepthEstimationModel()

# Define loss function and optimizer
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=initial_learning_rate)

# Learning rate scheduler
scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.5)

# Training loop
for epoch in range(num_epochs):
    model.train()  # Set the model to training mode
    total_loss = 0.0

    for batch_idx, (inputs, depth_targets) in enumerate(train_loader):
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, depth_targets)
        loss.backward()
        optimizer.step()

        total_loss += loss.item()

        if (batch_idx + 1) % 10 == 0:
            print(f'Train - Epoch [{epoch+1}/{num_epochs}], Batch [{batch_idx+1}/{len(train_loader)}], Loss: {loss.item():.4f}')

    avg_loss = total_loss / len(train_loader)
    print(f'Train - Epoch [{epoch+1}/{num_epochs}], Average Loss: {avg_loss:.4f}')

    # Validation loop
    model.eval()  # Set the model to evaluation mode
    with torch.no_grad():
        total_val_loss = 0.0
        for val_inputs, val_depth_targets in val_loader:
            val_outputs = model(val_inputs)
            val_loss = criterion(val_outputs, val_depth_targets)
            total_val_loss += val_loss.item()

        avg_val_loss = total_val_loss / len(val_loader)
        print(f'Validation - Epoch [{epoch+1}/{num_epochs}], Average Loss: {avg_val_loss:.4f}')

    # Update the learning rate
    scheduler.step()

# Save the trained model
torch.save(model.state_dict(), 'depth_estimation_model.pth')

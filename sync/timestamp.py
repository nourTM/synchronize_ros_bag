import pandas as pd

# Load the timestamps into a DataFrame
df = pd.read_csv("timestamps_100ms.txt", header=None, 
                 names=["ImageColor", "ImageDepth", "PointCloud", "GNSS", "IMU", "Common Time Assigned"])

# Convert the DataFrame to an Excel file
df.to_excel("timestamps.xlsx", index=False)

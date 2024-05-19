import pandas as pd
import numpy as np
from itertools import combinations

# Load the Excel file
file_path = 'timestamps.xlsx'
df = pd.read_excel(file_path)

# Automatically determine the sensor columns
# Exclude any columns that are not sensors; for example, the timestamp column
# Update the excluded_columns list with the name of any columns that shouldn't be treated as sensors
excluded_columns = ['Timestamp', 'Common Time Assigned']  # Add or remove columns based on your specific Excel file structure
sensor_columns = [col for col in df.columns if col not in excluded_columns]

# Calculate the differences between each pair of sensor columns using combinations
for (sensor1, sensor2) in combinations(sensor_columns, 2):
    df[f'Gap_{sensor1}_to_{sensor2}'] = np.abs(df[sensor2] - df[sensor1])

# Calculate statistics
stats_data = {}
for col in df.columns:
    if 'Gap' in col:
        stats_data[col] = {
            'Max Difference': df[col].max() / 1e9,
            'Min Difference': df[col].min() / 1e9,
            'Mean Difference': df[col].mean() / 1e9,
            'Standard Deviation': df[col].std() / 1e9
        }

# Convert stats data to DataFrame
stats_df = pd.DataFrame(stats_data).transpose()

# Save the statistics to a new Excel file
output_path = 'output_statistics_report.xlsx'
stats_df.to_excel(output_path, index=True)

print(f'Report generated successfully and saved to {output_path}')

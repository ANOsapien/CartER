import pandas as pd
import numpy as np
import os
import glob

def parse_and_load_data(file_path):
    """
    Parses the unique CSV file structure to extract metadata (PID params)
    and load the time-series data into a pandas DataFrame.

    Args:
        file_path (str): The path to the CSV file.

    Returns:
        tuple: A tuple containing:
            - dict: A dictionary of the extracted metadata (PID parameters, etc.).
            - pd.DataFrame: A DataFrame with the time-series data.
    """
    metadata = {}
    header_row_index = -1

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"Could not read file {file_path}: {e}")
        return None, None


    # First pass: Extract metadata and find the data header
    for i, line in enumerate(lines):
        line = line.strip()
        if not line:
            continue

        parts = line.split(',')

        if parts[0] == 'Kp' and parts[1] == 'Ki':
            # This is the PID parameter header line, the next line has the values
            param_names = parts
            # Handle cases where the next line might be out of bounds
            if i + 1 < len(lines):
                param_values = lines[i+1].strip().split(',')
                if len(param_names) == len(param_values):
                    for name, value in zip(param_names, param_values):
                        try:
                            metadata[name] = float(value)
                        except ValueError:
                            # Handle cases where a value might not be a float
                            metadata[name] = value
        elif parts[0] == 'time' and parts[1] == 'angle':
            # This is the header for the main data
            header_row_index = i
            break # Stop after finding the data header
        elif len(parts) == 2:
            # For key-value pairs like 'start_time,889.405'
            try:
                metadata[parts[0]] = float(parts[1])
            except ValueError:
                metadata[parts[0]] = parts[1]

    if header_row_index == -1:
        # Silently skip files that don't match the expected format
        return None, None

    # Second pass: Load the data using pandas, skipping the metadata rows
    try:
        df = pd.read_csv(file_path, skiprows=header_row_index)
    except Exception as e:
        print(f"Pandas could not parse {file_path} after header: {e}")
        return None, None


    return metadata, df

def calculate_fitness_score(df, metadata):
    """
    Calculates a fitness score based on the performance of the system.
    A lower score indicates better performance.

    Args:
        df (pd.DataFrame): The time-series data.
        metadata (dict): The extracted metadata.

    Returns:
        dict: A dictionary containing the individual performance metrics
              and the final combined fitness score.
    """
    if df is None or df.empty or len(df) < 2:
        # Return worst-case scenario if data is invalid
        return {
            'error': 'Dataframe is empty or has insufficient data.',
            'fitness_score': float('inf')
        }

    # --- Key Performance Indicators (KPIs) ---
    target_angle = 0.0
    target_position = 0.0

    # 1. Integral of Absolute Error for Angle
    iae_angle = np.sum(np.abs(df['angle'] - target_angle))

    # 2. Integral of Absolute Error for Position
    iae_position = np.sum(np.abs(df['position'] - target_position))

    # 3. Maximum Absolute Angle
    max_angle = np.max(np.abs(df['angle']))

    # 4. Maximum Absolute Position (Overshoot)
    max_position = np.max(np.abs(df['position']))

    # 5. Control Effort Proxy (Integral of Squared Angular Velocity)
    control_effort = np.sum(np.square(df['angular_velocity']))

    # --- Final Fitness Score Calculation ---
    weights = {
        'w_iae_angle': 2.0,
        'w_iae_position': 0.5,
        'w_max_angle': 3.0,
        'w_max_position': 0.2,
        'w_control_effort': 0.1
    }

    fitness_score = (
        weights['w_iae_angle'] * iae_angle +
        weights['w_iae_position'] * iae_position +
        weights['w_max_angle'] * max_angle +
        weights['w_max_position'] * max_position +
        weights['w_control_effort'] * control_effort
    )

    results = {
        'iae_angle': iae_angle,
        'iae_position': iae_position,
        'max_angle_deviation': max_angle,
        'max_position_overshoot': max_position,
        'control_effort_proxy': control_effort,
        'fitness_score': fitness_score
    }

    return results

def main():
    """
    Main function to find all relevant CSV files, analyze them,
    and output a summary of the results.
    """
    # --- Configuration ---
    # IMPORTANT: Change this to the root directory containing your data folders.
    base_directory = '.' # Use '.' for current directory or provide a full path.

    # This pattern will recursively find all .csv files inside folders matching the structure.
    # It starts in base_directory, looks for cart_pendulum_data_*, then pid_set_*,
    # and then searches all subdirectories (**) for any .csv file.
    search_pattern = os.path.join(base_directory, 'cart_pendulum_data_*', 'pid_set_*', '**', '*.csv')

    print(f"Searching for files in: {os.path.abspath(base_directory)}")
    print(f"Using search pattern: {search_pattern}\n")

    # Find all files matching the pattern
    csv_files = glob.glob(search_pattern, recursive=True)

    if not csv_files:
        print("No CSV files found matching the pattern. Please check your `base_directory` and folder structure.")
        # Create some dummy files and folders for demonstration if none are found.
        print("Creating a dummy directory structure for demonstration...")
        os.makedirs("cart_pendulum_data_9/pid_set_1/23-07-csv", exist_ok=True)
        dummy_filepath = "cart_pendulum_data_9/pid_set_1/23-07-csv/pid-15-26-17.csv"
        with open(dummy_filepath, 'w') as f:
            f.write("""special_info,
Kp,Ki,Kd,Kp_pos,Ki_pos,Kd_pos
450.0,315.0,3.00,-0.057,0,-0.0150
start_time,889.405
time,angle,position,angular_velocity,cart_velocity
0.0,0.0,0.0,0.0,0.0
0.20,-0.0077,19.0,-0.5371,461.3785
0.25,0.0092,-1.0,0.8142,-152.6042
""")
        csv_files = glob.glob(search_pattern, recursive=True)


    all_results = []
    print(f"Found {len(csv_files)} files to analyze. Processing...")

    # --- Analysis Loop ---
    for file_path in csv_files:
        metadata, df = parse_and_load_data(file_path)

        # Skip if the file couldn't be parsed correctly
        if metadata is None or df is None:
            continue

        analysis_results = calculate_fitness_score(df, metadata)

        # Combine metadata and analysis results into one record
        record = {
            'file_path': file_path,
            **metadata,
            **analysis_results
        }
        all_results.append(record)

    print("\nAnalysis complete.")

    # --- Summarize and Save Results ---
    if not all_results:
        print("Could not process any files.")
        return

    # Convert the list of dictionaries to a pandas DataFrame
    results_df = pd.DataFrame(all_results)

    # Sort the DataFrame by fitness score to find the best parameters
    results_df = results_df.sort_values(by='fitness_score', ascending=True)

    # Define the output file name
    output_csv_path = 'pid_analysis_results.csv'

    try:
        # Save the full results to a new CSV file
        results_df.to_csv(output_csv_path, index=False)
        print(f"\nFull analysis saved to: {os.path.abspath(output_csv_path)}")
    except Exception as e:
        print(f"\nCould not save results to CSV: {e}")


    # Display the top 5 best-performing parameter sets
    print("\n--- Top 10 Best Performing PID Sets  ---")
    # Define columns to display for clarity
    display_cols = [
        'fitness_score', 'Kp', 'Ki', 'Kd', 'Kp_pos', 'Ki_pos', 'Kd_pos', 'file_path'
    ]
    # Filter out columns that might not exist in all files for cleaner printing
    display_cols = [col for col in display_cols if col in results_df.columns]
    
    print(results_df[display_cols].head(10).to_string())


if __name__ == '__main__':
    main()
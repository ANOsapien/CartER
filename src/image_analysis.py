import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

def analyze_pid_data(file_path):
    """
    Analyzes PID controller data from a CSV file to find the top 5 parameter sets
    and generates several visualizations.

    Args:
        file_path (str): The path to the CSV file.
    """
    # --- Data Loading and Processing ---
    # Check if the file exists
    if not os.path.exists(file_path):
        print(f"Error: The file '{file_path}' was not found.")
        return

    # Read the CSV data from the file into a pandas DataFrame
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Error reading the CSV file: {e}")
        return

    # Sort the DataFrame by 'fitness_score' in ascending order to find the best-performing sets
    # and select the top 5.
    df_sorted = df.sort_values(by='fitness_score').head(5)

    print("--- Top 5 Parameter Sets ---")
    print(df_sorted)
    print("\nGenerating visualizations...")

    # Create a directory for the output plots if it doesn't exist
    output_dir = "analysis_results"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # --- Visualizations ---

    # 1. Bar chart of Fitness Score
    plt.figure(figsize=(10, 6))
    sns.barplot(x=df_sorted.index, y='fitness_score', data=df_sorted, palette='viridis', order=df_sorted.index)
    plt.title('Top 5 Parameter Sets by Fitness Score')
    plt.xlabel('Original Index from CSV')
    plt.ylabel('Fitness Score')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'fitness_score_barplot.png'))

    # 2. Correlation Heatmap
    plt.figure(figsize=(12, 10))
    cols_for_corr = [
        'Kp', 'Ki', 'Kd', 'Kp_pos', 'Ki_pos', 'Kd_pos',
        'iae_angle', 'iae_position', 'max_angle_deviation',
        'max_position_overshoot', 'control_effort_proxy', 'fitness_score'
    ]
    # Filter out columns that are not in the dataframe to prevent errors
    cols_for_corr = [col for col in cols_for_corr if col in df_sorted.columns]
    correlation_matrix = df_sorted[cols_for_corr].corr()
    sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', fmt=".2f")
    plt.title('Correlation Heatmap of Top 5 Parameter Sets')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'correlation_heatmap.png'))


    # 3. Pair Plot
    pairplot_cols = ['Kp', 'Ki', 'Kd', 'Kp_pos', 'Ki_pos', 'Kd_pos', 'fitness_score']
    pairplot_cols = [col for col in pairplot_cols if col in df_sorted.columns]
    sns.pairplot(df_sorted[pairplot_cols], diag_kind='kde')
    plt.suptitle('Pair Plot of PID Parameters and Fitness Score (Top 5 Sets)', y=1.02)
    plt.savefig(os.path.join(output_dir, 'pair_plot.png'))

    # 4. Parallel Coordinates Plot
    plt.figure(figsize=(12, 7))
    parallel_coords_cols = ['Kp', 'Ki', 'Kd', 'Kp_pos', 'Ki_pos', 'Kd_pos', 'fitness_score']
    parallel_coords_cols = [col for col in parallel_coords_cols if col in df_sorted.columns]
    pd.plotting.parallel_coordinates(df_sorted[parallel_coords_cols], 'fitness_score', colormap='viridis')
    plt.title('Parallel Coordinates Plot of Top 5 Parameter Sets')
    plt.xlabel('Parameters')
    plt.ylabel('Value')
    plt.xticks(rotation=45)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'parallel_coordinates_plot.png'))

    # --- Save Top 5 Data ---
    df_sorted.to_csv(os.path.join(output_dir, 'top_5_parameter_sets.csv'), index=False)

    print(f"\nAnalysis complete. All plots and the top 5 data CSV have been saved in the '{output_dir}' directory.")


if __name__ == '__main__':
    # -------------------------------------------------------------------
    # IMPORTANT: Change this path to the location of your CSV file.
    # For example: 'C:/Users/YourUser/Documents/data/pid_results.csv'
    # Or on Linux/Mac: '/home/youruser/data/pid_results.csv'
    # -------------------------------------------------------------------
    csv_file_location = 'pid_analysis_results.csv'

    # Create a dummy CSV for demonstration purposes if it doesn't exist
    if not os.path.exists(csv_file_location):
        print(f"'{csv_file_location}' not found. Creating a dummy file for demonstration.")
        dummy_data = """file_path,special_info,Kp,Ki,Kd,Kp_pos,Ki_pos,Kd_pos,start_time,omega,phase/pi,iae_angle,iae_position,max_angle_deviation,max_position_overshoot,control_effort_proxy,fitness_score
.\\cart_pendulum_data_4\\pid_set_1\\23-07-csv\\pid-12-26-42.csv,,700.0,490.0,2.75,-0.04,0.0,-0.008,3468.968,2.0,0.0,2.6041999999999996,35006.0,0.431,2493.0,79.63857414,18016.065257414
.\\cart_pendulum_data_1\\pid_set_10\\22-07-csv\\pid-17-29-56.csv,,500.0,350.0,2.75,-0.04,0.0,-0.008,1567.488,2.0,0.0,6.216,37356.0,1.0293,2575.0,187.63563666,19227.283463666
.\\cart_pendulum_data_2\\pid_set_7\\23-07-csv\\pid-11-39-19.csv,,600.0,420.0,2.75,-0.04,0.0,-0.008,626.592,2.0,0.0,4.5308,37994.0,0.7839,2624.0,124.33249813999998,19545.646549814
.\\cart_pendulum_data_1\\pid_set_16\\22-07-csv\\pid-17-37-32.csv,,500.0,350.0,2.75,-0.08,0.0,-0.008,2023.281,2.0,0.0,11.1244,40082.0,1.3514,1807.0,439.8224445000001,20472.68524445
.\\cart_pendulum_data_2\\pid_set_16\\23-07-csv\\pid-11-51-46.csv,,600.0,420.0,3.5,-0.04,0.0,-0.008,1373.577,2.0,0.0,3.2984,40218.0,0.5676,2676.0,85.56872956,20661.056472956
.\\cart_pendulum_data_1\\pid_set_26\\22-07-csv\\pid-17-50-07.csv,,500.0,350.0,3.5,-0.08,0.0,-0.014,2726.866,2.0,0.0,5.3538,40442.0,0.3344,2801.0,67.01845725999999,20799.612645726
.\\cart_pendulum_data_9\\pid_set_9\\23-07-csv\\pid-15-34-27.csv,,450.0,315.0,3.0,-0.057,0.0,-0.005,554.696,2.0,0.0,18.707800000000002,42420.0,2.5602,1339.0,1133.9963456599999,21636.295834566
.\\cart_pendulum_data_4\\pid_set_10\\23-07-csv\\pid-12-40-24.csv,,700.0,490.0,3.5,-0.04,0.0,-0.008,4291.409,2.0,0.0,1.5924,42402.0,0.2562,2697.0,18.14412618,21746.167812618
"""
        with open(csv_file_location, 'w') as f:
            f.write(dummy_data)

    analyze_pid_data(csv_file_location)
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans

def perform_pca_and_clustering(file_path, n_clusters=4):
    """
    Performs PCA and K-Means clustering on PID controller data from a CSV file.

    Args:
        file_path (str): The path to the CSV file.
        n_clusters (int): The number of clusters to form.
    """
    # --- 1. Data Loading and Preparation ---
    if not os.path.exists(file_path):
        print(f"Error: The file '{file_path}' was not found.")
        return
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Error reading the CSV file: {e}")
        return

    # Select numerical features for clustering
    features = [
        'Kp', 'Ki', 'Kd', 'Kp_pos', 'Ki_pos', 'Kd_pos', 'iae_angle',
        'iae_position', 'max_angle_deviation', 'max_position_overshoot',
        'control_effort_proxy', 'fitness_score'
    ]
    # Filter out columns that are not in the dataframe
    features = [col for col in features if col in df.columns]
    data_for_clustering = df[features].dropna()

    # Standardize the features before applying PCA
    scaler = StandardScaler()
    scaled_data = scaler.fit_transform(data_for_clustering)

    # --- 2. Elbow Method to Find Optimal k ---
    print("Generating Elbow Method plot to find optimal k...")
    sse = []
    k_range = range(1, 11)
    for k in k_range:
        kmeans = KMeans(n_clusters=k, random_state=42, n_init=10)
        kmeans.fit(scaled_data)
        sse.append(kmeans.inertia_) # Sum of squared distances to closest centroid

    plt.figure(figsize=(8, 5))
    plt.plot(k_range, sse, marker='o')
    plt.title('Elbow Method for Optimal k')
    plt.xlabel('Number of Clusters (k)')
    plt.ylabel('Sum of Squared Errors (SSE)')
    plt.xticks(k_range)
    plt.grid(True)
    plt.show() # Display the plot for the user to decide on k

    # --- 3. PCA for Dimensionality Reduction ---
    print(f"Performing PCA to reduce to 2 components...")
    pca = PCA(n_components=2)
    principal_components = pca.fit_transform(scaled_data)
    pca_df = pd.DataFrame(data=principal_components, columns=['PC1', 'PC2'])

    # --- 4. K-Means Clustering ---
    print(f"Performing K-Means clustering with k={n_clusters}...")
    kmeans = KMeans(n_clusters=n_clusters, random_state=42, n_init=10)
    clusters = kmeans.fit_predict(scaled_data)
    pca_df['cluster'] = clusters

    # --- 5. Visualization ---
    print("Generating final cluster visualization...")
    plt.figure(figsize=(12, 8))
    sns.scatterplot(
        x='PC1', y='PC2', hue='cluster', data=pca_df,
        palette=sns.color_palette('viridis', n_colors=n_clusters),
        s=100, alpha=0.9, legend='full'
    )
    # Plotting the centroids
    centroids_pca = pca.transform(kmeans.cluster_centers_)
    plt.scatter(
        centroids_pca[:, 0], centroids_pca[:, 1],
        marker='X', s=200, c='red', edgecolor='black', label='Centroids'
    )

    plt.title('K-Means Clusters on PCA-Reduced Data')
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.legend()
    plt.grid(True)

    # Create output directory and save the plot
    output_dir = "analysis_results"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    save_path = os.path.join(output_dir, 'pca_kmeans_clusters.png')
    plt.savefig(save_path)

    print(f"\nAnalysis complete. Cluster plot saved to '{save_path}'")


if __name__ == '__main__':
    # --- Configuration ---
    # IMPORTANT: Change this path to the location of your CSV file.
    csv_file_location = 'pid_analysis_results.csv'
    # Set the number of clusters based on the Elbow Method plot.
    # The "elbow" in the plot suggests an optimal value. A value of 3 or 4 looks reasonable.
    N_CLUSTERS = 4

    # --- Execution ---
    # Create a dummy CSV for demonstration if it doesn't exist
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
.\\cart_pendulum_data_4\\pid_set_2\\23-07-csv\\pid-12-27-35.csv,,700.0,490.0,2.75,-0.04,0.0,-0.014,3522.717,2.0,0.0,1.0368,43154.0,0.1779,2737.0,30.58220786,22130.065520786
.\\cart_pendulum_data_2\\pid_set_13\\23-07-csv\\pid-11-47-38.csv,,600.0,420.0,2.75,-0.08,0.0,-0.008,1125.12,2.0,0.0,13.3214,43612.0,1.7426,1396.0,572.2225503,22174.292855030002
.\\cart_pendulum_data_1\\pid_set_25\\22-07-csv\\pid-17-49-15.csv,,500.0,350.0,3.5,-0.08,0.0,-0.008,2597.845,2.0,0.0,4.724399999999999,43694.0,0.2777,2860.0,52.30236662,22434.512136661997
.\\cart_pendulum_data_3\\pid_set_5\\23-07-csv\\pid-12-03-28.csv,,600.0,420.0,3.5,-0.08,0.0,-0.008,2075.089,2.0,0.0,20.721400000000003,46568.0,2.8087,1410.0,1357.7898304599996,23751.647883046
.\\cart_pendulum_data_9\\pid_set_4\\23-07-csv\\pid-15-29-49.csv,,450.0,315.0,3.0,-0.03,0.0,-0.015,277.614,2.0,0.0,1.3279999999999998,46496.0,0.1856,2804.0,33.51695274,23815.364495273996
.\\cart_pendulum_data_3\\pid_set_8\\23-07-csv\\pid-12-08-08.csv,,700.0,490.0,2.0,-0.04,0.0,-0.008,2355.458,2.0,0.0,1.7336,48088.0,0.2761,2890.0,43.59041494,24630.654541494
.\\cart_pendulum_data_9\\pid_set_5\\23-07-csv\\pid-15-30-44.csv,,450.0,315.0,3.0,-0.043,0.0,-0.005,331.284,2.0,0.0,15.962800000000001,50432.0,2.212,1761.0,829.2204865799999,25689.683648658
"""
        with open(csv_file_location, 'w') as f:
            f.write(dummy_data)

    perform_pca_and_clustering(csv_file_location, n_clusters=N_CLUSTERS)
"""
Generate ArUco markers for testing
Creates printable marker images for the competition
"""
import cv2
import cv2.aruco as aruco
import numpy as np

def generate_marker(marker_id, dictionary=aruco.DICT_6X6_1000, size=500):
    """
    Generate a single ArUco marker
    
    Args:
        marker_id: ID of the marker to generate
        dictionary: ArUco dictionary to use
        size: Size of the marker image in pixels
    
    Returns:
        marker_image: Generated marker as numpy array
    """
    aruco_dict = aruco.getPredefinedDictionary(dictionary)
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, size)
    return marker_image

def save_marker(marker_id, filename=None, dictionary=aruco.DICT_6X6_1000, size=500):
    """
    Generate and save a marker to file
    
    Args:
        marker_id: ID of the marker
        filename: Output filename (default: marker_ID.png)
        dictionary: ArUco dictionary
        size: Size in pixels
    """
    if filename is None:
        filename = f"marker_{marker_id}.png"
    
    marker = generate_marker(marker_id, dictionary, size)
    cv2.imwrite(filename, marker)
    print(f"Saved marker {marker_id} to {filename}")

def generate_marker_sheet(marker_ids, output_file="marker_sheet.png", 
                         dictionary=aruco.DICT_6X6_1000, marker_size=400, 
                         padding=50, cols=3):
    """
    Generate a sheet with multiple markers for printing
    
    Args:
        marker_ids: List of marker IDs to generate
        output_file: Output filename
        dictionary: ArUco dictionary
        marker_size: Size of each marker in pixels
        padding: Space between markers
        cols: Number of columns in the sheet
    """
    num_markers = len(marker_ids)
    rows = (num_markers + cols - 1) // cols
    
    sheet_width = cols * marker_size + (cols + 1) * padding
    sheet_height = rows * marker_size + (rows + 1) * padding
    
    sheet = np.ones((sheet_height, sheet_width), dtype=np.uint8) * 255
    
    aruco_dict = aruco.getPredefinedDictionary(dictionary)
    
    for idx, marker_id in enumerate(marker_ids):
        row = idx // cols
        col = idx % cols
        
        x = col * (marker_size + padding) + padding
        y = row * (marker_size + padding) + padding
        
        marker = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        sheet[y:y+marker_size, x:x+marker_size] = marker
        
        # Add label below marker
        label = f"ID: {marker_id}"
        label_y = y + marker_size + 30
        cv2.putText(sheet, label, (x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.7, 0, 2, cv2.LINE_AA)
    
    cv2.imwrite(output_file, sheet)
    print(f"Saved marker sheet to {output_file}")
    return sheet

def main():
    """
    Generate markers for the competition
    ID 0 is typically the "target" marker for Challenge 2
    """
    # Generate individual markers
    print("Generating individual markers...")
    save_marker(0, "target_marker_0.png", size=800)  # Main target
    save_marker(1, "marker_1.png", size=800)
    save_marker(2, "marker_2.png", size=800)
    
    # Generate a test sheet with multiple markers
    print("\nGenerating marker sheet...")
    marker_ids = [0, 1, 2, 3, 4, 5]
    generate_marker_sheet(marker_ids, "test_marker_sheet.png")
    
    print("\nDone! Print markers at actual size:")
    print("- For 10cm markers: print at 100mm width")
    print("- For 15cm markers: print at 150mm width")
    print("- Ensure 'Actual Size' is selected when printing")

if __name__ == "__main__":
    main()

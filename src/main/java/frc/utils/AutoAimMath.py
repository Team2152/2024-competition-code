def calculate_angle_required(distance_to_target, closest_distance, farthest_distance, closest_angle, farthest_angle):
    if distance_to_target < closest_distance:
        distance_to_target = closest_distance
    elif distance_to_target > farthest_distance:
        distance_to_target = farthest_distance
        
    factor = (distance_to_target - closest_distance) / (farthest_distance - closest_distance)
    angle_required = closest_angle + factor * (farthest_angle - closest_angle)
    return angle_required

# Example usage:
closest_distance = 0
farthest_distance = 30
closest_angle = 0
farthest_angle = 60

while True:
    distance = int(input("Distance: "))
    angle_required = calculate_angle_required(distance, closest_distance, farthest_distance, closest_angle, farthest_angle)
    print("Angle required:", angle_required)
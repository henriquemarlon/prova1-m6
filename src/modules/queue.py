import csv
from modules import Pose
from collections import deque

# Create a Queue class that inherits from the deque class
class Queue(deque):
    # Create a constructor that takes in a csv file
    def __init__(self, csv_file="./utils/cordinates.csv"):
        
        super().__init__()
        with open(csv_file) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            for row in csv_reader:
                new_pose = Pose()
                new_pose.x, new_pose.y = [float(x) for x in row]
                self.enqueue(new_pose)
    # Create a enqueue method that takes in a value and appends it to the queue
    def enqueue(self, x):
        super().append(x)
    # Create a dequeue method that removes the first item in the queue
    def dequeue(self):
        return super().popleft()
    

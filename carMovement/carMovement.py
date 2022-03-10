from typing import Dict
import numpy as np
import cv2
import matplotlib.pyplot as plt
import sys
import os
import shutil
import time
from adafruit_servokit import ServoKit


# Global Variables
kit = ServoKit(channels=16)
numOfArgs = len(sys.argv)   # number of cli arguments
args = sys.argv             # list of cli arguments
tags = []                   # list for tags
times = []                   # Array for compute time of each image

##
## CLingArgs Class: Stores cli arguments neatly and correctly
##
class CLineArgs:

    ##
    ## __init__: Class constructor
    ##
    def __init__(self, weights, config, names, images):
        self.weights = weights
        self.config = config
        self.names = names
        self.images = images

    ##
    ## print_images: Prints all image paths; used in debugging
    ##
    def print_images(self):
        for img in self.images:
            print(img)

##
## YOLOModel Class: Defines the YOLO model using cli arguments
##
class YOLOModel:

    ##
    ## YOLOModel Constructor
    ##
    def __init__(self, cLineArgs:CLineArgs):

        # Command line arguments
        self.cLineArgs = cLineArgs;
        
        #Model details
        self.network = cv2.dnn.readNetFromDarknet(cLineArgs.config, cLineArgs.weights)
        self.layers = self.network.getLayerNames()
        self.yolo_layers = [self.layers[i-1] for i in self.network.getUnconnectedOutLayers()]
        self.label_names = self.GetLabels()

        # Total class breakdown
        self.totalBreakdown = self.Breakdown()

    ##
    ## Breakdown: Creates a dictionary for (class, instances) for total breakdown of all images
    ##
    def Breakdown(self):
        breakdownTuples = {}

        for label in self.label_names:
            breakdownTuples[label] = 0

        return breakdownTuples

    ##
    ## GetLabels: Puts all the labels from the .names file into a list
    ##
    def GetLabels(self):
        file = open(self.cLineArgs.names)
        labelNames = []

        for line in file:
            labelNames.append(line.rstrip())

        return labelNames


##
## ImageEvaluation: Class for evaluation each image
##
class ImageEvaluation():

    ##
    ## ImageEvaluation Constructor
    ##
    def __init__(self, i:int, model:YOLOModel):

        # Set the model
        self.model = model

        # Load the current image to perform the yolo task on.
        self.image = cv2.imread(self.model.cLineArgs.images[i])

        # Variables for keeping track of 
        self.bounding_boxes = []
        self.confidences = []
        self.classes = []
        self.probability_minimum = 0.5
        self.threshold = 0.3
        self.h, self.w = self.image.shape[:2]

        # Result of yolo
        self.output = self.EvaluateImage()

        # List of breakdown dictionaries for each image
        self.breakdown = self.Breakdown()

    ##
    ## PlotImage: Show the image we are editing. Used for debugging.
    ##
    def PlotImage(self):
        plt.rcParams['figure.figsize'] = (10.0, 10.0)
        plt.imshow(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB))
        plt.show()

    ##
    ## Breakdown: Creates a breakdown for the current image
    ##
    def Breakdown(self):
        breakdownTuples = {}
        for label in self.model.label_names:
            breakdownTuples[label] = 0
        return breakdownTuples

    ##
    ## EvaluateImage: Evaluates the current image using yolo
    ##
    def EvaluateImage(self):
        
        # Convert loaded image into a blob (4D array) and give to the network 
        input_blob = cv2.dnn.blobFromImage(self.image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.model.network.setInput(input_blob)
        output = self.model.network.forward(self.model.yolo_layers)

        return output

    ##
    ## GetVariables: Get bounding boxes, confidences, and classes for the images
    ##
    def GetVariables(self):

        for result in self.output:

            for detection in result:

                scores = detection[5:]                      # Skip the first 5 since we only have 80 classes
                class_current = np.argmax(scores)           # Choose class with highest confidence
                confidence_current = scores[class_current]  # Save that confidence

                # Only add result to the image if the confidence is above 50%
                if confidence_current > self.probability_minimum:

                    # Set the parameters for the bounding box of the current dectection
                    box_current = detection[0:4] * np.array([self.w, self.h, self.w, self.h])
                    x_center, y_center, box_width, box_height = box_current.astype('int')
                    x_min = int(x_center - (box_width / 2))
                    y_min = int(y_center - (box_height / 2))

                    # Add the results to their respective lists
                    self.bounding_boxes.append([x_min, y_min, int(box_width), int(box_height)])
                    self.confidences.append(float(confidence_current))
                    self.classes.append(class_current)
    
    ##
    ## DrawBoundingBoxes: Draws the bounding boxes, class, and confidence on the current image's output.
    ##
    def DrawBoundingBoxes(self, img_num):

        # Save our results from GetVariables stage
        results = cv2.dnn.NMSBoxes(self.bounding_boxes, self.confidences, self.probability_minimum, self.threshold)
        
        # Get the number of labels
        coco_labels = len(self.model.label_names)

        #Choose random colors for each class' bounding box
        np.random.seed(42)
        colours = np.random.randint(0, 255, size=(coco_labels, 3), dtype='uint8')

        # If we have any results
        if len(results) > 0:
            
            # Flatten results into one dimension.
            for i in results.flatten():

                # Add current class to the class breakdown of this image
                if self.model.label_names[self.classes[i]] in self.model.totalBreakdown:
                    self.model.totalBreakdown[self.model.label_names[self.classes[i]]] += 1
                    self.breakdown[self.model.label_names[self.classes[i]]] += 1

                # Get current image's bounding box parameters
                x_min, y_min = self.bounding_boxes[i][0], self.bounding_boxes[i][1]
                box_width, box_height = self.bounding_boxes[i][2], self.bounding_boxes[i][3]

                # Set the color of the bounding box
                colour_box = [int(j) for j in colours[self.classes[i]]]

                # Draw the bounding box and write the class + confidences on it
                cv2.rectangle(self.image, (x_min, y_min), (x_min + box_width, y_min + box_height), colour_box, 3)
                text_box =  self.model.label_names[self.classes[i]] + ' : {:.4f}'.format(self.confidences[i])
                cv2.putText(self.image, text_box, (x_min, y_min - 7), cv2.FONT_HERSHEY_SIMPLEX, 1, colour_box, 3)

        # Write all these results to the output image file.
        cv2.imwrite(self.model.cLineArgs.images[img_num], self.image)

##
## ExtractArgs: Iterates through cli arguments list.
## Returns CLineArgs Class if iteration is successful, error message string if error is encountered
##
def ExtractArgs(args:list):
    
    # Initialzing CLI argument variables. If they aren't changed, they are missing.
    weights = None 
    config = None
    names = None
    images = []

    # Report error if there are not enough arguments
    if numOfArgs < 1:
        return "You are missing at least one argument. Please use -help for usage."
    
    # Iterate through all of the cli arguments (start at idx 1 since idx zero is run_yolo.py)
    for i in range(1, numOfArgs):

        # .weights file encountered
        if (args[i].endswith('.weights')):

            # verify a .weights file hasn't already been encountered
            if weights == None:

                #verify the .weights file path exists
                if (os.path.isfile(args[i])):
                    weights = args[i]
                else:
                    return ".weights file at path " + args[i] + " could not be found. Please confirm your path is correct."

            else:
                return "Found too many .weights files. Please only include one."

        # .cfg file encountered
        elif (args[i].endswith('.cfg')):

            # verify a .cfg file hasn't already been encountered
            if config == None:

                # verify the .cfg file path exists
                if (os.path.isfile(args[i])):
                    config = args[i]
                else:
                    return ".cfg file at path " + args[i] + " could not be found. Please confirm your path is correct."

            else:
                return "Found too many .cfg files. Please only include one."

        # .names file encountered
        elif (args[i].endswith('.names')):

            # verify the .names file exists
            if names == None:

                #verify the .names file path exists
                if (os.path.isfile(args[i])):
                    names = args[i]
                else:
                    return ".names file at path " + args[i] + " could not be found. Please confirm your path is correct."

            else:
                return "Found too many .names files. Please only include one."

        # Single image file (.jpg) encountered
        elif (args[i].endswith('.jpg')):

            # Verify image file path exists
            if (os.path.isfile(args[i])):

                # Create output file name
                head, tail = os.path.split(args[i])
                image_name = tail[0:(len(tail)-4)] + '_out.jpg'

                # Create output file for image by copying the image to the new location
                shutil.copy2(args[i], 'out_imgs/' + image_name)
                
                # Add image to list of images that will be run through yolo.
                images.append('out_imgs/' + image_name)

            else:
                return "Image at path " + args[i] + " was not found. Please confirm your path is correct"

        # Folder encountered. Looking for .jpg images..
        elif (os.path.isdir(args[i])):

            dirContents = os.listdir(args[i])   # list containing the contents of the folder
            imgFound = False                    # Flag to determine if any images have been found

            # Iterating through folder contents. 
            for item in dirContents:

                # Save item to image list if it is a .jpg file
                if item.endswith('.jpg'):

                    # Create output file name
                    image_name = item[0:len(item)-4] + '_out.jpg'

                    # Create output file for image by copying the image to the new location
                    if args[i].endswith('/'):
                        imagePath = args[i] + item
                    else: 
                        imagePath = args[i] +'/' + item 
                    shutil.copy(imagePath, 'out_imgs/' + image_name)

                    # Add image to list of images that will be run through yolo.
                    images.append('out_imgs/' + image_name)

                    imgFound = True            # Found an image; activate flag.

            # Report error if no images were found in the directory
            if imgFound == False:
                return "No images found in the given directory. Please add images to your directory or chose a new path"

        # Tag encountered. Add to tag list.
        elif (args[i].startswith('-')):
            tags.append(args[i])

        # Did not encounter any of the above scenarios
        else:
            if (args[i].endswith('/')):
                return "Could not find the path " + args[i] + ". Please check that your path exists."
            else:
                return "Could not resolve the command line argument: " + args[i] + ". Please use -help for usage."
            
    # Confirming that all necessary inputs are accounted for
    if (weights == None):
        return "No .weights file found. Please use -help for usage."
    if (config == None):
        return "No .cfg file found. Please use -help for usage."
    if (names == None):
        return "No .names file found. Please use -help for usage."
    if (len(images) == 0): 
        return "No images found. Please use -help for usage."
    
    # Cli argument iteration is sucessful. Return CLineArgs class with results.
    return CLineArgs(weights, config, names, images)

##
## GetTotalClasses: Prints the total number of classes found across all images
##
def GetTotalClasses(totalBreakdown:Dict):

    totalClasses = 0
    for key,value in totalBreakdown.items():
        if value != 0:
            totalClasses += 1
    print("Total number of objects/classes detected: " + str(totalClasses))

##
## GetBreakdown: Prints the breakdown for all images combined
##
def GetBreakdown(totalBreakdown: Dict):
    for key,value in totalBreakdown.items():
        if value != 0:
            capKey = key[0].upper() + key[1:]
            print(capKey + ": " + str(value))

##
## PerImageBreakdown: Prints the breakdown for each image seperately
##
def PerImageBreakdown(indiv_breakdowns: list, cLineArgs: CLineArgs):
    for i in range(len(indiv_breakdowns)):
        head, tail = os.path.split(cLineArgs.images[i])
        print(tail + " =>")
        GetBreakdown(indiv_breakdowns[i])

        if i < (len(indiv_breakdowns)-1):
            print("\r")

##
## AverageInference: Prints the average inference time across all images
##
def AverageInference(times: list):
    averageInf = sum(times)/len(times)
    print("Average Inference Time: " + str(averageInf))

def main():

    # Create folder for output images if it does not already exist
    if (os.path.isdir('out_imgs/') == False):
        os.makedirs('out_imgs/')
    
    # Get the command line arguments
    cLineArgs = ExtractArgs(args)

    # Check for help tag, and print help if found.
    if (tags.count("-help") >= 1):
        print(open('help.txt').read())
        sys.exit()

    # Check for cli argument errors. cLineArgs will return a string with the error message, if there is one.
    if (type(cLineArgs) != CLineArgs):
        print(cLineArgs)
        sys.exit()

    # Catch unknown tags and report error.
    validTags = ["-help", "-find-people"]
    for tag in tags:
        if (validTags.count(tag) == 0):
            print("Unknown tag, '" + tag + "' found. Please use -help for usage.")
            sys.exit()

    # Create the model
    yoloModel = YOLOModel(cLineArgs)

    # Run the model
    indiv_breakdowns = []
    for i in range(len(cLineArgs.images)):

        startTime = time.time()                         # Start the time

        ImageEval = ImageEvaluation(i, yoloModel)       # Evaluate the image
        ImageEval.GetVariables()                        # Get the variables for drawing (class, confidence, bounding box)
        ImageEval.DrawBoundingBoxes(i)                  # Draw on the image

        endTime = time.time()                           # End the time
        times.append(endTime-startTime)                 # Add time to compute to list

        indiv_breakdowns.append(ImageEval.breakdown)    # Add current image breakdown to list
    
    print("\r")

    # If image has people, stop car. Else, run car. 
    if (tags.count("-find-people") >= 1):
        if (yoloModel.totalBreakdown["person"] > 0):
            print("Found a Person. Stopping car.")
            kit.continuous_servo[1].throttle = 0
        else:
            print("No people in sight")
            kit.continuous_servo[1].throttle = 1

    print("Annotated image stored as:")
    for image in cLineArgs.images:
        print(image)
    
main()
'''
*********************************************************************************
*
*        		===============================================
*           		        CYBORG OPENCV TASK 2
*        		===============================================
*
*
*********************************************************************************
'''

# Author Name:		[Your Name]
# Roll No:			[Your Roll Number]
# Filename:			task_2A_{your_name}.py
# Functions:		detect_faulty_squares


####################### IMPORT MODULES #######################
import cv2
import numpy as np
##############################################################

def detect_faulty_squares(input_image):
    """
    Purpose:
    ---
    This function takes the image as an argument and returns a dictionary
    containing the details of the faulty squares in the chessboard.

    Input Arguments:
    ---
    `input_image` :	[ numpy array ]
            numpy array of image returned by cv2 library

    Returns:
    ---
    `sorted_squares` : { dictionary }
            dictionary with keys as (faulty_color, position)
            and values as the original expected color (Black/White)

    Example call:
    ---
    sorted_squares = detect_faulty_squares(input_image)
    
    NB : Do not change any of the input and output parameters.
    
    You can create any number of functions as per your requirement.
    """
        
    sorted_squares = {}
    
	################################################################
    ##                     Add your code here                     ##
	################################################################

    return sorted_squares
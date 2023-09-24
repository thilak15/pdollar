# $P Point-Cloud Gesture Recognizer

## Overview
This repository contains a Java implementation of the $P Point-Cloud Recognizer, a powerful gesture recognition algorithm. The $P Point-Cloud Recognizer is an efficient and robust method designed for both real-time applications and rapid prototyping.

## Class Description
1. **Point**: Represents a single point in the gesture. Holds X, Y coordinates and an ID.
2. **_Point_cloud**: Represents a set of points that form a gesture. Contains methods to preprocess points for recognition.
3. **_Result**: Encapsulates the result after gesture recognition. Provides the name of the recognized gesture.
4. **PRecognizer**: Houses core algorithms and methods essential for gesture recognition.
5. **pdollar**: Main driver class. It offers functionalities to:
   - Add gesture templates
   - Clear gesture templates
   - Recognize gestures from event streams

## Usage
1. **Adding a Gesture**:
   ```bash
   pdollar -t <gesturefile>

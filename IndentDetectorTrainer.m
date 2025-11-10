%TRAIN YOLO Network
clear;
clc;

gpuDevice().reset;
gpuDevice(1);

%Inputs:
addpath('Data')
yoloDetectorName = 'VickersYOLODetector.mat';%Desired name of YOLO detector file
GTruthFileName = 'VickerDataGTruth.mat';
gTruth = load(GTruthFileName);
gTruth = gTruth.VickersDataGTruth;%Rename to whatever field name is
classes = ["indent"];
numAnchors = 6;
inputSize = [224 224 3];

trainingData = objectDetectorTrainingData(gTruth);
imds = imageDatastore(trainingData.imageFilename);
blds = boxLabelDatastore(trainingData(:, 2:end));
classNames = gTruth.LabelDefinitions.Name;
trainingData = combine(imds, blds);

[anchors, ~] = estimateAnchorBoxes(trainingData, numAnchors);
area = anchors(:,1).*anchors(:,2);
[~,idx] = sort(area,"descend");
anchors = anchors(idx,:);
anchorBoxes = {anchors(1:3,:);anchors(4:6,:)};

baseNetwork = 'tiny-yolov4-coco';
detector = yolov4ObjectDetector(baseNetwork,classes,anchorBoxes, InputSize=inputSize);
optionsYOLO = trainingOptions('sgdm', 'InitialLearnRate', 1e-4, 'MaxEpochs', 30, 'MiniBatchSize', 4, 'ExecutionEnvironment', 'gpu');
[YOLOdetector, info] = trainYOLOv4ObjectDetector(trainingData, detector, optionsYOLO);

disp('YOLO Training Complete!')
save(yoloDetectorName, 'YOLOdetector');
%%
%TRAIN CASCADE OBJECT DETECTOR
clear;
clc;

%Inputs:
%Change the path below to to the path of the folder containing the negative images for the training
negativeFolder = '"C:\Users\Eduar\Documents\MATLAB\Microindentation-Analyzer\Negative"';
indentDataTable = load('VickersDataTable.mat');
indentData = indentDataTable.VickersDataTable;%Rename to whatever field name is 
cascadeDetectorName = 'VickersCascadeDetector.xml';

trainCascadeObjectDetector(cascadeDetectorName, indentData, negativeFolder);
disp('Cascade Training Complete!')
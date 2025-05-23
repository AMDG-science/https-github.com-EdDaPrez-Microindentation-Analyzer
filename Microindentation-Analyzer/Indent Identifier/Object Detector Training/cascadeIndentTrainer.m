negativeFolder = 'E:\Pictures\Knoop\negativeIndent';%change this to whatever folder holds your negative images
NumStages = 20;
FAR = .025;

trainCascadeObjectDetector('realknoopIndentDetector_20_025.xml', realIndents, negativeFolder, 'NumCascadeStages', NumStages, 'FalseAlarmRate',FAR);%realIndents imported from matlab image segmentation app
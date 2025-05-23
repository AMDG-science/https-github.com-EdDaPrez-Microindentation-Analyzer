classdef IndentIdentifier < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TabGroup                        matlab.ui.container.TabGroup
        SetupTab                        matlab.ui.container.Tab
        RetryButton                     matlab.ui.control.Button
        TakeSnapshotButton              matlab.ui.control.Button
        ChooseWebcamDropDown            matlab.ui.control.DropDown
        ChooseWebcamDropDownLabel       matlab.ui.control.Label
        InputTypeSwitch                 matlab.ui.control.Switch
        InputTypeSwitchLabel            matlab.ui.control.Label
        ClearButton                     matlab.ui.control.Button
        RotateImageSlider               matlab.ui.control.Slider
        RotateImageSliderLabel          matlab.ui.control.Label
        IndentTypeSwitch                matlab.ui.control.Switch
        IndentTypeSwitchLabel           matlab.ui.control.Label
        CropImageButton                 matlab.ui.control.Button
        CalculateButton                 matlab.ui.control.Button
        MassgramsEditField              matlab.ui.control.NumericEditField
        MassgramsEditFieldLabel         matlab.ui.control.Label
        MicronPixelRatioEditField       matlab.ui.control.NumericEditField
        MicronPixelRatioEditFieldLabel  matlab.ui.control.Label
        AddIndentButton                 matlab.ui.control.Button
        BoundingBoxesSwitch             matlab.ui.control.Switch
        BoundingBoxesSwitchLabel        matlab.ui.control.Label
        LoadImageButton                 matlab.ui.control.Button
        ImagePreview                    matlab.ui.control.UIAxes
        ResultsTab                      matlab.ui.container.Tab
        ExportButton                    matlab.ui.control.Button
        skewLabel                       matlab.ui.control.Label
        ConfirmButton                   matlab.ui.control.Button
        RecalculateButton               matlab.ui.control.Button
        HKValueDiagonal                 matlab.ui.control.Label
        HKValueArea                     matlab.ui.control.Label
        HKdiagonalLabel                 matlab.ui.control.Label
        HKareaLabel                     matlab.ui.control.Label
        CycleForward                    matlab.ui.control.Button
        CycleBackward                   matlab.ui.control.Button
        ImagePreview_2                  matlab.ui.control.UIAxes
        IndentImage                     matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        originalImage %image as selected by user
        newImage %image  after bounding boxes added
        originalIndentImages
        indentImages %array of indent images
        currentIndent = 1 %index of currently displayed indent
        boundingBoxes %array of all indent bounding boxes
        hardnessConstantsArea %array of all knoop hardness constants by area
        hardnessConstantsDiagonal %array of all knoop hardness constants by diagonal length
        rotatingImage %image before rotation
        diamondROI %roi object for recaclculating indents
        cam %webcam object
        skewedList
        error = .5;
        folderName = 'Data'
        sigFigs = 3;%# of significant figures desired
    end

    methods (Access = private)
        
        function allevents(app, event)
            
            if app.TabGroup.SelectedTab == app.SetupTab
                app.boundingBoxes{end} = event.Position;
            else
                app.diamondROI = event;
            end
        end

        function FindIndents(app, angleLessThan, angleGreaterThan)
            functionname='indentIdentifierGUI.mlapp'; functiondir=which(functionname);
            functiondir=functiondir(1:end-length(functionname));
            addpath([functiondir 'Functions'])
            addpath([functiondir 'Object Detector Training'])
            numLines = 4;
            if strcmp(app.BoundingBoxesSwitch.Value, 'Automatic')
                if strcmp(app.IndentTypeSwitch.Value, 'Knoop')
                    detector = vision.CascadeObjectDetector('knoopIndentDetector_20_025.xml');%initialize machine vision
                else
                    detector = vision.CascadeObjectDetector('vickersIndentDetector_20_025.xml');%initialize machine vision
                end
                foundBoundingBoxes = step(detector,app.originalImage);%detect all indents and put positions into matrix of boxes
                numBoundingBoxes = height(foundBoundingBoxes);
            else
                foundBoundingBoxes = app.boundingBoxes;
                numBoundingBoxes = length(foundBoundingBoxes);                
            end
            %process individual objects
            verticalPoints.x = [];
            verticalPoints.y = [];
            app.indentImages = {};
            app.boundingBoxes = {};
            app.hardnessConstantsArea = [];
            app.hardnessConstantsDiagonal = [];
            app.originalIndentImages = [];
            app.skewedList = {};
            for i = 1:1:numBoundingBoxes
                %roi = images.roi.Rectangle(gca, 'Position', bbox(i,1:4), 'StripeColor', 'b');%draw roi
                if strcmp(app.BoundingBoxesSwitch.Value, 'Automatic')
                   currentBoundingBox = foundBoundingBoxes(i,1:4);
                else
                    currentBoundingBox = foundBoundingBoxes{i};
                end
                indentImage = imcrop(app.originalImage,currentBoundingBox);
                originalIndentImage = indentImage;
                grayIndentImage = rgb2gray(indentImage);
                binarizedIndentImage = imbinarize(grayIndentImage);
                edgeImage = edge(binarizedIndentImage, 'sobel');%detect edges            
                [H,theta,rho] = hough(edgeImage);
                exclude = [];
                while true %keep going until no more of the same line found twice
                    clear intersect;
                    clear allLines;
                    clear verticalPoints;
                    intersect.x = [];
                    intersect.y = [];
                    verticalPoints.x = [];
                    verticalPoints.y = [];
                    numPeaks = numLines + length(exclude);
                    P = houghpeaks(H,numPeaks,'threshold',ceil(0.3*max(H(:))));%choose # of points
                    lines = houghlines(edgeImage,theta,rho,P,'FillGap',1000,'MinLength',8);
                    for k = 1:1:length(exclude)
                        lines(exclude(k)) = [];
                    end
                    for k = 1:1:length(lines)
                        pto1 = lines(k).point1;  
                        pto2 = lines(k).point2;
                        % A vector along the ray from pto1 to pto2...
                        V1 = pto2 - pto1;
                        V2 = pto1 - pto2;        
                        factor_distance = 3;%line extended by 200%
                        % Extend the ray
                        pext1 = pto1 + V1*factor_distance;
                        pext2 = pto2 + V2*factor_distance;
                        
                        allLines(k).x = [pext1(1) pext2(1)];
                        allLines(k).y = [pext1(2) pext2(2)];
                    end
                   
                    %find intersections
                    intersections = 0;
                    currentExclusions = 0;
                    stopLooking = false;
                    for k = 1:1:(length(allLines)-1)
                        for n = (k + 1):1:length(allLines)
                            [xi, yi] = linexline(allLines(k).x, allLines(k).y, allLines(n).x, allLines(n).y, 0);%last parameter 1 to plot intersections
                            if(~isnan(xi))
                                intersect(intersections + 1).x = xi;
                                intersect(intersections + 1).y = yi;
                                intersections = intersections + 1;
                                angleValue = abs(lines(k).theta - lines(n).theta);
                                if strcmp(app.IndentTypeSwitch.Value, 'Knoop')
                                    if angleValue >= 90
                                        angleValue = 180-angleValue;
                                    end
                                end
                                if angleValue <= angleGreaterThan || angleValue >= angleLessThan
                                    exclude(end + 1) = n;
                                    currentExclusions = currentExclusions + 1;
                                    stopLooking = true;
                                    break;
                                end
                            end
                        end
                        if stopLooking == true
                            break;
                        end
                    end
                    if currentExclusions == 0
                        break;
                    end
                end
                %filter unwanted indents
                %insert any filter conditions after if statement
                if intersections ~= 4 && strcmp(app.BoundingBoxesSwitch.Value, 'Automatic')
                    continue
                else            
                    %search image for diagonals
                    %this loop will look through all points of the indent and find the
                    %leftmost and righmost points
                    dimensions = size(edgeImage);%1-dimensional array [rows,columns]
                    leftPoint = [dimensions(2) dimensions(1)];
                    rightPoint = [0 0];
                    for k = 1:1:length(intersect)
                        if intersect(k).x < leftPoint(1)
                            leftPoint = [intersect(k).x intersect(k).y];
                            n1 = k;
                        end
                        if intersect(k).x > rightPoint(1)
                            rightPoint = [intersect(k).x intersect(k).y];
                            n2 = k;
                        end
                    end
                    if length(intersect) >= 4
                        nonCounter = 1;
                        for k = 1:1:length(intersect)
                           if (k ~= n1) && (k ~= n2)
                                verticalPoints(nonCounter).x = intersect(k).x;
                                verticalPoints(nonCounter).y = intersect(k).y;
                                nonCounter = nonCounter + 1;
                           end
                        end
                        %plot edges
                        indentShape = [leftPoint verticalPoints(1).x verticalPoints(1).y rightPoint verticalPoints(2).x verticalPoints(2).y];
                            %xy pairs of indent coordinates
                        indentImage = insertShape(indentImage, "polygon", indentShape, ShapeColor="red", LineWidth=1);
                        %plot diagonals
                        indentImage = insertShape(indentImage, "line", [leftPoint rightPoint], ShapeColor="blue", LineWidth=1);
                        indentImage = insertShape(indentImage, "line", [verticalPoints(1).x verticalPoints(1).y verticalPoints(2).x verticalPoints(2).y],...
                            ShapeColor="blue", LineWidth=1);
                        L1 = sqrt((rightPoint(1)-leftPoint(1))^2+(rightPoint(2)-leftPoint(2))^2);%calculate indent length in pixels
                        L2 = sqrt((verticalPoints(1).x-verticalPoints(2).x)^2+(verticalPoints(1).y-verticalPoints(2).y)^2);%calculate indent length in pixels
                        topPoint = [verticalPoints(1).x verticalPoints(1).y];
                        bottomPoint = [verticalPoints(2).x verticalPoints(2).y];
                        if strcmp(app.IndentTypeSwitch.Value, 'Knoop')
                            indentArea = 0.5*L1*L2;
                            app.hardnessConstantsArea(end + 1) = ((app.MassgramsEditField.Value)/(indentArea*(app.MicronPixelRatioEditField.Value^2)))*10^3;
                            %xy pairs of indent coordinates
                            L1x = [bottomPoint(1) topPoint(1)];
                            L1y = [bottomPoint(2) topPoint(2)];
                            L2x = [leftPoint(1) rightPoint(1)];
                            L2y = [leftPoint(2) rightPoint(2)];
                            [xi, yi] = linexline(L1x, L1y, L2x, L2y, 0);%last parameter 1 to plot intersections
                            longDiagonal = L1;
                            if L2>L1
                                longDiagonal = L2;
                                halfDiagonal1 = sqrt((topPoint(1)-xi)^2+(topPoint(2)-yi)^2);%calculate diagonal length in pixels
                                halfDiagonal2 = sqrt((xi-bottomPoint(1))^2+(yi-bottomPoint(2))^2);%calculate diagonal length in pixels
                            else
                                halfDiagonal1 = sqrt((rightPoint(1)-xi)^2+(rightPoint(2)-yi)^2);%calculate diagonal length in pixels
                                halfDiagonal2 = sqrt((xi-leftPoint(1))^2+(yi-leftPoint(2))^2);%calculate diagonal length in pixels
                            end
                            if halfDiagonal1 < halfDiagonal2
                                if halfDiagonal2 > ((1 + app.error)*halfDiagonal1)
                                    app.skewedList{end + 1} = 1;
                                else
                                    app.skewedList{end + 1} = 0;
                                end
                            else
                                if halfDiagonal1 > ((1 + app.error)*halfDiagonal2)
                                    app.skewedList{end + 1} = 1;
                                else
                                    app.skewedList{end + 1} = 0;
                                end
                            end
                            app.hardnessConstantsDiagonal(end + 1) = 14229*(app.MassgramsEditField.Value/((longDiagonal*app.MicronPixelRatioEditField.Value)^2));
                        else
                            d = (L1 + L2)/2;
                            app.hardnessConstantsArea(end + 1) = ((app.MassgramsEditField.Value)/(d*app.MicronPixelRatioEditField.Value)^2)*2*sind(68)*10^3;
                            app.hardnessConstantsDiagonal(end + 1) = 0;
                        end
                    else
                        app.hardnessConstantsArea(end + 1) = 0;
                        app.hardnessConstantsDiagonal(end + 1) = 0;
                        app.skewedList{end + 1} = 0;
                    end
                    app.indentImages{end + 1} = indentImage;
                    app.boundingBoxes{end + 1} = currentBoundingBox;
                    app.originalIndentImages{end + 1} = originalIndentImage;
                end
            end
            if strcmp(app.BoundingBoxesSwitch.Value, 'Automatic')
               release(detector);
            end
        end

        function updateScreens(app)
            imshow(insertShape(app.newImage, "rectangle", app.boundingBoxes{app.currentIndent}, "ShapeColor", "blue", "LineWidth", 2), 'parent', app.ImagePreview_2);%display image
            imshow(app.indentImages{app.currentIndent}, 'parent', app.IndentImage);%display indent image
            app.HKValueArea.Text = string(round(app.hardnessConstantsArea(app.currentIndent), app.sigFigs, "significant"));
            app.HKValueDiagonal.Text = string(round(app.hardnessConstantsDiagonal(app.currentIndent), app.sigFigs, "significant"));
            if strcmp(app.IndentTypeSwitch.Value, 'Knoop')
                app.skewLabel.Visible = 1;
                if app.skewedList{app.currentIndent} == 0
                    app.skewLabel.Text = "No skew detected. Knoop hardness calculation by area recommended.";
                elseif app.skewedList{app.currentIndent} == 1
                    app.skewLabel.Text = "Skew detected. Knoop hardness calculation by diagonal recommended.";
                end
            end
        end
        
        function manualCalculate(app, roi)
            if length(roi.Position) == 4
                leftPoint=roi.Position(1,1:2);bottomPoint=roi.Position(2,1:2);rightPoint=roi.Position(3,1:2);topPoint=roi.Position(4,1:2);
                indentShape = [leftPoint bottomPoint rightPoint topPoint];
                indentImage = insertShape(app.originalIndentImages{app.currentIndent}, "polygon", indentShape, ShapeColor="red", LineWidth=1);
                %plot diagonals
                indentImage = insertShape(indentImage, "line", [leftPoint rightPoint], ShapeColor="blue", LineWidth=1);
                indentImage = insertShape(indentImage, "line", [topPoint bottomPoint],...
                    ShapeColor="blue", LineWidth=1);
                L1 = sqrt((rightPoint(1)-leftPoint(1))^2+(rightPoint(2)-leftPoint(2))^2);%calculate diagonal length in pixels
                L2 = sqrt((topPoint(1)-bottomPoint(1))^2+(topPoint(2)-bottomPoint(2))^2);%calculate diagonal length in pixels
                if strcmp(app.IndentTypeSwitch.Value, 'Knoop')
                    %xy pairs of indent coordinates
                    indentArea = 0.5*L1*L2;
                    app.hardnessConstantsArea(app.currentIndent) = ((app.MassgramsEditField.Value)/(indentArea*(app.MicronPixelRatioEditField.Value^2)))*10^3;
                    L1x = [bottomPoint(1) topPoint(1)];
                    L1y = [bottomPoint(2) topPoint(2)];
                    L2x = [leftPoint(1) rightPoint(1)];
                    L2y = [leftPoint(2) rightPoint(2)];
                    [xi, yi] = linexline(L1x, L1y, L2x, L2y, 0);%last parameter 1 to plot intersections
                    longDiagonal = L1;
                    if L2>L1
                        longDiagonal = L2;
                        halfDiagonal1 = sqrt((topPoint(1)-xi)^2+(topPoint(2)-yi)^2);%calculate diagonal length in pixels
                        halfDiagonal2 = sqrt((xi-bottomPoint(1))^2+(yi-bottomPoint(2))^2);%calculate diagonal length in pixels
                    else
                        halfDiagonal1 = sqrt((rightPoint(1)-xi)^2+(rightPoint(2)-yi)^2);%calculate diagonal length in pixels
                        halfDiagonal2 = sqrt((xi-leftPoint(1))^2+(yi-leftPoint(2))^2);%calculate diagonal length in pixels
                    end
                    if halfDiagonal1 < halfDiagonal2
                        if halfDiagonal2 > ((1 + app.error)*halfDiagonal1)
                            app.skewedList{app.currentIndent} = 1;
                        else
                            app.skewedList{app.currentIndent} = 0;
                        end
                    else
                        if halfDiagonal1 > ((1 + app.error)*halfDiagonal2)
                            app.skewedList{app.currentIndent} = 1;
                        else
                            app.skewedList{app.currentIndent} = 0;
                        end
                    end
                    app.skewLabel.Visible = 1;
                    app.hardnessConstantsDiagonal(app.currentIndent) = 14229*(app.MassgramsEditField.Value/((longDiagonal*app.MicronPixelRatioEditField.Value)^2));
                    app.indentImages{app.currentIndent} = indentImage;
                else
                    d = (L1 + L2)/2;
                    app.hardnessConstantsArea(app.currentIndent) = ((app.MassgramsEditField.Value)/(d*app.MicronPixelRatioEditField.Value)^2)*2*sind(68)*10^3;
                    app.hardnessConstantsDiagonal(app.currentIndent) = 0;
                    app.indentImages{app.currentIndent} = indentImage;
                end
            updateScreens(app);              
            else
                msgbox(sprintf('Please create a 4-sided polygon.'), 'Error', 'Warning');
                imshow(app.originalIndentImages{app.currentIndent}, 'parent', app.IndentImage);%display indent image
            end
        end
        
        function showUI(app)
            app.CropImageButton.Visible = 1;
            app.CalculateButton.Visible = 1;
            app.RotateImageSlider.Visible = 1;
            app.IndentTypeSwitch.Visible = 1;
            app.IndentTypeSwitchLabel.Visible = 1;
            app.BoundingBoxesSwitch.Visible = 1;
            app.BoundingBoxesSwitchLabel.Visible = 1;
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: LoadImageButton
        function LoadImageButtonPushed(app, event)
            [path, nofile] = imgetfile();
            if nofile
                msgbox(sprintf('Image not Found'), 'Error', 'Warning');
                return
            end
            app.originalImage = imread(path);
            app.ImagePreview.Visible = 1;
            imshow(app.originalImage, 'parent', app.ImagePreview);
            showUI(app);
            app.boundingBoxes =  {};
            app.rotatingImage = app.originalImage;
        end

        % Button pushed function: CalculateButton
        function CalculateButtonPushed(app, event)
            app.skewLabel.Visible = 0;
            app.ExportButton.Visible = 1;
            if strcmp(app.IndentTypeSwitch.Value, 'Knoop')
                FindIndents(app, 22, 10);%median is 16.23
                app.HKValueDiagonal.Visible = 1;
                app.HKdiagonalLabel.Visible = 1;
                app.HKareaLabel.Text = 'HK (area)';                
            else
                FindIndents(app, 110, 70);
                app.HKareaLabel.Text = 'HV';
                app.HKValueDiagonal.Visible = 0;
                app.HKdiagonalLabel.Visible = 0;
            end
            if isempty(app.boundingBoxes) && strcmp(app.BoundingBoxesSwitch.Value, 'Automatic')
                msgbox(sprintf('No bounding boxes detected. Try manually inserting bounding boxes.'), 'Error', 'Warning');
                return
            elseif isempty(app.boundingBoxes) && strcmp(app.BoundingBoxesSwitch.Value, 'Manual')
                msgbox(sprintf('No bounding boxes detected. Add bounding boxes or switch mode to automatic.'), 'Error', 'Warning');
                return
            end
            app.newImage = app.originalImage;
            for k = 1:1:length(app.boundingBoxes)
                app.newImage = insertShape(app.newImage, "rectangle", app.boundingBoxes{k}, "ShapeColor", "yellow", "LineWidth", 2);
            end
            app.currentIndent = 1;
            updateScreens(app);
            app.TabGroup.SelectedTab = app.ResultsTab;
            app.ConfirmButton.Visible = 0;
        end

        % Button pushed function: CycleForward
        function CycleForwardButtonPushed(app, event)
            app.currentIndent = app.currentIndent + 1;
            if app.currentIndent > length(app.indentImages)
                app.currentIndent = 1;
            end
            updateScreens(app);
        end

        % Button pushed function: CycleBackward
        function CycleBackwardButtonPushed(app, event)
            app.currentIndent = app.currentIndent - 1;
            if app.currentIndent < 1
                app.currentIndent = length(app.indentImages);
            end
            updateScreens(app);
        end

        % Button pushed function: AddIndentButton
        function AddIndentButtonPushed(app, event)
            if ~isempty(app.boundingBoxes)
                app.newImage = insertShape(app.newImage, "rectangle", app.boundingBoxes{end}, "ShapeColor", "yellow", "LineWidth", 2);
                imshow(app.newImage, 'parent', app.ImagePreview)
            else
                app.newImage = app.originalImage;
            end
            roi = drawrectangle(app.ImagePreview);
            app.boundingBoxes{end + 1} = roi.Position;
            addlistener(roi,'ROIMoved',@(varargin)allevents(app, roi));
        end

        % Value changed function: BoundingBoxesSwitch
        function BoundingBoxesSwitchValueChanged(app, event)
            value = app.BoundingBoxesSwitch.Value;
            app.boundingBoxes =  {};
            if strcmp(value, 'Automatic')
                app.AddIndentButton.Visible = 0;%0 makes it invisible, vice versa
                app.ClearButton.Visible = 0;
            else
                app.AddIndentButton.Visible = 1;
                app.ClearButton.Visible = 1;
            end
            imshow(app.originalImage, 'parent', app.ImagePreview);
        end

        % Button pushed function: CropImageButton
        function CropImageButtonPushed(app, event)
            roi = drawrectangle(app.ImagePreview);
            app.originalImage = imcrop(app.originalImage, roi.Position);
            imshow(app.originalImage, 'parent', app.ImagePreview);
            app.boundingBoxes = {};
            app.rotatingImage = app.originalImage;
        end

        % Button pushed function: RecalculateButton
        function RecalculateButtonPushed(app, event)
            app.ExportButton.Visible = 0;
            imshow(app.originalIndentImages{app.currentIndent}, 'parent', app.IndentImage);%display indent image
            roi = drawpolygon(app.IndentImage);
            app.diamondROI = roi;
            addlistener(roi,'ROIMoved',@(varargin)allevents(app, roi));
            app.ConfirmButton.Visible = 1;
        end

        % Value changed function: IndentTypeSwitch
        function IndentTypeSwitchValueChanged(app, event)
            value = app.IndentTypeSwitch.Value;
            imshow(app.originalImage, 'parent', app.ImagePreview);
            app.boundingBoxes =  {};
        end

        % Value changed function: RotateImageSlider
        function RotateImageSliderValueChanged(app, event)
            value = app.RotateImageSlider.Value;
            app.originalImage = imrotate(app.rotatingImage, value);
            app.boundingBoxes = {};
            imshow(app.originalImage, 'Parent', app.ImagePreview);
        end

        % Button pushed function: ClearButton
        function ClearButtonPushed(app, event)
            app.boundingBoxes = {};
            imshow(app.originalImage, 'Parent', app.ImagePreview);
        end

        % Button pushed function: ConfirmButton
        function ConfirmButtonPushed(app, event)
            roi = app.diamondROI;
            manualCalculate(app, roi);
            app.ConfirmButton.Visible = 0;
            app.ExportButton.Visible = 1;
        end

        % Value changed function: InputTypeSwitch
        function InputTypeSwitchValueChanged(app, event)
            value = app.InputTypeSwitch.Value;
            app.boundingBoxes =  {};
            if strcmp(value, 'File')
                app.RetryButton.Visible = 0;%0 makes it invisible, vice versa
                app.TakeSnapshotButton.Visible = 0;%0 makes it invisible, vice versa
                app.LoadImageButton.Visible = 1;
                app.ChooseWebcamDropDown.Visible = 0;
                app.ChooseWebcamDropDownLabel.Visible = 0;
                app.BoundingBoxesSwitch.Value = 'Automatic';
                app.BoundingBoxesSwitch.Visible = 0;
                imshow(1, 'Parent', app.ImagePreview);
                app.cam = [];
            else
                app.TakeSnapshotButton.Visible = 1;%0 makes it invisible, vice versa
                app.LoadImageButton.Visible = 0;
                app.ChooseWebcamDropDown.Visible = 1;
                app.ChooseWebcamDropDownLabel.Visible = 1;
                app.CropImageButton.Visible = 0;
                app.CalculateButton.Visible = 0;
                app.RotateImageSlider.Visible = 0;
                app.ClearButton.Visible = 0;
                app.AddIndentButton.Visible = 0;
                app.IndentTypeSwitch.Visible = 0;
                app.IndentTypeSwitchLabel.Visible = 0;
                app.BoundingBoxesSwitch.Visible = 0;
                app.BoundingBoxesSwitchLabel.Visible = 0;
                app.ChooseWebcamDropDown.Items = webcamlist;
                app.ChooseWebcamDropDown.Items{end + 1} = 'None';
                app.ChooseWebcamDropDown.Value = 'None';
                imshow(1, 'Parent', app.ImagePreview);
            end
        end

        % Value changed function: ChooseWebcamDropDown
        function ChooseWebcamDropDownValueChanged(app, event)
            value = app.ChooseWebcamDropDown.Value;
            if ~strcmp(value, 'None')
                app.TakeSnapshotButton.Visible = 1;%0 makes it invisible, vice versa
                app.cam = webcam(value);
                frame = snapshot(app.cam);
                im = image(app.ImagePreview, zeros(size(frame), 'uint8'));
                axis(app.ImagePreview,'image');
                preview(app.cam, im);
                pause;
            else
                app.TakeSnapshotButton.Visible = 0;%0 makes it invisible, vice versa
                app.cam = [];
            end
        end

        % Button pushed function: TakeSnapshotButton
        function TakeSnapshotButtonPushed(app, event)
            app.TakeSnapshotButton.Visible = 0;
            closePreview(app.cam)
            app.originalImage = snapshot(app.cam);
            app.cam = [];
            imshow(app.originalImage, 'Parent', app.ImagePreview);
            app.RetryButton.Visible = 1;%1 makes it invisible, vice versa
            showUI(app);
            app.rotatingImage = app.originalImage;
        end

        % Button pushed function: RetryButton
        function RetryButtonPushed(app, event)
            app.boundingBoxes = {};
            app.RetryButton.Visible = 0;
            app.TakeSnapshotButton.Visible = 1;
            app.cam = webcam(app.ChooseWebcamDropDown.Value);
            frame = snapshot(app.cam);
            im = image(app.ImagePreview, zeros(size(frame), 'uint8'));
            axis(app.ImagePreview,'image');
            preview(app.cam, im);
            pause;
        end

        % Button pushed function: ExportButton
        function ExportButtonPushed(app, event)
            functionname='indentIdentifierGUI.mlapp'; functiondir=which(functionname);
            functiondir=functiondir(1:end-length(functionname));
            addpath([functiondir 'Data']);
            if ~contains(cd, 'Data') 
                cd 'Data';
            end
            x = 0;
            counter = 1;
            while x == 0
                currentFile = "IndentImage" + string(counter) + ".jpg";
                fstruct = dir(currentFile);
                if ~isempty(fstruct)%specific .csv file not found
                    counter = counter + 1;
                else
                    x = 1;
                end
            end
            imwrite(app.newImage, currentFile);
            csvFile = "IndentData.csv";
            fstruct = dir(csvFile);
            if isempty(fstruct)%specific .csv file not found
                mat = ["Original Image", "Indent Image", "Indent Type", "Skewed?", "HK (area)", "HK (diagonal)", "HV"];
                writematrix(mat, 'IndentData.csv', 'Delimiter', ',');
            end
            for i = 1:1:length(app.boundingBoxes)
                currentFile1 = "IndentImage" + string(counter) + "-" + string(i) + ".jpg";
                ogImage = currentFile;
                indentImage = currentFile1;
                indentType = string(app.IndentTypeSwitch.Value);
                if strcmp(app.IndentTypeSwitch.Value, "Knoop")
                    skewed = app.skewedList(i);
                    HKa = app.hardnessConstantsArea(i);
                    HKd = app.hardnessConstantsDiagonal(i);
                    HV = "N/A";
                else
                    skewed = "N/A";
                    HKa = "N/A";
                    HKd = "N/A";
                    HV = app.hardnessConstantsArea(i);
                end
                imwrite(app.indentImages{i}, currentFile1);
                M2(i,:) = [ogImage, indentImage, indentType, skewed, HKa, HKd, HV];
            end
            writematrix(M2,'IndentData.csv','WriteMode','append')
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 620 417];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [1 1 619 417];

            % Create SetupTab
            app.SetupTab = uitab(app.TabGroup);
            app.SetupTab.Title = 'Setup';

            % Create ImagePreview
            app.ImagePreview = uiaxes(app.SetupTab);
            app.ImagePreview.XColor = 'none';
            app.ImagePreview.XTick = [];
            app.ImagePreview.YColor = 'none';
            app.ImagePreview.YTick = [];
            app.ImagePreview.Tag = 'UIAxes_2';
            app.ImagePreview.Visible = 'off';
            app.ImagePreview.Position = [16 62 386 265];

            % Create LoadImageButton
            app.LoadImageButton = uibutton(app.SetupTab, 'push');
            app.LoadImageButton.ButtonPushedFcn = createCallbackFcn(app, @LoadImageButtonPushed, true);
            app.LoadImageButton.Position = [281 342 120 30];
            app.LoadImageButton.Text = 'Load Image';

            % Create BoundingBoxesSwitchLabel
            app.BoundingBoxesSwitchLabel = uilabel(app.SetupTab);
            app.BoundingBoxesSwitchLabel.HorizontalAlignment = 'center';
            app.BoundingBoxesSwitchLabel.FontWeight = 'bold';
            app.BoundingBoxesSwitchLabel.Visible = 'off';
            app.BoundingBoxesSwitchLabel.Position = [464 278 100 22];
            app.BoundingBoxesSwitchLabel.Text = 'Bounding Boxes';

            % Create BoundingBoxesSwitch
            app.BoundingBoxesSwitch = uiswitch(app.SetupTab, 'slider');
            app.BoundingBoxesSwitch.Items = {'Automatic', 'Manual'};
            app.BoundingBoxesSwitch.ValueChangedFcn = createCallbackFcn(app, @BoundingBoxesSwitchValueChanged, true);
            app.BoundingBoxesSwitch.Visible = 'off';
            app.BoundingBoxesSwitch.Position = [491 254 45 20];
            app.BoundingBoxesSwitch.Value = 'Automatic';

            % Create AddIndentButton
            app.AddIndentButton = uibutton(app.SetupTab, 'push');
            app.AddIndentButton.ButtonPushedFcn = createCallbackFcn(app, @AddIndentButtonPushed, true);
            app.AddIndentButton.Visible = 'off';
            app.AddIndentButton.Position = [26 16 111 32];
            app.AddIndentButton.Text = 'Add Indent';

            % Create MicronPixelRatioEditFieldLabel
            app.MicronPixelRatioEditFieldLabel = uilabel(app.SetupTab);
            app.MicronPixelRatioEditFieldLabel.HorizontalAlignment = 'right';
            app.MicronPixelRatioEditFieldLabel.Position = [417 209 102 22];
            app.MicronPixelRatioEditFieldLabel.Text = 'Micron/Pixel Ratio';

            % Create MicronPixelRatioEditField
            app.MicronPixelRatioEditField = uieditfield(app.SetupTab, 'numeric');
            app.MicronPixelRatioEditField.Position = [534 209 60 22];
            app.MicronPixelRatioEditField.Value = 0.396349331382252;

            % Create MassgramsEditFieldLabel
            app.MassgramsEditFieldLabel = uilabel(app.SetupTab);
            app.MassgramsEditFieldLabel.HorizontalAlignment = 'right';
            app.MassgramsEditFieldLabel.Position = [429 170 78 22];
            app.MassgramsEditFieldLabel.Text = 'Mass (grams)';

            % Create MassgramsEditField
            app.MassgramsEditField = uieditfield(app.SetupTab, 'numeric');
            app.MassgramsEditField.Position = [522 170 60 22];
            app.MassgramsEditField.Value = 300;

            % Create CalculateButton
            app.CalculateButton = uibutton(app.SetupTab, 'push');
            app.CalculateButton.ButtonPushedFcn = createCallbackFcn(app, @CalculateButtonPushed, true);
            app.CalculateButton.Visible = 'off';
            app.CalculateButton.Position = [408 79 196 43];
            app.CalculateButton.Text = 'Calculate';

            % Create CropImageButton
            app.CropImageButton = uibutton(app.SetupTab, 'push');
            app.CropImageButton.ButtonPushedFcn = createCallbackFcn(app, @CropImageButtonPushed, true);
            app.CropImageButton.Visible = 'off';
            app.CropImageButton.Position = [284 16 102 32];
            app.CropImageButton.Text = 'Crop Image';

            % Create IndentTypeSwitchLabel
            app.IndentTypeSwitchLabel = uilabel(app.SetupTab);
            app.IndentTypeSwitchLabel.HorizontalAlignment = 'center';
            app.IndentTypeSwitchLabel.FontWeight = 'bold';
            app.IndentTypeSwitchLabel.Visible = 'off';
            app.IndentTypeSwitchLabel.Position = [476 340 71 22];
            app.IndentTypeSwitchLabel.Text = 'Indent Type';

            % Create IndentTypeSwitch
            app.IndentTypeSwitch = uiswitch(app.SetupTab, 'slider');
            app.IndentTypeSwitch.Items = {'Knoop', 'Vickers'};
            app.IndentTypeSwitch.ValueChangedFcn = createCallbackFcn(app, @IndentTypeSwitchValueChanged, true);
            app.IndentTypeSwitch.Visible = 'off';
            app.IndentTypeSwitch.Position = [488 316 45 20];
            app.IndentTypeSwitch.Value = 'Knoop';

            % Create RotateImageSliderLabel
            app.RotateImageSliderLabel = uilabel(app.SetupTab);
            app.RotateImageSliderLabel.HorizontalAlignment = 'right';
            app.RotateImageSliderLabel.WordWrap = 'on';
            app.RotateImageSliderLabel.Visible = 'off';
            app.RotateImageSliderLabel.Position = [385 14 46 34];
            app.RotateImageSliderLabel.Text = 'Rotate Image';

            % Create RotateImageSlider
            app.RotateImageSlider = uislider(app.SetupTab);
            app.RotateImageSlider.Limits = [0 180];
            app.RotateImageSlider.MajorTicks = [0 30 60 90 120 150 180];
            app.RotateImageSlider.MajorTickLabels = {'0', '30', '60', '90', '120', '150', '180'};
            app.RotateImageSlider.ValueChangedFcn = createCallbackFcn(app, @RotateImageSliderValueChanged, true);
            app.RotateImageSlider.Visible = 'off';
            app.RotateImageSlider.Position = [452 35 135 3];

            % Create ClearButton
            app.ClearButton = uibutton(app.SetupTab, 'push');
            app.ClearButton.ButtonPushedFcn = createCallbackFcn(app, @ClearButtonPushed, true);
            app.ClearButton.Visible = 'off';
            app.ClearButton.Position = [162 16 94 32];
            app.ClearButton.Text = 'Clear';

            % Create InputTypeSwitchLabel
            app.InputTypeSwitchLabel = uilabel(app.SetupTab);
            app.InputTypeSwitchLabel.HorizontalAlignment = 'center';
            app.InputTypeSwitchLabel.FontWeight = 'bold';
            app.InputTypeSwitchLabel.Position = [41 364 65 22];
            app.InputTypeSwitchLabel.Text = 'Input Type';

            % Create InputTypeSwitch
            app.InputTypeSwitch = uiswitch(app.SetupTab, 'slider');
            app.InputTypeSwitch.Items = {'File', 'Webcam'};
            app.InputTypeSwitch.ValueChangedFcn = createCallbackFcn(app, @InputTypeSwitchValueChanged, true);
            app.InputTypeSwitch.Position = [51 342 45 20];
            app.InputTypeSwitch.Value = 'File';

            % Create ChooseWebcamDropDownLabel
            app.ChooseWebcamDropDownLabel = uilabel(app.SetupTab);
            app.ChooseWebcamDropDownLabel.HorizontalAlignment = 'right';
            app.ChooseWebcamDropDownLabel.FontWeight = 'bold';
            app.ChooseWebcamDropDownLabel.Visible = 'off';
            app.ChooseWebcamDropDownLabel.Position = [157 364 101 22];
            app.ChooseWebcamDropDownLabel.Text = 'Choose Webcam';

            % Create ChooseWebcamDropDown
            app.ChooseWebcamDropDown = uidropdown(app.SetupTab);
            app.ChooseWebcamDropDown.Items = {'Webcam'};
            app.ChooseWebcamDropDown.ValueChangedFcn = createCallbackFcn(app, @ChooseWebcamDropDownValueChanged, true);
            app.ChooseWebcamDropDown.Visible = 'off';
            app.ChooseWebcamDropDown.Position = [161 342 100 22];
            app.ChooseWebcamDropDown.Value = 'Webcam';

            % Create TakeSnapshotButton
            app.TakeSnapshotButton = uibutton(app.SetupTab, 'push');
            app.TakeSnapshotButton.ButtonPushedFcn = createCallbackFcn(app, @TakeSnapshotButtonPushed, true);
            app.TakeSnapshotButton.Visible = 'off';
            app.TakeSnapshotButton.Position = [283 342 120 30];
            app.TakeSnapshotButton.Text = 'Take Snapshot';

            % Create RetryButton
            app.RetryButton = uibutton(app.SetupTab, 'push');
            app.RetryButton.ButtonPushedFcn = createCallbackFcn(app, @RetryButtonPushed, true);
            app.RetryButton.Visible = 'off';
            app.RetryButton.Position = [282 342 120 30];
            app.RetryButton.Text = 'Retry';

            % Create ResultsTab
            app.ResultsTab = uitab(app.TabGroup);
            app.ResultsTab.Title = 'Results';
            app.ResultsTab.HandleVisibility = 'off';

            % Create IndentImage
            app.IndentImage = uiaxes(app.ResultsTab);
            app.IndentImage.XColor = 'none';
            app.IndentImage.XTick = [];
            app.IndentImage.YColor = 'none';
            app.IndentImage.YTick = [];
            app.IndentImage.Tag = 'UIAxes_1';
            app.IndentImage.Position = [270 81 347 254];

            % Create ImagePreview_2
            app.ImagePreview_2 = uiaxes(app.ResultsTab);
            app.ImagePreview_2.XColor = 'none';
            app.ImagePreview_2.XTick = [];
            app.ImagePreview_2.YColor = 'none';
            app.ImagePreview_2.YTick = [];
            app.ImagePreview_2.Tag = 'UIAxes_2';
            app.ImagePreview_2.Position = [26 114 222 268];

            % Create CycleBackward
            app.CycleBackward = uibutton(app.ResultsTab, 'push');
            app.CycleBackward.ButtonPushedFcn = createCallbackFcn(app, @CycleBackwardButtonPushed, true);
            app.CycleBackward.FontSize = 24;
            app.CycleBackward.FontWeight = 'bold';
            app.CycleBackward.Position = [279 348 139 38];
            app.CycleBackward.Text = '<';

            % Create CycleForward
            app.CycleForward = uibutton(app.ResultsTab, 'push');
            app.CycleForward.ButtonPushedFcn = createCallbackFcn(app, @CycleForwardButtonPushed, true);
            app.CycleForward.FontSize = 24;
            app.CycleForward.FontWeight = 'bold';
            app.CycleForward.Position = [477 348 140 38];
            app.CycleForward.Text = '>';

            % Create HKareaLabel
            app.HKareaLabel = uilabel(app.ResultsTab);
            app.HKareaLabel.FontSize = 18;
            app.HKareaLabel.Position = [8 35 132 43];
            app.HKareaLabel.Text = 'HK (area)= ';

            % Create HKdiagonalLabel
            app.HKdiagonalLabel = uilabel(app.ResultsTab);
            app.HKdiagonalLabel.FontSize = 18;
            app.HKdiagonalLabel.Position = [8 9 182 43];
            app.HKdiagonalLabel.Text = 'HK (diagonal)= ';

            % Create HKValueArea
            app.HKValueArea = uilabel(app.ResultsTab);
            app.HKValueArea.FontSize = 18;
            app.HKValueArea.Position = [187 34 120 44];
            app.HKValueArea.Text = '_____';

            % Create HKValueDiagonal
            app.HKValueDiagonal = uilabel(app.ResultsTab);
            app.HKValueDiagonal.FontSize = 18;
            app.HKValueDiagonal.Position = [187 9 120 44];
            app.HKValueDiagonal.Text = '_____';

            % Create RecalculateButton
            app.RecalculateButton = uibutton(app.ResultsTab, 'push');
            app.RecalculateButton.ButtonPushedFcn = createCallbackFcn(app, @RecalculateButtonPushed, true);
            app.RecalculateButton.FontSize = 18;
            app.RecalculateButton.Position = [269 21 131 52];
            app.RecalculateButton.Text = 'Recalculate';

            % Create ConfirmButton
            app.ConfirmButton = uibutton(app.ResultsTab, 'push');
            app.ConfirmButton.ButtonPushedFcn = createCallbackFcn(app, @ConfirmButtonPushed, true);
            app.ConfirmButton.FontSize = 18;
            app.ConfirmButton.Visible = 'off';
            app.ConfirmButton.Position = [482 21 131 52];
            app.ConfirmButton.Text = 'Confirm';

            % Create skewLabel
            app.skewLabel = uilabel(app.ResultsTab);
            app.skewLabel.HorizontalAlignment = 'center';
            app.skewLabel.WordWrap = 'on';
            app.skewLabel.FontSize = 14;
            app.skewLabel.Visible = 'off';
            app.skewLabel.Position = [8 72 272 43];
            app.skewLabel.Text = 'Placeholder Text';

            % Create ExportButton
            app.ExportButton = uibutton(app.ResultsTab, 'push');
            app.ExportButton.ButtonPushedFcn = createCallbackFcn(app, @ExportButtonPushed, true);
            app.ExportButton.FontSize = 18;
            app.ExportButton.Visible = 'off';
            app.ExportButton.Position = [482 21 131 52];
            app.ExportButton.Text = 'Export';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = IndentIdentifier

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end
function varargout = StereoDist(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @StereoDist_OpeningFcn, ...
    'gui_OutputFcn',  @StereoDist_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end


end


function StereoDist_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;


guidata(hObject, handles);



global fl cameraAngle camerasDistance  vidDeviceL vidDeviceR
clc

str = get(handles.editCamDist,'String');
camerasDistance = str2double(str);

str = get(handles.editCamAngle,'String');
cameraAngle = str2double(str)*pi/180;

% Инициализация
redThresh = 0.25; 
greenThresh = 0.05; 
blueThresh = 0.15; 



vidDeviceL = imaq.VideoDevice('winvideo', 1, 'YUY2_320x240', 'ReturnedColorSpace', 'rgb');
vidDeviceR = imaq.VideoDevice('winvideo', 2, 'YUY2_320x240', 'ReturnedColorSpace', 'rgb');


vidInfo = imaqhwinfo(vidDeviceL); 
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true', ...
    'MinimumBlobArea', 400, ...          
    'MaximumCount', 50);

hshapeinsBox = vision.ShapeInserter('BorderColorSource', 'Input port', ... 
                                        'Fill', true, ...
                                        'FillColorSource', 'Input port', ...
                                        'Opacity', 0.4);

hshapeinsRedBox = vision.TextInserter('Text', 'Red   : %2d', ... 
                                    'Location',  [5 2], ...
                                    'Color', [1 0 0], ... 
                                    'Font', 'Courier New', ...
                                    'FontSize', 14);

hshapeinsBlueBox = vision.TextInserter('Text', 'Blue  : %2d', ... %
                                    'Location',  [5 34], ...
                                    'Color', [0 0 1], ... 
                                    'Font', 'Courier New', ...
                                    'FontSize', 14);

hshapeinsGreenBox = vision.TextInserter('Text', 'Green : %2d', ... 
                                    'Location',  [5 18], ...
                                    'Color', [0 1 0], ... 
                                    'Font', 'Courier New', ...
                                    'FontSize', 14);



hshapeinsLine = vision.ShapeInserter('Shape','Lines', ...
    'BorderColor', 'Custom', 'CustomBorderColor', [0 255 0]);
htextins = vision.TextInserter('Text', 'Distanse is: %s mm', ... 
    'Location',  [7 2], ...
    'Color', [1 1 1], ... 
    'Font', 'Courier New', ...
    'FontSize', 12);
htextinsCent = vision.TextInserter('Text', '+', ...
    'LocationSource', 'Input port', ...
    'Color', [0 0 0], ... 
    'FontSize', 12);

%%


nFrame = 0;
while (nFrame < 3000)
    
    rgbFrameL = step(vidDeviceL); 
    rgbFrameR = step(vidDeviceR);
    
   rgbFrameL = flipdim(rgbFrameL,2);
   rgbFrameR = flipdim(rgbFrameR,2);
    
    
    imSizeX = size(rgbFrameL,2);
    %%Red
    diffFrameL = imsubtract(rgbFrameL(:,:,1), rgb2gray(rgbFrameL)); 
    diffFrameR = imsubtract(rgbFrameR(:,:,1), rgb2gray(rgbFrameR)); 
    
    diffFrameL = medfilt2(diffFrameL, [3 3]); 
    diffFrameR = medfilt2(diffFrameR, [3 3]); 
    
    binFrameL = im2bw(diffFrameL, redThresh); 
    binFrameR = im2bw(diffFrameR, redThresh); 
    
    %%Green
    diffFrameGreenL = imsubtract(rgbFrameL(:,:,2), rgb2gray(rgbFrameL)); 
    diffFrameGreenR = imsubtract(rgbFrameR(:,:,2), rgb2gray(rgbFrameR));
    
    
    diffFrameGreenL = medfilt2(diffFrameGreenL, [3 3]); 
    diffFrameGreenR = medfilt2(diffFrameGreenR, [3 3]);
    
    binFrameGreenL = im2bw(diffFrameGreenL, greenThresh); 
    binFrameGreenR = im2bw(diffFrameGreenR, greenThresh);
    
    %%Blue
    
    diffFrameBlueL = imsubtract(rgbFrameL(:,:,3), rgb2gray(rgbFrameL)); 
    diffFrameBlueR = imsubtract(rgbFrameR(:,:,3), rgb2gray(rgbFrameR));
    
    diffFrameBlueL = medfilt2(diffFrameBlueL, [3 3]); 
    diffFrameBlueR = medfilt2(diffFrameBlueR, [3 3]);
    
    
    binFrameBlueL = im2bw(diffFrameBlueL, blueThresh); 
    binFrameBlueR = im2bw(diffFrameBlueR, blueThresh);

    [centroidL, bboxL] = step(hblob, binFrameL); 
    [centroidR, bboxR] = step(hblob, binFrameR); 
    centroidL = uint16(centroidL); 
    centroidR = uint16(centroidR);
    
    [centroidGreenL, bboxGreenL] = step(hblob, binFrameGreenL);
    [centroidGreenR, bboxGreenR] = step(hblob, binFrameGreenR);
    centroidGreenL = uint16(centroidGreenL);
    centroidGreenR = uint16(centroidGreenR);
    
    [centroidBlueL, bboxBlueL] = step(hblob, binFrameBlueL);
    [centroidBlueR, bboxBlueR] = step(hblob, binFrameBlueR);
    centroidBlueL = uint16(centroidBlueL);
    centroidBlueR = uint16(centroidBlueR);
    
    
    rgbFrameL(1:20,1:200,:) = 0; 
    rgbFrameR(1:20,1:200,:) = 0;
    
    %%
    vidInL = step(hshapeinsBox, rgbFrameL, bboxL,single([1 0 0]));
    vidInR = step(hshapeinsBox, rgbFrameR, bboxR, single([1 0 0])); 
    
    vidInL = step(hshapeinsBox, vidInL, bboxGreenL, single([0 1 0])); 
    vidInR = step(hshapeinsBox, vidInR, bboxGreenR, single([0 1 0]));
    
    vidInL = step(hshapeinsBox, vidInL, bboxBlueL, single([0 0 1])); 
    vidInR = step(hshapeinsBox, vidInR, bboxBlueR, single([0 0 1])); 
  
    %%Работа с  центроиды
    for k = 1:length(bboxL(:,1)) 
        centRedX = centroidL(k,1);
        centRedY = centroidL(k,2);
        vidInL = step(htextinsCent, vidInL, [centRedX centRedY]);
    end
    
    for k = 1:length(bboxR(:,1))
        centRedX = centroidR(k,1);
        centRedY = centroidR(k,2);
        vidInR = step(htextinsCent, vidInR, [centRedX centRedY]);
    end
    
    %%Green
      for k = 1:length(bboxGreenL(:,1)) 
        centGreenX = centroidGreenL(k,1);
        centGreenY = centroidGreenL(k,2);
        vidInL = step(htextinsCent, vidInL, [centGreenX centGreenY]);
    end
    
    for k = 1:length(bboxGreenR(:,1)) 
        centGreenX = centroidGreenR(k,1);
        centGreenY = centroidGreenR(k,2);
        vidInR = step(htextinsCent, vidInR, [centGreenX centGreenY]);
    end
    
    %%Blue
      for k = 1:length(bboxBlueL(:,1)) 
        centBlueX = centroidBlueL(k,1);
        centBlueY = centroidBlueL(k,2);
        vidInL = step(htextinsCent, vidInL, [centBlueX centBlueY]);
    end
    
    for k = 1:length(bboxBlueR(:,1)) 
        centBlueX = centroidBlueR(k,1);
        centBlueY = centroidBlueR(k,2);
        vidInR = step(htextinsCent, vidInR, [centBlueX centBlueY]);
    end

    stereo = imfuse(vidInL, vidInR,'falsecolor','Scaling','joint','ColorChannels', [1 0 2]); % imfuse- слияние двух изобр в одно
    m = min(size(centroidL,1),size(centroidR,1));
    if m>0
        for k = 1:m
            stereo = step(hshapeinsLine,stereo,int32([centroidL(k,:) centroidR(k,:)]));
            
        end
    end
    
    %%Работа с данными: дистанция ,угол
    pixelDistance = NaN;
    if size(centroidR,1)==size(centroidL,1)
        pixelDistance = abs(mean(centroidR(:,1)-centroidL(:,1)));
    elseif ~isempty(centroidR)&~isempty(centroidL)
        pixelDistance = abs(mean(centroidR(1,1)-centroidL(1,1)));
    end
  % подсчет расстояния до объекта
    estimatedDistance = camerasDistance*imSizeX/(2*tan(cameraAngle/2)*pixelDistance);
    
% Подсчет погрешности
    errorDistance = estimatedDistance.^2 * tan(cameraAngle/imSizeX)/camerasDistance;
    
    str = strcat(num2str(estimatedDistance,3),'±',num2str(errorDistance,3));
    vidInL = step(htextins, vidInL, uint8(str)); % Подсчитайте количество больших двоичных объектов
    vidInR = step(htextins, vidInR, uint8(str)); % Count the number of blobs
    set(handles.text8,'String',str)
    
    %% Вывод изображений видео на экран.
    axes(handles.axesL)
    image(vidInL); axis off
    
    axes(handles.axesR);
    image(vidInR); axis off
    
    axes(handles.axesC);
    image(stereo); axis off
    
    nFrame = nFrame+1;
end


function varargout = StereoDist_OutputFcn(hObject, eventdata, handles)

end


function pushbutton1_Callback(hObject, eventdata, handles)
global vidDeviceL vidDeviceR
end


function editCamDist_Callback(hObject, eventdata, handles)
global camerasDistance
str = get(handles.editCamDist,'String');
camerasDistance = str2double(str);
end


function editCamDist_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function editCamAngle_Callback(hObject, eventdata, handles)
global cameraAngle
str = get(handles.editCamAngle,'String');
cameraAngle = str2double(str)*pi/180;
end

function editCamAngle_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

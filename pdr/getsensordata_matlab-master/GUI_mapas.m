function varargout = GUI_mapas(varargin)
% GUI_MAPAS M-file for GUI_mapas.fig
%      GUI_MAPAS, by itself, creates a new GUI_MAPAS or raises the existing
%      singleton*.
%
%      H = GUI_MAPAS returns the handle to a new GUI_MAPAS or the handle to
%      the existing singleton*.
%
%      GUI_MAPAS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_MAPAS.M with the given input arguments.
%
%      GUI_MAPAS('Property','Value',...) creates a new GUI_MAPAS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_mapas_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_mapas_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_mapas

% Last Modified by GUIDE v2.5 12-Apr-2016 15:52:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_mapas_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_mapas_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before GUI_mapas is made visible.
function GUI_mapas_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_mapas (see VARARGIN)

% Choose default command line output for GUI_mapas
handles.output = hObject;

% inicializacion
clc
handles.Sensibilidad=1;
handles.h_CAD=NaN;   % Crear imagen inicial
handles.CADfilename='';
%Calrgar matriz calibrado edificio completo (m plantas) y cada planta tiene: Filename, X, Y, Rot y Scale
handles.Calib=[];
handles.Planta_idx=NaN;  % indice para saber cual es la planta activa
handles.radio_Tierra=6371000; % metros
handles.ratio_Lat_metro=atan(1/handles.radio_Tierra)*180/pi;  % grados Lat por cada metro en la superficie terrestre
handles.ratio_Lon_metro=atan(1/handles.radio_Tierra)*(180/pi)/cos(40.5*pi/180);  % grados Lon por cada metro en la superficie terrestre
handles.rutarelativa_mapas='.\mapas\';
handles.traject=[];  % Array nx5 con los puntos de la trajectoria a recorrer
handles.trajectory_lastpoint=0;
handles.h_impoly=[];
handles.datacursormode = datacursormode(hObject);
set(handles.datacursormode,'Updatefcn',@mi_funcion_customizar_datacursor);
handles.datos_LogFile_new_deleted=[];
handles.datos_LogFile_new=[];
% visibilidad por defecto:
set(handles.pushbutton_Draw_NewSection,'Visible','off');
set(handles.text22,'Visible','off');
set(handles.edit_numPointRoute,'Visible','off');
set(handles.pushbutton_SaveTraject,'Visible','off');
set(handles.pushbutton_AssignGroundTruth,'Enable','off');
handles.string_fichero_LogFile_actual='No LogFile Opened';
handles.contador_capturas=0;



handles.delta_desplaza=0.00004;

% Cargar mapa inicial
%plot(-3.35,40.513,'.r','MarkerSize',20);  % marcar un punto cualquiera
% marcar ptos ref del CNR Pisa:
plot(10.4224303815462,43.7191733350706,'.r','MarkerSize',20); hold on;
plot(10.4227492989194,43.7187899662822,'.r','MarkerSize',20); 
plot(10.4217191636067,43.7191058012223,'.r','MarkerSize',20);
plot(10.4215250945291,43.7181254344932,'.r','MarkerSize',20);
plot(10.4215957574101,43.7180690908951,'.r','MarkerSize',20); hold off;

% cargar mapa de google:
plot_google_map;  % pintar el mapa de googlemaps de fondo
hold on;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI_mapas wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_mapas_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function [handles]=pinta_imagen_superpuesta(handles);
%if ishandle(handles.h_CAD)  % borrar objeto imagen si habia otra antes
%    delete(handles.h_CAD);
%end
handles.CAD_original=imread([char(handles.rutarelativa_mapas) char(handles.Calib(handles.Planta_idx).Filename)]);  % cargar la matriz del bitmap
if size(handles.CAD_original,3)==3  % si el jpg es RGB
    handles.CAD_original(:,:,1)=flipud(handles.CAD_original(:,:,1)); % flipud RGB
    handles.CAD_original(:,:,2)=flipud(handles.CAD_original(:,:,2));
    handles.CAD_original(:,:,3)=flipud(handles.CAD_original(:,:,3));
else  % si el jpg no es RGB, lo pongo con sus 3 componentes
    handles.CAD_original(:,:,1)=flipud(handles.CAD_original(:,:)); % flipud
    handles.CAD_original(:,:,2)=handles.CAD_original(:,:,1);
    handles.CAD_original(:,:,3)=handles.CAD_original(:,:,1);
end
index_ceros=find(handles.CAD_original(:,:,:)==0); % lo que vale negro (cero)
handles.CAD_original(index_ceros)=1;  % lo pongo un poco mas claro (para que no haya ceros y se pueda distinguir de lo rotado que imrotate me lo pone a cero)
handles.CAD = imrotate(handles.CAD_original, handles.Calib(handles.Planta_idx).Rot, 'loose', 'bilinear');  % rotarla lo deseado (ampliado puesto a cero-negro)
%handles.CAD = handles.CAD_original;
%keyboard,
handles.mascara_rotado=find(handles.CAD(:,:,1)==0); % lo que vale negro (cero) que es lo rotado
handles.mascara_original=find(handles.CAD(:,:,1)>0); % lo que no vale negro, que es la imagen CAD 
handles.Alpha=ones(size(handles.CAD,1),size(handles.CAD,2),1);
handles.Alpha(handles.mascara_original)=handles.Alpha(handles.mascara_original)*get(handles.slider_trasparencia,'Value'); % la parte de la imagen CAD que se vea con la transparencia deseada
handles.Alpha(handles.mascara_rotado)=handles.Alpha(handles.mascara_rotado)*0; % para que no se vea lo rotado (transparencia pura)
handles.pixels_X_CAD=size(handles.CAD,2);   handles.pixels_Y_CAD=size(handles.CAD,1);  % ver su nuevo tamaño
if ishandle(handles.h_CAD)
    set(handles.h_CAD,'CDAta',handles.CAD);
    set(handles.h_CAD,'XData',[handles.Calib(handles.Planta_idx).Lon-0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_X_CAD*handles.ratio_Lon_metro  ...
        handles.Calib(handles.Planta_idx).Lon+0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_X_CAD*handles.ratio_Lon_metro]);
    set(handles.h_CAD,'YData', [handles.Calib(handles.Planta_idx).Lat-0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_Y_CAD*handles.ratio_Lat_metro  ...
        handles.Calib(handles.Planta_idx).Lat+0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_Y_CAD*handles.ratio_Lat_metro]   );
else
    handles.h_CAD=image([handles.Calib(handles.Planta_idx).Lon-0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_X_CAD*handles.ratio_Lon_metro  ...
                         handles.Calib(handles.Planta_idx).Lon+0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_X_CAD*handles.ratio_Lon_metro],...
                         [handles.Calib(handles.Planta_idx).Lat-0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_Y_CAD*handles.ratio_Lat_metro  ...
                         handles.Calib(handles.Planta_idx).Lat+0.5*handles.Calib(handles.Planta_idx).Scale*handles.pixels_Y_CAD*handles.ratio_Lat_metro],...
                         handles.CAD);  % crear el objeto imagen
end
%set(handles.h_CAD, 'AlphaData', get(handles.slider_trasparencia,'Value'));  % poner al objeto imagen la propiedad de transparencia
set(handles.h_CAD, 'AlphaData', handles.Alpha); % transparencia matricial
disp(['Pintada Imagen: handles.pixels_X_CAD: ',num2str(handles.pixels_X_CAD), '  handles.pixels_Y_CAD: ',num2str(handles.pixels_Y_CAD)]);



% --- Executes on button press in pushbuttonF3.
function pushbuttonF3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonF3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%keyboard;
handles.Planta_idx=4;
[handles]=pinta_imagen_superpuesta(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbuttonF2.
function pushbuttonF2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonF2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Planta_idx=3;
[handles]=pinta_imagen_superpuesta(handles);
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbuttonF1.
function pushbuttonF1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonF1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Planta_idx=2;
[handles]=pinta_imagen_superpuesta(handles);
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbuttonF0.
function pushbuttonF0_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonF0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Planta_idx=1;
[handles]=pinta_imagen_superpuesta(handles);
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_Calib_Up.
function pushbutton_Calib_Up_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Calib_Up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Calib(handles.Planta_idx).Lat=handles.Calib(handles.Planta_idx).Lat+handles.delta_desplaza*handles.Sensibilidad;
[handles]=pinta_imagen_superpuesta(handles);
set(handles.text_Latitude,'String',num2str(handles.Calib(handles.Planta_idx).Lat,11));
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_Calib_Left.
function pushbutton_Calib_Left_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Calib_Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Calib(handles.Planta_idx).Lon=handles.Calib(handles.Planta_idx).Lon-handles.delta_desplaza*handles.Sensibilidad;
[handles]=pinta_imagen_superpuesta(handles);
set(handles.text_Longitude,'String',num2str(handles.Calib(handles.Planta_idx).Lon,11));
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_Calib_Right.
function pushbutton_Calib_Right_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Calib_Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Calib(handles.Planta_idx).Lon=handles.Calib(handles.Planta_idx).Lon+handles.delta_desplaza*handles.Sensibilidad;
[handles]=pinta_imagen_superpuesta(handles);
set(handles.text_Longitude,'String',num2str(handles.Calib(handles.Planta_idx).Lon,11));
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_Calib_Down.
function pushbutton_Calib_Down_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Calib_Down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Calib(handles.Planta_idx).Lat=handles.Calib(handles.Planta_idx).Lat-handles.delta_desplaza*handles.Sensibilidad;
[handles]=pinta_imagen_superpuesta(handles);
set(handles.text_Latitude,'String',num2str(handles.Calib(handles.Planta_idx).Lat,11));
% Update handles structure
guidata(hObject, handles);

% --- Executes on slider movement.
function slider_Rotate_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.Calib(handles.Planta_idx).Rot=get(hObject,'Value');
[handles]=pinta_imagen_superpuesta(handles);
set(handles.text_Rotation,'String',num2str(handles.Calib(handles.Planta_idx).Rot,11));
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_Rotate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Rotate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
    set(hObject,'Value',(get(hObject,'Max')+get(hObject,'Min'))/2);
end


% --- Executes on button press in pushbutton_SelectImage.
function pushbutton_SelectImage_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_SelectImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile('*.jpg','Select the CAD image to superimpose');
handles.pathname=PathName;
handles.CADfilename=FileName;

%Cargar imagen:
[handles]=pinta_imagen_superpuesta(handles);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_Calibrate.
function pushbutton_Calibrate_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Calibrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isempty(handles.Calib)
    disp('Creando log file...');
%     fecha_hora_str=datestr(now,'dd_mmm_yyyy_HH_MM_SS');
%     if ~exist('mapas','dir')
%         mkdir('mapas');
%     end
%     path=[cd,'\mapas\'];
 
    fid=fopen(handles.string_fichero_calibrado_actual,'w+');
    fprintf(fid,'%% This is a Calib File created with a Matlab GUI ("GUI_mapas")\n');
    fprintf(fid,'%%\n');
    fprintf(fid,'%% This file is used to store the calibration parameters that georeference several floormap images representing \n');
    fprintf(fid,'%% the indoor structure in a building in the WGS-84 datum coordinate reference frame. \n');
    fprintf(fid,'%%\n');
    fprintf(fid,'%% A building is composed of several floors. Each floor has 6 calibration data:\n');
    fprintf(fid,'%% 1) the filename of the bitmap\n');
    fprintf(fid,'%% 2) the floor number (e.g. -2, 0, 3)\n');
    fprintf(fid,'%% 3) the Building number (e.g. 1, 2, 3)\n');
    fprintf(fid,'%% 4) Latidude (in degrees) of image center, \n');
    fprintf(fid,'%% 5) Longitude (in degrees) of image center, \n');
    fprintf(fid,'%% 6) Rotation (in degrees) of image to be aligned to the geometric north\n');
    fprintf(fid,'%% 7) Scale (meters/pixel).\n');
    fprintf(fid,'\n');
    % 3) the Building number (e.g. 1, 2, 3)
% 4) Latidude (in degrees) of image center, 
% 5) Longitude (in degrees) of image center, 
% 6) Rotation (in degrees) of image to be aligned to the geometric north
% 7) Scale (meters/pixel).

    
    
    
    num_images=length(handles.Calib);
    for i=1:num_images
        fprintf(fid,'%s ',char(handles.Calib(i).Filename)); % name of image file
    end
    fprintf(fid,'\n');
    for i=1:num_images
        fprintf(fid,'%.0f ',handles.Calib(i).Planta);   % Planta del edificio (numero entre e.g. -2 y 5)
    end
    fprintf(fid,'\n');
    for i=1:num_images
        fprintf(fid,'%.0f ',handles.Calib(i).Building);   % Building ID (numero entre e.g. 1, 2 y 3)
    end
    fprintf(fid,'\n');
    for i=1:num_images
        fprintf(fid,'%.8f ',handles.Calib(i).Lat);  % Latitude of center of image
    end
    fprintf(fid,'\n');
    for i=1:num_images
        fprintf(fid,'%.8f ',handles.Calib(i).Lon);  % Longitude of center of image
    end
    fprintf(fid,'\n');
    for i=1:num_images
        fprintf(fid,'%.8f ',handles.Calib(i).Rot);     % angle in degrees to North
    end
    fprintf(fid,'\n');
    for i=1:num_images
        fprintf(fid,'%.8f ',handles.Calib(i).Scale);  % Scale of image in meterss/pixel
    end
    fprintf(fid,'\n');
     
    fclose(fid);
    disp(['Calibration File: ',handles.string_fichero_calibrado_actual,' updated']);
end


% --- Executes during object creation, after setting all properties.
function edit_Filename_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_Filename (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on slider movement.
function slider_Scale_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
 
handles.Calib(handles.Planta_idx).Scale=get(hObject,'Value'); % Scale of image in meterss/pixel
[handles]=pinta_imagen_superpuesta(handles);
set(handles.text_Scale,'String',num2str(handles.Calib(handles.Planta_idx).Scale,11));
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_Scale_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Scale (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_trasparencia_Callback(hObject, eventdata, handles)
% hObject    handle to slider_trasparencia (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
valor=get(handles.slider_trasparencia,'Value');
set(handles.edit_trasparencia,'String',num2str(valor));
[handles]=pinta_imagen_superpuesta(handles);

if ishandle(handles.h_CAD)
    handles.Alpha(handles.mascara_original)=handles.Alpha(handles.mascara_original)*valor; % la parte de la imagen CAD que se vea con la transparencia deseada
    handles.Alpha(handles.mascara_rotado)=handles.Alpha(handles.mascara_rotado)*0; % para que no se vea lo rotado (transparencia pura)
    set(handles.h_CAD, 'AlphaData', handles.Alpha);  % poner al objeto imagen la propiedad de transparencia
    %    set(handles.h_CAD, 'AlphaData', valor);  % poner al objeto imagen la propiedad de transparencia
end

% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_trasparencia_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_trasparencia (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
valor_defecto=0.5;
set(hObject,'Value',valor_defecto);




function edit_trasparencia_Callback(hObject, eventdata, handles)
% hObject    handle to edit_trasparencia (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_trasparencia as text
%        str2double(get(hObject,'String')) returns contents of edit_trasparencia as a double
valor=str2num(get(handles.edit_trasparencia,'String'));
set(handles.slider_trasparencia,'Value',valor);
if ishandle(handles.h_CAD)
   set(handles.h_CAD, 'AlphaData', valor);  % poner al objeto imagen la propiedad de transparencia
end
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_trasparencia_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_trasparencia (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
valor_defecto=0.5;
set(hObject,'String',num2str(valor_defecto));


% --- Executes on button press in pushbutton_OpenCalibFile.
function pushbutton_OpenCalibFile_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_OpenCalibFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile('*.cal','Select the Calibration file for your building');
handles.pathname=PathName;
handles.rutarelativa_mapas=PathName;
handles.string_fichero_calibrado_actual=[PathName,FileName];
handles.Calib_data=importdata(handles.string_fichero_calibrado_actual);
num_images=size(handles.Calib_data.data,2);
string_popupmenu=[];
for i=1:num_images
    handles.Calib(i).Filename=handles.Calib_data.colheaders(i);  % name of image file
    handles.Calib(i).Planta=handles.Calib_data.data(1,i);  % Planta del edificio (numero entre e.g. -2 y 5)
    handles.Calib(i).Building=handles.Calib_data.data(2,i);  % Edificio ID (numero entre e.g. 1, 2 y 3)
    handles.Calib(i).Lat=handles.Calib_data.data(3,i);  % Latitude of center of image
    handles.Calib(i).Lon=handles.Calib_data.data(4,i);  % Longitude of center of image
    handles.Calib(i).Rot=handles.Calib_data.data(5,i);     % angle in degrees to North
    handles.Calib(i).Scale=handles.Calib_data.data(6,i);  % Scale of image in meterss/pixel
    string_popupmenu{i}=char(handles.Calib(i).Filename);
end

set(handles.edit_BuildingID,'String',num2str(handles.Calib(1).Building));  % Etiqueta de edificio
set(handles.edit_NumImagesInCalibFile,'String',num2str(num_images));
handles.Planta_idx=1; % selecciono una planta o imagen
handles.ratio_Lon_metro=atan(1/handles.radio_Tierra)*(180/pi)/cos(handles.Calib(1).Lat*pi/180);  % grados Lon por cada metro en la superficie terrestre
set(handles.slider_Rotate,'Value',handles.Calib(handles.Planta_idx).Rot);
set(handles.slider_Rotate,'Min',-180);  set(handles.slider_Rotate,'Max',+180);
set(handles.slider_Scale,'Value',handles.Calib(handles.Planta_idx).Scale);
set(handles.slider_Scale,'Min',0.05);  set(handles.slider_Scale,'Max',0.25);
set(handles.popupmenu_SelectedImage,'String',string_popupmenu);
set(handles.edit_FloorNumber,'String',num2str(handles.Calib(handles.Planta_idx).Planta));
set(handles.text_Longitude,'String',num2str(handles.Calib(handles.Planta_idx).Lon,11));
set(handles.text_Latitude,'String',num2str(handles.Calib(handles.Planta_idx).Lat,11));
set(handles.text_Rotation,'String',num2str(handles.Calib(handles.Planta_idx).Rot,11));
set(handles.text_Scale,'String',num2str(handles.Calib(handles.Planta_idx).Scale,11));
% centrar imagen googlemaps de fondo en zona donde tengo las imagenes de CAD
span=0.002;
set(gca,'XLim',[handles.Calib(handles.Planta_idx).Lon-span handles.Calib(handles.Planta_idx).Lon+span]);
set(gca,'YLim',[handles.Calib(handles.Planta_idx).Lat-span handles.Calib(handles.Planta_idx).Lat+span]);
zoom(1.0);
% poner imagen CAD superpuesta:
[handles]=pinta_imagen_superpuesta(handles);
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_CreateNewCalibFile.
function pushbutton_CreateNewCalibFile_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_CreateNewCalibFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function edit_NumImagesInCalibFile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_NumImagesInCalibFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on selection change in popupmenu_SelectedImage.
function popupmenu_SelectedImage_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_SelectedImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_SelectedImage contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_SelectedImage
handles.Planta_idx=get(hObject,'Value');
set(handles.edit_FloorNumber,'String',num2str(handles.Calib(handles.Planta_idx).Planta));
[handles]=pinta_imagen_superpuesta(handles);

planta_actual=handles.Calib(handles.Planta_idx).Planta;
if ~isempty(handles.traject)
   plantas_poligono=handles.traject(:,4);  % ,planta_actual,plantas_poligono
   mejorar_legibilidad_poligono(handles.h_impoly.getPosition(),planta_actual,plantas_poligono,handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function popupmenu_SelectedImage_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_SelectedImage (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_FloorNumber_Callback(hObject, eventdata, handles)
% hObject    handle to edit_FloorNumber (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_FloorNumber as text
%        str2double(get(hObject,'String')) returns contents of edit_FloorNumber as a double
handles.Calib(handles.Planta_idx).Planta=str2num(get(hObject,'String'));
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function edit_FloorNumber_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_FloorNumber (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_Sensibilidad_Callback(hObject, eventdata, handles)
% hObject    handle to slider_Sensibilidad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.Sensibilidad=get(hObject,'Value');
disp(['Sensibilidad nueva: ',num2str(handles.Sensibilidad)]);
set(handles.textSensibilidad,'String',[num2str(handles.Sensibilidad),' X']);
set(handles.slider_Rotate,'SliderStep',[0.01*handles.Sensibilidad  0.1*handles.Sensibilidad]);
set(handles.slider_Scale,'SliderStep',[0.01*handles.Sensibilidad  0.1*handles.Sensibilidad]);
% Update handles structure
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function slider_Sensibilidad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_Sensibilidad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes when selected object is changed in uipanel4.
function uipanel4_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uipanel4 
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)
Terrestre=get(handles.radiobutton_MapTerrestrial,'Value'),
Satellite=get(handles.radiobutton_MapSatellite,'Value'),
if Terrestre==1
    plot_google_map('MapType','roadmap');  % pintar el mapa de googlemaps de fondo
end
if Satellite==1
    plot_google_map('MapType','satellite');  % pintar el mapa de googlemaps de fondo
end


% --- Executes on button press in pushbutton_Draw_NewSection.
function pushbutton_Draw_NewSection_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Draw_NewSection (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

h_impoly_anterior=handles.h_impoly;
% Solicitar crear un poly nuevo con ratón:
disp('Dibujar nueva seción. START');
handles.h_impoly= impoly(gca,'Closed',false);

% Colocar los puntos de la sección en el array de trayectoria
last_point=handles.trajectory_lastpoint;
LonLat=handles.h_impoly.getPosition(),
planta=str2num(get(handles.edit_FloorNumber,'String'));
if isempty(planta), planta=0; end
building=str2num(get(handles.edit_BuildingID,'String')); 
if isempty(building), building=1; end
num_points_section=size(LonLat,1);
for i=1:num_points_section
   handles.traject(i+last_point,1:5)=[i+last_point LonLat(i,2) LonLat(i,1) planta building]; 
end
handles.traject,
handles.trajectory_lastpoint=last_point+num_points_section;
num_points_route=size(handles.traject,1);
set(handles.edit_numPointRoute,'String',num2str(num_points_route));
set(handles.listbox_Trajectory,'String',num2str(handles.traject));

% Borrar los impoly anteriores (el previo y la nueva seccion)
disp('borro im_poly anteriores');
if ~isempty(h_impoly_anterior), delete(h_impoly_anterior); end  % borro los anteriores
if ~isempty(handles.h_impoly), delete(handles.h_impoly); handles.h_impoly=[]; end  % borro los anteriores
% Crear un poly nuevo con todos los puntos acumulados (previos + nuevos de seccion)
handles.h_impoly=impoly(gca,[handles.traject(:,3) handles.traject(:,2)],'Closed',false);  % pinto el agregado completo
if ~isempty(handles.Calib)
    planta_actual=handles.Calib(handles.Planta_idx).Planta;
else
    planta_actual=0;
end
plantas_poligono=handles.traject(:,4);  % ,planta_actual,plantas_poligono
mejorar_legibilidad_poligono(handles.h_impoly.getPosition(),planta_actual,plantas_poligono,handles);
setColor(handles.h_impoly,'magenta');

addNewPositionCallback(handles.h_impoly,@display_numero_punto);

disp('Dibujar nueva seción. END');
% Update handles structure
guidata(hObject, handles);


function display_numero_punto(hObject)
%disp('Ini: Display_numero_punto');
handles=guidata(gcf);

planta_actual=handles.Calib(handles.Planta_idx).Planta,
plantas_poligono=handles.traject(:,4),  % ,planta_actual,plantas_poligono
p=hObject;  % matrix nx2 con los puntos del poligono
mejorar_legibilidad_poligono(p,planta_actual,plantas_poligono,handles);

% Actualizar datos de lista de puntos (si impoly lo muevo, cambio puntos, etc...)
%planta_actual=str2num(get(handles.edit_FloorNumber,'String')),
%edificio_actual=str2num(get(handles.edit_BuildingID,'String')),

puntos=handles.h_impoly.getPosition();  % coger los puntos trajectoria de planta actual
num_puntos_impoly=size(puntos,1);
num_puntos_traject=size(handles.traject,1);
traject_aux=zeros(num_puntos_impoly,5);
for i=1:num_puntos_impoly
 traject_aux(i,1)=i;
 traject_aux(i,2)=puntos(i,2); % Latitude
 traject_aux(i,3)=puntos(i,1); % Longitude
 traject_aux(i,4)=handles.traject(min(i,num_puntos_traject),4);  % supongo que el punto está en la misma planta que antes estaba un punto numerado igual
 traject_aux(i,5)=handles.traject(min(i,num_puntos_traject),5);  %edificio_actual;
end
% copio a la estructura válida y actualizo el list_box
%traject_aux,
%handles.traject,
handles.traject=traject_aux;  
set(handles.listbox_Trajectory,'String',num2str(handles.traject));

% Update handles structure
guidata(gcf, handles);
%disp('Fin: Display_numero_punto');



function mejorar_legibilidad_poligono(p,planta_actual,plantas_poligono,handles)
% p:  matrix nx2 con los puntos del poligono

 % quitar los numeros de la trajectoria
 text_seco = findall(gca,'type','text');
 delete(text_seco);
    
 disp('Mejorar legibilidad poligono. START.');
 disp(['planta_actual: ',num2str(planta_actual),' plantas_poligono:', num2str(plantas_poligono')]);
 

% % Borrar todos los objetos "text" anteriores
% handle_objetos=get(gca,'Children');
% for indobjeto=1:length(handle_objetos),
%     handle_actual=handle_objetos(indobjeto);
%     % Texto
%     if(findstr(get(handle_actual,'type'),'text')),
%         %set(handle_actual,'FontSize',tam_fuente);
%         delete(handle_actual);
%     end
% end   % cierra el bucle de los objetos

% Pintar nuevos objetos "text"
if ~isempty(p)
    XLim=get(gca,'XLim');   YLim=get(gca,'YLim');
    dX=(XLim(2)-XLim(1))/5000;  % desplazamiento un 2% respecto al tamaño del eje
    dY=(YLim(2)-YLim(1))/5000;  % desplazamiento un 2% respecto al tamaño del eje
    dXY=400*(dX+dY)/2;
    
    % poner nombre de fichero trayectorias, y numero planta:
     FileName=handles.string_FileName_trajectory_actual;
     text(mean(p(:,1)),mean(p(:,2))+1.6*dXY,12,num2str(FileName),'FontSize',8,'BackgroundColor',[1 1 0]); hold on;
     text(mean(p(:,1)),mean(p(:,2))+1.1*dXY,12,['FloorID: ',num2str(planta_actual)],'FontSize',8,'BackgroundColor',[1 1 0]);
    
    %  keyboard,
    for i=1:size(p,1)
            if planta_actual==plantas_poligono(i)
                % text(p(i,1)+dX,p(i,2)+dY,12,num2str(i),'FontSize',8,'BackgroundColor',[1 1 0]);
                angulo=rand(1)*2*pi;
                text(p(i,1)+dXY*cos(angulo),p(i,2)+dXY*sin(angulo),12,num2str(i),'FontSize',8,'BackgroundColor',[1 1 0]); hold on;
                plot3([p(i,1),p(i,1)+dXY*cos(angulo)]',[p(i,2),p(i,2)+dXY*sin(angulo)]',[11,11]','.-','LineWidth',8,'Color',[1 1 0]);
            end
    end
    hold off;
end
% cambiar el ancho de linea:
lines = findall(gca,'type','line');
set(lines,'linewidth',1,'MarkerSize',8);
disp('Mejorar legibilidad poligono. END.');


% --- Executes on button press in pushbutton_OpenTrajectFile.
function pushbutton_OpenTrajectFile_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_OpenTrajectFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile('*.tra','Select the Trajectory file with your ground-truth path');
if FileName~=0   % He cargado un fichero
    handles.string_fichero_trajectory_actual=[PathName,FileName];
    handles.string_FileName_trajectory_actual=FileName;
    handles.Traject_data=importdata(handles.string_fichero_trajectory_actual);
    %handles.Traject_data,
    array_puntos=handles.Traject_data.data,
    num_points=size(array_puntos,1);
    handles.traject=[];
    for i=1:num_points
        handles.traject(i,1:5)=handles.Traject_data.data(i,1:5);
    end
    handles.trajectory_lastpoint=num_points;
    num_points_route=size(handles.traject,1);
    handles.idx_traj_section_ini=1;
    set(handles.edit_numPointRoute,'String',num2str(num_points_route));
    % completar el listbox
    set(handles.listbox_Trajectory,'String',num2str(handles.traject));
    % Crear un poly nuevo con todos los puntos en el fichero de trayectorias
    handles.h_impoly=impoly(gca,[handles.traject(:,3) handles.traject(:,2)],'Closed',false);  % pinto el agregado completo
    if ~isempty(handles.Calib)
       planta_actual=handles.Calib(handles.Planta_idx).Planta;
    else
        planta_actual=0;
    end
    plantas_poligono=handles.traject(:,4);  % ,planta_actual,plantas_poligono
    pos=handles.h_impoly.getPosition();
    mejorar_legibilidad_poligono(pos,planta_actual,plantas_poligono,handles);
    setColor(handles.h_impoly,'magenta');
    addNewPositionCallback(handles.h_impoly,@display_numero_punto);
        
    % visibilidad activa:
    set(handles.pushbutton_Draw_NewSection,'Visible','on');
    set(handles.text22,'Visible','on');
    set(handles.edit_numPointRoute,'Visible','on');
    set(handles.pushbutton_SaveTraject,'Visible','on');
    set(handles.pushbutton_AssignGroundTruth,'Enable','on');
    set(handles.text_trajectoryfile,'String',FileName);
end

% Update handles structure
guidata(hObject, handles);




% --- Executes on selection change in listbox_Trajectory.
function listbox_Trajectory_Callback(hObject, eventdata, handles)
% hObject    handle to listbox_Trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox_Trajectory contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox_Trajectory


% --- Executes during object creation, after setting all properties.
function listbox_Trajectory_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox_Trajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_SaveTraject.
function pushbutton_SaveTraject_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_SaveTraject (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fecha_hora_str=datestr(now,'dd_mmm_yyyy_HH_MM_SS');
if ~exist('trajectories','dir')
    mkdir('trajectories');
end
path=[cd,'\trajectories\'];
nombre_fichero=['trajectory_',fecha_hora_str,'.tra'];
set(handles.text_trajectoryfile,'String',nombre_fichero);
handles.string_fichero_trajectory_actual=[path,nombre_fichero];
fid=fopen(handles.string_fichero_trajectory_actual,'w+');
fprintf(fid,'%% This is a trajectory file created with a Matlab GUI ("GUI_mapas")\n\n');
fprintf(fid,'Point_Index  Latitude Longitude Floor_ID Building_ID\n');
num_points_route=size(handles.traject,1);
set(handles.edit_numPointRoute,'String',num2str(num_points_route));
for i=1:num_points_route
   fprintf(fid,[num2str(handles.traject(i,1:5),11),'\n']);
end
fclose(fid);
disp('Fichero de trayectorias grabado');

% --- Executes on button press in pushbutton_NewTrajectory.
function pushbutton_NewTrajectory_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_NewTrajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% Borrar trajectorias antiguas:
disp('borro trayectorias antiguas. Start');
if ~isempty(handles.h_impoly)
    disp('Hay trayectorias antiguas. Start');
    if ~isempty(handles.Calib)
        planta_actual=handles.Calib(handles.Planta_idx).Planta;
    else
        planta_actual=0;
    end
    if ~isempty(handles.traject)
        plantas_poligono=handles.traject(:,4);  % ,planta_actual,plantas_poligono
    else
        plantas_poligono=ones(1,length(handles.h_impoly.getPosition()))*planta_actual,
    end
    mejorar_legibilidad_poligono([],planta_actual,plantas_poligono,handles);  % quitar los numeros de la trajectoria
    disp('Hay trayectorias antiguas. Middle');
   %if ishandle(handles.h_impoly)
       delete(handles.h_impoly);  handles.h_impoly=[];
   %end
   disp('hay trayectorias antiguas. End');
end
% No tienen nada las trayectorias internas (handles.traject)
handles.traject=[];  % Array nx5 con los puntos de la trajectoria a recorrer
handles.trajectory_lastpoint=0;
num_points_route=size(handles.traject,1);
set(handles.edit_numPointRoute,'String',num2str(num_points_route));
set(handles.listbox_Trajectory,'String',num2str(handles.traject));
set(handles.text_trajectoryfile,'String',' ');
% visibilidad activa:
set(handles.pushbutton_Draw_NewSection,'Visible','on');
set(handles.listbox_Trajectory,'Visible','on');
set(handles.text22,'Visible','on');
set(handles.edit_numPointRoute,'Visible','on');
set(handles.pushbutton_SaveTraject,'Visible','on');

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_numPointRoute_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_numPointRoute (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function edit_BuildingID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_BuildingID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


function output_txt = mi_funcion_customizar_datacursor(obj,event_obj)
% Display the position of the data cursor
% obj          Currently not used (empty)
% event_obj    Handle to event object
% output_txt   Data cursor text string (string or cell array of strings).

pos = get(event_obj,'Position');
output_txt = {['Latitude: ',num2str(pos(2),11)],...
    ['Longitude: ',num2str(pos(1),11)]};

% If there is a Z-coordinate in the position, display it as well
if length(pos) > 2
    output_txt{end+1} = ['Z: ',num2str(pos(3),11)];
end


% --------------------------------------------------------------------
function uipushtool2_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uipushtool2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
h = imdistline(gca);
handles.api = iptgetapi(h);
%fcn = makeConstrainToRectFcn('imline',get(gca,'XLim'),get(gca,'YLim'));
%handles.api.setDragConstraintFcn(fcn);
%handles.api.setLabelTextFormatter('%02.8f degrees'); 
handles.api.setLabelVisible(false);
handles.api.addNewPositionCallback(@funcion_medida_distancia);
pos=handles.api.getPosition();
handles.h_text_distancia=text((pos(1,1)+pos(2,1))/2,(pos(1,2)+pos(2,2))/2,2,'Move to measure distance');
set(handles.h_text_distancia,'FontSize',15,'BackgroundColor',[1 0.7 0.5]);
% Update handles structure
guidata(hObject, handles);


function funcion_medida_distancia(pos)
handles=guidata(gcf);
% pos: col1:Longitude, col2 latitude
d_Lon=abs(pos(1,1)-pos(2,1));  % delta_longitude (grados)
d_Lat=abs(pos(1,2)-pos(2,2));  % delta_latitude (grados)
radio=handles.radio_Tierra; % metros
latitude_media=(pos(1,2)+pos(2,2))/2;
NS=radio*tan(d_Lat*pi/180);  % North-South in meters
WE=radio*tan(d_Lon*pi/180)*cos(latitude_media*pi/180);  % West-East in meters
distancia_metros=sqrt(NS^2+WE^2),
%delete(handles.h_text_distancia);
set(handles.h_text_distancia,'Position',[(pos(1,1)+pos(2,1))/2,(pos(1,2)+pos(2,2))/2, 2],'String',[num2str(distancia_metros,4),' m']);
% Update handles structure
%guidata(gcf, handles);


% --------------------------------------------------------------------
function uipushtool3_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to uipushtool3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% borrar cosas sobre el plano:
lines = findall(gca,'type','line');
delete(lines);
text_seco = findall(gca,'type','text');
delete(text_seco);


% --- Executes on button press in pushbutton_OpenPhoneLogFile.
function pushbutton_OpenPhoneLogFile_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_OpenPhoneLogFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uigetfile('*.txt','Select the LogFile file created with SmartPhone');
handles.pathname=PathName;
handles.string_fichero_LogFile_actual=[PathName,FileName];
filename_sin_extension=[PathName,FileName(1:end-4)];
handles.string_fichero_LogFile_new=[filename_sin_extension,'_POSI_assigned.txt'];
handles.string_fichero_LogFile_new_deleted=[filename_sin_extension,'_POSI_deleted.txt'];
set(handles.edit_OpenedLogFile,'String','Wait...');   % No se!!, pero no actualiza este texto
guidata(hObject, handles);  % Update handles structure
pause(.001)
% parsear:
[handles.datos_LogFile]=leer_LogFile(handles.string_fichero_LogFile_actual);
[numlineas,numposi,aux_idx_Posi,num_Dif_Wifi_AP,handles]=parsear_LogFile(handles.datos_LogFile,filename_sin_extension,handles);
% actualizar datos en panel visualizacion:
set(handles.text_NumLines_output,'String',numlineas);
set(handles.text_Total_POSI_output,'String',numposi);
set(handles.text_Num_diff_WiFi_AP,'String',num_Dif_Wifi_AP);
set(handles.edit_OpenedLogFile,'String',FileName);
% contar POSIs vacios y llenos
if ~isempty(aux_idx_Posi)
    handles.idx_Posi=aux_idx_Posi;
   [numPOSI_empty,numPOSI_filled]=contarPOSI_LogFile(handles.datos_LogFile,handles.idx_Posi);
else
    numPOSI_empty=0; numPOSI_filled=0;
end
set(handles.edit_NumPosiEmpty_output,'String',num2str(numPOSI_empty));
set(handles.edit_NumPosiFilled_output,'String',num2str(numPOSI_filled));
% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function edit_OpenedLogFile_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_OpenedLogFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function edit_NumPosiEmpty_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_NumPosiEmpty_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function edit_NumPosiFilled_output_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_NumPosiFilled_output (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton_AssignGroundTruth.
function pushbutton_AssignGroundTruth_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_AssignGroundTruth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.traject) && ~isempty(handles.idx_Posi)
   [handles.datos_LogFile_new]=Assign_POSI_LogFile(handles.datos_LogFile,handles.idx_Posi,handles.traject);
   disp('Assigned POSI data to new LogFilefile');
else
    disp('ERROR: No Trajctory file or LogFilefile loaded');
    handles.datos_LogFile_new=[];
end
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_ClearPOSImarksFromLogFile.
function pushbutton_ClearPOSImarksFromLogFile_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ClearPOSImarksFromLogFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if  ~isempty(handles.idx_Posi)
   [handles.datos_LogFile_new_deleted]=ClearPOSImarksFromLogFile(handles.datos_LogFile,handles.idx_Posi);
   disp('Deleted POSI from new LogFilefile');
else
    disp('ERROR: No LogFilefile loaded');
    handles.datos_LogFile_new_deleted=[];
end
% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_SaveLogFile.
function pushbutton_SaveLogFile_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_SaveLogFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.datos_LogFile_new)
    fid = fopen(handles.string_fichero_LogFile_new,'w');
    fwrite(fid,handles.datos_LogFile_new);  % escribo el fichero y lo cargo todo en datos
    fclose(fid);
    disp(['New Log_File (POSI  assigned) saved as ',handles.string_fichero_LogFile_new]);
end
if ~isempty(handles.datos_LogFile_new_deleted)
    fid = fopen(handles.string_fichero_LogFile_new_deleted,'w');
    fwrite(fid,handles.datos_LogFile_new_deleted);  % escribo el fichero y lo cargo todo en datos
    fclose(fid);
    disp(['New Log_File (POSI deleted) saved as ',handles.string_fichero_LogFile_new_deleted]);
end

function [datos_LogFile]=leer_LogFile(filename)
% Abrir fichero y volcar contenido en "datos_fichero"
fid = fopen(filename);
datos_LogFile=fread(fid);  % leo el fichero y lo cargo todo en datos
fclose(fid);


function [numlineas,numposi,idx_Posi,num_Dif_Wifi_AP,handles]=parsear_LogFile(datos_LogFile,filename_sin_extension,handles)
% OUTPUT: 
%   numlineas:  el numero de lineas del fichero
%   numposi:    el numero de lineas de tipo POSI
%   idx_Posi:   los indices dentro de "datos_LogFile" que definen las lineas con POSI
%   num_Dif_Wifi_AP:  número de Wifi AP diferentes vistos (con cualquier valor de RSS)

set(handles.text_Num_diff_WiFi_AP,'String','?');
posfinlineas=find(datos_LogFile==10);  % busco saltos de linea (10 = LF(LineFeed),  13=CR(Carrige Return))
posfinlineas=[0; posfinlineas]; %
numlineas=length(posfinlineas)-1; % numero de lineas en fichero
disp(['Numero de lineas: ',num2str(numlineas)]);

% Parsear los datos y Grabarlos en un .mat
idx_linea=0;  % reseteo el indice de la linea del fichero
% Reservar memoria
Posi=ones(numlineas,6)*NaN; index_Posi=1;
idx_Posi=ones(numlineas,2)*NaN; 
Acce=ones(numlineas,6)*NaN; index_Acce=1;
Gyro=ones(numlineas,6)*NaN; index_Gyro=1;
Magn=ones(numlineas,6)*NaN; index_Magn=1;
Pres=ones(numlineas,4)*NaN; index_Pres=1;
Ligh=ones(numlineas,4)*NaN; index_Ligh=1;
Prox=ones(numlineas,4)*NaN; index_Prox=1;
Soun=ones(numlineas,4)*NaN; index_Soun=1;
Ahrs=ones(numlineas,9)*NaN; index_Ahrs=1;
Gnss=ones(numlineas,10)*NaN; index_Gnss=1;
Rfid=ones(numlineas,5)*NaN; index_Rfid=1;% datos RFID con 5 columas: 1)time stamp, 2) id_reader, 3) id_tag, 4) RSS1  y RSS2
Wifi=ones(numlineas,4)*NaN; Wifi_extra=cell(numlineas,2); index_Wifi=1;
Ble4=ones(numlineas,4)*NaN; index_Ble4=1;
Imux=ones(numlineas,21)*NaN; index_Imux=1;

eof=false;  datos=[];  tipo='';

tamventana2=35;    % tamaño de la ventana donde sacas el nombre del archivo (en caracteres)
dummycheck=round(numlineas/tamventana2);

while (~eof)
    % Leer una medida %[tipo,datos,eof]=obj.getSiguienteMedida();
    linea_con_medida=false;
    while (~linea_con_medida && ~eof)
        idx_linea=idx_linea+1;
        
        if(mod(idx_linea,dummycheck)==1),
            cadena_progreso=char(35*ones(1,round(tamventana2*idx_linea/numlineas)));
            set(handles.text_NumLines_output,'String',idx_linea);
            set(handles.edit_OpenedLogFile,'String',cadena_progreso);
            pause(.001)
        end
        
        if ( idx_linea<=numlineas) % hay lineas por procesar
            idx_ini=posfinlineas(idx_linea)+1; % posicion absoluta de principio de linea en 'data'
            idx_fin=posfinlineas(idx_linea+1)-1; % posicion absoluta de final de linea actual
            linea=char(datos_LogFile(idx_ini:idx_fin)');
            % disp(['Linea: ',linea]);
            [tipo,datos,datos_extra]=procesa_linea_LogFile(linea);
            if ~isempty(datos)
                linea_con_medida=true; % ya hemos cogido una medida buena (el resto son comentarios)
                %disp(['tipo: ',tipo,', Datos: ',num2str(datos),]);
            end
        else
            eof=true;
            disp('No hay mas que leer. Fin de datos');
        end
    end
    if eof
        break
    end
    % Almacenar el tipo de medida en la matriz corrrespondiente:
    if strcmp(tipo,'POSI')
        Posi(index_Posi,1:6)=datos;   
        idx_Posi(index_Posi,1:2)=[idx_ini idx_fin];  % marcadores de linea con Posis
        index_Posi=index_Posi+1;
    end
    if strcmp(tipo,'ACCE')
        Acce(index_Acce,1:6)=datos;   index_Acce=index_Acce+1;
    end
    if strcmp(tipo,'GYRO')
        Gyro(index_Gyro,1:6)=datos;   index_Gyro=index_Gyro+1;
    end
    if strcmp(tipo,'MAGN')
        Magn(index_Magn,1:6)=datos;   index_Magn=index_Magn+1;
    end
    if strcmp(tipo,'PRES')
        Pres(index_Pres,1:4)=datos;   index_Pres=index_Pres+1;
    end
    if strcmp(tipo,'LIGH')
        Ligh(index_Ligh,1:4)=datos;   index_Ligh=index_Ligh+1;
    end
    if strcmp(tipo,'PROX')
        Prox(index_Prox,1:4)=datos;   index_Prox=index_Prox+1;
    end
    if strcmp(tipo,'SOUN')
        Soun(index_Soun,1:4)=datos;   index_Soun=index_Soun+1;
    end
    if strcmp(tipo,'AHRS')
        Ahrs(index_Ahrs,1:9)=datos;   index_Ahrs=index_Ahrs+1;
    end
    if strcmp(tipo,'GNSS')
        Gnss(index_Gnss,1:10)=datos;   index_Gnss=index_Gnss+1;
    end
    if strcmp(tipo,'RFID')
        Rfid(index_Rfid,1:5)=datos;   index_Rfid=index_Rfid+1;
    end
    if strcmp(tipo,'WIFI')  % 1: timestamp  2: timestamp_sensor   3: MAC_dec   4: RSS
        Wifi(index_Wifi,1:4)=datos;   
        Wifi_extra{index_Wifi,1}=datos_extra{1};
        Wifi_extra{index_Wifi,2}=datos_extra{2};
        index_Wifi=index_Wifi+1;
    end
    if strcmp(tipo,'BLE4')
        Ble4(index_Ble4,1:4)=datos;   index_Ble4=index_Ble4+1;
    end
    if strcmp(tipo,'IMUX')
        Imux(index_Imux,1:21)=datos;   index_Imux=index_Imux+1;
    end
end
% Recortar arrays datos
Posi=Posi(1:index_Posi-1,:);
idx_Posi=idx_Posi(1:index_Posi-1,:);
Acce=Acce(1:index_Acce-1,:);
Gyro=Gyro(1:index_Gyro-1,:);
Magn=Magn(1:index_Magn-1,:);
Pres=Pres(1:index_Pres-1,:);
Ligh=Ligh(1:index_Ligh-1,:);
Prox=Prox(1:index_Prox-1,:);
Soun=Soun(1:index_Soun-1,:);
Ahrs=Ahrs(1:index_Ahrs-1,:);
Gnss=Gnss(1:index_Gnss-1,:);
Rfid=Rfid(1:index_Rfid-1,:);
Wifi=Wifi(1:index_Wifi-1,:);
Wifi_extra=Wifi_extra(1:index_Wifi-1,1:2);
Ble4=Ble4(1:index_Ble4-1,:);
Imux=Imux(1:index_Imux-1,:);

% anotar el numero de marcas POSI
numposi=size(Posi,1);

% contar el numero de WiFi distintos vistos:
lecturas_Wifi=size(Wifi,1);
Wifi_Dif=zeros(lecturas_Wifi,1);  % Acumulo solo los diferentes
Wifi_extra_Dif=cell(lecturas_Wifi,2);
idx_Wifi_Dif=0;
for ii=1:lecturas_Wifi
    if ~any(Wifi(ii,3)==Wifi_Dif(:,1))  % ver si no está esa MAC en el acumulador de diferentes Wifi_Dif
        idx_Wifi_Dif=idx_Wifi_Dif+1; % si no está añado otra linea
        Wifi_Dif(idx_Wifi_Dif,1)=Wifi(ii,3);  % almaceno MAC_dec
        Wifi_extra_Dif{idx_Wifi_Dif,1}=Wifi_extra{ii,1};  % BSSID string
        Wifi_extra_Dif{idx_Wifi_Dif,2}=Wifi_extra{ii,2};   % MAC string
    end
end
Wifi_Dif=Wifi_Dif(1:idx_Wifi_Dif,:);
Wifi_extra_Dif=Wifi_extra_Dif(1:idx_Wifi_Dif,1:2);
Wifi_extra_Dif=sortrows(Wifi_extra_Dif,2),
num_Dif_Wifi_AP=idx_Wifi_Dif;

% crear fichero .mat con array medidas:
save([filename_sin_extension,'.mat'],'Posi','Acce','Gyro','Magn','Pres','Ligh','Prox','Soun','Ahrs','Gnss','Rfid','Wifi','Wifi_extra','Wifi_extra_Dif','Ble4','Imux');

% guardar arrays en handles:
handles.LogFile_Posi=Posi;
handles.LogFile_Acce=Acce;
handles.LogFile_Gyro=Gyro;
handles.LogFile_Magn=Magn;
handles.LogFile_Pres=Pres;
handles.LogFile_Ligh=Ligh;
handles.LogFile_Prox=Prox;
handles.LogFile_Soun=Soun;
handles.LogFile_Ahrs=Ahrs;
handles.LogFile_Gnss=Gnss;
handles.LogFile_Rfid=Rfid;
handles.LogFile_Ble4=Ble4;
handles.LogFile_Wifi=Wifi;
handles.LogFile_Wifi_extra=Wifi_extra;
handles.LogFile_Imux=Imux;




function [tipo,datos,datos_extra]=procesa_linea_LogFile(linea)
% Procesamos la línea
% LogFile Data format:
% Accelerometer data: 	'ACCE;AppTimestamp(s);SensorTimestamp(s);Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Accuracy(integer)'
% Gyroscope data:     	'GYRO;AppTimestamp(s);SensorTimestamp(s);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Accuracy(integer)'
% Magnetometer data:  	'MAGN;AppTimestamp(s);SensorTimestamp(s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Accuracy(integer)'
% Pressure data:      	'PRES;AppTimestamp(s);SensorTimestamp(s);Pres(mbar);Accuracy(integer)'
% Light data:         	'LIGH;AppTimestamp(s);SensorTimestamp(s);Light(lux);Accuracy(integer)'
% Proximity data:     	'PROX;AppTimestamp(s);SensorTimestamp(s);prox(?);Accuracy(integer)'
% Humidity data:      	'HUMI;AppTimestamp(s);SensorTimestamp(s);humi(%);Accuracy(integer)'
% Temperature data:   	'TEMP;AppTimestamp(s);SensorTimestamp(s);temp(ÂºC);Accuracy(integer)'
% Orientation data:   	'AHRS;AppTimestamp(s);SensorTimestamp(s);PitchX(Âº);RollY(Âº);YawZ(Âº);RotVecX();RotVecY();RotVecZ();Accuracy(int)'
% GNSS/GPS data:        'GNSS;AppTimestamp(s);SensorTimeStamp(s);Latit(º);Long(º);Altitude(m);Bearing(º);Accuracy(m);Speed(m/s);SatInView;SatInUse'
% WIFI data:          	'WIFI;AppTimestamp(s);SensorTimeStamp(s);Name_SSID;MAC_BSSID;RSS(dBm);'
% Bluetooth data:     	'BLUE;AppTimestamp(s);Name;MAC_Address;RSS(dBm);'
% BLE 4.0 data:       	'BLE4;AppTimestamp(s);MajorID;MinorID;RSS(dBm);'
% Sound data:         	'SOUN;AppTimestamp(s);RMS;Pressure(Pa);SPL(dB);'
% RFID Reader data:   	'RFID;AppTimestamp(s);ReaderNumber(int);TagID(int);RSS_A(dBm);RSS_B(dBm);'
% IMU XSens data:     	'IMUX;AppTimestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(Âº);Pitch(Âº);Yaw(Âº);Pressure(mbar);Temp(ÂºC)'
% IMU LPMS-B data:    	'IMUL;AppTimestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(Âº);Pitch(Âº);Yaw(Âº);Pressure(mbar);Temp(ÂºC)'
% POSI Reference:  		'POSI;AppTimestamp(s);Counter;Latitude(degrees); Longitude(degrees);floor ID(0,1,2..4);Building ID(0,1,2..3);'
% Note that there are two timestamps:
%  -'AppTimestamp' is set by the Android App as data is read. It is not representative of when data is actually captured by the sensor (but has a common time reference for all sensors)
%  -'SensorTimestamp' is set by the sensor itself (the delta_time=SensorTimestamp(k)-SensorTimestamp(k-1) between two consecutive samples is an accurate estimate of the sampling interval). This timestamp is better for integrating inertial data.

% Inicializo la salida de datos. Si no es nada reconocible -> los datos y tipo vacios
datos=[];  datos_extra=[];
tipo='';

if (numel(linea)>0 && ~strcmp(linea(1),'%') )
    if ( strfind(linea,'POSI'))  % Es una linea de Marca Posicion
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')'; %output=textscan(linea,'%*4s;%f;%f;%f;%f;%f')';
        tipo='POSI';
    end
    if ( strfind(linea,'ACCE'))  % Es una linea de Acelerometro
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')'; %output=textscan(linea,'%*4s;%f;%f;%f;%f;%f')';
        tipo='ACCE';
    end
    if ( strfind(linea,'GYRO'))  % Es una linea de Gyroscopo
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')';
        tipo='GYRO';
    end
    if ( strfind(linea,'MAGN'))  % Es una linea de Magnetometro
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f')';
        tipo='MAGN';
    end
    if ( strfind(linea,'PRES'))  % Es una linea de Presion
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
        tipo='PRES';
    end
    if ( strfind(linea,'LIGH'))  % Es una linea de Luminosidad. p.ej: LIGH;14;22.370;2565.0;0
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
        tipo='LIGH';
    end
    if ( strfind(linea,'PROX'))  % Es una linea de Proximidad. p.ej: PROX;0.23;22.370;0.0;0
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
        tipo='PROX';
    end
    if ( strfind(linea,'SOUN'))  % Es una linea de Sonido. p.ej.: SOUN;0.200;576.05;0.01758;58.88
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
        tipo='SOUN';
    end
    if ( strfind(linea,'AHRS'))  % Es una linea de Orientacion
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
        tipo='AHRS';
    end
    if ( strfind(linea,'GNSS'))  % Es una linea de GPS
        % 'GNSS;Timestamp(s);SensorTimeStamp(s);Latitude(º);Longitude(º);Altitude(m);Bearing(º),Accuracy(m),Speed(m/s),satellites_in_view;num_satellites_in_use'
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
        tipo='GNSS';
    end
    if ( strfind(linea,'RFID'))  % Es una linea de RFID. p.ej.: "RFID;22.365;29;67937;80;80"
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f')';
        tipo='RFID';
    end
    if ( strfind(linea,'BLE4'))  % Es una linea de BLE4. p.ej.: "BLE4;0.750;2016;3;-87"
                    datos=sscanf(linea,'%*4s;%f;%f;%f;%f')';
                    tipo='BLE4';
    end
    if ( strfind(linea,'WIFI'))  % Es una linea de WIFI.
        % Formato nuevo WIFI data: 'WIFI;AppTimestamp(s);SensorTimeStamp(s);Name_SSID;MAC_BSSID;RSS(dBm);'
        % p.ej.: "WIFI;22.365;8051.234;portal-csic;00:0b:86:27:36:c1;-71"
        cell_array=textscan(linea,'%*s %f %f %s %s %f','delimiter',';');
        datos(1)=cell_array{1,1}; % timestamp
        datos(2)=cell_array{1,2}; % Sensortimestamp
        datos(4)=cell_array{1,5}; % RSS
        MAC_str=cell_array{1,4}{1,1}; % MAC string
        SSID_str=cell_array{1,3}{1,1}; % SSID nombre AP string
        MAC_dec_array=sscanf(MAC_str,'%x:%x:%x:%x:%x:%x'); % quitar ":" y convertir a numero
        MAC_dec=MAC_dec_array(1)*256^5+MAC_dec_array(2)*256^4+MAC_dec_array(3)*256^3+MAC_dec_array(4)*256^2+MAC_dec_array(5)*256+MAC_dec_array(6);
        datos(3)=MAC_dec;
        datos_extra={SSID_str, MAC_str};
        tipo='WIFI';
        % disp(['Linea: ',linea]);
    end
    if ( strfind(linea,'IMUL'))  % Es una linea de IMUL. p.ej.: "IMUL;0.929;1907.2437;11782;0.22992;-0.15328;9.31184;-0.01853;0.01550;0.00004;-24.02985;56.41791;22.50000;-1.28843;1.03222;121.61204;936.870;0.00"
        % 21 campos: Timestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(º);Pitch(º);Yaw(º);Quat1;Quat2;Quat3;Quat4;Pressure(mbar);Temp(ºC)'");
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
        % Calibro magnetometro por hard iron effects:
        datos(10:12)=datos(10:12)-[-22.7386, 44.0594, 59.0431]; % Quitar cuando calibre internamente en la flash del LPMS-B
        tipo='IMUL';
    end
    if ( strfind(linea,'IMUX'))  % Es una linea de IMUX. p.ej.: "IMUL;0.929;1907.2437;11782;0.22992;-0.15328;9.31184;-0.01853;0.01550;0.00004;-24.02985;56.41791;22.50000;-1.28843;1.03222;121.61204;936.870;0.00"
        % 21 campos: Timestamp(s);SensorTimestamp(s);Counter;Acc_X(m/s^2);Acc_Y(m/s^2);Acc_Z(m/s^2);Gyr_X(rad/s);Gyr_Y(rad/s);Gyr_Z(rad/s);Mag_X(uT);;Mag_Y(uT);Mag_Z(uT);Roll(º);Pitch(º);Yaw(º);Quat1;Quat2;Quat3;Quat4;Pressure(mbar);Temp(ºC)'");
        datos=sscanf(linea,'%*4s;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f')';
        tipo='IMUX';
    end
end

function [numPOSI_empty,numPOSI_filled]=contarPOSI_LogFile(datos_LogFile,idx_Posi)
%INPUT:
%   datos_LogFile:    Buffer de chars con todo el fichero del  LogFile
%   idx_Posi:         Indices dentro de "datos_LogFile" que definen donde encontar las lineas con POSI.  

numPOSI_empty=0;
numPOSI_filled=0;
for i=1:size(idx_Posi,1)
    linea_POSI=char(datos_LogFile(idx_Posi(i,1):idx_Posi(i,2))');
    disp(['linea_POSI: ',linea_POSI]);
    idx_separador=find(linea_POSI==';');  % idx_separador relativo a la linea (debe haber 6 puntos y coma en una linea POSI)
    tiempo=linea_POSI(idx_separador(1)+1:idx_separador(2)-1);
    number=linea_POSI(idx_separador(2)+1:idx_separador(3)-1);
    Latitude=str2num(linea_POSI(idx_separador(3)+1:idx_separador(4)-1));
    Longitude=str2num(linea_POSI(idx_separador(4)+1:idx_separador(5)-1));
    floorID=str2num(linea_POSI(idx_separador(5)+1:idx_separador(6)-1));
    buildingID=str2num(linea_POSI(idx_separador(6)+1:end));
    if Latitude==0 && Longitude==0
        numPOSI_empty=numPOSI_empty+1;
    else
        numPOSI_filled=numPOSI_filled+1;
    end
end

function [datos_LogFile_new]=Assign_POSI_LogFile(datos_LogFile,idx_Posi,GroundTruth)
% INPUT:
%   datos_LogFile:    Buffer de chars con todo el fichero del  LogFile
%   idx_Posi:         Indices dentro de "datos_LogFile" que definen donde encontar las lineas con POSI.
%   GroundTruth:      The real position of points (Lattitude, Longitude, FloorID and BuildingID)
% OUTPUT:
%   datos_LogFile:    Buffer de chars con todo el fichero del  LogFile actualizado (handles.Traject_data), 
%                     es un array nx5 con las columnas: 1) contador, 2)Latitude; 3) Longitude, 4)FloorID, 5) BuildingID

datos_LogFile_new=[]; % La construyo progresivamente
for i=1:size(idx_Posi,1)  % por cada linea POSI
    linea_POSI=char(datos_LogFile(idx_Posi(i,1):idx_Posi(i,2))');
    idx_separador=find(linea_POSI==';');  % idx_separador relativo a la linea (debe haber 6 puntos y coma en una linea POSI)
    tiempo=linea_POSI(idx_separador(1)+1:idx_separador(2)-1);
    number=linea_POSI(idx_separador(2)+1:idx_separador(3)-1);
    Latitude=str2num(linea_POSI(idx_separador(3)+1:idx_separador(4)-1));
    Longitude=str2num(linea_POSI(idx_separador(4)+1:idx_separador(5)-1));
    floorID=str2num(linea_POSI(idx_separador(5)+1:idx_separador(6)-1));
    buildingID=str2num(linea_POSI(idx_separador(6)+1:end));
    % Poner lo nuevo
    Latitude_new=GroundTruth(i,2);
    Longitude_new=GroundTruth(i,3);
    FloorID_new=GroundTruth(i,4);
    BuildingID_new=GroundTruth(i,5);
    linea_POSI_new=['POSI;',num2str(tiempo,6),';',num2str(number),';',num2str(Latitude_new,11),';',num2str(Longitude_new,11),';',num2str(FloorID_new),';',num2str(BuildingID_new)];
    disp(['linea_POSI_new: ',linea_POSI_new]);
     % acumular datos_LogFile_new
     if i==1  % La primera vez
         datos_LogFile_new=[datos_LogFile_new; datos_LogFile(1:idx_Posi(i,1)-1); linea_POSI_new'];  % añado lo ultimo entre las ultimas lineas POSI y la nueva generada
     else
        datos_LogFile_new=[datos_LogFile_new; datos_LogFile(idx_Posi(i-1,2)+1:idx_Posi(i,1)-1); linea_POSI_new'];  % añado lo ultimo entre las ultimas lineas POSI y la nueva generada
     end
end
% añado lo ultimo del LogFile tras la ultima linea POSI:
datos_LogFile_new=[datos_LogFile_new; datos_LogFile(idx_Posi(size(idx_Posi,1),2)+1:end)];
disp('Asignados POSI a datos_LogFile_new');



function [datos_LogFile_new_deleted]=ClearPOSImarksFromLogFile(datos_LogFile,idx_Posi)
% INPUT:
%   datos_LogFile:    Buffer de chars con todo el fichero del  LogFile
%   idx_Posi:         Indices dentro de "datos_LogFile" que definen donde encontar las lineas con POSI.
% OUTPUT:
%   datos_LogFile:    Buffer de chars con todo el fichero del  LogFile actualizado (handles.Traject_data), 
%                     es un array nx5 con las columnas: 1) contador, 2)Latitude; 3) Longitude, 4)FloorID, 5) BuildingID

datos_LogFile_new_deleted=[]; % La construyo progresivamente
for i=1:size(idx_Posi,1)  % por cada linea POSI
     % acumular datos_LogFile_new
     if i==1  % La primera vez
         datos_LogFile_new_deleted=[datos_LogFile_new_deleted; datos_LogFile(1:idx_Posi(i,1)-2)];  % añado lo ultimo entre las ultimas lineas POSI
     else
        datos_LogFile_new_deleted=[datos_LogFile_new_deleted; datos_LogFile(idx_Posi(i-1,2)+1:idx_Posi(i,1)-2)];  % añado lo ultimo entre las ultimas lineas POSI
     end
end
% añado lo ultimo del LogFile tras la ultima linea POSI:
datos_LogFile_new_deleted=[datos_LogFile_new_deleted; datos_LogFile(idx_Posi(size(idx_Posi,1),2)+1:end)];
disp('Borrados los POSI en datos_LogFile_new_deleted');


% --- Executes on button press in pushbutton_ScreenCapture.
function pushbutton_ScreenCapture_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_ScreenCapture (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.contador_capturas=handles.contador_capturas+1;
FileName=handles.string_FileName_trajectory_actual;
screencapture(gca,['captura_',FileName,'_',num2str(handles.contador_capturas),'.jpg']);

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_Tra_NEXT.
function pushbutton_Tra_NEXT_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Tra_BACK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
largo_trayectoria=size(handles.traject,1);
idx_traj_section_ini=handles.idx_traj_section_ini;
idx_traj_section_fin=largo_trayectoria; % inicializo con algo (luego se calcula)
floor_ini=handles.traject(min(idx_traj_section_ini,largo_trayectoria),4);


if idx_traj_section_ini>=largo_trayectoria
    disp(' En ultima seccion de trayectoria. No se puede ir mas para delante');
    handles.idx_traj_section_ini=largo_trayectoria+1;  % preparo para siguiente sección que vanda
else
    % cambiar el bitmap de la planta sobre el  mapa
    total_plantas=size(handles.Calib,2);
    Planta_idx=1;
    for ii=1:total_plantas
        if handles.Calib(ii).Planta==floor_ini
            Planta_idx=ii;
        end
    end
    set(handles.popupmenu_SelectedImage,'Value',Planta_idx);
    popupmenu_SelectedImage_Callback(handles.popupmenu_SelectedImage, [], handles);
    % ver la secion d ela trayectoria y pintar la trayectoria y las etiquetas de los GT
    i=idx_traj_section_ini;
    igual=true;
    while i<largo_trayectoria && igual
        if handles.traject(i,4)==floor_ini
            i=i+1;
        else
            igual=false;
            idx_traj_section_fin=i-1; % ya tengo el idx de ini y fin de una seccion en una misma planta
        end
    end
    disp(['Planta:',num2str(floor_ini),'. Seccion de trayectoria de ',num2str(idx_traj_section_ini),' a ',num2str(idx_traj_section_fin)]);
    draw_section(handles,idx_traj_section_ini,idx_traj_section_fin);
    handles.idx_traj_section_ini=min(largo_trayectoria,idx_traj_section_fin+1);  % preparo para siguiente sección que vanda
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in pushbutton_Tra_BACK.
function pushbutton_Tra_BACK_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Tra_BACK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
largo_trayectoria=size(handles.traject,1);
idx_traj_section_fin=max(1,handles.idx_traj_section_ini-1);
idx_traj_section_ini=1; % inicializo con algo (luego se calcula)
floor_fin=handles.traject(idx_traj_section_fin,4);

if idx_traj_section_fin==1
    disp(' En primera seccion de trayectoria. No se puede ir mas para atras');
    idx_traj_section_ini=1;
else
    % cambiar el bitmap de la planta sobre el  mapa
    total_plantas=size(handles.Calib,2);
    Planta_idx=1;
    for ii=1:total_plantas
        if handles.Calib(ii).Planta==floor_fin
            Planta_idx=ii;
        end
    end
    set(handles.popupmenu_SelectedImage,'Value',Planta_idx);
    popupmenu_SelectedImage_Callback(handles.popupmenu_SelectedImage, [], handles);
    % ver la secion d ela trayectoria y pintar la trayectoria y las etiquetas de los GT
    i=idx_traj_section_fin;
    igual=true;
    while i>0 && igual
        if handles.traject(i,4)==floor_fin
            i=i-1;
        else
            igual=false;
            idx_traj_section_ini=i+1; % ya tengo el idx de ini y fin de una seccion en una misma planta
        end
    end
    disp(['Planta:',num2str(floor_fin),'. Seccion de trayectoria de ',num2str(idx_traj_section_ini),' a ',num2str(idx_traj_section_fin)]);
    draw_section(handles,idx_traj_section_ini,idx_traj_section_fin);
end
handles.idx_traj_section_ini=idx_traj_section_ini;  % preparo para siguiente sección que vanda

% Update handles structure
guidata(hObject, handles);


function draw_section(handles,idx_ini,idx_fin)
% borro todo
lines = findall(gca,'type','line');
delete(lines);
text_seco = findall(gca,'type','text');
delete(text_seco);
% precalculo cosas para pintar bien
XLim=get(gca,'XLim');   YLim=get(gca,'YLim');
dX=(XLim(2)-XLim(1))/500;  % desplazamiento un 20% respecto al tamaño del eje
dY=(YLim(2)-YLim(1))/500;  % desplazamiento un 20% respecto al tamaño del eje

% pinto la seccion de trayectoria para una planta
plot(handles.traject(idx_ini:idx_fin,3)+dX,handles.traject(idx_ini:idx_fin,2)+dY,'-om','linewidth',2,'MarkerSize',7);   % las lineas

% y ahora pinto el texto
text(handles.traject(idx_ini:idx_fin,3),handles.traject(idx_ini:idx_fin,2),5*ones(idx_fin-idx_ini+1,1),num2str(handles.traject(idx_ini:idx_fin,1)),'FontSize',8,'BackgroundColor',[1 1 0]);


% --- Executes on button press in puchbutton_VisualizarLogFile.
function puchbutton_VisualizarLogFile_Callback(hObject, eventdata, handles)
% hObject    handle to puchbutton_VisualizarLogFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of puchbutton_VisualizarLogFile
%handles.LogFile_
idx_fig=1;

if ~isempty(handles.LogFile_Posi)
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos_Posi=handles.LogFile_Posi(:,1);
    plot(tiempos_Posi,0,'m*');
    xlabel('time(s)');
    for i=1:size(handles.LogFile_Posi,1), text(tiempos_Posi(i),0.1,num2str(i)); end
    title('POSI marks');
end

if ~isempty(handles.LogFile_Acce)
    figure(idx_fig); idx_fig=idx_fig+1;
    Acce_mag=sqrt(sum(handles.LogFile_Acce(:,3:5).^2,2));
    tiempos=handles.LogFile_Acce(:,2)-handles.LogFile_Acce(1,2);
    plot(tiempos,handles.LogFile_Acce(:,3),'r-'); hold on;
    plot(tiempos,handles.LogFile_Acce(:,4),'g-'); plot(tiempos,handles.LogFile_Acce(:,5),'b-');
    plot(tiempos,Acce_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Acceleration(m/s^2)');
    freq_Acce=(size(handles.LogFile_Acce,1)-1)/(handles.LogFile_Acce(end,1)-handles.LogFile_Acce(1,1));
    title(['Range:[-39.24:39.24] m/s^2, Resolution: 0.00981 m/s^2 (13 bits), Freq.: ',num2str(freq_Acce),' Hz'])
    legend({'Acc_x','Acc_y','Acc_z','Acc_{mag}'});
end

if ~isempty(handles.LogFile_Gyro)
    figure(idx_fig); idx_fig=idx_fig+1;
    Gyro_mag=sqrt(handles.LogFile_Gyro(:,3).^2+handles.LogFile_Gyro(:,4).^2+handles.LogFile_Gyro(:,5).^2);
    tiempos=handles.LogFile_Gyro(:,2)-handles.LogFile_Gyro(1,2);
    plot(tiempos,handles.LogFile_Gyro(:,3),'r-'); hold on;
    plot(tiempos,handles.LogFile_Gyro(:,4),'g-'); plot(tiempos,handles.LogFile_Gyro(:,5),'b-');
    plot(tiempos,Gyro_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Angular Rate(rad/s)');
    freq_Gyro=(size(handles.LogFile_Gyro,1)-1)/(handles.LogFile_Gyro(end,1)-handles.LogFile_Gyro(1,1));
    title(['Range: [-34.9:34.9] rad/s, Resolution: 0.0011 rad/s (16 bits), Freq: ',num2str(freq_Gyro),' Hz'])
    legend({'Gyr_x','Gyr_y','Gyr_z','Gyr_{mag}'});
end

if ~isempty(handles.LogFile_Magn)
    figure(idx_fig); idx_fig=idx_fig+1;
    Magn_mag=sqrt(handles.LogFile_Magn(:,3).^2+handles.LogFile_Magn(:,4).^2+handles.LogFile_Magn(:,5).^2);
    tiempos=handles.LogFile_Magn(:,2)-handles.LogFile_Magn(1,2);
    plot(tiempos,handles.LogFile_Magn(:,3),'r-'); hold on;
    plot(tiempos,handles.LogFile_Magn(:,4),'g-'); plot(tiempos,handles.LogFile_Magn(:,5),'b-');
    plot(tiempos,Magn_mag,'k-'); hold off;
    xlabel('time(s)'); ylabel('Magnetic Field(uT - microTeslas)');
    freq_Magn=(size(handles.LogFile_Magn,1)-1)/(handles.LogFile_Magn(end,1)-handles.LogFile_Magn(1,1));
    title(['Range:[-2000:2000] uT (on Earth typically 25-65 uT), Resolution: 0.0625 uT (16 bits), Freq: ',num2str(freq_Magn),' Hz'])
    legend({'Mag_x','Mag_y','Mag_z','Mag_{mag}'});
end

if ~isempty(handles.LogFile_Pres)
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=handles.LogFile_Pres(:,2)-handles.LogFile_Pres(1,2);
    plot(tiempos,handles.LogFile_Pres(:,3),'r-'); hold on;
    xlabel('time(s)'); ylabel('Pressure (mbar)'); hold off;
    freq_Pres=(size(handles.LogFile_Pres,1)-1)/(handles.LogFile_Pres(end,1)-handles.LogFile_Pres(1,1));
    title(['Range:[625-1250] mbar, Resolution: 0.01 mbar (16 bits), Frequency: ',num2str(freq_Pres),' Hz'])
    legend({'Pressure'});
end

if ~isempty(handles.LogFile_Ligh)
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=handles.LogFile_Ligh(:,2)-handles.LogFile_Ligh(1,2);
    plot(tiempos,handles.LogFile_Ligh(:,3),'r-'); hold on;
    xlabel('time(s)'); ylabel('Light (lux)'); hold off;
    title(['Range: [0:208076 lux] (>1000 outdoors), Resolution: 58 lux (12 bits), Freq.: ',num2str(size(handles.LogFile_Ligh,1)/handles.LogFile_Ligh(end,1)),' Hz'])
    legend({'Light'});
end

if ~isempty(handles.LogFile_Prox)
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=handles.LogFile_Prox(:,2)-handles.LogFile_Prox(1,2);
    plot(tiempos,handles.LogFile_Prox(:,3),'r-'); hold on;
    xlabel('time(s)'); ylabel('Prox (a.u)'); hold off;
    title(['Range: [0 o 8] , Freq.: ',num2str(size(handles.LogFile_Prox,1)/handles.LogFile_Prox(end,1)),' Hz'])
    legend({'Proximity'});
end

if ~isempty(handles.LogFile_Ahrs)
    figure(idx_fig); idx_fig=idx_fig+1;
    tiempos=handles.LogFile_Ahrs(:,2)-handles.LogFile_Ahrs(1,2);
    plot(tiempos,handles.LogFile_Ahrs(:,3),'r-'); hold on;
    plot(tiempos,handles.LogFile_Ahrs(:,4),'g-'); plot(tiempos,handles.LogFile_Ahrs(:,5),'b-'); hold off;
    xlabel('time(s)'); ylabel('Ahrs (º)');
    title(['Range:[?]  , Resolution: ? (? bits), Freq: ',num2str(size(handles.LogFile_Ahrs,1)/handles.LogFile_Ahrs(end,1)),' Hz'])
    legend({'Ahrs_x','Ahrs_y','Ahrs_z'});
end

if ~isempty(handles.LogFile_Soun)
    figure(idx_fig); idx_fig=idx_fig+1;
    subplot(3,1,1); plot(handles.LogFile_Soun(:,1),handles.LogFile_Soun(:,2),'r-');  ylabel('RMS');
    title(['Sound, Freq.: ',num2str(size(handles.LogFile_Soun,1)/handles.LogFile_Soun(end,1)),' Hz']);
    subplot(3,1,2); plot(handles.LogFile_Soun(:,1),handles.LogFile_Soun(:,3),'g-'); ylabel('Pressure(Pa)');
    subplot(3,1,3); plot(handles.LogFile_Soun(:,1),handles.LogFile_Soun(:,4),'b-');xlabel('time(s)'); ylabel('SPL(dB)');
end

if ~isempty(handles.LogFile_Wifi)
    figure(idx_fig); idx_fig=idx_fig+1;
    plot(handles.LogFile_Wifi(:,1),handles.LogFile_Wifi(:,4),'r.'); hold on;
    xlabel('time(s)'); ylabel('RSS (dBm)'); hold off;
    title(['Wifi, Freq.: ',num2str(size(handles.LogFile_Wifi,1)/handles.LogFile_Wifi(end,1)),' Hz'])
    legend({'Wifi'});
end

if ~isempty(handles.LogFile_Ble4)
    figure(idx_fig); idx_fig=idx_fig+1;
    plot(handles.LogFile_Ble4(:,1),handles.LogFile_Ble4(:,4),'r.'); hold on;
    xlabel('time(s)'); ylabel('RSS (dBm)'); hold off;
    title(['Ble4, Freq.: ',num2str(size(handles.LogFile_Ble4,1)/handles.LogFile_Ble4(end,1)),' Hz'])
    legend({'Ble4'});
end

if ~isempty(handles.LogFile_Gnss)
    figure(idx_fig); idx_fig=idx_fig+1;
    subplot(4,2,1);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,3),'r-');ylabel('Lat(º)');
    title(['GNSS data - Freq: ',num2str(size(handles.LogFile_Gnss,1)/handles.LogFile_Gnss(end,1)),' Hz']);
    subplot(4,2,2);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,4),'r-'); ylabel('Long(º)');
    title(['Resolution=[1e-4º, 1.1 m, 0.3 m/s] ']);
    subplot(4,2,3);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,5),'r-'); ylabel('Height(m)');
    subplot(4,2,4);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,7),'r-'); ylabel('Accuracy(m)');
    subplot(4,2,5);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,6),'r-'); ylabel('Bearing(º)');
    subplot(4,2,6);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,8),'r-'); ylabel('Speed(m/s)');
    subplot(4,2,7);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,9),'r-'); ylabel('# Sat in View'); xlabel('time(s)');
    subplot(4,2,8);
    plot(handles.LogFile_Gnss(:,1),handles.LogFile_Gnss(:,10),'r-'); ylabel('# Sat in Use'); xlabel('time(s)');
end

clc;
clear;

rosshutdown;
rosinit('172.19.135.224','NodeHost','172.19.121.18');

sub = rossubscriber('/camera/color/image_raw/compressed');

velpub = rospublisher('/cmd_vel','geometry_msgs/Twist');
velmsg = rosmessage(velpub);

velmsg.Linear.X = 0.0;
        send(velpub,velmsg)

bluethreshhold = 100; %define the threshold value for blue
redthreshhold = 200;
greenthreshhold = 190;
maxorangecount = 20; %how much orange is being found

for i = 1:70
    
    x1 = 0;
    x2 = 0;
    
    y1 = 0;
    y2 = 0;
    maxy = 0;

    image = readImage(receive(sub,10)); %get image from camera
    %figure(1);
    %imshow(image);%show original image
    [rows, columns, channels] = size(image); %get size of image

    orangeimage = zeros(rows, columns); %set up blue detection image

    for x = 1:columns
        orangecount = 0;
        for y = 1:rows
            if image(y,x,1) >= redthreshhold && image(y,x,2) <= greenthreshhold && image(y,x,3) <= bluethreshhold
                orangeimage(y,x) = 255;
                orangecount = orangecount + 1;
                
                if orangecount >= maxy && orangecount >= maxorangecount
                    maxy = orangecount;
                    y2 = y;
                    y1 = y - maxy;
                end
                
            else
                orangeimage(y,x) = 0;
                orangecount = 0;
            end
            
            if orangecount == maxorangecount && x1 == 0
                x1 = x;
            elseif orangecount == maxorangecount && x1 ~=0
                x2 = x;
            end
            
        end
        
        maxy = 0;
    end
    
    figure(2);
    imshow(orangeimage);%show blue detection image

    direction(x1, x2, columns,velpub,velmsg)
    
    disp(y1)
    disp(y2)
    disp(x1)
    disp(x2)
    
    coorddisplay(image,x1,x2,y1,y2)
    
    
    
    pause(0.1);
end

velmsg.Linear.X = 0.0;
send(velpub,velmsg)

function [] = direction(x1, x2, columns,velpub,velmsg)
    if (x2-x1) <= columns/2 && x2 ~=0
        velmsg.Linear.X = 0.1;
        send(velpub,velmsg)
%         if (x2+x1)/2 <= columns/3
%             disp('LEFT')
%         elseif (x2+x1)/2 >= columns*(2/3)
%             disp('RIGHT')
%         else
%             disp('STRAIGHT')
%         end
    elseif (x2-x1) > columns/4
        velmsg.Linear.X = 0.0;
send(velpub,velmsg)
        disp('STOP')
    elseif x2 == 0
        velmsg.Linear.X = 0.0;
send(velpub,velmsg)
        disp('LOOK')
    end
    
end


function [] = coorddisplay(image,x1,x2,y1,y2)
        
    if y2 ~= 0 && x2 ~= 0
        y = fix((y1+y2)/2);
        for x = x1:x2
            image(y,x,1) = 255;
            image(y,x,2) = 0;
            image(y,x,3) = 0;
        end

        x = fix((x1+x2)/2);
        for y = y1:y2
            image(y,x,1) = 255;
            image(y,x,2) = 0;
            image(y,x,3) = 0;
        end
        
    end
    figure(1)
    imshow(image)
end


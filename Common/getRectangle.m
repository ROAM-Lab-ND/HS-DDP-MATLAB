function Rectangle = getRectangle(leftTop, rightBottom, space)
leftBottom = [leftTop(1), rightBottom(2)]';
rightTop   = [rightBottom(1), leftTop(2)]';

[leftEdgeX,leftEdgeY] = meshgrid(leftTop(1), leftBottom(2):space:leftTop(2));
[topEdgeX,topEdgeY] = meshgrid(leftTop(1):space:rightTop(1), leftTop(2));
[rightEdgeX,rightEdgeY] = meshgrid(rightTop(1), rightBottom(2):space:rightTop(2));
[bottomEdgeX,bottomEdgeY] = meshgrid(leftBottom(1):space:rightBottom(1), leftBottom(2));

Rectangle = [leftEdgeX, leftEdgeY;
             topEdgeX',  topEdgeY';
             rightEdgeX, rightEdgeY;
             bottomEdgeX', bottomEdgeY'];
end
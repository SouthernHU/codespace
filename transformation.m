classdef transformation
    properties
        sdt, % [rx,ry,rz,x,y,z]
        transMatrix_XYZ,% 齐次坐标变换矩阵
        transMatrix_ZYX,
        transMatrix,
        rotMatrix_XYZ,% 旋转矩阵
        rotMatrix_ZYX
    end

    methods

    function obj=transformation(initSdt)
        % SDT
        obj.sdt=initSdt;
        alpha=initSdt(1);beta=initSdt(2);gamma=initSdt(3);u=initSdt(4);v=initSdt(5);w=initSdt(6);
        % 绕各个轴的旋转矩阵
        Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
        Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
        Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
        % 旋转矩阵,此处为外旋，X-Y-Z
        obj.rotMatrix_XYZ=Rz * Ry * Rx;
        % 旋转矩阵,此处为内旋，Z-Y-X
        obj.rotMatrix_ZYX=Rx * Ry * Rz;
        % 齐次坐标变换矩阵
        obj.transMatrix_XYZ=[[obj.rotMatrix_XYZ,[u;v;w]];0,0,0,1];
        obj.transMatrix_ZYX=[[obj.rotMatrix_ZYX,[u;v;w]];0,0,0,1];
        % 默认为X-Y-Z
        obj.transMatrix=obj.transMatrix_XYZ;

    end

    function vector2=calMoved_Transform(obj,vector,seq)
        % 输入:
        %   vector:  nx3数组 | 变换前点云/向量
        %   seq:     旋转顺序| 'Z-Y-X'或'X-Y-Z'，默认'X-Y-Z'
        %输出
        %   vector2: nx3数组 | 变换后的点云/向量
        if nargin < 3
            seq = 'X-Y-Z';  % 如果k未被赋值，则设置默认值为1
        end
        if isequal(seq,'X-Y-Z')
            vector_qici=[vector,ones(size(vector,1),1)];
            temp=(obj.transMatrix_XYZ)*(vector_qici');
            vector2=temp(1:3,:)';
        elseif isequal(seq,'Z-Y-X')
            vector_qici=[vector,ones(size(vector,1),1)];
            temp=(obj.transMatrix_ZYX)*(vector_qici');
            vector2=temp(1:3,:)';
        end

    end

    function vector2=calMoved_SDT(obj,vector)
        % 输入:
        %   vector: nx3数组 | 变换前点云/向量
        % 输出：
        %   vector2: nx3数组 | 移动后点云
        % Creat in:  2022/11/14
        % Update in: 2023/02/25
        localSdt=obj.sdt;
        C=mean(vector);%中心点
        a=localSdt(1);b=localSdt(2);c=localSdt(3);u=localSdt(4);v=localSdt(5);w=localSdt(6);
        r=[a,b,c];t=[u,v,w];
        % 小位移旋量计算
        vector2=zeros(size(vector));
        for i = 1:length(vector(:,1))
            vector2(i,:)=vector(i,:)+t+cross(C-vector(i,:),r);
        end
    end



end

end
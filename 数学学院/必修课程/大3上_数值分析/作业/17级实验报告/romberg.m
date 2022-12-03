function I=romberg(fun,a,b,e)
% 使用龙贝格(Romberg数值求解公式)
% 例如：
% I=romberg(@(x)x^(3/2),0,1,0.000001)
%
% T =
%
%     1 至 3 列
%  
%     0.500000000000000                   0                   0
%     0.426776695296637   0.402368927062182                   0
%     0.407018110857901   0.400431916044989   0.400302781977176
%     0.401812464799974   0.400077249447332   0.400053605007488
%     0.400463401302048   0.400013713469406   0.400009477737544
%     0.400117671209778   0.400002427845688   0.400001675470774
%     0.400029739862529   0.400000429413446   0.400000296184629
%     0.400007491908991   0.400000075924478   0.400000052358547
%  
%     4 至 6 列
%  
%                     0                   0                   0
%                     0                   0                   0
%                     0                   0                   0
%     0.400049649817493                   0                   0
%     0.400008777304688   0.400008617020324                   0
%     0.400001551625270   0.400001523289272   0.400001516355028
%     0.400000274291198   0.400000269282045   0.400000268056232
%     0.400000048488292   0.400000047602790   0.400000047386095
%
%     7 至 8 列
%  
%                     0                   0
%                     0                   0
%                     0                   0
%                     0                   0
%                     0                   0
%                     0                   0
%     0.400000267751397                   0
%     0.400000047332207   0.400000047318753
%
%
% I =
%
%     0.400000047318753
 
% 判断输入参数是否足够
if nargin~=4
    error('请输入需要求积分的f、上界和下界以及误差e')
end
 
k=0; % 迭代次数
n=1; % 区间划分个数
h=b-a; %上下界间距
T(1,1)=h/2*(fun(a)+fun(b));
d=b-a; %误差间距
while e<=d
    k=k+1;
    h=h/2;
    sum=0;
    % 计算第一列T
    for i=1:n
        sum=sum+fun(a+(2*i-1)*h);
    end
    T(k+1,1)=T(k)/2+h*sum;
    % 迭代
    for j=1:k
        T(k+1,j+1)=T(k+1,j)+(T(k+1,j)-T(k,j))/(4^j-1);
    end
    n=n*2;
    d=abs(T(k+1,k+1)-T(k,k));
end
T
I=T(k+1,k+1);
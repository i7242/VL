-- test lua caliper---------------------------------------------------------------------

matrix = require "matrix"
metrology = require "metrology"

D1=matrix{1,1.1,1}
D2=matrix{1,0.9,1}
DPL=5

SDT, CRT=metrology.Caliper_A(D1,D2,DPL)

print('-------SDT---------')
local i=1
while (i<7) do
print(SDT[i])
i=i+1
end
print('-------CRT---------')
print(CRT)
-- -------------------------------------------------------------------------------------


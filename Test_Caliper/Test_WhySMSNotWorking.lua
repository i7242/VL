-- load packages----------------------------
matrix = require "matrix"
metrology = require "metrology"

-- r correspond to "z" direction------------
local num_r=3
local range_r1=-1
local range_r2= 1
local dr=(range_r2-range_r1)/(num_r-1)
-- c correspond to "y" direction------------
local num_c=3
local range_c1=-1
local range_c2= 1
local dc=(range_c2-range_c1)/(num_c-1)
-- measurement points-----------------------
local PTS=matrix(num_r*num_c,3,0)
local count=1
for i=1,num_r do
	pst_r=range_r1+dr*(i-1)
	for j=1,num_c do
		pst_c=range_c1+dc*(j-1)
		PTS[count][1]=0
		PTS[count][2]=pst_c
		PTS[count][3]=pst_r
		count=count+1
	end
end

-- measurement distance D1 by ray-----------
local D1=matrix(num_r*num_c,1,0.001)

D1[2][1]=0.09
D1[4][1]=0.09
D1[6][1]=0.09
D1[8][1]=0.09

print(D1)

local SDT,CRT = metrology.Caliper_A(D1,PTS)
-- finish-----------------------------------




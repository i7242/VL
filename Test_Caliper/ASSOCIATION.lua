-- test for caliper--------------------------------------------------
--[[ using the two outer measuring faces of
     caliper at first.                  ]]--

-- load namespace----------------------------------------------------
package.path = package.path .. ';'
		.. unprojectizeFilename(Caliper_Path.localPath) .. '/?.lua'

-- load packages-----------------------------------------------------
metrology = require "metrology"
matrix = require "matrix"
optimize=require "optimize"

-- criteria to stop association--------------------------------------
--[[	here CRT is the absolute value of
			relative displacement between the caliper and slide.	]]--
local CRT=1
local loop_num=1
local empty=true

--[[	measurement points on each plane
			position is relative to the
			plane center												]]--
-- r correspond to "z" direction
local num_r=8
--	!!! notice the direction for row
--	negetive direction is the longer direction
local range_r1=-0.03
local range_r2= 0.003
local dr=(range_r2-range_r1)/(num_r-1)
-- c correspond to "y" direction
local num_c=4
local range_c1=-0.001
local range_c2= 0.001
local dc=(range_c2-range_c1)/(num_c-1)


-- loop to associate-------------------------------------------------
--while (loop_num<10) do
while ((CRT>1e-5) and (loop_num<10)) do

	
	-- construct measurement points on caliper----------
	--[[	this point coordinate is relative to the
				center of measurement surface
				no relation to the global coordinate	]]--
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

	--[[V_normal_1:	normal direction of the
				first measurement plane it is generated
				by rotation from initial position      ]]--
	local V_normal_1=Vec3(1,0,0)
	local a=Caliper:getOrientation()
	V_normal_1=a:rotate(V_normal_1)
	local V_normal_2=Vec3(0,0,0)-V_normal_1
	
	-- get position of two plane center-----------------
	local	PS1=Caliper:getStagePosition()
	local PS2=Slide:getStagePosition()
	
	-- distance between two planes----------------------
	--	"0.025" is the initial distance, when
	--	the caliper is "closed"
	local DPL=(PS2-PS1)*V_normal_1
	DPL=DPL-0.025
	
	-- print('> > > > loop: '..loop_num..' > > > >')

	-- measurement distance D1 by ray-------------------
	--	give D1 initially a large value
	--	if find no intersection point, using this value
	--	and the constraint will be satisfied
	local D1=matrix(num_r*num_c,1,9e9)
	for i=1,(num_r*num_c) do
		local MPT=Vec3(PTS[i][1],PTS[i][2],PTS[i][3])
		MPT=a:rotate(MPT)
		--	slite move along normal direction, to avoid
		--	"rayIntersect" find no point
		MPT=MPT+PS1-V_normal_1*0.001
		local r = Ray(MPT, V_normal_1)
		local t = rayIntersect(Stage.SceneNode, r, false, true)

		if (t~= nil) then
			local num_t=1
			local num_d=0
			local d1={}
			while (t[num_t]~=nil) do
				if ((t[num_t].Entity.Name~='Caliper')and(t[num_t].Entity.Name~='Slide')) then
					num_d=num_d+1
					--	"-0.001" corresponding to before
					--	a distance was added to avoid empty intersection
					d1[num_d]=t[num_t].IntersectedPoint.distance-0.001
				end
				num_t=num_t+1
			end
			
			if (num_d>0) then
				D1[i][1]=d1[1]
			end
			
			if (num_d>1) then
				for j=2,num_d do
					if (math.abs(d1[j])<math.abs(D1[i][1])) then
						D1[i][1]=d1[j]
					end
				end
			end
			
		end
		
		if (D1[i][1]<DPL) then
			empty=false
		end

	end
	
	-- measurement distance D2 by ray-------------------
	local D2=matrix(num_r*num_c,1,9e9)
	for i=1,(num_r*num_c) do
		local MPT=Vec3(PTS[i][1],PTS[i][2],PTS[i][3])
		MPT=a:rotate(MPT)
		MPT=MPT+PS1+V_normal_1*(DPL+0.001)
		local r = Ray(MPT, V_normal_2)
		local t = rayIntersect(Stage.SceneNode, r, false, true)
		
		if (t~= nil) then
			local num_t=1
			local num_d=0
			local d2={}
			while (t[num_t]~=nil) do
				if ((t[num_t].Entity.Name~='Caliper')and(t[num_t].Entity.Name~='Slide')) then
					num_d=num_d+1
					d2[num_d]=t[num_t].IntersectedPoint.distance-0.001
				end
				num_t=num_t+1
			end
			
			if (num_d>0) then
				D2[i][1]=d2[1]
			end
			
			if (num_d>1) then
				for j=2,num_d do
					if (math.abs(d2[j])<math.abs(D2[i][1])) then
						D2[i][1]=d2[j]
					end
				end
			end
			
		end
		
		if (D2[i][1]<DPL) then
			empty=false
		end

	end

	-- conduct association------------------------------
	if (empty) then
		print('No measurement part !')
		Result=matrix(7,1,0)
		Result[7][1]=DPL
		CRT=0
	else
		Result,CRT = metrology.Caliper_A(D1,D2,DPL,PTS)
	end

	-- translate the position of caliper----------------
	--[[translation corresponding to the left
			contact surface of the caliper.   ]]--
	local tr_c=Vec3(-Result[1][1],Result[2][1],-Result[3][1])
	--[[7th variable of SDT is the changing
			of distance between two faces.
			positive is increasing distance.  ]]--
	local tr_s=Vec3(-Result[7][1],0,0)
	Caliper:translateLocal(tr_c)
	Slide:translateParent(tr_s)
	
	-- rotation the cqliper, Vec3(y,x,z)----------------
	--[[assume the rotation angle is small,
			the SDT is used as
			Eular Angle directly							]]--
	local RTQT=Quat(0,0,0,1)
	RTQT:setEulerAngles(Vec3(-Result[5][1],Result[4][1],-Result[6][1]))
	Caliper:rotateLocal(RTQT)
	
	loop_num=loop_num+1
	
--	print('CRT:',CRT)
	
end

--	Re-calculate the distance between the planes of caliper and slide
--	Result "DPL" and its calculation is the same as before in the "while" loop
	local V_normal_1=Vec3(1,0,0)
	local a=Caliper:getOrientation()
	V_normal_1=a:rotate(V_normal_1)
	
	local	PS1=Caliper:getStagePosition()
	local PS2=Slide:getStagePosition()
	
	local DPL=(PS2-PS1)*V_normal_1
	Distance=(DPL-0.025)*1000
	
	--	output the result
	output('Distance',Distance)

-- finish------------------------------------------------------------



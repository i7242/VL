-- set namespace-----------------------------------------------------
package.path = package.path .. ';'
		.. unprojectizeFilename(Caliper_Path.localPath) .. '/?.lua'

-- load packages-----------------------------------------------------
metrology = require "metrology"
matrix	  = require "matrix"
optimize  = require "optimize"

-- read imput variables----------------------------------------------
--[[
	measurement_type
	measurement_range_lower
	measurement_range_upper
	MT_1: first measurement tool
	MT_2: second measurement tool
]]--

-- Change the method for different measurement types-----------------
if (measurement_type=='Caliper_Outside') then
	-- measure the parts inside the measurement surfaces-------------
	-- MT_1: Caliper
	-- MT_2: Slider

	-- criteria to stop association----------------------------------
	--[[	here CRT is the absolute value of
			relative displacement between the caliper and slide. ]]--
	local CRT=1
	local loop_num=1
	local empty=true

	--[[	N_1:	normal direction of the
			first measurement plane it is generated
			by rotation from initial position     				 ]]--
	local N_1=Vec3(1,0,0)
	local a=MT_1:getOrientation()
		  N_1=a:rotate(N_1)
	local N_2=Vec3(0,0,0)-N_1
	
	-- get position of two plane center------------------------------
	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	
	-- distance between two planes-----------------------------------
	--	"0.025" is the initial distance, when
	--	the caliper is "closed"
	local DPL=(CT_2-CT_1)*N_1
	DPL=DPL-0.025
	-- re-calculate the CT_2 by offsetting
	CT_2=CT_1+N_1*DPL

	--[[	measurement points on each plane
			position is relative to the
			plane center CT_1 and CT_2					         ]]--
	-- correspond to "z" direction
	local num_z=8
	--	!!! notice the direction for row
	--	negetive direction is the longer direction
	local range_z1=-0.03
	local range_z2= 0.003
	local dz=(range_z2-range_z1)/(num_z-1)
	-- correspond to "y" direction
	local num_y=4
	local range_y1=-0.001
	local range_y2= 0.001
	local dy=(range_y2-range_y1)/(num_y-1)

	-- construct measurement points on caliper-----------------------
	-- PTS1 is for the first plane-----------------------------------
	local PTS1=matrix(num_z*num_y,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_y do
			pst_y=range_y1+dy*(j-1)
			PTS1[count][1]=0
			PTS1[count][2]=pst_y
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_y) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_1.x
		PTS1[i][2]=MPT.y+CT_1.y
		PTS1[i][3]=MPT.z+CT_1.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10)) do

		-- measurement distance D1 by ray----------------------------
		--	give D1 initially a large value
		--	if find no intersection point, using this value
		--	and the constraint will be satisfied
		local D1=matrix(num_z*num_y,1,9e9)
		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			--	slite move along normal direction, to avoid
			--	"rayIntersect" find no point
			MPT=MPT-N_1*0.001
			local r = Ray(MPT, N_1)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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
	
		-- measurement distance D2 by ray----------------------------
		local D2=matrix(num_z*num_y,1,9e9)
		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			MPT=MPT+N_1*(DPL+0.001)
			local r = Ray(MPT, N_2)
			local t = rayIntersect(Stage.SceneNode, r, false, true)
			
			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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

		-- conduct association---------------------------------------
		--	transform the variables from "Vec3" to "matrix"
		--	so they could be used in "metrology" package
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N1=matrix{N_1.x,N_1.y,N_1.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		if (empty) then
			print('No measurement part !')
			Result=matrix(7,1,0)
			Result[7][1]=DPL
			CRT=0
		else
			Result,CRT = metrology.Caliper_A(C1,C2,N1,N2,D1,D2,DPL,PTS1)
		end

		-- Translation & Rotation: CT_1,CT_2,N_1,N_2,PTS1------------
		--[[	After each step of association, update the
				position and orientation of the points,
				not the caliper									 ]]--
		-- rotate N_1 & N_2------------------------------------------
		N_1=N_1-N_1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_1:normalize()
		N_2=Vec3(0,0,0)-N_1
		N_2:normalize()
		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_y) do
			-- translation caused by rotation
			local r_PTS1=Vec3((CT_1.x-PTS1[i][1]),(CT_1.y-PTS1[i][2]),(CT_1.z-PTS1[i][3]))
			r_PTS1=r_PTS1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])+r_PTS1
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_1--------------------------------------------
		CT_1=CT_1+Vec3(Result[1][1],Result[2][1],Result[3][1])
		-- translate CT_2--------------------------------------------
		DPL=DPL-Result[7][1]
		CT_2=CT_1+N_1*DPL
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- rotate the caliper--------------------------------------------
	local v_1=Vec3(1,0,0)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
		  v_1:normalize()
	local v_2=N_1
	-- calculate the rotation axis
	local r_axis=v_1:cross(v_2)
		  r_axis:normalize()
	-- translate World rotation axis to Local rotation axis
	local M=MT_1:getLocalFrame()
		  M:invert()
		  r_axis=M*r_axis
		  r_axis:normalize()
	-- calculate the rotation angle
	local r_angle=math.acos(v_1:dot(v_2))

	-- translate the position of caliper-----------------------------
	--[[	translation corresponding to the left
			contact surface of the caliper.						 ]]--
	local tr_caliper=CT_1-MT_1:getStagePosition()
	--[[	the displacement of slide is calculated by
			changing of "DPL"									 ]]--
	local v_1=Vec3(1,0,0)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	local DPL2=(CT_2-CT_1)*v_1-0.025
	local tr_slide=Vec3((DPL-DPL2),0,0)

	local duration = 2
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		local r_qt=Quat(r_axis,r_angle/duration*frametime)
		--[[	if not using local rotation
				additional displacement will be introduced		 ]]--
		MT_1:rotateLocal(r_qt)
		MT_1:translateWorld(tr_caliper/duration*frametime)
		MT_2:translateParent(tr_slide/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end
	
	--	output the result
	Distance=DPL*1000
	output('Distance',Distance)

elseif (measurement_type=='Caliper_Inside') then
	-- measure the parts outside the measurement surfaces------------

	-- criteria to stop association----------------------------------
	--[[	here CRT is the absolute value of
			relative displacement between the caliper and slide. ]]--
	local CRT=1
	local loop_num=1
	local empty=false

	--[[	N_11:	normal direction of the
			first measurement plane it is generated
			by rotation from initial position     				 ]]--
	local N_11=Vec3(-1,0,0)
	-- angle for the rotation of normal direction
	local a_normal=7

	local N_12=Vec3(-math.cos(a_normal),math.sin(a_normal),0)
	local N_13=Vec3(-math.cos(a_normal),-math.sin(a_normal),0)
	-- rotation from original position to current position
	local a=MT_1:getOrientation()
		N_11=a:rotate(N_11)
		N_12=a:rotate(N_12)
		N_13=a:rotate(N_13)
	local N_21=Vec3(0,0,0)-N_11
	local N_22=Vec3(0,0,0)-N_12
	local N_23=Vec3(0,0,0)-N_13
	--[[	dir_cb: direction from part center to the
				center of measurement plane, caliper B			 ]]--
	local dir_cb=Vec3(-0.008,0,-0.08)
		dir_cb=a:rotate(dir_cb)
	
	-- get position of two plane center------------------------------
	local CT_1=MT_1:getStagePosition()
		CT_1=CT_1+dir_cb
	local CT_2=MT_2:getStagePosition()
		CT_2=CT_2+dir_cb
	
	-- distance between two planes-----------------------------------
	--	"0.025" is the initial distance, when
	--	the caliper is "closed"
	local DPL=(CT_2-CT_1)*N_21
		DPL=DPL-0.025
	-- re-calculate the CT_2 by offsetting
		CT_2=CT_1+N_21*DPL

	--[[	measurement points on each plane
			position is relative to the
			plane center CT_1 and CT_2					         ]]--
	-- correspond to "z" direction
	local num_z=8
	--	!!! notice the direction for row
	--	negetive direction is the longer direction
	local range_z1=-0.003
	local range_z2= 0.003
	local dz=(range_z2-range_z1)/(num_z-1)
	-- correspond to "y" direction
	local num_y=4
	local range_y1=-0.00001
	local range_y2= 0.00001
	local dy=(range_y2-range_y1)/(num_y-1)

	-- construct measurement points on caliper-----------------------
	-- PTS1 is for the first plane-----------------------------------
	local PTS1=matrix(num_z*num_y,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_y do
			pst_y=range_y1+dy*(j-1)
			PTS1[count][1]=0
			PTS1[count][2]=pst_y
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_y) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_1.x
		PTS1[i][2]=MPT.y+CT_1.y
		PTS1[i][3]=MPT.z+CT_1.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10)) do

		-- measurement distance D1 by ray----------------------------
		--	give D1 initially a large value
		--	if find no intersection point, using this value
		--	and the constraint will be satisfied
		local D1=matrix(num_z*num_y*3,1,9e9)

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			--	slite move along normal direction, to avoid
			--	"rayIntersect" find no point
				MPT=MPT-N_11*0.001
			local r = Ray(MPT, N_11)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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
		end

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			--	slite move along normal direction, to avoid
			--	"rayIntersect" find no point
			MPT=MPT-N_12*0.001
			local r = Ray(MPT, N_12)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
						num_d=num_d+1
						--	"-0.001" corresponding to before
						--	a distance was added to avoid empty intersection
						d1[num_d]=t[num_t].IntersectedPoint.distance-0.001
					end
					num_t=num_t+1
				end
			
				if (num_d>0) then
					D1[num_z*num_y+i][1]=d1[1]
				end
				
				if (num_d>1) then
					for j=2,num_d do
						if (math.abs(d1[j])<math.abs(D1[num_z*num_y+i][1])) then
							D1[num_z*num_y+i][1]=d1[j]
						end
					end
				end

			end
		end

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			--	slite move along normal direction, to avoid
			--	"rayIntersect" find no point
			MPT=MPT-N_13*0.001
			local r = Ray(MPT, N_13)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
						num_d=num_d+1
						--	"-0.001" corresponding to before
						--	a distance was added to avoid empty intersection
						d1[num_d]=t[num_t].IntersectedPoint.distance-0.001
					end
					num_t=num_t+1
				end
			
				if (num_d>0) then
					D1[num_z*num_y*2+i][1]=d1[1]
				end
				
				if (num_d>1) then
					for j=2,num_d do
						if (math.abs(d1[j])<math.abs(D1[num_z*num_y*2+i][1])) then
							D1[num_z*num_y*2+i][1]=d1[j]
						end
					end
				end
			
			end
		end
	
		-- measurement distance D2 by ray----------------------------
		local D2=matrix(num_z*num_y*3,1,9e9)

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			MPT=MPT+N_21*DPL-N_21*0.001
			local r = Ray(MPT, N_21)
			local t = rayIntersect(Stage.SceneNode, r, false, true)
			
			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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
		end

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			MPT=MPT+N_21*DPL-N_22*0.001
			local r = Ray(MPT, N_22)
			local t = rayIntersect(Stage.SceneNode, r, false, true)
			
			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
						num_d=num_d+1
						d2[num_d]=t[num_t].IntersectedPoint.distance-0.001
					end
					num_t=num_t+1
				end
				
				if (num_d>0) then
					D2[num_z*num_y+i][1]=d2[1]
				end
				
				if (num_d>1) then
					for j=2,num_d do
						if (math.abs(d2[j])<math.abs(D2[num_z*num_y+i][1])) then
							D2[num_z*num_y+i][1]=d2[j]
						end
					end
				end
				
			end
		end

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			MPT=MPT+N_21*DPL-N_23*0.001
			local r = Ray(MPT, N_23)
			local t = rayIntersect(Stage.SceneNode, r, false, true)
			
			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
						num_d=num_d+1
						d2[num_d]=t[num_t].IntersectedPoint.distance-0.001
					end
					num_t=num_t+1
				end
				
				if (num_d>0) then
					D2[num_z*num_y*2+i][1]=d2[1]
				end
				
				if (num_d>1) then
					for j=2,num_d do
						if (math.abs(d2[j])<math.abs(D2[num_z*num_y*2+i][1])) then
							D2[num_z*num_y*2+i][1]=d2[j]
						end
					end
				end
				
			end
		end

		-- conduct association---------------------------------------
		--	transform the variables from "Vec3" to "matrix"
		--	so they could be used in "metrology" package
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N11=matrix{N_11.x,N_11.y,N_11.z}
		local N12=matrix{N_12.x,N_12.y,N_12.z}
		local N13=matrix{N_13.x,N_13.y,N_13.z}
		local N21=matrix{N_21.x,N_21.y,N_21.z}
		local N22=matrix{N_22.x,N_22.y,N_22.z}
		local N23=matrix{N_23.x,N_23.y,N_23.z}

		local min_D1=9e9
		for i=1,(num_z*num_y*3) do
			if (D1[i][1]<min_D1) then
				min_D1=D1[i][1]
			end
		end
		local min_D2=9e9
		for i=1,(num_z*num_y*3) do
			if (D2[i][1]<min_D2) then
				min_D2=D2[i][1]
			end
		end
		if ((min_D1+min_D2+DPL)>measurement_range_upper) then
			empty=true
		end

		if (empty) then
			print('Parts are beyond the measurement range !')
			Result=matrix(7,1,0)
			Result[7][1]=-DPL
			CRT=0
		else
			Result,CRT = metrology.Caliper_B(C1,C2,N11,N12,N13,N21,N22,N23,D1,D2,DPL,PTS1)
		end

		-- Translation & Rotation: CT_1,CT_2,N_1,N_2,PTS1------------
		--[[	After each step of association, update the
				position and orientation of the points,
				not the caliper									 ]]--
		-- rotate N_1 & N_2------------------------------------------
		N_11=N_11-N_11:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_11:normalize()
		N_21=Vec3(0,0,0)-N_11
		N_21:normalize()
		-- rotate N_1 & N_2------------------------------------------
		N_12=N_12-N_12:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_12:normalize()
		N_22=Vec3(0,0,0)-N_12
		N_22:normalize()
		-- rotate N_1 & N_2------------------------------------------
		N_13=N_13-N_13:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_13:normalize()
		N_23=Vec3(0,0,0)-N_13
		N_23:normalize()
		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_y) do
			-- translation caused by rotation
			local r_PTS1=Vec3((CT_1.x-PTS1[i][1]),(CT_1.y-PTS1[i][2]),(CT_1.z-PTS1[i][3]))
			r_PTS1=r_PTS1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])+r_PTS1
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_1--------------------------------------------
		CT_1=CT_1+Vec3(Result[1][1],Result[2][1],Result[3][1])
		-- translate CT_2--------------------------------------------
		DPL=DPL+Result[7][1]
		CT_2=CT_1+N_21*DPL
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- rotate the caliper--------------------------------------------
	local v_1=Vec3(-1,0,0)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
		  v_1:normalize()
	local v_2=N_11
	-- calculate the rotation axis
	local r_axis=v_1:cross(v_2)
		  r_axis:normalize()
	-- translate World rotation axis to Local rotation axis
	local M=MT_1:getLocalFrame()
		  M:invert()
		  r_axis=M*r_axis
		  r_axis:normalize()
	-- calculate the rotation angle
	local r_angle=math.acos(v_1:dot(v_2))

	-- translate the position of caliper-----------------------------
	--[[	translation corresponding to the left
			contact surface of the caliper
			this considered:
				1. changing of measurement surface
				2. displacement caused by rotation				 ]]--
	local tr_caliper=CT_1-MT_1:getStagePosition()-dir_cb+dir_cb:cross(r_axis*r_angle)
	--[[	the displacement of slide is calculated by
			changing of "DPL"									 ]]--
	local v_1=Vec3(-1,0,0)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	local DPL2=(CT_1-CT_2)*v_1-0.025
	local tr_slide=Vec3((DPL-DPL2),0,0)

	local duration = 2
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		local r_qt=Quat(r_axis,r_angle/duration*frametime)
		--[[	if not using local rotation
				additional displacement will be introduced		 ]]--
		MT_1:rotateLocal(r_qt)
		MT_1:translateWorld(tr_caliper/duration*frametime)
		MT_2:translateParent(tr_slide/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end
	
	--	output the result
	Distance=DPL*1000
	output('Distance',Distance)

elseif (measurement_type=='Caliper_Shoulder') then

	local CRT=9e9
	local loop_num=1
	local empty=false

	local N_1=Vec3(-1,0,0)
	local N_2=Vec3(-1,0,0)
	
	local a=MT_1:getOrientation()
		N_1=a:rotate(N_1)
		N_2=a:rotate(N_2)
	--[[	dir_c1: direction from part center to the
				center of measurement plane			 				
			dir_c3: samilier to dir_c1							 ]]--
	local dir_c1=Vec3(-0.0157,0.002,-0.055)
	local dir_c3=Vec3(0,-0.006,0)
		dir_c1=a:rotate(dir_c1)
		dir_c3=a:rotate(dir_c3)

	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	
	local DPL=(CT_1-CT_2)*N_1
		DPL=DPL-0.025

	-- dir_c2: samilier to dir_c1	
	dir_c2=dir_c1+dir_c3-N_2*DPL
	
	CT_2=CT_1+dir_c2
	CT_1=CT_1+dir_c1

	local num_z=8
	
	local range_z1=-0.004
	local range_z2= 0.004
	local dz=(range_z2-range_z1)/(num_z-1)
	
	local num_y=6
	local range_y1=-0.002
	local range_y2= 0.002
	local dy=(range_y2-range_y1)/(num_y-1)

	local PTS1=matrix(num_z*num_y,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_y do
			pst_y=range_y1+dy*(j-1)
			PTS1[count][1]=0
			PTS1[count][2]=pst_y
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_y) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_2.x
		PTS1[i][2]=MPT.y+CT_2.y
		PTS1[i][3]=MPT.z+CT_2.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10)) do

		local D2=matrix(num_z*num_y,1,9e9)

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
				MPT=MPT-N_2*0.001
			local r = Ray(MPT, N_2)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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
		end

		-- conduct association---------------------------------------
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N1=matrix{N_1.x,N_1.y,N_1.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		Result, Objective = metrology.Caliper_C2(C2,N2,D2,PTS1)
		
		if (loop_num==1) then
			CRT=Objective
		else
			CRT=math.abs(CRT-Objective)
		end

		N_2=N_2-N_2:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_2:normalize()
		N_1=N_1-N_1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_1:normalize()

		dir_c3=dir_c3-dir_c3:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))

		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_y) do
			-- translation caused by rotation
			local r_PTS1=Vec3((CT_2.x-PTS1[i][1]),(CT_2.y-PTS1[i][2]),(CT_2.z-PTS1[i][3]))
			r_PTS1=r_PTS1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])+r_PTS1
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_2--------------------------------------------
		CT_2=CT_2+Vec3(Result[1][1],Result[2][1],Result[3][1])
		-- translate CT_1--------------------------------------------
		CT_1=CT_2+N_2*DPL-dir_c3
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- rotate the caliper--------------------------------------------
	local v_1=Vec3(-1,0,0)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
		  v_1:normalize()
	local v_2=N_2
	-- calculate the rotation axis
	local r_axis=v_1:cross(v_2)
		  r_axis:normalize()
	-- translate World rotation axis to Local rotation axis
	local M=MT_1:getLocalFrame()
		  M:invert()
		  r_axis=M*r_axis
		  r_axis:normalize()
	-- calculate the rotation angle
	local r_angle=math.acos(v_1:dot(v_2))

	-- translate the position of caliper-----------------------------
	local tr_caliper=CT_1-MT_1:getStagePosition()-dir_c1+dir_c1:cross(r_axis*r_angle)
	if (math.abs(tr_caliper.x)>measurement_range_upper) then
		empty=true
	end
	if (math.abs(tr_caliper.y)>measurement_range_upper) then
		empty=true
	end
	if (math.abs(tr_caliper.z)>measurement_range_upper) then
		empty=true
	end

	if (empty==true) then
		tr_caliper=Vec3(0,0,0)
		r_angle=0
		print('Too far from measurement part !')
	end

	local duration = 1
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		local r_qt=Quat(r_axis,r_angle/duration*frametime)
		--[[	if not using local rotation
				additional displacement will be introduced		 ]]--
		MT_1:rotateLocal(r_qt)
		MT_1:translateWorld(tr_caliper/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end

	-- Second step---------------------------------------------------
	local CRT=9e9
	local loop_num=1

	local N_1=Vec3(-1,0,0)
	local N_2=Vec3(-1,0,0)
	
	local a=MT_1:getOrientation()
		N_1=a:rotate(N_1)
		N_2=a:rotate(N_2)

	local dir_c1=Vec3(-0.0157,0.002,-0.055)
	local dir_c3=Vec3(0,-0.006,0)
		dir_c1=a:rotate(dir_c1)
		dir_c3=a:rotate(dir_c3)

	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	
	local DPL=(CT_1-CT_2)*N_1
		DPL=DPL-0.025

	-- dir_c2
	dir_c2=dir_c1+dir_c3-N_2*DPL
	
	CT_2=CT_1+dir_c2
	CT_1=CT_1+dir_c1

	local num_z=8
	
	local range_z1=-0.004
	local range_z2= 0.004
	local dz=(range_z2-range_z1)/(num_z-1)
	
	local num_y=6
	local range_y1=-0.002
	local range_y2= 0.002
	local dy=(range_y2-range_y1)/(num_y-1)

	local PTS1=matrix(num_z*num_y,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_y do
			pst_y=range_y1+dy*(j-1)
			PTS1[count][1]=0
			PTS1[count][2]=pst_y
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_y) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_1.x
		PTS1[i][2]=MPT.y+CT_1.y
		PTS1[i][3]=MPT.z+CT_1.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10) and (empty==false)) do

		local D1=matrix(num_z*num_y,1,9e9)

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
				MPT=MPT-N_1*0.001
			local r = Ray(MPT, N_1)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
						num_d=num_d+1
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
		end

		-- conduct association---------------------------------------
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N1=matrix{N_1.x,N_1.y,N_1.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		Result, Objective = metrology.Caliper_C1(C1,N1,D1,PTS1)
		
		if (loop_num==1) then
			CRT=Objective
		else
			CRT=math.abs(CRT-Objective)
		end

		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_y) do
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_1--------------------------------------------
		CT_1=CT_1+Vec3(Result[1][1],Result[2][1],Result[3][1])
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- translate the position of caliper-----------------------------
	local tr_caliper=CT_1-MT_1:getStagePosition()-dir_c1
	local tr_slide=Vec3(tr_caliper:dot(N_1),0,0)
	if ((tr_caliper:dot(N_1)+DPL)>measurement_range_upper) then
		empty=true
	end

	if (empty==true) then
		tr_caliper=Vec3(0,0,0)
		tr_slide=Vec3(0,0,0)
		print('Beyond measurement range !')
	end
	
	local duration = 1
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		MT_2:translateLocal(tr_slide/duration*frametime)
		MT_1:translateWorld(tr_caliper/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end

	--	output the result
	Distance=DPL*1000
	output('Distance',Distance)

elseif (measurement_type=='Caliper_Depth') then

	local CRT=9e9
	local loop_num=1
	local empty=false

	local N_1=Vec3(1,0,0)
	local N_2=Vec3(1,0,0)
	
	local a=MT_1:getOrientation()
		N_1=a:rotate(N_1)
		N_2=a:rotate(N_2)
	--[[	dir_c1: direction from part center to the
				center of measurement plane			 				
			dir_c3: samilier to dir_c1							 ]]--
	local dir_c1=Vec3(0.2746,0.005,-0.055)
	local dir_c3=Vec3(0,-0.006,0)
		dir_c1=a:rotate(dir_c1)
		dir_c3=a:rotate(dir_c3)

	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	
	local DPL=(CT_2-CT_1)*N_1
		DPL=DPL-0.025

	-- dir_c2: samilier to dir_c1	
	dir_c2=dir_c1+dir_c3+N_1*DPL
	
	CT_2=CT_1+dir_c2
	CT_1=CT_1+dir_c1

	local num_z=8
	
	local range_z1=-0.004
	local range_z2= 0.004
	local dz=(range_z2-range_z1)/(num_z-1)
	
	local num_y=6
	local range_y1=-0.002
	local range_y2= 0.002
	local dy=(range_y2-range_y1)/(num_y-1)

	local PTS1=matrix(num_z*num_y,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_y do
			pst_y=range_y1+dy*(j-1)
			PTS1[count][1]=0
			PTS1[count][2]=pst_y
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_y) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_1.x
		PTS1[i][2]=MPT.y+CT_1.y
		PTS1[i][3]=MPT.z+CT_1.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10)) do

		local D1=matrix(num_z*num_y,1,9e9)

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
				MPT=MPT-N_1*0.001
			local r = Ray(MPT, N_1)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
						num_d=num_d+1
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
		end

		-- conduct association---------------------------------------
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N1=matrix{N_1.x,N_1.y,N_1.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		Result, Objective = metrology.Caliper_C2(C1,N1,D1,PTS1)
		
		if (loop_num==1) then
			CRT=Objective
		else
			CRT=math.abs(CRT-Objective)
		end

		N_2=N_2-N_2:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_2:normalize()
		N_1=N_1-N_1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_1:normalize()

		dir_c3=dir_c3-dir_c3:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))

		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_y) do
			-- translation caused by rotation
			local r_PTS1=Vec3((CT_1.x-PTS1[i][1]),(CT_1.y-PTS1[i][2]),(CT_1.z-PTS1[i][3]))
			r_PTS1=r_PTS1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])+r_PTS1
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_1--------------------------------------------
		CT_1=CT_1+Vec3(Result[1][1],Result[2][1],Result[3][1])
		-- translate CT_2--------------------------------------------
		CT_2=CT_1+N_1*DPL+dir_c3
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- rotate the caliper--------------------------------------------
	local v_1=Vec3(1,0,0)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
		  v_1:normalize()
	local v_2=N_1
	-- calculate the rotation axis
	local r_axis=v_1:cross(v_2)
		  r_axis:normalize()
	-- translate World rotation axis to Local rotation axis
	local M=MT_1:getLocalFrame()
		  M:invert()
		  r_axis=M*r_axis
		  r_axis:normalize()
	-- calculate the rotation angle
	local r_angle=math.acos(v_1:dot(v_2))

	-- translate the position of caliper-----------------------------
	local tr_caliper=CT_1-MT_1:getStagePosition()-dir_c1+dir_c1:cross(r_axis*r_angle)
	if (math.abs(tr_caliper.x)>measurement_range_upper) then
		empty=true
	end
	if (math.abs(tr_caliper.y)>measurement_range_upper) then
		empty=true
	end
	if (math.abs(tr_caliper.z)>measurement_range_upper) then
		empty=true
	end

	if (empty==true) then
		tr_caliper=Vec3(0,0,0)
		r_angle=0
		print('Too far from measurement part !')
	end

	local duration = 1
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		local r_qt=Quat(r_axis,r_angle/duration*frametime)
		--[[	if not using local rotation
				additional displacement will be introduced		 ]]--
		MT_1:rotateLocal(r_qt)
		MT_1:translateWorld(tr_caliper/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end

	-- Second step---------------------------------------------------
	local CRT=9e9
	local loop_num=1

	local N_1=Vec3(1,0,0)
	local N_2=Vec3(1,0,0)
	
	local a=MT_1:getOrientation()
		N_1=a:rotate(N_1)
		N_2=a:rotate(N_2)
	
	local dir_c1=Vec3(0.2746,0.005,-0.055)
	local dir_c3=Vec3(0,-0.006,0)
		dir_c1=a:rotate(dir_c1)
		dir_c3=a:rotate(dir_c3)

	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	
	local DPL=(CT_2-CT_1)*N_1
		DPL=DPL-0.025

	-- dir_c2
	dir_c2=dir_c1+dir_c3+N_1*DPL
	
	CT_2=CT_1+dir_c2
	CT_1=CT_1+dir_c1

	local num_z=2
	
	local range_z1=-0.001
	local range_z2= 0.001
	local dz=(range_z2-range_z1)/(num_z-1)
	
	local num_y=2
	local range_y1=-0.001
	local range_y2= 0.001
	local dy=(range_y2-range_y1)/(num_y-1)

	local PTS1=matrix(num_z*num_y,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_y do
			pst_y=range_y1+dy*(j-1)
			PTS1[count][1]=0
			PTS1[count][2]=pst_y
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_y) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_2.x
		PTS1[i][2]=MPT.y+CT_2.y
		PTS1[i][3]=MPT.z+CT_2.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10) and (empty==false)) do

		local D2=matrix(num_z*num_y,1,9e9)

		for i=1,(num_z*num_y) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
				MPT=MPT-N_2*0.001
			local r = Ray(MPT, N_2)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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
		end

		-- conduct association---------------------------------------
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N1=matrix{N_1.x,N_1.y,N_1.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		Result, Objective = metrology.Caliper_C1(C2,N2,D2,PTS1)
		
		if (loop_num==1) then
			CRT=Objective
		else
			CRT=math.abs(CRT-Objective)
		end

		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_y) do
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_2--------------------------------------------
		CT_2=CT_2+Vec3(Result[1][1],Result[2][1],Result[3][1])
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- translate the position of caliper-----------------------------
	local tr_slide=Vec3(N_2:dot(CT_2-MT_1:getStagePosition()-dir_c2),0,0)
	if ((tr_slide+DPL)>measurement_range_upper) then
		empty=true
	end

	if (empty==true) then
		tr_slide=Vec3(0,0,0)
		print('Too far from measurement part !')
	end
	
	local duration = 1
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		MT_2:translateLocal(tr_slide/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end

	--	output the result
	Distance=DPL*1000
	output('Distance',Distance)

elseif (measurement_type=='MicroMeter') then
	-- measure the parts inside the measurement surfaces-------------

	-- criteria to stop association----------------------------------
	--[[	here CRT is the absolute value of
			relative displacement of micro meter				 ]]--
	local CRT=1
	local loop_num=1
	local empty=true

	--[[	N_1:	normal direction of the
			first measurement plane it is generated
			by rotation from initial position     				 ]]--
	local N_1=Vec3(0,0,-1)
	local a=MT_1:getOrientation()
		  N_1=a:rotate(N_1)
	local N_2=Vec3(0,0,0)-N_1
	
	-- get position of two plane center------------------------------
	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	
	-- distance between two planes-----------------------------------
	local DPL=(CT_2-CT_1)*N_1
	-- re-calculate the CT_2 by offsetting
		  CT_2=CT_1+N_1*DPL

	--[[	measurement points on each plane
			position is relative to the
			plane center CT_1 and CT_2					         ]]--
	-- correspond to radius
	local num_r=3
	--	!!! notice the direction for row
	--	negetive direction is the longer direction
	local range_r1= 0
	local range_r2= 0.003
	local dr=(range_r2-range_r1)/num_r
	-- correspond to angle
	local num_a=5
	local range_a1= 0
	local range_a2= math.pi*2
	local da=(range_a2-range_a1)/num_a

	-- construct measurement points on caliper-----------------------
	-- PTS1 is for the first plane-----------------------------------
	local PTS1=matrix(num_r*num_a,3,0)
	local count=1
	for i=1,num_r do
		rds=dr*i
		for j=1,num_a do
			agl=da*j
			PTS1[count][1]=math.cos(agl)*rds
			PTS1[count][2]=math.sin(agl)*rds
			PTS1[count][3]=0
			count=count+1
		end
	end
	for i=1,(num_r*num_a) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_1.x
		PTS1[i][2]=MPT.y+CT_1.y
		PTS1[i][3]=MPT.z+CT_1.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10)) do

		-- measurement distance D1 by ray----------------------------
		--	give D1 initially a large value
		--	if find no intersection point, using this value
		--	and the constraint will be satisfied
		local D1=matrix(num_r*num_a,1,9e9)
		for i=1,(num_r*num_a) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			--	slite move along normal direction, to avoid
			--	"rayIntersect" find no point
			MPT=MPT-N_1*0.001
			local r = Ray(MPT, N_1)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d1={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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
	
		-- measurement distance D2 by ray----------------------------
		local D2=matrix(num_r*num_a,1,9e9)
		for i=1,(num_r*num_a) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
			MPT=MPT+N_1*(DPL+0.001)
			local r = Ray(MPT, N_2)
			local t = rayIntersect(Stage.SceneNode, r, false, true)
			
			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if ((t[num_t].Entity~=MT_1)and(t[num_t].Entity~=MT_2)) then
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

		-- conduct association---------------------------------------
		--	transform the variables from "Vec3" to "matrix"
		--	so they could be used in "metrology" package
		local C1=matrix{CT_1.x,CT_1.y,CT_1.z}
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N1=matrix{N_1.x,N_1.y,N_1.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		if (empty) then
			print('No measurement part !')
			Result=matrix(7,1,0)
			Result[7][1]=DPL-measurement_range_lower
			CRT=0
		else
			Result,CRT = metrology.Caliper_A(C1,C2,N1,N2,D1,D2,DPL,PTS1)
		end

		-- Translation & Rotation: CT_1,CT_2,N_1,N_2,PTS1------------
		--[[	After each step of association, update the
				position and orientation of the points			 ]]--
		-- rotate N_1 & N_2------------------------------------------
		N_1=N_1-N_1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
		N_1:normalize()
		N_2=Vec3(0,0,0)-N_1
		N_2:normalize()
		-- translate PTS1--------------------------------------------
		for i=1,(num_r*num_a) do
			-- translation caused by rotation
			local r_PTS1=Vec3((CT_1.x-PTS1[i][1]),(CT_1.y-PTS1[i][2]),(CT_1.z-PTS1[i][3]))
			r_PTS1=r_PTS1:cross(Vec3(Result[4][1],Result[5][1],Result[6][1]))
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])+r_PTS1
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_1--------------------------------------------
		CT_1=CT_1+Vec3(Result[1][1],Result[2][1],Result[3][1])
		-- translate CT_2--------------------------------------------
		DPL=DPL-Result[7][1]
		CT_2=CT_1+N_1*DPL
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- rotate the micro meter----------------------------------------
	local v_1=Vec3(0,0,-1)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
		  v_1:normalize()
	local v_2=N_1
	-- calculate the rotation axis
	local r_axis=v_1:cross(v_2)
		  r_axis:normalize()
	-- translate World rotation axis to Local rotation axis
	local M=MT_1:getLocalFrame()
		  M:invert()
		  r_axis=M*r_axis
		  r_axis:normalize()
	-- calculate the rotation angle
	local r_angle=math.acos(v_1:dot(v_2))

	-- translate the position of micro meter-------------------------
	--[[	translation corresponding to the left
			contact surface 									 ]]--
	local tr_MM1=CT_1-MT_1:getStagePosition()
	--[[	the displacement of MM2 is calculated by
			changing of "DPL"									 ]]--
	local v_1=Vec3(0,0,-1)
	local a=MT_1:getOrientation()
		  v_1=a:rotate(v_1)
	local CT_1=MT_1:getStagePosition()
	local CT_2=MT_2:getStagePosition()
	local DPL2=(CT_2-CT_1)*v_1
	local tr_MM2=Vec3(0,0,-(DPL-DPL2))

	if ((DPL<measurement_range_lower) and (empty==false)) then
		r_angle=0
		tr_MM1=Vec3(0,0,0)
		tr_MM2=Vec3(0,0,0)
		print('Beyond measurement range ! The part is too small !')
	end

	local duration = 2
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		local r_qt=Quat(r_axis,r_angle/duration*frametime)
		--[[	if not using local rotation
				additional displacement will be introduced		 ]]--
		MT_1:rotateLocal(r_qt)
		MT_1:translateWorld(tr_MM1/duration*frametime)
		MT_2:translateParent(tr_MM2/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end
	
	--	output the result
	Distance=DPL*1000
	output('Distance',Distance)
	
elseif (measurement_type=='DialIndicator') then
	local CRT=9e9
	local loop_num=1
	local empty=false

	local CT_2=MT_2:getStagePosition()
	local a=MT_2:getOrientation()
	local N_2=Vec3(0,-1,0)
		N_2=a:rotate(N_2)
	local dir_c2=Vec3(0,-0.0003,0)
		dir_c2=a:rotate(dir_c2)
	CT_2=CT_2+dir_c2

	local num_z=2
	local range_z1=-0.0001
	local range_z2= 0.0001
	local dz=(range_z2-range_z1)/(num_z-1)
	
	local num_x=2
	local range_x1=-0.0001
	local range_x2= 0.0001
	local dx=(range_x2-range_x1)/(num_x-1)

	local PTS1=matrix(num_z*num_x,3,0)
	local count=1
	for i=1,num_z do
		pst_z=range_z1+dz*(i-1)
		for j=1,num_x do
			pst_x=range_x1+dx*(j-1)
			PTS1[count][1]=pst_x
			PTS1[count][2]=0
			PTS1[count][3]=pst_z
			count=count+1
		end
	end
	for i=1,(num_z*num_x) do
		local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
		MPT=a:rotate(MPT)
		PTS1[i][1]=MPT.x+CT_2.x
		PTS1[i][2]=MPT.y+CT_2.y
		PTS1[i][3]=MPT.z+CT_2.z
	end

	-- loop to associate---------------------------------------------
	while ((CRT>1e-5) and (loop_num<10)) do

		local D2=matrix(num_z*num_x,1,9e9)

		for i=1,(num_z*num_x) do
			local MPT=Vec3(PTS1[i][1],PTS1[i][2],PTS1[i][3])
				MPT=MPT-N_2*0.001
			local r = Ray(MPT, N_2)
			local t = rayIntersect(Stage.SceneNode, r, false, true)

			if (t~= nil) then
				local num_t=1
				local num_d=0
				local d2={}
				while (t[num_t]~=nil) do
					if (t[num_t].Entity~=MT_2) then
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
		end

		-- conduct association---------------------------------------
		local C2=matrix{CT_2.x,CT_2.y,CT_2.z}
		local N2=matrix{N_2.x,N_2.y,N_2.z}

		Result, Objective = metrology.Caliper_C1(C2,N2,D2,PTS1)
		
		if (loop_num==1) then
			CRT=Objective
		else
			CRT=math.abs(CRT-Objective)
		end

		-- translate PTS1--------------------------------------------
		for i=1,(num_z*num_x) do
			-- sum of translation for each point
			local tr_PTS1=Vec3(Result[1][1],Result[2][1],Result[3][1])
			-- translate
			PTS1[i][1]=PTS1[i][1]+tr_PTS1.x
			PTS1[i][2]=PTS1[i][2]+tr_PTS1.y
			PTS1[i][3]=PTS1[i][3]+tr_PTS1.z
		end
		-- translate CT_1--------------------------------------------
		CT_2=CT_2+Vec3(Result[1][1],Result[2][1],Result[3][1])
		
		-- mark the number of loop-----------------------------------
		loop_num=loop_num+1
	end

	-- translate the position of Dial Indicator2---------------------
	local tr_value=N_2:dot(CT_2-MT_2:getStagePosition()-dir_c2)
	local tr_DialIndicator2=Vec3(0,0,-tr_value)
	if (tr_value>measurement_range_upper) then
		empty=true
		tr_DialIndicator2=Vec3(0,0,0)
		print('Too far from measurement part !')
	end
	
	local duration = 1
	local time = 0
	while (time<duration) do
		local frametime=Simulation:getFrameTime()
		MT_2:translateLocal(tr_DialIndicator2/duration*frametime)
		time=time+frametime
		waitAndUpdate()
	end

	--	output the result
	Distance=tr_value*1000
	output('Distance',Distance)
end

-- finish------------------------------------------------------------


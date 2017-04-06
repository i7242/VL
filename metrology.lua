-- Metrology Module---------------------------------
metrology = {}

-- Function Associate-------------------------------
function metrology.Caliper_A(D1,D2,DPL)
    -- Caliper_A:-----------------------------------
    --  conduct measurement by two measurement planes
    --  the size between these planes are evaluated
    -- ---------------------------------------------

    -- Inputs:--------------------------------------
    --  D1: distance between 1st measurement plane & skin model shape
    --  D2: distance between 2nd measurement plane & skin model shape
    --  DPL: distance between 1st & 2nd plane
    -- ---------------------------------------------
    
    -- Define the number of "contact points"--------
     local n_ctpts=3
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local C1=matrix{0,0,0}
    local N1=matrix{-1,0,0}
        -- Problem!!! the difference makes rotation SDT need to be amplified
        -- the modification should be at both place at the same time
    local P1=matrix{{0,1,0},{0,-1,0},{0,0,1}}
    -- ---------------------------------------------

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local C2=matrix{-DPL,0,0}
    local N2=matrix{1,0,0}
        -- Problem!!! the difference makes rotation SDT need to be amplified
        -- the modification should be at both place at the same time
    local P2=matrix{{-DPL,1,0},{-DPL,-1,0},{-DPL,0,1}}
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx,Ty,Tz,Rx,Ry,Rz,DA,DF}
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  DA: change of distance between 2 measurement planes
    --  DF: optimization objective value (maximum distance between plane and points)
    -- For each plane, there are:
    --  1. matrial condition
    --  2. objective equation
    -- And there are two planes
    -- So the row number of COE is:
    --      2*2*n_ctpts
    -- "COE": coefficients of linear equations
    local COE=matrix(4*n_ctpts,8,0)
    -- "COEC": constant value part of linear equations
    local COEC=matrix(4*n_ctpts,1,0)
    -- This is for the 1st measurement plane--------
    -- Including:
    --  1. the material conditions
    --  2. the optimization objective
    for i=1,n_ctpts do
        -- material conditions----------------------
        COE[i][1]=N1[1][1]
        COE[i][2]=N1[2][1]
        COE[i][3]=N1[3][1]
        -- objective--------------------------------
        COE[i+n_ctpts][1]=-COE[i][1]
        COE[i+n_ctpts][2]=-COE[i][2]
        COE[i+n_ctpts][3]=-COE[i][3]
        -- material conditions----------------------
        r=matrix{P1[i][1],P1[i][2],P1[i][3]}+N1*D1[i][1]
        r=r:cross(N1)
        COE[i][4]=r[1][1]
        COE[i][5]=r[2][1]
        COE[i][6]=r[3][1]
        -- objective--------------------------------
        COE[i+n_ctpts][4]=-COE[i][4]
        COE[i+n_ctpts][5]=-COE[i][5]
        COE[i+n_ctpts][6]=-COE[i][6]
        COE[i+n_ctpts][8]=-1
        -- -----------------------------------------
        COEC[i][1]=D1[i][1]
        -- -----------------------------------------
        COEC[i+n_ctpts][1]=-D1[i][1]
        -- -----------------------------------------
    end
    -- This is for the 2nd plane--------------------
    -- similar to the first one
    -- but the changing of distance between 2 planes
    -- are considered
    for i=2*n_ctpts+1,3*n_ctpts do
        -- -----------------------------------------
        r=matrix{P2[i-2*n_ctpts][1],P2[i-2*n_ctpts][2],P2[i-2*n_ctpts][3]}+N2*D2[i-2*n_ctpts][1]-C2
        r=r:cross(N2)
        r=r-N2:cross(C2)
        COE[i][4]=r[1][1]
        COE[i][5]=r[2][1]
        COE[i][6]=r[3][1]
        COE[i][7]=-1
        -- -----------------------------------------
        COE[i+n_ctpts][4]=-COE[i][4]
        COE[i+n_ctpts][5]=-COE[i][5]
        COE[i+n_ctpts][6]=-COE[i][6]
        COE[i+n_ctpts][7]=1
        COE[i+n_ctpts][8]=-1
        -- -----------------------------------------
        COEC[i][1]=D2[i-2*n_ctpts][1]
        -- -----------------------------------------
        COEC[i+n_ctpts][1]=-D2[i-2*n_ctpts][1]
        -- -----------------------------------------
    end

    -- Using Lua Simplex----------------------------
    local luasimplex = require("luasimplex")
    local rsm = require("luasimplex.rsm")
    -- ---------------------------------------------
    elm={}
    n=1
    for i=1,4*n_ctpts do
        for j=1,8 do
            elm[n]=COE[i][j]
            n=n+1
        end
    end
    -- ---------------------------------------------
    COECT={}
    for i=1,4*n_ctpts do
        COECT[i]=COEC[i][1]
    end
    -- ---------------------------------------------
    rs={}
    rs[1]=1
    for i=1,4*n_ctpts do
        rs[i+1]=1+8*i
    end
    -- ---------------------------------------------
    -- -----------1----------------2----------------
    -- indexes = {1,2,3,4,5,6,7,8, 1,2,3,..........}
    idx={}
    for i=1,4*n_ctpts do
        for j=1,8 do
            idx[(i-1)*8+j]=j
        end
    end
    -- ---------------------------------------------
    local M =
        {
        nvars = 8,
        nrows = 4*n_ctpts,
        indexes = idx,
        elements = elm,
        row_starts = rs,
        b = COECT,
        c = {0,0,0,0,0,0,0,1},
        xl = {-10,-10,-10,-10,-10,-10,-10,-10},
        xu = {10,10,10,10,10,10,10,10},
        }
    -- ---------------------------------------------
    local I = luasimplex.new_instance(M.nrows, M.nvars)
    rsm.initialise(M, I, {})
    objective, x = rsm.solve(M, I, {})
    -- ---------------------------------------------
    x[2]=0
    x[3]=0
    x[4]=0
    x[8]=nil
    
    crt=(x[1]^2+x[2]^2+x[3]^2+x[4]^2+x[5]^2+x[6]^2+x[7]^2)^0.5

    return x, crt
    -- ---------------------------------------------
end

-- -------------------------------------------------

return metrology

-- -------------------------------------------------


-- Metrology Module---------------------------------
metrology = {}

-- Function Associate-------------------------------
function metrology.Caliper_A(D1,PTS)
    
    -- Caliper_A:-----------------------------------
    --  conduct measurement by two measurement planes
    --  the size between these planes are evaluated
    -- ---------------------------------------------

    -- Inputs:--------------------------------------
    --  D1: distance between 1st measurement plane & skin model shape
    --  PTS: measurement points on 1st plane
    -- ---------------------------------------------
    
    -- Define the number of "contact points"--------
    local n_ctpts,b = matrix.size(PTS)
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local C1=matrix{0,0,0}
    local N1=matrix{-1,0,0}
    local P1=PTS
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx,Ty,Tz,Rx,Ry,Rz,F}
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  F: optimization objective value (maximum distance between plane and points)
    --  1. matrial condition
    --  2. objective equation
    -- "COE": coefficients of linear equations
    local COE=matrix(2*n_ctpts,4,0)
    -- "COEC": constant value part of linear equations
    local COEC=matrix(2*n_ctpts,1,0)
    -- This is for the 1st measurement plane--------
    -- Including:
    --  1. the material conditions
    --  2. the optimization objective
    for i=1,n_ctpts do
        -- r: --------------------------------------
        local r=matrix{D1[i][1],P1[i][2],P1[i][3]}
        r=r:cross(N1)
        -- material conditions----------------------
        COE[i][1]=N1[1][1]
        --COE[i][2]=N1[2][1]
        --COE[i][3]=N1[3][1]
        --COE[i][4]=r[1][1]
        COE[i][2]=r[2][1]
        COE[i][3]=r[3][1]
        -- -----------------------------------------
        COEC[i][1]=D1[i][1]
        -- objective--------------------------------
        COE[i+n_ctpts][1]=-COE[i][1]
        --COE[i+n_ctpts][2]=-COE[i][2]
        --COE[i+n_ctpts][3]=-COE[i][3]
        --COE[i+n_ctpts][4]=-COE[i][4]
        COE[i+n_ctpts][2]=-COE[i][2]
        COE[i+n_ctpts][3]=-COE[i][3]
        COE[i+n_ctpts][4]=-1
        -- -----------------------------------------
        COEC[i+n_ctpts][1]=-D1[i][1]
        -- -----------------------------------------
    end

    -- Using Lua Simplex----------------------------
    local luasimplex = require("luasimplex")
    local rsm = require("luasimplex.rsm")
    -- ---------------------------------------------
    local elm={}
    local n=1
    for i=1,2*n_ctpts do
        for j=1,4 do
            elm[n]=COE[i][j]
            n=n+1
        end
    end
    -- ---------------------------------------------
    local COECT={}
    for i=1,2*n_ctpts do
        COECT[i]=COEC[i][1]
    end
    -- ---------------------------------------------
    local rs={}
    rs[1]=1
    for i=1,2*n_ctpts do
        rs[i+1]=1+4*i
    end
    -- ---------------------------------------------
    -- -----------1--------2----------------
    -- indexes = {1,2,3,4, 1,2,3,..........}
    local idx={}
    for i=1,2*n_ctpts do
        for j=1,4 do
            idx[(i-1)*4+j]=j
        end
    end
    -- ---------------------------------------------
    local M =
        {
        nvars = 4,
        nrows = 2*n_ctpts,
        indexes = idx,
        elements = elm,
        row_starts = rs,
        b = COECT,
        c = {0,0,0,1},
        xl = {-10,-10,-10,-10},
        xu = {10,10,10,10},
        }
    -- ---------------------------------------------
    local I = luasimplex.new_instance(M.nrows, M.nvars)
    rsm.initialise(M, I, {})
    local objective, x = rsm.solve(M, I, {})
    -- ---------------------------------------------
    print('----------')
    print(x[1])
    print(x[2])
    print(x[3])
    print(x[4])
    print(objective)
    print('----------')
    return x, objective
    -- ---------------------------------------------
end

-- -------------------------------------------------

return metrology

-- -------------------------------------------------


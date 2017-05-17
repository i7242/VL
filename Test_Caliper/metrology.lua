-- Metrology Module---------------------------------
metrology = {}

-- Function Associate-------------------------------
function metrology.Caliper_A(D1,D2,DPL,PTS)
    
    -- Caliper_A:-----------------------------------
    --  conduct measurement by two measurement planes
    --  the distance between these planes
    --  are minimized
    -- ---------------------------------------------

    -- Inputs:--------------------------------------
    --  D1: distance between 1st measurement plane & skin model shape
    --  D2: distance between 2nd measurement plane & skin model shape
    --  DPL: distance between 1st & 2nd plane
    --  PTS: measurement points on 1st plane
    -- ---------------------------------------------
    
    -- Define the number of "contact points"--------
    local a,b = matrix.size(PTS)
    local n_ctpts=a
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

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local C2=matrix{-DPL,0,0}
    local N2=matrix{1,0,0}
    local P2=PTS
    for i=1,n_ctpts do
        P2[i][1]=-DPL
    end
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx1,Tx2,Ty1,Ty2,Tz1,Tz2,Rx1,Rx2,Ry1,Ry2,Rz1,Rz2,DA1,DA2}
    --  14 variables
    --  "1" will be plus
    --  "2" will be minus
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  DA1: displacement along normal direction of the second plane
    --  DA2: inverse displacement of DA1
    
    -- For each plane
    --  there are matrial conditions
    --  and there are two planes
    --  and one minimization objective
    --  all equations are in form "<="
    --  no "=" or ">="
    
    --  So the row number of matrix "A" is:
    --      2*n_ctpts+2  (one column more to avoide error in require nil value)
    --  the column number of matrix "A" is:
    --      14+2*n_ctpts+2
    local A=matrix(2*n_ctpts+2,14+2*n_ctpts+2,0)
    
    -- This is for the 1st measurement plane--------
    for i=1,n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N1[1][1]
        A[i][2+1]=-N1[1][1]

        A[i][3+1]=N1[2][1]
        A[i][4+1]=-N1[2][1]

        A[i][5+1]=N1[3][1]
        A[i][6+1]=-N1[3][1]

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}+N1*D1[i][1]
        r=r:cross(N1)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][14+2*n_ctpts+2]=D1[i][1]
    end
    
    -- This is for the 2nd plane--------------------
    -- similar to the first one
    -- but the changing of distance between 2 planes
    -- are considered
    for i=n_ctpts+1,2*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N2[1][1]
        A[i][2+1]=-N2[1][1]

        A[i][3+1]=N2[2][1]
        A[i][4+1]=-N2[2][1]
        
        A[i][5+1]=N2[3][1]
        A[i][6+1]=-N2[3][1]
        
        local r=matrix{P2[i-n_ctpts][1],P2[i-n_ctpts][2],P2[i-n_ctpts][3]}+N2*D2[i-n_ctpts][1]-C2
        r=r:cross(N2)
        r=r-N2:cross(C2)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][13+1]=1
        A[i][14+1]=-1
        
        A[i][14+2*n_ctpts+2]=D2[i-n_ctpts][1]
    end

    --  Add objective line----------------------
    --  maximize displacement along normal direction of second plane
    --  that means the distance between two planes is minimized
    A[2*n_ctpts+1][13+1]=1
    A[2*n_ctpts+1][14+1]=-1

    -- Using Simplex----------------------------
    local L=2*n_ctpts
    local E=0
    local G=0
    local N=14
    local F=1

    Result, Objective=optimize.simplex(L,E,G,N,F,A)

    local Result2=matrix(7,1,0)
    for i=1,7 do
        Result2[i][1]=Result[2*i-1][1]-Result[2*i][1]
    end

    return Result2,math.abs(Result2[7][1])

    -- ---------------------------------------------
end
-- -------------------------------------------------

-- Function Associate-------------------------------
function metrology.Caliper_B(D1,D2,DPL,PTS)
    
    -- Caliper_B:-----------------------------------
    --  conduct measurement by two measurement planes
    --  the distance between these planes
    --  are maximized
    -- ---------------------------------------------

    -- Inputs:--------------------------------------
    --  D1: distance between 1st measurement plane & skin model shape
    --  D2: distance between 2nd measurement plane & skin model shape
    --  DPL: distance between 1st & 2nd plane
    --  PTS: measurement points on 1st plane
    -- ---------------------------------------------
    
    -- Define the number of "contact points"--------
    local a,b = matrix.size(PTS)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local C1=matrix{0,0,0}
    local N1=matrix{1,0,0}
    local P1=PTS
    -- ---------------------------------------------

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local C2=matrix{-DPL,0,0}
    local N2=matrix{-1,0,0}
    local P2=PTS
    for i=1,n_ctpts do
        P2[i][1]=-DPL
    end
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx1,Tx2,Ty1,Ty2,Tz1,Tz2,Rx1,Rx2,Ry1,Ry2,Rz1,Rz2,DA1,DA2}
    --  14 variables
    --  "1" will be plus
    --  "2" will be minus
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  DA1: displacement along normal direction of the second plane
    --  DA2: inverse displacement of DA1
    
    -- For each plane
    --  there are matrial conditions
    --  and there are two planes
    --  and one minimization objective
    --  all equations are in form "<="
    --  no "=" or ">="
    
    --  So the row number of matrix "A" is:
    --      2*n_ctpts+2  (one column more to avoide error in require nil value)
    --  the column number of matrix "A" is:
    --      14+2*n_ctpts+2
    local A=matrix(2*n_ctpts+2,14+2*n_ctpts+2,0)
    
    -- This is for the 1st measurement plane--------
    for i=1,n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N1[1][1]
        A[i][2+1]=-N1[1][1]

        A[i][3+1]=N1[2][1]
        A[i][4+1]=-N1[2][1]

        A[i][5+1]=N1[3][1]
        A[i][6+1]=-N1[3][1]

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}+N1*D1[i][1]
        r=r:cross(N1)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][14+2*n_ctpts+2]=D1[i][1]
    end
    
    -- This is for the 2nd plane--------------------
    -- similar to the first one
    -- but the changing of distance between 2 planes
    -- are considered
    for i=n_ctpts+1,2*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N2[1][1]
        A[i][2+1]=-N2[1][1]

        A[i][3+1]=N2[2][1]
        A[i][4+1]=-N2[2][1]
        
        A[i][5+1]=N2[3][1]
        A[i][6+1]=-N2[3][1]
        
        local r=matrix{P2[i-n_ctpts][1],P2[i-n_ctpts][2],P2[i-n_ctpts][3]}+N2*D2[i-n_ctpts][1]-C2
        r=r:cross(N2)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][13+1]=1
        A[i][14+1]=-1
        
        A[i][14+2*n_ctpts+2]=D2[i-n_ctpts][1]
    end

    --  Add objective line----------------------
    --  maximize distence between two plane
    --  that means the displacement between plane should be maximized
    A[2*n_ctpts+1][13+1]=1
    A[2*n_ctpts+1][14+1]=-1

    -- Using Simplex----------------------------
    local L=2*n_ctpts
    local E=0
    local G=0
    local N=14
    local F=1

    Result, Objective=optimize.simplex(L,E,G,N,F,A)

    local Result2=matrix(7,1,0)
    for i=1,7 do
        Result2[i][1]=Result[2*i-1][1]-Result[2*i][1]
    end

    return Result2,math.abs(Result2[7][1])

    -- ---------------------------------------------
end
-- -------------------------------------------------

-- Function Associate-------------------------------
function metrology.Caliper_C(D1,D2,DPL,PTS,dir_c3)
    
    -- Caliper_C:-----------------------------------
    --  conduct measurement by two measurement planes
    --  these planes are on the top of caliper
    --  the maximum distance between these planes and
    --  measurement poits are minimized
    -- ---------------------------------------------

    -- Inputs:--------------------------------------
    --  D1: distance between 1st measurement plane & skin model shape
    --  D2: distance between 2nd measurement plane & skin model shape
    --  DPL: distance between 1st & 2nd plane
    --  PTS: measurement points on 1st plane
    -- ---------------------------------------------
    
    -- Define the number of "contact points"--------
    local a,b = matrix.size(PTS)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local C1=matrix{0,0,0}
    local N1=matrix{1,0,0}
    local P1=PTS
    -- ---------------------------------------------

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    -- ??? using dir_c3 ???
    local C2=matrix{-DPL,0,0}
    local N2=matrix{1,0,0}
    local P2=PTS
    for i=1,n_ctpts do
        P2[i][1]=-DPL
    end
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx1,Tx2,Ty1,Ty2,Tz1,Tz2,Rx1,Rx2,Ry1,Ry2,Rz1,Rz2,DA1,DA2,Dmax}
    --  15 variables
    --  "1" will be plus
    --  "2" will be minus
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  DA1: displacement along normal direction of the second plane
    --  DA2: inverse displacement of DA1
    --  "Dmax" will be minimized
    
    -- For each plane
    --  there are matrial conditions
    --  there are "Dmax" conditions
    --  and there are two planes
    --  and one minimization objective
    --  all equations are in form "<="
    --  no "=" or ">="
    
    --  So the row number of matrix "A" is:
    --      4*n_ctpts+2  (one column more to avoide error in require nil value)
    --  the column number of matrix "A" is:
    --      15+2*n_ctpts+2
    local A=matrix(4*n_ctpts+2,15+2*n_ctpts+2,0)
    
    -- This is for the 1st measurement plane--------
    for i=1,n_ctpts do
        -- material conditions----------------------
        A[2*i-1][1+1]=N1[1][1]
        A[2*i-1][2+1]=-N1[1][1]

        A[2*i-1][3+1]=N1[2][1]
        A[2*i-1][4+1]=-N1[2][1]

        A[2*i-1][5+1]=N1[3][1]
        A[2*i-1][6+1]=-N1[3][1]

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}+N1*D1[i][1]
        r=r:cross(N1)
        A[2*i-1][7+1]=r[1][1]
        A[2*i-1][8+1]=-r[1][1]

        A[2*i-1][9+1]=r[2][1]
        A[2*i-1][10+1]=-r[2][1]

        A[2*i-1][11+1]=r[3][1]
        A[2*i-1][12+1]=-r[3][1]

        A[2*i-1][15+2*n_ctpts+2]=D1[i][1]

        -- "Dmax" conditions----------------------
        A[2*i][1+1]=-N1[1][1]
        A[2*i][2+1]=N1[1][1]

        A[2*i][3+1]=-N1[2][1]
        A[2*i][4+1]=N1[2][1]

        A[2*i][5+1]=-N1[3][1]
        A[2*i][6+1]=N1[3][1]

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}+N1*D1[i][1]
        r=r:cross(N1)
        A[2*i][7+1]=-r[1][1]
        A[2*i][8+1]=r[1][1]

        A[2*i][9+1]=-r[2][1]
        A[2*i][10+1]=r[2][1]

        A[2*i][11+1]=-r[3][1]
        A[2*i][12+1]=r[3][1]

        A[2*i][15+1]=-1

        A[2*i][15+2*n_ctpts+2]=0
    end
    
    -- This is for the 2nd plane--------------------
    -- similar to the first one
    -- but the changing of distance between 2 planes
    -- are considered
    for i=n_ctpts+1,2*n_ctpts do
        -- material conditions----------------------
        A[2*i-1][1+1]=N2[1][1]
        A[2*i-1][2+1]=-N2[1][1]

        A[2*i-1][3+1]=N2[2][1]
        A[2*i-1][4+1]=-N2[2][1]
        
        A[2*i-1][5+1]=N2[3][1]
        A[2*i-1][6+1]=-N2[3][1]
        
        local r=matrix{P2[i-2*n_ctpts][1],P2[i-2*n_ctpts][2],P2[i-2*n_ctpts][3]}+N2*D2[i-n_ctpts][1]-C2
        r=r:cross(N2)
        A[2*i-1][7+1]=r[1][1]
        A[2*i-1][8+1]=-r[1][1]

        A[2*i-1][9+1]=r[2][1]
        A[2*i-1][10+1]=-r[2][1]

        A[2*i-1][11+1]=r[3][1]
        A[2*i-1][12+1]=-r[3][1]

        A[2*i-1][13+1]=1
        A[2*i-1][14+1]=-1
        
        A[2*i-1][15+2*n_ctpts+2]=D2[i-n_ctpts][1]

        -- "Dmax" conditions----------------------
        A[2*i][1+1]=-N1[1][1]
        A[2*i][2+1]=N1[1][1]

        A[2*i][3+1]=-N1[2][1]
        A[2*i][4+1]=N1[2][1]

        A[2*i][5+1]=-N1[3][1]
        A[2*i][6+1]=N1[3][1]

        local r=matrix{P2[i-2*n_ctpts][1],P2[i-2*n_ctpts][2],P2[i-2*n_ctpts][3]}+N2*D2[i-n_ctpts][1]-C2
        r=r:cross(N2)
        A[2*i][7+1]=-r[1][1]
        A[2*i][8+1]=r[1][1]

        A[2*i][9+1]=-r[2][1]
        A[2*i][10+1]=r[2][1]

        A[2*i][11+1]=-r[3][1]
        A[2*i][12+1]=r[3][1]

        A[2*i][15+1]=-1

        A[2*i][15+2*n_ctpts+2]=0
    end

    --  Add objective line----------------------
    --  minimize the maximum distance "Dmax"
    A[4*n_ctpts+1][15+1]=1

    -- Using Simplex----------------------------
    local L=4*n_ctpts
    local E=0
    local G=0
    local N=15
    local F=-1

    Result, Objective=optimize.simplex(L,E,G,N,F,A)

    local Result2=matrix(7,1,0)
    for i=1,7 do
        Result2[i][1]=Result[2*i-1][1]-Result[2*i][1]
    end

    return Result2,math.abs(Result2[7][1])

    -- ---------------------------------------------
end
-- -------------------------------------------------

-- Function Associate-------------------------------
function metrology.MicroMeter(D1,D2,DPL,PTS)
    
    -- MicroMeter:-----------------------------------
    --  conduct measurement by two measurement planes
    --  the distance between these planes
    --  are minimized
    -- ---------------------------------------------

    -- Inputs:--------------------------------------
    --  D1: distance between 1st measurement plane & skin model shape
    --  D2: distance between 2nd measurement plane & skin model shape
    --  DPL: distance between 1st & 2nd plane
    --  PTS: measurement points on 1st plane
    -- ---------------------------------------------
    
    -- Define the number of "contact points"--------
    local a,b = matrix.size(PTS)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local C1=matrix{0,0,0}
    local N1=matrix{0,0,-1}
    local P1=PTS
    -- ---------------------------------------------

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local C2=matrix{0,0,-DPL}
    local N2=matrix{0,0,1}
    local P2=PTS
    for i=1,n_ctpts do
        P2[i][3]=-DPL
    end
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx1,Tx2,Ty1,Ty2,Tz1,Tz2,Rx1,Rx2,Ry1,Ry2,Rz1,Rz2,DA1,DA2}
    --  14 variables
    --  "1" will be plus
    --  "2" will be minus
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  DA1: displacement along normal direction of the second plane
    --  DA2: inverse displacement of DA1
    
    -- For each plane
    --  there are matrial conditions
    --  and there are two planes
    --  and one minimization objective
    --  all equations are in form "<="
    --  no "=" or ">="
    
    --  So the row number of matrix "A" is:
    --      2*n_ctpts+2  (one column more to avoide error in require nil value)
    --  the column number of matrix "A" is:
    --      14+2*n_ctpts+2
    local A=matrix(2*n_ctpts+2,14+2*n_ctpts+2,0)
    
    -- This is for the 1st measurement plane--------
    for i=1,n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N1[1][1]
        A[i][2+1]=-N1[1][1]

        A[i][3+1]=N1[2][1]
        A[i][4+1]=-N1[2][1]

        A[i][5+1]=N1[3][1]
        A[i][6+1]=-N1[3][1]

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}+N1*D1[i][1]
        r=r:cross(N1)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][14+2*n_ctpts+2]=D1[i][1]
    end
    
    -- This is for the 2nd plane--------------------
    -- similar to the first one
    -- but the changing of distance between 2 planes
    -- are considered
    for i=n_ctpts+1,2*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N2[1][1]
        A[i][2+1]=-N2[1][1]

        A[i][3+1]=N2[2][1]
        A[i][4+1]=-N2[2][1]
        
        A[i][5+1]=N2[3][1]
        A[i][6+1]=-N2[3][1]
        
        local r=matrix{P2[i-n_ctpts][1],P2[i-n_ctpts][2],P2[i-n_ctpts][3]}+N2*D2[i-n_ctpts][1]-C2
        r=r:cross(N2)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][13+1]=1
        A[i][14+1]=-1
        
        A[i][14+2*n_ctpts+2]=D2[i-n_ctpts][1]
    end

    --  Add objective line----------------------
    --  maximize displacement along normal direction of second plane
    --  that means the distance between two planes is minimized
    A[2*n_ctpts+1][13+1]=1
    A[2*n_ctpts+1][14+1]=-1

    -- Using Simplex----------------------------
    local L=2*n_ctpts
    local E=0
    local G=0
    local N=14
    local F=1

    Result, Objective=optimize.simplex(L,E,G,N,F,A)

    local Result2=matrix(7,1,0)
    for i=1,7 do
        Result2[i][1]=Result[2*i-1][1]-Result[2*i][1]
    end

    return Result2,math.abs(Result2[7][1])

    -- ---------------------------------------------
end
-- -------------------------------------------------


return metrology

-- -------------------------------------------------


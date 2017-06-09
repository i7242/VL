-- Metrology Module---------------------------------
metrology = {}

-- Function Associate-------------------------------
function metrology.Caliper_Outside(C1,C2,N1,N2,D1,D2,DPL,PTS1)
    
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
    local a,b = matrix.size(PTS1)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local P1=matrix(a,b,0)
    for i=1,n_ctpts do
        P1[i][1]=PTS1[i][1]
        P1[i][2]=PTS1[i][2]
        P1[i][3]=PTS1[i][3]
    end
    -- ---------------------------------------------

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local P2=matrix(a,b,0)
    for i=1,n_ctpts do
        P2[i][1]=PTS1[i][1]+DPL*N1[1][1]
        P2[i][2]=PTS1[i][2]+DPL*N1[2][1]
        P2[i][3]=PTS1[i][3]+DPL*N1[3][1]
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

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}-C1+N1*D1[i][1]
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
        
        local r1=matrix{P2[i-n_ctpts][1],P2[i-n_ctpts][2],P2[i-n_ctpts][3]}+N2*D2[i-n_ctpts][1]-C2
        r1=r1:cross(N2)
        local r2=N2:cross(C1-C2)
        local r=r1+r2
        
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

    --  Add objective line--------------------------
    --  maximize displacement along normal direction of second plane
    --  that means the distance between two planes is minimized
    A[2*n_ctpts+1][13+1]=1
    A[2*n_ctpts+1][14+1]=-1

    -- Using Simplex--------------------------------
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
function metrology.Caliper_Inside(C1,C2,N11,N12,N13,N21,N22,N23,D1,D2,DPL,PTS1)
    
    -- Caliper_B:-----------------------------------
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
    local a,b = matrix.size(PTS1)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    local P1=matrix(a,b,0)
    for i=1,n_ctpts do
        P1[i][1]=PTS1[i][1]
        P1[i][2]=PTS1[i][2]
        P1[i][3]=PTS1[i][3]
    end
    -- ---------------------------------------------

    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local P2=matrix(a,b,0)
    for i=1,n_ctpts do
        P2[i][1]=PTS1[i][1]+DPL*N21[1][1]
        P2[i][2]=PTS1[i][2]+DPL*N21[2][1]
        P2[i][3]=PTS1[i][3]+DPL*N21[3][1]
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
    --  and each point has 3 normal directions
    --  and one minimization objective
    --  all equations are in form "<="
    --  no "=" or ">="
    
    --  So the row number of matrix "A" is:
    --      3*2*n_ctpts+2  (one column more to avoide error in require nil value)
    --  the column number of matrix "A" is:
    --      14+3*2*n_ctpts+2
    local A=matrix(3*2*n_ctpts+2,14+3*2*n_ctpts+2,0)
    
    -- This is for the 1st measurement plane--------
    -- N11
    for i=1,n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N11[1][1]
        A[i][2+1]=-N11[1][1]

        A[i][3+1]=N11[2][1]
        A[i][4+1]=-N11[2][1]

        A[i][5+1]=N11[3][1]
        A[i][6+1]=-N11[3][1]

        local r=matrix{P1[i][1],P1[i][2],P1[i][3]}-C1+N11*D1[i][1]
        r=r:cross(N11)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][14+3*2*n_ctpts+2]=D1[i][1]
    end
    -- N12
    for i=n_ctpts+1,2*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N12[1][1]
        A[i][2+1]=-N12[1][1]

        A[i][3+1]=N12[2][1]
        A[i][4+1]=-N12[2][1]

        A[i][5+1]=N12[3][1]
        A[i][6+1]=-N12[3][1]

        local r=matrix{P1[i-n_ctpts][1],P1[i-n_ctpts][2],P1[i-n_ctpts][3]}-C1+N12*D1[i][1]
        r=r:cross(N12)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][14+3*2*n_ctpts+2]=D1[i][1]
    end
    -- N13
    for i=2*n_ctpts+1,3*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N13[1][1]
        A[i][2+1]=-N13[1][1]

        A[i][3+1]=N13[2][1]
        A[i][4+1]=-N13[2][1]

        A[i][5+1]=N13[3][1]
        A[i][6+1]=-N13[3][1]

        local r=matrix{P1[i-2*n_ctpts][1],P1[i-2*n_ctpts][2],P1[i-2*n_ctpts][3]}-C1+N13*D1[i][1]
        r=r:cross(N13)
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][14+3*2*n_ctpts+2]=D1[i][1]
    end

    -- This is for the 2nd plane--------------------
    -- similar to the first one
    -- but the changing of distance between 2 planes
    -- are considered
    -- N21
    for i=3*n_ctpts+1,4*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N21[1][1]
        A[i][2+1]=-N21[1][1]

        A[i][3+1]=N21[2][1]
        A[i][4+1]=-N21[2][1]
        
        A[i][5+1]=N21[3][1]
        A[i][6+1]=-N21[3][1]
        
        local r1=matrix{P2[i-3*n_ctpts][1],P2[i-3*n_ctpts][2],P2[i-3*n_ctpts][3]}+N21*D2[i-3*n_ctpts][1]-C2
        r1=r1:cross(N21)
        local r2=N21:cross(C1-C2)
        local r=r1+r2
        
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][13+1]=1
        A[i][14+1]=-1
        
        A[i][14+3*2*n_ctpts+2]=D2[i-3*n_ctpts][1]
    end
    -- N22
    for i=4*n_ctpts+1,5*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N22[1][1]
        A[i][2+1]=-N22[1][1]

        A[i][3+1]=N22[2][1]
        A[i][4+1]=-N22[2][1]
        
        A[i][5+1]=N22[3][1]
        A[i][6+1]=-N22[3][1]
        
        local r1=matrix{P2[i-4*n_ctpts][1],P2[i-4*n_ctpts][2],P2[i-4*n_ctpts][3]}+N22*D2[i-3*n_ctpts][1]-C2
        r1=r1:cross(N22)
        local r2=N22:cross(C1-C2)
        local r=r1+r2
        
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][13+1]=1
        A[i][14+1]=-1
        
        A[i][14+3*2*n_ctpts+2]=D2[i-3*n_ctpts][1]
    end
    -- N23
    for i=5*n_ctpts+1,6*n_ctpts do
        -- material conditions----------------------
        A[i][1+1]=N23[1][1]
        A[i][2+1]=-N23[1][1]

        A[i][3+1]=N23[2][1]
        A[i][4+1]=-N23[2][1]
        
        A[i][5+1]=N23[3][1]
        A[i][6+1]=-N23[3][1]
        
        local r1=matrix{P2[i-5*n_ctpts][1],P2[i-5*n_ctpts][2],P2[i-5*n_ctpts][3]}+N23*D2[i-3*n_ctpts][1]-C2
        r1=r1:cross(N23)
        local r2=N23:cross(C1-C2)
        local r=r1+r2
        
        A[i][7+1]=r[1][1]
        A[i][8+1]=-r[1][1]

        A[i][9+1]=r[2][1]
        A[i][10+1]=-r[2][1]

        A[i][11+1]=r[3][1]
        A[i][12+1]=-r[3][1]

        A[i][13+1]=1
        A[i][14+1]=-1
        
        A[i][14+3*2*n_ctpts+2]=D2[i-3*n_ctpts][1]
    end

    --  Add objective line--------------------------
    --  maximize displacement along normal direction of second plane
    --  that means the distance between two planes is maximized
    A[3*2*n_ctpts+1][13+1]=1
    A[3*2*n_ctpts+1][14+1]=-1

    -- Using Simplex--------------------------------
    local L=3*2*n_ctpts
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
function metrology.Caliper_Shoulder2(C2,N2,D2,PTS1)
    
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
    local a,b = matrix.size(PTS1)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    
    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local P2=matrix(a,b,0)
    for i=1,n_ctpts do
        P2[i][1]=PTS1[i][1]
        P2[i][2]=PTS1[i][2]
        P2[i][3]=PTS1[i][3]
    end
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx1,Tx2,Ty1,Ty2,Tz1,Tz2,Rx1,Rx2,Ry1,Ry2,Rz1,Rz2,Dmax1,Dmax2}
    --  13 variables
    --  "1" will be plus
    --  "2" will be minus
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  (Dmax1-Dmax2): maximum distance to be minimized, always positive
    
    -- There is only one plane:
    --  there are matrial conditions
    --  there are constraints for "Dmax"
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

        local r=matrix{P2[i][1],P2[i][2],P2[i][3]}+N2*D2[i][1]-C2
        r=r:cross(N2)
        -- material conditions----------------------
        A[2*i-1][1+1]=N2[1][1]
        A[2*i-1][2+1]=-N2[1][1]

        A[2*i-1][3+1]=N2[2][1]
        A[2*i-1][4+1]=-N2[2][1]

        A[2*i-1][5+1]=N2[3][1]
        A[2*i-1][6+1]=-N2[3][1]

        A[2*i-1][7+1]=r[1][1]
        A[2*i-1][8+1]=-r[1][1]

        A[2*i-1][9+1]=r[2][1]
        A[2*i-1][10+1]=-r[2][1]

        A[2*i-1][11+1]=r[3][1]
        A[2*i-1][12+1]=-r[3][1]

        A[2*i-1][14+2*n_ctpts+2]=D2[i][1]

        -- "f" up boundary condition----------------
        A[2*i][1+1]=-N2[1][1]
        A[2*i][2+1]=N2[1][1]

        A[2*i][3+1]=-N2[2][1]
        A[2*i][4+1]=N2[2][1]

        A[2*i][5+1]=-N2[3][1]
        A[2*i][6+1]=N2[3][1]

        A[2*i][7+1]=-r[1][1]
        A[2*i][8+1]=r[1][1]

        A[2*i][9+1]=-r[2][1]
        A[2*i][10+1]=r[2][1]

        A[2*i][11+1]=-r[3][1]
        A[2*i][12+1]=r[3][1]

        A[2*i][13+1]=-1
        A[2*i][14+1]=1

        A[2*i][14+2*n_ctpts+2]=-D2[i][1]
    end

    --  Add objective line--------------------------
    --  maximize displacement along normal direction of second plane
    --  that means the distance between two planes is minimized
    A[2*n_ctpts+1][13+1]=1
    A[2*n_ctpts+1][14+1]=-1

    -- Using Simplex--------------------------------
    local L=2*n_ctpts
    local E=0
    local G=0
    local N=14
    local F=-1

    Result, Objective=optimize.simplex(L,E,G,N,F,A)

    local Result2=matrix(6,1,0)
    for i=1,6 do
        Result2[i][1]=Result[2*i-1][1]-Result[2*i][1]
    end

    return Result2, Objective

    -- ---------------------------------------------
end
-- -------------------------------------------------

-- Function Associate-------------------------------
function metrology.Caliper_Shoulder1(C1,N1,D1,PTS1)
    
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
    local a,b = matrix.size(PTS1)
    local n_ctpts=a
    -- ---------------------------------------------

    -- Definition of the 1st plane------------------
    --  C1: center point
    --  N1: normal vector
    --  P1: position of measurement contact points
    --      this is relative to "C1"
    --      and should be same as in "D1"
    
    -- Definition of the 2nd plane------------------
    --  similar to 1st plane
    local P1=matrix(a,b,0)
    for i=1,n_ctpts do
        P1[i][1]=PTS1[i][1]
        P1[i][2]=PTS1[i][2]
        P1[i][3]=PTS1[i][3]
    end
    -- ---------------------------------------------

    -- Formulating the linear programming-----------
    --  {Tx1,Tx2,Ty1,Ty2,Tz1,Tz2,Rx1,Rx2,Ry1,Ry2,Rz1,Rz2,Dmax1,Dmax2}
    --  13 variables
    --  "1" will be plus
    --  "2" will be minus
    --  Tx,Ty,Tz: translation torsor
    --  Rx,Ry,Rz: rotation torsor
    --  (Dmax1-Dmax2): maximum distance to be minimized, always positive
    
    -- There is only one plane:
    --  there are matrial conditions
    --  there are constraints for "Dmax"
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
        A[2*i-1][1+1]=N1[1][1]
        A[2*i-1][2+1]=-N1[1][1]

        A[2*i-1][3+1]=N1[2][1]
        A[2*i-1][4+1]=-N1[2][1]

        A[2*i-1][5+1]=N1[3][1]
        A[2*i-1][6+1]=-N1[3][1]

        A[2*i-1][14+2*n_ctpts+2]=D1[i][1]

        -- "f" up boundary condition----------------
        A[2*i][1+1]=-N1[1][1]
        A[2*i][2+1]=N1[1][1]

        A[2*i][3+1]=-N1[2][1]
        A[2*i][4+1]=N1[2][1]

        A[2*i][5+1]=-N1[3][1]
        A[2*i][6+1]=N1[3][1]

        A[2*i][13+1]=-1
        A[2*i][14+1]=1

        A[2*i][14+2*n_ctpts+2]=-D1[i][1]
    end

    --  Add objective line--------------------------
    --  maximize displacement along normal direction of second plane
    --  that means the distance between two planes is minimized
    A[2*n_ctpts+1][13+1]=1
    A[2*n_ctpts+1][14+1]=-1

    -- Using Simplex--------------------------------
    local L=2*n_ctpts
    local E=0
    local G=0
    local N=14
    local F=-1

    Result, Objective=optimize.simplex(L,E,G,N,F,A)

    local Result2=matrix(6,1,0)
    for i=1,6 do
        Result2[i][1]=Result[2*i-1][1]-Result[2*i][1]
    end

    return Result2, Objective

    -- ---------------------------------------------
end

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


-- Optimization Package---------------------------------
--[[ Currently, only have simplex
     Maybe will be improved with other methods
--------------------------------------------------- ]]--
optimize={}

--------------------1 For Simplex-----------------------
--[[ ---------------------------------------------------
    1>Type of variables:
        L:number of constraints"<="
        E:....................."="
        G:.....................">="
        N:.......... variables
        F: = 1, if maximize
           =-1, if minimize
        A:coefficient matrix of the simples
          matrix is arranged in the following sequance:
          1. <=
          2. =
          3. >=
          4. objective

    2>How to use it
        Simplex(L,G,E,N,F,A)

    3>Dependent
        Matrix in Lua
--------------------------------------------------- ]]--

-- Simplex calculation processes------------------------
function optimize.simplex( L,G,E,N,F,A )

    --  Initialize Variable Value
    --  give all variables zero value
    --  because if they are not changing to base, they should be zero
    local Result=matrix( N,1,0 )
    
    --  Initialization, add artificial variables, for each kind of constraints
    local A,Error,C,M,B,W=Initialization( L,G,E,N,F,A )
    local Q=0
    local Objective=0
    local R=-1

    --  add additional artifical variables for "=" & ">=" constraints
    if ( (E~=0) or (G~=0) ) then
        C,Q,A,W=BlocA( L,G,E,N,A,M,W )
    end
    local Stop=false
    Error=0

    --  if no =,>= constraints, go to BlocC directly
    if ( (G+E)==0 ) then
        C,Q,A=BlocC( L,G,N,A,C,W )
    end

    while ( not Stop ) do

        --  "Q" is the value corresponding to the change in variable
        --  if Q==0, means no more change in, optimization will be stoped
        if ( Q==0 ) then
            --  "W" is the row number of objective, or the last row
            --  this is to change out all artifical variables for "=" & ">="
            --  then solve the optimization problem
            if ( W~=(M+1) ) then
                --  "W+1" in "BlocA", so here minus
                W=W-1
            else
                --  this is to know if there exist solutions
                --  it is actually "BlocD", and "BlocD" is not used...
                i=1
                while ( (not Stop) and (i<=(M+1)) ) do
                    --  "N+G+L+2" is the begining of artifical variable for equation constraint
                    --  because equation constraint must be satisfied, so it should be changed out
                    --  if "A[i][B]~=0", means it is not changed out
                    --  can't find solution satisfy equation constraint
                    if ( (A[i][1]>=(N+G+L+2)) and (A[i][B]~=0) ) then
                        Stop=true
                        Error=2
                    end
                    i=i+1
                end
                --  if exist solutions, save them and out put
                if ( Error==0 ) then
                    Result,Objective=BlocE( N,F,A,B,W,Result )
                end
                Stop=true
            end
        else    --  problem still could be optimized
            Q,R=BlocB1( Q,A,C,M,B )
            --  initially R=-1. if R<0, means no change out variable
            --  there is no definite solutions
            if ( R<0 ) then
                Stop=true
                Error=1
            else
                A=BlocB2( A,C,B,W,R )
            end
        end
        --  if not stop, continue to find change in variable
        if ( not Stop ) then
            C,Q,A=BlocC( L,G,N,A,C,W )
        end
    end

    --  print results or error information
    if (Error==0) then
        --print('Results are:')
        --print(Result)
        --print(Objective)
    elseif (Error==1) then
        print('Error1: No definite solution!')
    elseif (Error==2) then
        print('Error2: No feasable solution!')
    end

    --  output calculation results
    return Result, Objective

end

-- Initialization: add artificial variables-------------
function Initialization( L,G,E,N,F,A )
    --  last row of "A" is objective
    --  in the form:
    --  z-a1*x1-a2*x2....=0
    for i=2,(N+1) do    --  variable starts from second column
        A[L+E+G+1][i]=-F*A[L+E+G+1][i]
    end
    local Error=0
    
    -- column number of change in variable
    local C=2
    --  number of all constraints
    local M=L+G+E
    --  column number of constant value part of constraints
    --  Ax=B
    --  1+N+G+L+E+G+1
    local B=M+N+G+2
    --  row number of objective
    --  also the last row of matrix "A"
    --  because "W+1" in "BlocA"
    local W=M+1
    --  because in pascal, matrix starts from zero row
    --  M=M-1
    
    --  ??? H ???
    --  maybe mark a number of optimization loops
    --H=1

    for k=1,M do
        --  add artifical variables for all constraints
        --  N+G+k+1:    1 jump the first column
        --              N jump the variables
        --              G jump the >= constraints, to save place for other variables
        --              k is for each constraints, <=,=,>=
        A[k][N+G+k+1]=1
        --  the numbering of initial base column
        A[k][1]=N+G+k+1
    end

    return A,Error,C,M,B,W

end

-- BlocA: artifical variables for "=" & >="-------------
function BlocA( L,G,E,N,A,M,W )
    --  add variables for ">=" constraints
    for k=(L+E+1),M do
        A[k][k-L-E+N]=-1
    end
    --  here, "W" is the last row
    W=W+1
    local Q=0
    for j=2,(N+G+1) do
        local S=0
        for i=(M-G-E+1),M do
            S=S+A[i][j]
        end
        A[W][j]=-S
        if A[W][j]<=Q then
            Q=A[W][j]
            C=j
        end
    end

    return C,Q,A,W

end

-- BlocB1: determine the change out variable------------
function BlocB1( Q,A,C,M,B )
    -- H=H+1
    --  using "Q"
    Q=9.9e37
    --  "R" indicate the row number of change out
    --  give it an impossible value "-1" initially
    local R=-1
    for i=1,M do
        --  "C" is the column number of change in variable
        --  it has been decided in "BlocC"
        if ( A[i][C]>0 ) then
            if ( (A[i][B]/A[i][C])<=Q ) then
                Q=A[i][B]/A[i][C]
                --  A[R][C] is used. this can be found in "BlocB2"
                R=i
            end
        end
    end

    return Q,R

end

-- BlocB2: rotation operations to change base variables
--         one step of optimization is done-------------
function BlocB2( A,C,B,W,R )
    local P=A[R][C]
    --  the first column is the number of base vectors
    --  this is to change base to new one
    A[R][1]=C
    for j=2,B do
        A[R][j]=A[R][j]/P
    end
    for i=1,W do
        if ( i~=R ) then
            for j=2,B do
                if ( j~=C ) then
                    A[i][j]=A[i][j]-A[R][j]*A[i][C]
                    if ( math.abs(A[i][j])<1e-9 ) then
                        A[i][j]=0
                    end
                end
            end
        end
    end
    --  this is to set the new base vector
    for i=1,W do
        A[i][C]=0
    end
    A[R][C]=1

    return A

end

-- BlocC: determine the change in variable--------------
function BlocC( L,G,N,A,C,W )
    local Q=0
    for j=2,(1+N+L+G) do
        --  because "-F", finding smallest "Q" means finding the biggest coefficient
        if ( A[W][j]<=Q ) then
            Q=A[W][j]
            --  corresponding column number of change in variable
            C=j
        end
    end

    return C,Q,A

end

-- BlocD: not used -------------------------------------
function BlocD()
    if ( not Stop ) then
        i=1
        while ( (not Stop) and (i<=(M+1)) ) do
            if ( (A[i][1]>=(N+G+L+1)) and (A[i][B]~=0) ) then
                Stop=true
                Error=2
            end
            i=i+1
        end
    end
end

-- BlocE: find the results and outputs------------------
function BlocE( N,F,A,B,W,Result )
    for i=1,N do
        j=1
        local Stop1=false
        --  "~=i+1" this is because the numbering of variable should +1 in matrix A
        --  the loop will be finish if find the variable
        --  the row number is saved in "j"
        while ( (A[j][1]~=i+1) and (not Stop1) ) do
            if ( j>=W ) then    -- use ">=" to avoid index of nil
                Result[i][1]=0
                Stop1=true
            end
            j=j+1
        end
        if ( not Stop1 ) then
            Result[i][1]=A[j][B]
        end
    end
    local Objective=F*A[W][B]

    return Result,Objective

end

-- Fine-------------------------------------------------
return optimize

-- -----------------------------------------------------
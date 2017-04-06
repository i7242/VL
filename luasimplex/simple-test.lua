local luasimplex = require("luasimplex")
local rsm = require("luasimplex.rsm")

local M =
{
  -- number of variables
  nvars = 2,
  -- number of constraints
  nrows = 1,

  --[[index for the variables of constraints
      the first one value in the array is the total number of indexes
      following, is the index
      for example, "1" is the first variable, and "2" is the second variable
      this is corresponding to the value of "elements"]]
  indexes = luasimplex.iarray(2, 1, 2),
  --[[elements are the coefficients in the linear constraints
      this is corresponding to indexes]]
  elements = luasimplex.darray(2, 1, 2),
  --[[because all the constraints were written in the one row_starts
      an index to say where to segment them is required
      row_starts is the index to say the starting and ending of a row of constraints
      first value is the total number of array
      the second value usually is "1"
      for the following, depending on where to stop the constraint equation
      is the number of position in "indexes" or "constraints"
      (they should be the same value if no problem)]]
  row_starts = luasimplex.iarray(2,1,3),
  --[["b" is the right side constants of the linear constraints]]
  b = luasimplex.darray(1,1.5),

  -- objective function
  c = luasimplex.darray(2, -1, -1),

  -- lower & upper bounds
  xl = luasimplex.darray(2, 0, 0),
  xu = luasimplex.darray(2, 1, 1),

}

local I = luasimplex.new_instance(M.nrows, M.nvars)
rsm.initialise(M, I, {})

objective, x = rsm.solve(M, I, {})

io.stderr:write(("Objective: %g\n"):format(objective))
io.stderr:write("  x:")
for i = 1, M.nvars do io.stderr:write((" %g"):format(x[i])) end
io.stderr:write("\n")


-- EOF -------------------------------------------------------------------------


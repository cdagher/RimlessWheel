
#!/bin/bash

julia_script="$JULIA_PROJECT/evaluateController.jl"
julia $julia_script $SAVED_WEIGHT_DIR


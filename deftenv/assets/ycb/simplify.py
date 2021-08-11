import meshlabxml as mlx
import sys

original_mesh = sys.argv[1] # input file
simplified_mesh = sys.argv[2] # output file

simplified_mesh = mlx.FilterScript(file_in=original_mesh, file_out=simplified_mesh, ml_version='2016.12') # Create FilterScript object 
mlx.remesh.simplify(simplified_mesh, texture=False, faces=1000,
                    target_perc=0.0, quality_thr=0.3)
simplified_mesh.run_script() # Run the script
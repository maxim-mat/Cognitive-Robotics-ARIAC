from check import solve_problems

kit_rob_loc = (0,0)  # x,y
ass_rob_loc = (0,0)  # x,y
ass_stations_loc = ((2,2), (0,2))  # x,y
kit_stations_loc = ((-2,0), (2,0))  # x,y
parts_required_ass = ((1,2,3),(1,3))  # type of part
parts_required_kit = ((2,4,4),(1,3,5))  # type of part
avilable_parts = ((1,0,1),(1,0,1),(1,0,1),(-1,0,4),(-1,0,4),(1.5,0,2),(1.5,0,2),(1,1,3),(-1,1,3),(3,3,6)) # x,y,type
"""
# for this test one 3 is missing, 3 are in diffrent locations, 6 isnt required, 5 is not avilable
# goal: parts required is empty or no more required parts are avilable.

the solution by running the algorithm:
[(('kit_rob', 'kit', 0, 3), ('ass_rob', 'kit', 0, 4)), (('kit_rob', 'kit', 1, 0), ('ass_rob', 'ass', 1, 6)), (('kit_rob', 'kit', 0, 2), ('ass_rob', 'kit', 1, 4)), ('ass_rob', 'ass', 0, 0), ('ass_rob', 'ass', 0, 1), ('ass_rob', 'ass', 1, 0)]

"""

problems = [(kit_rob_loc,ass_rob_loc,ass_stations_loc,kit_stations_loc,parts_required_ass,parts_required_kit,avilable_parts)]
strategy = solve_problems(problems)
print(strategy)


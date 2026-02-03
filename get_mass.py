#get mass function 
# ''This function computes rover mass in kilograms. It accounts for the chassis, power subsystem, science payload,
#and six wheel assemblies, which itself is comprised of a motor, speed reducer, and the wheel itself'''

#start with rover dictionary, should just be plug and chug from table values
# masses in kg
rover = { "chasis": 659.0, 
        " power subsystem" : 90.0 ,
        " science payload" : 75.0, 
        "wheel assembly" : {      # dictionary within, called by v = rover[wheel assembly][name]
            "wheel mass" : 6.0*1.0,
            "motor": 6.0*5.0 ,
            "speed reducer": 6.0*1.5 }}              #### multiplying by 6 to begin with, problem for future?

# make function, just a summation 
#''This function should validate that (a) the input is a dict. If this condition fails, 
    #call raise Exception(‘<message here>’) with a meaningful message to the user'

def get_mass(rover):
    if not isinstance(rover, dict): # cite GPT or Stack Overflow
        raise Exception('Input must be a dictionary, try again please')
    return sum(
        v if not isinstance(v, dict) else get_mass(v)
        for v in rover.values())

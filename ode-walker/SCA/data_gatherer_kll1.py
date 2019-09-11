import sys, string, os
import glob
import shutil
import random
import time

amount_iterations = 10000

settings = []

##Test pattern


settings.append(['NO_MACHINE_LEARNING\nNO_NEAREST_NEIGHBOR\nL1\nELU\n1\n0.1\n0.0'])
settings.append(['NO_MACHINE_LEARNING\nNEAREST_NEIGHBOR\nL1\nELU\n1\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNO_NEAREST_NEIGHBOR\nL1\nELU\n5\n0.1\n0.0'])

settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nL1\nELU\n5\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nL1\nBSELU\n5\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nL1\nBSELU\n5\n0.0\n0.001'])

settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n1\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n2\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n3\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n4\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n5\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n6\n0.1\n0.0'])
settings.append(['MACHINE_LEARNING\nNEAREST_NEIGHBOR\nKL\nBSELU\n7\n0.1\n0.0'])


##### Create folder to store the data

dist = "Data/"
folder_name_length = 10  

for setting in settings:
    
    tmp_folder = ''.join(random.choice(string.ascii_lowercase) for _ in range(folder_name_length))
    tmp_folder = dist + tmp_folder

    created = False

    while (not created):
        try:
            print("Made folder: " + tmp_folder)
            os.makedirs(tmp_folder)
            created = True
        except OSError:
            print(tmp_folder + " exists already")
            tmp_folder = ''.join(random.choice(string.ascii_lowercase) for _ in range(folder_name_length))
            tmp_folder = dist + tmp_folder

    setting.append(tmp_folder)

for _ in range(amount_iterations):
    for settings_string, target_folder in settings:

        file=open('settings.txt', 'w')
        file.write(settings_string)
        file.close()

        os.system("drawstuffrenderer.exe")
        os.system("awake_script.py")

        extension = 'csv'
        result = [i for i in glob.glob('*.{}'.format(extension))]
        for file in result:
            print(file)
            shutil.move(file,target_folder+'/'+file)





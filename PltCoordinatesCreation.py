import numpy as np

positioni=[]
with open("SquarePltPoses.txt","r") as f_read: # CirclePltPoses //  SquarePltPoses
    for line in f_read:        
        positioni.append(line.rstrip('\n').split(","))
pltPos=np.array(positioni)

pltPosList=pltPos.tolist()

# Q1=np.zeros( (len(positioni),7) )
# Q2=np.zeros( (len(positioni),7) )
# Q3=np.zeros( (len(positioni),7) )
# for j in range(0, len(positioni)):
#     Q1[j,:] = np.hstack( ( float(ActPos[j,0]) + float(PPi[indexQ1,0]), PPi[indexQ1,1:] ))
#     Q2[j,:] = np.hstack( ( float(ActPos[j,1]) + float(PPi[indexQ2,0]), PPi[indexQ2,1:] ))
#     Q3[j,:] = np.hstack( ( float(ActPos[j,2]) + float(PPi[indexQ3,0]) , PPi[indexQ3,1:] ))
# Q1=Q1.tolist()
# Q2=Q2.tolist()
# Q3=Q3.tolist()
# print(len(positioni))
print('Platform poses created')
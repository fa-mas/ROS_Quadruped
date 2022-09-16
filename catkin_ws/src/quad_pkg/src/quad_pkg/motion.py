import numpy as np
from numpy import array as arr
import numpy.linalg as la
from quad_pkg import config
from math import floor

# max/min angles that servos should move (2<=x<=178)
minAngle = 2
maxAngle = 178

class Leg:

    def __init__(self, vec, vRef1, vRefW, initAlpha, initBeta, initGamma, alphaOffset, betaOffset, gammaOffset, legLen1, legLen2, legLen3):

        # vec -> start vec for iteration
        # vRef1 -> reference vector for orientation in XY-Plane
        # gammaOffset -> offset angle for gamma
        # vecOffset -> 3D array with z-offset (only for reference, not calc!)

        self.vecOffset = arr ([0, 0, 30]) # offset in Z-direction, use only for reference not for calc!!

        self.vRef1 = vRef1 # vRef1 -> Reference vector XY for absolute angle calc at Hip 1 (e R^[2x1])
        self.vRef2 = arr([-1, 0]) # vRef2 -> Reference vector ZX for absolute angle calc at Hip 2 (e R^[2x1])
        self.initRefW = vRefW
        self.vRefW = self.initRefW # vRefW -> Reference vector for angle calc at Hip 1 for BodyTurn 
        # actual leg lengths
        self.legLen1 = legLen1
        self.legLen2 = legLen2
        self.legLen3 = legLen3
        # legLen modification for smoother movement
        self.legLen1Mod = 1
        self.legLen2Mod = 1
        self.legLen3Mod = 1
        self.initVec = vec # initial Legvector
        self.vec = self.initVec  # current Vector in XY-Plane, give initial leg vector (used as previous vector for iteration in functions, updated after function finished)
        
        # Reference vectors for overview only
        self.vPrev = vec + self.vecOffset # previous Vector before function was executed, DONT USE FOR ITERATION in functions
        self.vCurr = vec + self.vecOffset # current Vector in XYZ, DONT USE FOR ITERATION in functions
        self.vNext = vec + self.vecOffset # Vector after function is executed, DONT USE FOR ITERATION in functions

        # Correction
        self.corr = 0.97        # Servo correction
        self.alphaOffset = alphaOffset
        self.betaOffset = betaOffset
        self.gammaOffset = gammaOffset  # gamma offset

        # initial angles
        self.initAlpha = initAlpha
        self.initBeta = initBeta
        self.initGamma = initGamma

        # angles
        self.alpha = initAlpha
        self.beta = initBeta
        self.gamma = initGamma

        # corrected and offset angles
        self.alphaCorr = self.alpha*self.corr + self.alphaOffset
        self.betaCorr = self.beta*self.corr + self.betaOffset
        self.gammaCorr = self.gamma*self.corr + self.gammaOffset

    def GoToInit(self):

        self.vec = self.initVec

        # Reference vectors for overview only
        self.vPrev = self.vec + self.vecOffset # previous Vector before function was executed, DONT USE FOR ITERATION in functions
        self.vCurr = self.vec + self.vecOffset # current Vector in XYZ, DONT USE FOR ITERATION in functions
        self.vNext = self.vec + self.vecOffset # Vector after function is executed, DONT USE FOR ITERATION in functions

        self.alpha = self.initAlpha
        self.beta = self.initBeta
        self.gamma = self.initGamma

        # reset Reference Vectors for Turn calc
        self.vRefW = self.initRefW

        # reset leg length modification
        self.legLen1Mod = 1
        self.legLen2Mod = 1
        self.legLen3Mod = 1

        self.alphaCorr = self.initAlpha*self.corr + self.alphaOffset
        self.betaCorr = self.initBeta*self.corr + self.betaOffset
        self.gammaCorr = self.initGamma*self.corr + self.gammaOffset

    def COMIncrementalShift(self, vInpTot, nInc, incSum):
        # calc each angle for Center of Mass shift 
        
        # INPUT:
        # vInpTot -> total inputvector (e R^[3x1])
        # nInc -> number of increments (e N)
        # incSum -> Sum of increments (e R^[3x1])

        # legLen modification for smoother movement
        legLen1 = self.legLen1*self.legLen1Mod
        legLen2 = self.legLen2*self.legLen2Mod
        legLen3 = self.legLen3*self.legLen3Mod

        # calc increment of Inputvector
        vInc = vInpTot/nInc # single Increment
        incSum = incSum + vInc

        # calc new Vector
        vNew = self.vec - vInc

        # current Vector (only for reference)
        self.vCurr = vNew + self.vecOffset

        # initial Vector (before Function was executed)
        vInit = vNew + incSum
        self.vPrev = vInit + self.vecOffset

        # End Vector which will be reached after Function is executed
        vEnd = vInit - vInpTot
        self.vNext = vEnd + self.vecOffset

        # calc alpha (angle 1 at hip)
        vNewXY = arr([ vNew[0], vNew[1] ]) # Transform to 2D
        vNewXYLen = la.norm(vNewXY) # length
        scalProdAlpha = np.inner(self.vRef1, vNewXY)
        normsAlpha = la.norm(self.vRef1) * vNewXYLen
        cosAlpha = scalProdAlpha / normsAlpha
        self.alpha = np.rad2deg(np.arccos(np.clip(cosAlpha, -1.0, 1.0))) # np.clip limits value to defined range (definition range of arccos [-1,1])
        self.alphaCorr = self.alpha*self.corr + self.alphaOffset

        # calc auxilliary vector w
        vNewT = np.array([ la.norm([vNew[0], vNew[1]]), vNew[2] ]) # Transform to new KoSys ZX'
        s = arr([legLen1,0]) # Leg length 1 (first at Hip) as vector
        w = vNewT - s 
        wLen = la.norm(w)

        # calc beta (angle 2 at hip)
        cosTeta = ( np.power(legLen3, 2)-np.power(legLen2, 2)-np.power(wLen, 2) )/( -2*legLen2*wLen ) # cosinussatz
        teta = np.rad2deg(np.arccos(np.clip(cosTeta, -1.0, 1.0))) # np.clip limits value to defined range (definition range of arccos [-1,1]) values greater or smaller than one aro caused by machin precision (probably)
        scalProdPhi = np.inner(self.vRef2, w)
        normsPhi = la.norm(self.vRef2) * wLen
        cosPhi = scalProdPhi / normsPhi
        phi = np.rad2deg(np.arccos(np.clip(cosPhi, -1.0, 1.0)))
        self.beta = teta + phi - 90
        self.betaCorr = self.beta*self.corr + self.betaOffset

        # calc gamma (angle at knee)
        cosGamma = (np.power(wLen, 2)-np.power(legLen2, 2)-np.power(legLen3, 2))/(-2*legLen2*legLen3)
        self.gamma = np.rad2deg(np.arccos(np.clip(cosGamma, -1.0, 1.0))) # np.clip limits value to defined range (definition range of arccos [-1,1])
        self.gammaCorr = self.gamma*self.corr + self.gammaOffset

        # print(cosAlpha, cosTeta, cosPhi, cosGamma) # should not be >1 or <-1

        # update vector
        self.vec = vNew

        return incSum
    
    def IncrementalStep(self, vInpTot, nInc, incSum, stepHeight):

        # calc each angle incremental Step
        
        # INPUT:
        # vInpTot -> total inputvector (e R^[3x1])
        # nInc -> number of increments (e N)
        # incSum -> Sum of increments (e R^[3x1])
        # stepHeight -> Height of Step (measured in positive direction from highest point of step) (e Q)

        # legLen modification for smoother movement
        legLen1 = self.legLen1*self.legLen1Mod
        legLen2 = self.legLen2*self.legLen2Mod
        legLen3 = self.legLen3*self.legLen3Mod
        
        # calc increment of Inputvector
        vInc = vInpTot/nInc # single Increment
        incSum = incSum + vInc

        # calc new Vector
        vNew = self.vec + vInc # New vector after increment

        # initial Vector (before Function was executed)
        vInit = vNew - incSum
        self.vPrev = vInit + self.vecOffset

        # End Vector which will be reached after Function is executed
        vEnd = vInit + vInpTot
        self.vNext = vEnd + self.vecOffset

        # calc alpha (angle 1 at hip)
        vNewXY = arr([ vNew[0], vNew[1] ]) # Transform to 2D
        vNewXYLen = la.norm(vNewXY) # length
        scalProdAlpha = np.inner(self.vRef1, vNewXY)
        normsAlpha = la.norm(self.vRef1) * vNewXYLen
        cosAlpha = scalProdAlpha / normsAlpha
        self.alpha = np.rad2deg(np.arccos(np.clip(cosAlpha, -1.0, 1.0))) # np.clip limits value to defined range (definition range of arccos [-1,1])
        self.alphaCorr = self.alpha*self.corr + self.alphaOffset

        # calculate Total height for step
        if (vInc[2] + self.vec[2]) > self.vec[2]: # step up
            totHeight = stepHeight + abs(vInit[2] - vEnd[2])
        else: # step down or even
            totHeight = stepHeight     

        # calc auxilliary vector hi (for stepheight of increment)
        eXY = la.norm([ vInpTot[0], vInpTot[1] ]) # steplength resulting from vInp
        ei = la.norm([ incSum[0], incSum[1] ]) # Sum of increments transformed to KoSys of Parabola
        a = ((2*vInpTot[2]) - (4*totHeight)) / (eXY**2) # coeff1 for Parabola
        b = ((4*totHeight) - vInpTot[2]) / eXY # coeff2 for Parabola
        h = a*(ei**2) + b*ei # Parabola
        hi = arr([ 0, (h-incSum[2]) ]) # heightvector

        self.vCurr = arr([ vNew[0], vNew[1], vNew[2]+hi[1]]) + self.vecOffset 

        # calc auxilliary vector w
        vNewT = np.array([ la.norm([vNew[0], vNew[1]]), vNew[2] ]) # Transform di to new KoSys ZX'
        s = arr([legLen1,0]) # Leg length 1 (first at Hip) as vector
        w = vNewT + hi - s
        wLen = la.norm(w)

        # calc beta (angle 2 at hip)
        cosTeta = (np.power(legLen3, 2)-np.power(legLen2, 2)-np.power(wLen, 2))/(-2*legLen2*wLen) # cosinussatz
        teta = np.rad2deg(np.arccos(np.clip(cosTeta, -1.0, 1.0))) # np.clip limits value to defined range (definition range of arccos [-1,1]) values greater or smaller than one are caused by machin precision (probably)
        scalProdPhi = np.inner(self.vRef2, w)
        normsPhi = la.norm(self.vRef2) * wLen
        cosPhi = scalProdPhi / normsPhi
        phi = np.rad2deg(np.arccos(np.clip(cosPhi, -1.0, 1.0)))
        self.beta = teta + phi - 90
        self.betaCorr = self.beta*self.corr + self.betaOffset

        # calc gamma (angle at knee)
        cosGamma = (np.power(wLen, 2)-np.power(legLen2, 2)-np.power(legLen3, 2))/(-2*legLen2*legLen3)
        # print(np.clip(cosGamma, -1.0, 1.0)) # should not be >1 or <-1
        self.gamma = np.rad2deg(np.arccos(np.clip(cosGamma, -1.0, 1.0)))
        self.gammaCorr = self.gamma*self.corr + self.gammaOffset

        # update Vector
        self.vec = vNew # New vector after increment

        return incSum

    def BodyVecTurn(self, Angle): # calculate vInp to turn with COMIncrementalShift 
        # Rotation matrix
        rMat =arr([[ np.cos(Angle), -np.sin(Angle), 0 ],
                   [ np.sin(Angle),  np.cos(Angle), 0 ],
                   [ 0               , 0                , 1 ]])

        vRefWn = np.matmul(rMat, self.vRefW) # next reference vector 
        turnVec = (self.vRefW - vRefWn) # incremental input vector

        # update reference Vector 
        self.vRefW = vRefWn

        return turnVec

    def StepVecTurn(self, Angle): # totAngle [rad]
        # returns input vector for steps in 'WholeTurn'
        rMat =arr([[ np.cos(Angle), -np.sin(Angle), 0 ],
                   [ np.sin(Angle),  np.cos(Angle), 0 ],
                   [ 0               , 0          , 1 ]])
    
        stepVec = (self.vec) - np.matmul(rMat, self.vec)

        # update vRefW
        self.vRefW = np.matmul(rMat, self.vRefW)

        return stepVec

class Quadruped:
    def __init__(self, legA, legB, legC, legD):
        self.legA = legA
        self.legB = legB
        self.legC = legC
        self.legD = legD

    def ServoFeed(self):
        config.kit.servo[2].angle = limitAngle(self.legA.alphaCorr, minAngle, maxAngle)
        config.kit.servo[3].angle = limitAngle(self.legA.betaCorr, minAngle, maxAngle)
        config.kit.servo[4].angle = limitAngle(self.legA.gammaCorr, minAngle, maxAngle)

        config.kit.servo[5].angle = limitAngle(self.legB.alphaCorr, minAngle, maxAngle)
        config.kit.servo[6].angle = limitAngle(self.legB.betaCorr, minAngle, maxAngle)
        config.kit.servo[7].angle = limitAngle(self.legB.gammaCorr, minAngle, maxAngle)

        config.kit.servo[8].angle = limitAngle(self.legC.alphaCorr, minAngle, maxAngle)
        config.kit.servo[9].angle = limitAngle(self.legC.betaCorr, minAngle, maxAngle)
        config.kit.servo[10].angle = limitAngle(self.legC.gammaCorr, minAngle, maxAngle)

        config.kit.servo[11].angle = limitAngle(self.legD.alphaCorr, minAngle, maxAngle)
        config.kit.servo[12].angle = limitAngle(self.legD.betaCorr, minAngle, maxAngle)
        config.kit.servo[13].angle = limitAngle(self.legD.gammaCorr, minAngle, maxAngle)

    def IncrementalBodyTurn4Legs(self, vA, vB, vC, vD, incSum):

        incSum = self.legA.COMIncrementalShift(vA, 1, incSum)
        self.legB.COMIncrementalShift(vB, 1, incSum)
        self.legC.COMIncrementalShift(vC, 1, incSum)
        self.legD.COMIncrementalShift(vD, 1, incSum)
        return incSum

    def COMIncrementalShift4Legs(self, vInpTot, nInc, incSum):
        incSum = self.legA.COMIncrementalShift(vInpTot, nInc, incSum)
        self.legB.COMIncrementalShift(vInpTot, nInc, incSum)
        self.legC.COMIncrementalShift(vInpTot, nInc, incSum)
        self.legD.COMIncrementalShift(vInpTot, nInc, incSum)
        return incSum

    def CrossCrawlSl(self, vInpTot, nInc, stepHeight):

        # inputvector length
        vInpTotLen = la.norm(vInpTot) 

        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200
        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        # define leg order and COM-vectors depending on direction
        if (vInpTot[0] > 0) and (vInpTot[1] == 0): # forward
            leg = [self.legD, self.legB, self.legC, self.legA]
            relComp = 0 # relevant component of vInpTot, used to calc COM vectors (either x- or y-component of vInpTot)

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           -vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        elif (vInpTot[0] < 0) and (vInpTot[1] == 0): # backwards
            leg = [self.legB, self.legD, self.legA, self.legC]
            relComp = 0

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           -vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] > 0): # left
            leg = [self.legC, self.legA, self.legB, self.legD]
            relComp = 1

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([-vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] < 0): # right
            leg = [self.legA, self.legC, self.legD, self.legB]
            relComp = 1

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([-vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        else:
            print('\n\nInputvectors for steps can either have x&z- or y&z-components, not x&y (one must be 0)\n\n')
            return 0

        # COM vector lengths
        c1Len = la.norm(c1)
        c2Len = la.norm(c2)
        c3Len = la.norm(c3)
        c4Len = la.norm(c4)

        # START MOVEMENT

        # Go to initial position
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()
        self.ServoFeed()
        print('Slow cross-crawl started, initial Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr, )

        # COM shift 1
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < c1Len:
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = self.COMIncrementalShift4Legs(c1, nInc/2, incSum) # ninc/2 because of short inpVec
            incSumLen = la.norm(incSum)
            # print('COM:', c1)
            # FEED SERVOS
            self.ServoFeed()

        # step 1
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < vInpTotLen:
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = leg[0].IncrementalStep(vInpTot, nInc, incSum, stepHeight)
            incSumLen = la.norm(incSum)
            # print('Step previous:', leg[0].vPrev, 'current:', leg[0].vCurr,'next:', leg[0].vNext)
            # FEED SERVOS
            self.ServoFeed()

        # COM shift 2
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < c2Len:
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = self.COMIncrementalShift4Legs(c2, nInc, incSum)
            incSumLen = la.norm(incSum)
            # print('COM:', c2)
            # FEED SERVOS
            self.ServoFeed()

        # step 2
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < vInpTotLen:
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = leg[1].IncrementalStep(vInpTot, nInc, incSum, stepHeight)
            incSumLen = la.norm(incSum)
            # print('Step previous:', leg[1].vPrev, 'current:', leg[1].vCurr,'next:', leg[1].vNext)
            # FEED SERVOS
            self.ServoFeed()

        # step 3
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < vInpTotLen:  
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = leg[2].IncrementalStep(vInpTot, nInc, incSum, stepHeight)
            incSumLen = la.norm(incSum)
            # print('Step previous:', leg[2].vPrev, 'current:', leg[2].vCurr,'next:', leg[2].vNext)
            # FEED SERVOS
            self.ServoFeed()

        # COM shift 3
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < c3Len:
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = self.COMIncrementalShift4Legs(c3, nInc, incSum)
            incSumLen = la.norm(incSum)
            # print('COM:', c3)
            # FEED SERVOS
            self.ServoFeed()


        # step 4
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < vInpTotLen:  
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = leg[3].IncrementalStep(vInpTot, nInc, incSum, stepHeight)
            incSumLen = la.norm(incSum)
            # print('Step previous:', leg[3].vPrev, 'current:', leg[3].vCurr,'next:', leg[3].vNext)
            # FEED SERVOS
            self.ServoFeed()


        # COM shift 4
        incSum = arr([0, 0, 0])
        incSumLen = la.norm(incSum)
        while incSumLen < c4Len:
            # UPDATE ANGLES AND VECTORS AND INCREASE INCSUMLEN
            incSum = self.COMIncrementalShift4Legs(c4, nInc/2, incSum) # ninc/2 bec short inpVec
            incSumLen = la.norm(incSum)
            # print('COM:', c4)
            # FEED SERVOS
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)

        # return to initial position (deviation because of machine precision)
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()
        self.ServoFeed()
        print('Slow cross-crawl finished, current Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr,'\n' )

    def CrossCrawlFs(self, vInpTot, nInc, stepHeight):

        # inputvector length
        vInpTotLen = la.norm(vInpTot) 

        # legLen modification for smoother movement (values found by testing)
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200
        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        # define leg order and COM-vectors depending on direction
        if (vInpTot[0] > 0) and (vInpTot[1] == 0): # forward
            leg = [self.legD, self.legB, self.legC, self.legA]
            relComp = 0 # relevant component of vInpTot, used to calc COM vectors (either x- or y-component of vInpTot)

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           -vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        elif (vInpTot[0] < 0) and (vInpTot[1] == 0): # backwards
            leg = [self.legB, self.legD, self.legA, self.legC]
            relComp = 0

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           -vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] > 0): # left
            leg = [self.legC, self.legA, self.legB, self.legD]
            relComp = 1

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([-vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] < 0): # right
            leg = [self.legA, self.legC, self.legD, self.legB]
            relComp = 1

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],     (1/4)*vInpTot[2]])
            c2 = arr([vInpTot[relComp],           vInpTot[relComp],        (1/4)*vInpTot[2]])
            c3 = arr([-vInpTot[relComp],           vInpTot[relComp],         (1/4)*vInpTot[2]])    
            c4 = arr([0.5*vInpTot[relComp],      -0.5*vInpTot[relComp],    (1/4)*vInpTot[2]])

        else:
            print('\n\nInputvectors for steps can either have x&z- or y&z-components, not x&y (one must be 0)\n\n')
            return 0

        # COM vector lengths
        c1Len = la.norm(c1)
        c2Len = la.norm(c2)
        c3Len = la.norm(c3)
        c4Len = la.norm(c4)

        # START MOVEMENT
        # Go to initial position
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()
        self.ServoFeed()
        print('Fast cross-crawl started, initial Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr, )

        ratio = 5   # COM vector will be executed until distance (1/ratio)*c (measured from diagonal) is reached before step is started
                    # alter if COM shift not proper (the smaller "ratio" the safer the stand, the slower the movement)
                    # must be bigger than 2

        # COM SHIFT AND STEP1
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < c1Len/(ratio/2): # COM shift start
            
            incSumC = self.COMIncrementalShift4Legs(c1, nInc/2, incSumC) #  nInc/2 because of short COM shift vector
            incSumLenC = la.norm(incSumC)

            self.ServoFeed()

        while incSumLenC < c1Len: # COM shift & step

            incSumC = self.COMIncrementalShift4Legs(c1, nInc/2, incSumC) 
            incSumLenC = la.norm(incSumC)

            incSumV = leg[0].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)
            
            self.ServoFeed()
        
        # COM SHIFT AND STEP2
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < ((c2Len/2)+(c2Len/ratio)): # COM shift start  
            
            incSumC = self.COMIncrementalShift4Legs(c2, nInc, incSumC)
            incSumLenC = la.norm(incSumC)

            self.ServoFeed()

        while incSumLenC < c2Len: # COM shift & step
            
            incSumC = self.COMIncrementalShift4Legs(c2, nInc, incSumC)
            incSumLenC = la.norm(incSumC)
            
            incSumV = leg[1].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)

            self.ServoFeed()

        # STEP 3
        incSumV = arr([0, 0, 0])
        incSumLenV = la.norm(incSumV)
        while incSumLenV < vInpTotLen:  
        
            incSumV = leg[2].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)
        
            self.ServoFeed()

        # COM SHIFT AND STEP 4
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < ((c2Len/2)+(c2Len/ratio)): # COM shift start 
            
            incSumC = self.COMIncrementalShift4Legs(c3, nInc, incSumC)
            incSumLenC = la.norm(incSumC)
            
            self.ServoFeed()

        # FINISH COM SHIFT AND START STEP 4
        while incSumLenC < c3Len: # COM shift & step
            
            incSumC = self.COMIncrementalShift4Legs(c3, nInc, incSumC)
            incSumLenC = la.norm(incSumC)
            
            incSumV = leg[3].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)

            self.ServoFeed()


        # COM SHIFT 4
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
    
        while incSumLenC < c4Len:
            incSumC = self.COMIncrementalShift4Legs(c4, nInc/2, incSumC)
            incSumLenC = la.norm(incSumC)
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)

        # return to initial position (deviation because of machine precision)
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()
        self.ServoFeed()
        print('Fast cross-crawl finished, current Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr,'\n' )

    def CircleCrawlFs(self, vInpTot, nInc, stepHeight):

        # inputvector length
        vInpTotLen = la.norm(vInpTot) 

        # legLen modification for smoother movement (values found by testing)
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200
        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3
        
        # define leg order and COM-vectors depending on direction
        if (vInpTot[0] > 0) and (vInpTot[1] == 0): # forward
            leg = [self.legD, self.legC, self.legB, self.legA]

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTotLen,      0.5*vInpTotLen,     (1/5)*vInpTot[2]])
            c2 = arr([0,                    -vInpTotLen,        (1/5)*vInpTot[2]])
            c3 = arr([(3/2)*vInpTotLen,     0,                  (1/5)*vInpTot[2]])    
            c4 = arr([0.5*vInpTotLen,       vInpTotLen,         (1/5)*vInpTot[2]])
            c5 = arr([-0.5*vInpTotLen,      -0.5*vInpTotLen,    (1/5)*vInpTot[2]])

        elif (vInpTot[0] < 0) and (vInpTot[1] == 0): # backwards
            leg = [self.legB, self.legA, self.legD, self.legC]

            # COM vectors vInpTot[0]
            c1 = arr([0.5*vInpTotLen,       -0.5*vInpTotLen,    (1/5)*vInpTot[2]])
            c2 = arr([0,                    vInpTotLen,         (1/5)*vInpTot[2]])
            c3 = arr([-(3/2)*vInpTotLen,    0,                  (1/5)*vInpTot[2]])    
            c4 = arr([-0.5*vInpTotLen,      -vInpTotLen,        (1/5)*vInpTot[2]])
            c5 = arr([0.5*vInpTotLen,       0.5*vInpTotLen,     (1/5)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] > 0): # left
            leg = [self.legC, self.legB, self.legA, self.legD]

            # COM vectors vInpTot[0]
            c1 = arr([-0.5*vInpTotLen,      -0.5*vInpTotLen,    (1/4)*vInpTot[2]])
            c2 = arr([vInpTotLen,           0,                  (1/5)*vInpTot[2]])
            c3 = arr([0,                    (3/2)*vInpTotLen,   (1/5)*vInpTot[2]])    
            c4 = arr([-vInpTotLen,          0.5*vInpTotLen,     (1/5)*vInpTot[2]])
            c5 = arr([0.5*vInpTotLen,       -0.5*vInpTotLen,    (1/5)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] < 0): # right
            leg = [self.legA, self.legD, self.legC, self.legB]

            # COM vectors vInpTot[0]
            c1 = arr([0.5*vInpTotLen,       0.5*vInpTotLen,     (1/4)*vInpTot[2]])
            c2 = arr([-vInpTotLen,          0,                  (1/5)*vInpTot[2]])
            c3 = arr([0,                    -(3/2)*vInpTotLen,  (1/5)*vInpTot[2]])    
            c4 = arr([vInpTotLen,           -0.5*vInpTotLen,    (1/5)*vInpTot[2]])
            c5 = arr([-0.5*vInpTotLen,      0.5*vInpTotLen,     (1/5)*vInpTot[2]])

        else: 
            print('\n\nInputvectors for steps can either have x&z- or y&z-components, not x&y (one must be 0)\n\n')
            return 0


        # COM vector lengths
        c1Len = la.norm(c1)
        c2Len = la.norm(c2)
        c3Len = la.norm(c3)
        c4Len = la.norm(c4)
        c5Len = la.norm(c5)

        # START MOVEMENT
        print('Fast circle-crawl started, initial Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr, )

        ratio = 5   # COM vector will be executed until distance (1/ratio)*c (measured from diagonal) is reached before step is started
                    # alter if COM shift not proper (the smaller "ratio" the safer the stand, the slower the movement)

        # COM SHIFT AND STEP1
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < c1Len/(ratio/2): # COM shift start
            
            incSumC = self.COMIncrementalShift4Legs(c1, nInc/2, incSumC) #  nInc/2 because of short COM shift vector
            incSumLenC = la.norm(incSumC)

            self.ServoFeed()

        while incSumLenC < c1Len: # COM shift & step

            incSumC = self.COMIncrementalShift4Legs(c1, nInc/2, incSumC) 
            incSumLenC = la.norm(incSumC)

            incSumV = leg[0].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)
           
            self.ServoFeed()

        # input('stop')
        
        # COM SHIFT AND STEP2
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < ((c2Len/2)+(c2Len/ratio)): # COM shift start  
            
            incSumC = self.COMIncrementalShift4Legs(c2, nInc, incSumC)
            incSumLenC = la.norm(incSumC)

            self.ServoFeed()

        while incSumLenC < c2Len: # COM shift & step
            
            incSumC = self.COMIncrementalShift4Legs(c2, nInc, incSumC)
            incSumLenC = la.norm(incSumC)
            
            incSumV = leg[1].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)

            self.ServoFeed()

        # input('stop')

        # COM SHIFT AND STEP 3
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < ((c3Len/2)+(c3Len/ratio)): # COM shift start  
            
            incSumC = self.COMIncrementalShift4Legs(c3, nInc, incSumC)
            incSumLenC = la.norm(incSumC)

            self.ServoFeed()

        while incSumLenC < c3Len: # COM shift & step
            
            incSumC = self.COMIncrementalShift4Legs(c3, nInc, incSumC)
            incSumLenC = la.norm(incSumC)
            
            incSumV = leg[2].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)

            self.ServoFeed()

        # input('stop')

        # COM SHIFT AND STEP 4
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
        incSumV = arr([0, 0, 0])
        incSumV = la.norm(incSumV)

        while incSumLenC < ((c4Len/2)+(c4Len/ratio)): # COM shift start  
            
            incSumC = self.COMIncrementalShift4Legs(c4, nInc, incSumC)
            incSumLenC = la.norm(incSumC)

            self.ServoFeed()

        while incSumLenC < c4Len: # COM shift & step
            
            incSumC = self.COMIncrementalShift4Legs(c4, nInc, incSumC)
            incSumLenC = la.norm(incSumC)
            
            incSumV = leg[3].IncrementalStep(vInpTot, nInc*(((ratio/2)-1)/ratio), incSumV, stepHeight)
            incSumLenV = la.norm(incSumV)

            self.ServoFeed()

        # input('stop')

        # COM SHIFT 5
        incSumC = arr([0, 0, 0])
        incSumLenC = la.norm(incSumC)
    
        while incSumLenC < c5Len:
            incSumC = self.COMIncrementalShift4Legs(c5, nInc/2, incSumC)
            incSumLenC = la.norm(incSumC)
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)

        # return to initial position (deviation because of machine precision)
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()
        self.ServoFeed()
        print('Fast circle-crawl finished, current Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr,'\n' )

    def Run2x2Interm(self, vInpTot, nInc, stepHeight): 

        leg = [self.legA, self.legC, self.legB, self.legD]

        # legLen modification for smoother movement (values found by testing)
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200

        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        # COM Vector
        c1 = vInpTot/2

        # vector lengths
        vInpTotLen = la.norm(vInpTot)
        c1Len = la.norm(c1)
        stepLen = la.norm(vInpTot)

        # COM SHIFT AND STEP1
        print("Run2x2 interm")
        incSumS1 = arr([0, 0, 0])
        incSumS2 = arr([0, 0, 0])
        incSumLenS = la.norm(incSumS1)
        incSumC = arr([0, 0, 0])

        while incSumLenS < stepLen: 
            # com
            incSumC = self.COMIncrementalShift4Legs(c1, nInc, incSumC)
            # step
            incSumS1 = leg[0].IncrementalStep(vInpTot, nInc, incSumS1, stepHeight)
            incSumS2 = leg[1].IncrementalStep(vInpTot, nInc, incSumS2, stepHeight)
            incSumLenS = la.norm(incSumS1)
            self.ServoFeed()

        # COM SHIFT AND STEP2
        incSumS1 = arr([0, 0, 0])
        incSumS2 = arr([0, 0, 0])
        incSumLenS = la.norm(incSumS1)
        incSumC = arr([0, 0, 0])
        
        while incSumLenS < stepLen: 
            # com
            incSumC = self.COMIncrementalShift4Legs(c1, nInc, incSumC)
            # step
            incSumS1 = leg[2].IncrementalStep(vInpTot, nInc, incSumS1, stepHeight)
            incSumS2 = leg[3].IncrementalStep(vInpTot, nInc, incSumS2, stepHeight)
            incSumLenS = la.norm(incSumS1)
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)

    def Run2x2IntermExp(self, vInpTot, nInc, stepHeight): # experimental

        # inputvector length
        vInpTotLen = la.norm(vInpTot) 

        # define leg order and COM-vectors depending on direction

        if (vInpTot[0] > 0) and (vInpTot[1] == 0): # forward
            leg = [self.legA, self.legC, self.legB, self.legD]
            # COM vectors 
            c1 = arr([0.5*vInpTotLen,      -0.5*vInpTotLen,     (1/2)*vInpTot[2]])
            c2 = arr([0.5*vInpTotLen,       0.5*vInpTotLen,     (1/2)*vInpTot[2]])

        elif (vInpTot[0] < 0) and (vInpTot[1] == 0): # backwards
            leg = [self.legA, self.legC, self.legB, self.legD]
            # COM vectors 
            c1 = arr([-0.5*vInpTotLen,      0.5*vInpTotLen,     (1/2)*vInpTot[2]])
            c2 = arr([-0.5*vInpTotLen,     -0.5*vInpTotLen,     (1/2)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] > 0): # LEFT
            leg = [self.legB, self.legD, self.legA, self.legC]
            # COM vectors 
            c1 = arr([0.5*vInpTotLen,       0.5*vInpTotLen,     (1/2)*vInpTot[2]])
            c2 = arr([-0.5*vInpTotLen,      0.5*vInpTotLen,     (1/2)*vInpTot[2]])

        elif (vInpTot[0] == 0) and (vInpTot[1] < 0): # RIGHT
            leg = [self.legB, self.legD, self.legA, self.legC]
            # COM vectors 
            c1 = arr([-0.5*vInpTotLen,       -0.5*vInpTotLen,     (1/2)*vInpTot[2]])
            c2 = arr([ 0.5*vInpTotLen,        -0.5*vInpTotLen,     (1/2)*vInpTot[2]])

        else: 
            print('\n\nInputvectors for steps can either have x&z- or y&z-components, not x&y (one must be 0)\n\n')
            return 0

        # legLen modification for smoother movement (values found by testing)
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200

        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        # COM & step vector
        stepVec = vInpTot

        # vector lengths
        c1Len = la.norm(c1)
        stepLen = la.norm(stepVec)

        # COM SHIFT AND STEP1
        print("Run2x2 interm")
        incSumS1 = arr([0, 0, 0])
        incSumS2 = arr([0, 0, 0])
        incSumLenS = la.norm(incSumS1)
        incSumC = arr([0, 0, 0])

        while incSumLenS < stepLen: 
            # com
            incSumC = self.COMIncrementalShift4Legs(c1, nInc, incSumC)
            # step
            incSumS1 = leg[0].IncrementalStep(stepVec, nInc, incSumS1, stepHeight)
            incSumS2 = leg[1].IncrementalStep(stepVec, nInc, incSumS2, stepHeight)
            incSumLenS = la.norm(incSumS1)
            self.ServoFeed()

        print("COM VECTORS: ", c1, incSumC)
        print("STEP VECTORS: ", stepVec, incSumS1, incSumS2)

        # COM SHIFT AND STEP2
        incSumS1 = arr([0, 0, 0])
        incSumS2 = arr([0, 0, 0])
        incSumLenS = la.norm(incSumS1)
        incSumC = arr([0, 0, 0])
        
        while incSumLenS < stepLen: 
            # com
            incSumC = self.COMIncrementalShift4Legs(c2, nInc, incSumC)
            # step
            incSumS1 = leg[2].IncrementalStep(stepVec, nInc, incSumS1, stepHeight)
            incSumS2 = leg[3].IncrementalStep(stepVec, nInc, incSumS2, stepHeight)
            incSumLenS = la.norm(incSumS1)
            self.ServoFeed()
        
        print("COM VECTORS: ", c1, incSumC)
        print("STEP VECTORS: ", stepVec, incSumS1, incSumS2)

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)

    def Run2x2Init(self, vInpTot, nInc, stepHeight):

        # inputvector length
        vInpTotLen = la.norm(vInpTot) 

        # legLen modification for smoother movement (values found by testing)
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200

        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        # COM & step vector
        stepVec = vInpTot/2

        # vector lengths
        stepLen = la.norm(stepVec)

        # STEP1
        print("Run2x2 init")
        incSumS1 = arr([0, 0, 0])
        incSumS2 = arr([0, 0, 0])
        incSumLenS = la.norm(incSumS1)

        while incSumLenS < stepLen: 
            # step
            incSumS1 = self.legB.IncrementalStep(stepVec, nInc, incSumS1, stepHeight)
            incSumS2 = self.legD.IncrementalStep(stepVec, nInc, incSumS2, stepHeight)
            # increment
            incSumLenS = la.norm(incSumS1)
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)
    
    def Run2x2Term(self, vInpTot, nInc, stepHeight):

        # inputvector length
        vInpTotLen = la.norm(vInpTot) 

        # legLen modification for smoother movement (values found by testing)
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 60/55
        mod3 = 210/200

        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        # COM & step vector
        comVec = vInpTot/2
        stepVec = vInpTot/2

        # vector lengths
        stepLen = la.norm(stepVec)

        # COM SHIFT AND STEP1
        print("Run2x2 term")
        incSumS1 = arr([0, 0, 0])
        incSumS2 = arr([0, 0, 0])
        incSumLenS = la.norm(incSumS1)
        incSumC = arr([0, 0, 0])
        
        while incSumLenS < stepLen: 
            # com
            incSumC = self.COMIncrementalShift4Legs(comVec, nInc, incSumC)
            # step
            incSumS1 = self.legA.IncrementalStep(stepVec, nInc, incSumS1, stepHeight)
            incSumS2 = self.legC.IncrementalStep(stepVec, nInc, incSumS2, stepHeight)
            # increment
            incSumLenS = la.norm(incSumS1)
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)

        # return to initial position (deviation because of machine precision)
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()
        self.ServoFeed()
    
    def Run2x2(self, dest, nInc, stepHeight):

        stepLen = 200
        stepVec = dest/(la.norm(dest))*stepLen
        n = floor(la.norm(dest)/stepLen)

        print('Run2x2 started, initial Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr, )

        self.Run2x2Init(stepVec, nInc, stepHeight)
        for i in range((n-1)):
            self.Run2x2Interm(stepVec, nInc, stepHeight)
        self.Run2x2Term(stepVec, nInc, stepHeight)
        
        print('Run2x2 finished, current Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr,'\n' )

    def BodyTurn(self, totAngle, nInc):

        incSum = 0
        incAngle = np.deg2rad(totAngle/nInc)

        print('Body turn started, initial Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr, )

        while abs(incSum) < abs(np.deg2rad(totAngle)): # while sum of increments < total angle

            # calculate input vectors for Turn
            vInpA = self.legA.BodyVecTurn(incAngle)
            vInpB = self.legB.BodyVecTurn(incAngle)
            vInpC = self.legC.BodyVecTurn(incAngle)
            vInpD = self.legD.BodyVecTurn(incAngle)

            # write shift for each leg
            self.IncrementalBodyTurn4Legs(vInpA, vInpB, vInpC, vInpD, incSum)
            incSum += incAngle
            
            # angles must be in range, otherwise Leg moves in wrong direction (dot product definition)
            if AngleOutOfRange(self.legA.alpha, minAngle, maxAngle)[0] or AngleOutOfRange(self.legB.alpha, minAngle, maxAngle)[0]:
                break
            
            # feed servos
            self.ServoFeed()

        print('angles:', self.legA.alpha, self.legA.beta, self.legA.gamma)
        print('Body turn finished, current Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr,'\n' )

    def WholeTurn(self, totAngle, stepHeight, nInc):
        
        """
        To Do: find good modification values
        Hint: make leg shorter
        """
        # leg lengths for calculation are multiplied with mod and reset after Function is done
        mod1 = 1
        mod2 = 1
        mod3 = 1.03
        self.legA.legLen1Mod, self.legB.legLen1Mod, self.legC.legLen1Mod, self.legD.legLen1Mod = mod1, mod1, mod1, mod1
        self.legA.legLen2Mod, self.legB.legLen2Mod, self.legC.legLen2Mod, self.legD.legLen2Mod = mod2, mod2, mod2, mod2
        self.legA.legLen3Mod, self.legB.legLen3Mod, self.legC.legLen3Mod, self.legD.legLen3Mod = mod3, mod3, mod3, mod3

        nInc /= 1.5 #reduce nInc to match walk functions

        self.BodyTurn(totAngle, nInc)

        # input vectors for IncrementalStep(self, vInpTot, nInc, incSum, stepHeight)
        vStepA = self.legA.initVec - self.legA.vec
        vStepB = self.legB.initVec - self.legB.vec
        vStepC = self.legC.initVec - self.legC.vec
        vStepD = self.legD.initVec - self.legD.vec

        # get COM vectors and order of legs
        cLen = 90
        if totAngle > 0: # CCW
            # COM vectors

            
            c = [arr([  -0.5*cLen,   -0.5*cLen,   0,   ]),
                arr([       1*cLen,          0,   0,   ]), 
                arr([            0,     1*cLen,   0,   ]),
                arr([    -1*cLen,            0,   0,   ]),
                arr([       0.5*cLen,     -0.5*cLen,   0,   ])]

            """
            c = [arr([  -0.5*cLen,   -0.5*cLen,   0,   ]),
                arr([       cLen,               0,   0,   ]), 
                arr([   0.5*cLen,    0.5*cLen,   0,   ]),
                arr([    -2*cLen,               0,   0,   ]),
                arr([       cLen,               0,   0,   ])]
            """

            # Leg list (to access legs depending on rotation CCW/CW)
            leg = [self.legC, self.legB, self.legA, self.legD]
            vStep = [vStepC, vStepB, vStepA, vStepD]

        elif totAngle < 0: # CW
            #COM vectors
            c = [arr([  -0.5*cLen,   0.5*cLen,   0,   ]),
                arr([       1*cLen,          0,   0,   ]), 
                arr([            0,    -1*cLen,   0,   ]),
                arr([      -1*cLen,            0,   0,   ]),
                arr([       0.5*cLen,     0.5*cLen,   0,   ])]
            """
            c = [arr([   -0.5*cLen,    0.5*cLen,   0,   ]),
                arr([        cLen,               0,   0,   ]),
                arr([    0.5*cLen,   -0.5*cLen,   0,   ]),
                arr([     -2*cLen,               0,   0,   ]),
                arr([        cLen,               0,   0,   ])]
            """
    
            # Leg list (to access legs depending on rotation CCW/CW)
            leg = [self.legD, self.legA, self.legB, self.legC]
            vStep = [vStepD, vStepA, vStepB, vStepC]

        else:
            return 0

        print('Whole turn started, initial Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr, )
        
        for i in range(0,4):

            # COM SHIFT
            incSumC = arr([0, 0, 0])
            incSumV = arr([0, 0, 0])

            while la.norm(incSumC) < la.norm(c[i]): # COM shift start
                
                incSumC = self.COMIncrementalShift4Legs(c[i], nInc, incSumC) #  nInc/2 because of short COM shift vector

                self.ServoFeed()

            # STEP
            while la.norm(incSumV) < la.norm(vStep[i]): # COM shift & step

                incSumV = leg[i].IncrementalStep(vStep[i], nInc, incSumV, stepHeight)

                self.ServoFeed()

        # COM SHIFT
        incSumC = arr([0, 0, 0])
        incSumV = arr([0, 0, 0])

        while la.norm(incSumC) < la.norm(c[4]):
                
            incSumC = self.COMIncrementalShift4Legs(c[4], nInc, incSumC) #  nInc/2 because of short COM shift vector

            self.ServoFeed()  

        # return to init pos
        self.legA.GoToInit()
        self.legB.GoToInit()
        self.legC.GoToInit()
        self.legD.GoToInit()

        self.ServoFeed()

        print('Whole turn finished, current Vectors:', self.legA.vCurr, self.legB.vCurr, self.legC.vCurr, self.legD.vCurr,'\n' )

def AngleOutOfRange(angle, minVal, maxVal): 
    """check if angle out of range, return 1 for too big, -1 for too small, 0 for in range"""
    result = 0
    if angle <= minVal:
        print('angle too small:', angle)
        return [-1, minVal, maxVal]
    elif angle >= maxVal:
        print('angle too big:', angle)
        return [1, minVal, maxVal]
    else:
        return [0, minVal, maxVal]

def limitAngle(angle, minVal, maxVal):
    """limit angles to max/min values"""
    values = AngleOutOfRange(angle, minVal, maxVal)
    if values[0] < 0: # small
        return  values[1]
    elif values[0] > 0: # big
        return  values[2]
    else: # in range
        return  angle
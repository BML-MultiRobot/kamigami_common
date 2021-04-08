function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end
function __getObjectQuaternion__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectQuaternion(a,b)
end

-- for publishing ROS messages
function setMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    --sim.setJointTargetVelocity(rightMotor,msg.data)
    cycleFreqL = msg.motor_left
    cycleFreqR = msg.motor_right
end

function setPathNum_cb(msg)
    -- Right motor speed subscriber callback
    --sim.setJointTargetVelocity(rightMotor,msg.data)
    pathNum = msg.x
end


function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSimulationTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function abAngleFunc(phase, lr)
    abAngle = 0.6096 + 1.002*math.tanh(1.1011 + 1.1816*math.cos(0.9507 - phase))- math.pi/2
    
    if lr == -1 then
        abAngle = -abAngle
    end
    
    return abAngle
end

-- Abduction velocity
-- given freq, phase and lr, calculate the abduction angle
function abAngleVelFunc(freq, phase, lr)
    abAngleVel = 7.43957929294585*freq*math.pow(1/math.cosh(1.1011222381516315 + 1.1816061808371887*math.cos(0.9507092821679702 - phase)),2)*math.sin(0.9507092821679702 - phase)
    
    if lr == -1 then
        abAngleVel = -abAngleVel
    end
    
    return abAngleVel
end

-- Protraction angle
-- given phase and lr, calculate the protraction angle
function proAngleFunc(phase, lr)
    --Pure SIN, requires protractAmp, protractPhase, protractCenter to be set
    proAngle = protractAmp * math.sin(phase + protractPhase) + protractCenter
    return proAngle
end

-- Protraction velocity
-- given freq, phase and lr, calculate the protraction angle
function proAngleVelFunc(freq, phase, lr)
    proAngleVel = 2*math.pi* protractAmp * freq * math.cos(phase + protractPhase)
    return proAngleVel
end


function setLegAngles(cycleFreq, cyclePhase, leftright)
    for i=1, 6, 1 do
        targetAbduction      = abAngleFunc(cyclePhase[i], leftright[i])
        targetAbductionVel   = abAngleVelFunc(cycleFreq[i], cyclePhase[i], leftright[i])
        targetProtraction    = proAngleFunc(cyclePhase[i], leftright[i])
        targetProtractionVel = proAngleVelFunc(cycleFreq[i], cyclePhase[i], leftright[i])

        -- Setters
        sim.setJointTargetPosition(abHandles[i], targetAbduction)
        sim.setJointTargetVelocity(abHandles[i], targetAbductionVel)
        sim.setJointTargetPosition(proHandles[i], targetProtraction)
        sim.setJointTargetVelocity(proHandles[i], targetProtractionVel)

        -- REVISED
        -- using impulse to turning rather than change the frequency
        
        -- for test turning
        turnSignal = 1
        if turnSignal > 0 then
            if i < 4 then
                if targetProtraction < 0.1 and targetProtraction > -0.1 then
                    if targetProtractionVel > 0 then
                         -- for test -- revise position
                         -- cyclePhase[i] = cyclePhase[i] - 5
                         sim.setJointTargetVelocity(proHandles[i], targetProtractionVel + 10)
                    else
                         sim.setJointTargetVelocity(proHandles[i], targetProtractionVel - 10)
                    end
                end
            end
        else
            if i > 3 then
                if targetProtraction < 0.1 and targetProtraction > -0.1 then
                    if targetProtractionVel > 0 then
                         -- for test -- revise position
                         -- cyclePhase[i] = cyclePhase[i] - 5
                         sim.setJointTargetVelocity(proHandles[i], targetProtractionVel + 10)
                    else
                         sim.setJointTargetVelocity(proHandles[i], targetProtractionVel - 10)
                    end
                end
            end
        end
    end
end

function runLegControllers()

    -- My



    --[[ below: adding in python API functionality. Communicate floats representing left and right cycle frequencies
    cycleFreqL = sim.getFloatSignal("CycleLeft")
    cycleFreqR = sim.getFloatSignal("CycleRight")
    if not cycleFreqL then
        cycleFreqL = 0--sim.getScriptSimulationParameter(sim.handle_self, 'cycleFreqL')
        cycleFreqR = 0--sim.getScriptSimulationParameter(sim.handle_self, 'cycleFreqR')
    end]]

    -- End My

    cycleFreq = {cycleFreqL, cycleFreqL, cycleFreqL, cycleFreqR, cycleFreqR, cycleFreqR}
    
    dt = sim.getSimulationTimeStep()
    dPhiL = dt * cycleFreqL * 2.0 * math.pi
    dPhiR = dt * cycleFreqR * 2.0 * math.pi
    -- phase seemed same for left and right- better make explicit	
    for i=1, 3, 1 do   -- left side	
        cyclePhase[i] = (cyclePhase[i] + dPhiL)	
    end	
    for i=3, 6, 1 do   -- right side 	
        cyclePhase[i] = (cyclePhase[i] + dPhiR)	
    end
    

    setLegAngles(cycleFreq, cyclePhase, leftright)
end

function scriptSimBool(name)
    return sim.getScriptSimulationParameter(sim.handle_self, name) == 1
end

if (sim_call_type==sim.syscb_init) then
    sim.setScriptAttribute(sim.handle_self,sim.childscriptattribute_automaticcascadingcalls,false)
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self) -- test

    abHandles = {nil, nil, nil, nil, nil, nil}
    proHandles = {nil, nil, nil, nil, nil, nil}
    abHandles[1] = sim.getObjectHandle('abductionJointL1')
    abHandles[2] = sim.getObjectHandle('abductionJointL2')

    abHandles[3] = sim.getObjectHandle('abductionJointL3')

    abHandles[4] = sim.getObjectHandle('abductionJointR1')
    abHandles[5] = sim.getObjectHandle('abductionJointR2')
    abHandles[6] = sim.getObjectHandle('abductionJointR3')

    proHandles[1] = sim.getObjectHandle('protractionJointL1')
    proHandles[2] = sim.getObjectHandle('protractionJointL2')
    proHandles[3] = sim.getObjectHandle('protractionJointL3')

    proHandles[4] = sim.getObjectHandle('protractionJointR1')
    proHandles[5] = sim.getObjectHandle('protractionJointR2')
    proHandles[6] = sim.getObjectHandle('protractionJointR3')
    
    body = sim.getObjectHandle('body#1') 
    path = sim.getObjectHandle('Path#1') 

    leftright = {-1, -1, -1, 1, 1, 1}
    cycleFreqL = 0
    cycleFreqR = 0
  
    ------ alternating tetrapod
    ------ can change the leg offsets here
    cyclePhase = {math.pi,0,math.pi,0,math.pi,0}

    -- For sinusoidal movement
    protractMax = -42.0 * math.pi/180 --After AH
    protractMin = 42.0* math.pi/180   --AFter AH
    protractCenter = (protractMax + protractMin)/2
    protractAmp = (protractMax - protractMin)/2
    protractPhase = math.pi/2

    -- INITIAL joint position
    for i=1,6,1 do
        abductInitial = abAngleFunc(cyclePhase[i], leftright[i])
        sim.setJointPosition(abHandles[i], abductInitial)
        sim.setJointTargetPosition(abHandles[i], abductInitial)
        
        protractInitial = proAngleFunc(cyclePhase[i], leftright[i])
        sim.setJointPosition(proHandles[i], protractInitial)
        sim.setJointTargetPosition(proHandles[i], protractInitial)
    end

    --for ros
        -- prepare the topic names
        robotPosTopicName = 'cockroachPos' 
        robotVelTopicName = 'cockroachVel'
        robotPathTopicName = 'cockroachPath'
        robotPathNumTopicName = 'pathNum'
        -- Prepare the sensor publisher and the motor speed subscribers:
        robotPosPub=simROS.advertise('/'..robotPosTopicName,'geometry_msgs/Point32')
        robotPathPub = simROS.advertise('/'..robotPathTopicName,'geometry_msgs/Vector3')
        pathNumSub=simROS.subscribe('/'..robotPathNumTopicName,'geometry_msgs/Vector3','setPathNum_cb')
        MotorVelSub=simROS.subscribe('/'..robotVelTopicName,'kamigami_msgs/KamigamiCommand','setMotorVelocity_cb')
    -- end
    pathLen = sim.getPathLength(path)
    pathNum = 0
end

if (sim_call_type==sim.syscb_cleanup) then 
 
end 
if (sim_call_type==sim.syscb_sensing) then 
  sim.handleChildScripts(sim_call_type) 
end

if (sim_call_type==sim.syscb_actuation) then
    -- get and send path -- tenp
    pathPos = sim.getPositionOnPath(path, (pathNum / 16))
    print(pathPos)
    local tabPath = {}
    tabPath['x'] = pathPos[1]
    tabPath['y'] = pathPos[2]
    simROS.publish(robotPathPub, tabPath)
    -- end

    -- get position of body#1
    local tab = {}
    rPos = sim.getObjectPosition(body,-1)
    rOri = sim.getObjectOrientation(body,-1)
    tab['x'] = rPos[1]
    tab['y'] = rPos[2]
    tab['z'] = rOri[3]
    -- send pos msg
    simROS.publish(robotPosPub, tab)
    
    -- run controller
    runLegControllers()
end

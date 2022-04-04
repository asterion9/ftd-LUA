CONTROLLER_CPS = 1  -- cycles per second, this is the number of step taken per second. lower is more stable but higher is faster
SMOOTHER_SPEED = 0.05  -- rate of variation of the value toward target value per call (1/40s). shouldn't be changed
-- name of the strafer ad forwarder spinner to relay commands
INPUT_STRAFER_NAME = "strafer"
INPUT_FORWARDER_NAME = "forwarder"
INPUT_SPINNER_POWER = 0.5  -- the power expected on the input spinners (0 > power > 1, 0 and 1 won't work)
-- the ratio fo time a leg spend on ground vs returning to the start point.
-- higher is faster but risk maxing out the spinner speed and hitting ground too strongly (and looking weird)
GAIT_GROUND_RATIO = 0.7
GAIT_MIN_HEIGHT = 4  -- min height for the return move of the gait
GAIT_SIZE_FACTOR = 0.7  -- global size factor for the gait (1 is default, 0 is total disable)
GAIT_FORWARD_FACTOR = 1  -- forward size factor for the gait (1 is default, 0 is total disable)
GAIT_STRAFE_FACTOR = 0.7  -- strafe size factor for the gait (1 is default, 0 is total disable)
GAIT_YAW_FACTOR = 0.7  -- yaw size factor for the gait (1 is default, 0 is total disable)

IK_MODEL = {INSECT = 1, HUMAN = 2, CHICKEN = 3}  -- model for leg, INSECT is horizontal, HUMAN is vertical with forward knee, CHICKEN is vertical with inverse knee

ADJUSTER_HEIGHT_OFFSET = 0.4  -- offset between the calculated foot position and the actual, should be 0 when well configured, but sometimes around that
ADJUSTER_SPRING_STRENGTH = 0.9  -- strength of the adjuster software spring, 0 means infinite strength, 1 means no strength


controller = nil

function Update(I)
    if controller == nil then
        controller = Controller.newAmplitudeController(I, buildArms(I))
    end
    controller:move(I)
end

function buildArms(I)
    --spot 4 legs

    local segment0 = { len = Vector3(0, 0, 1), spinOffset = 0, spinDirection = 1 }
    local segment1 = { len = Vector3(0, 0, 15), spinOffset = 0, spinDirection = 1 }
    local segment2 = { len = Vector3(0, 0, 17), spinOffset = 0, spinDirection = 1 }
    local segment3 = { len = Vector3(0, 0, 2.5), spinOffset = 0, spinDirection = 1 }
    return PrefabLegBuilder.buildLegs(I,
                {
                    { phase = 0.25, ikModel = IK_MODEL.CHICKEN, gaitCenter = Vector3(0, -24, 0), segments = { segment0, segment1, segment2, segment3 } }, -- bottom right
                    { phase = 0, ikModel = IK_MODEL.CHICKEN, gaitCenter = Vector3(0, -24, 0), segments = { segment0, segment1, segment2, segment3 } }, -- top right
                    { phase = 0.5, ikModel = IK_MODEL.CHICKEN, gaitCenter = Vector3(0, -24, 0), segments = { segment0, segment1, segment2, segment3 } }, -- top left
                    { phase = 0.75, ikModel = IK_MODEL.CHICKEN, gaitCenter = Vector3(0, -24, 0), segments = { segment0, segment1, segment2, segment3 } }  -- bottom left
                })
end

Adjuster = {
    new = function(I, legs)
        return {
            comAdjust = Vector3.zero,
            maxOffset = legs[next(legs)].segments[2].len.z * 0.4,
            legOffsets = {},
            getForLeg = function(self, leg)
                local legOffset = Vector3(0, self.legOffsets[leg], 0)
                if (legOffset == nil) then
                    legOffset = Vector3.zero
                end
                return self.comAdjust + legOffset
            end,
            calculateLegOffsets = function(self, I, legTargets, stepTime)
                local vehicleWorldPosition = I:GetConstructCenterOfMass() + I:GetVelocityVector() * stepTime
                local vehicleWorldRotationVector = Vector3((I:GetConstructPitch() + 360) % 360,
                        (I:GetConstructYaw() + 360) % 360,
                        (I:GetConstructRoll() + 360) % 360)-- + I:GetAngularVelocity() * stepTime
                local vehicleWorldRotation = Quaternion.Euler(vehicleWorldRotationVector)
                self.legOffsets = {}
                local log = ""
                for leg, target in pairs(legTargets) do
                    local targetLocalPosition = leg.position + target
                    local targetWorldPos = vehicleWorldPosition + vehicleWorldRotation * targetLocalPosition
                    local heightDiff = targetWorldPos.y - ADJUSTER_HEIGHT_OFFSET - I:GetTerrainAltitudeForPosition(targetWorldPos.x, targetWorldPos.y, targetWorldPos.z)
                    log = log .. string.format("heightDiff=%f, ", heightDiff)

                    if(heightDiff < 0) then
                        self.legOffsets[leg] = -heightDiff*ADJUSTER_SPRING_STRENGTH
                    end
                end
                I:Log(log)
            end
        }
    end
}

-- gathers input commands, calculate the target position for each leg and moves them
Controller = {
    -- build each individual movement gait for each leg
    buildLegsGaits = function(I, legs)
        local legGaits = {}
        for id, leg in pairs(legs) do
            legGaits[leg] = {
                Walking = Gait.Walking.autoconfig(I, leg, -Mathf.PI / 2),
                Strafing = Gait.Walking.autoconfig(I, leg, 0),
                Turning = Gait.Turning.autoconfig(I, leg),
                Resting = Gait.Resting.autoconfig(I, leg)
            }
        end
        return legGaits
    end,
    -- calculate the target for each leg using the supplied commands
    calculateTarget = function(leg, gaits, curStep, forward, yaw, strafe)
        local fTarget
        local sTarget
        local yTarget
        local rTarget = gaits['Resting'].getPoint((curStep + leg.phase) % 1)
        local fRatio = math.abs(forward) * GAIT_SIZE_FACTOR * GAIT_FORWARD_FACTOR
        local sRatio = math.abs(strafe) * GAIT_SIZE_FACTOR * GAIT_STRAFE_FACTOR
        local yRatio = math.abs(yaw) * GAIT_SIZE_FACTOR * GAIT_YAW_FACTOR
        local fDirection = Mathf.Sign(forward)
        local sDirection = Mathf.Sign(strafe)
        local yDirection = Mathf.Sign(yaw)
        if (fRatio > 0.01) then
            fTarget = (fRatio * gaits['Walking'].getPoint((fDirection * (curStep + leg.phase) % 1)) + (1 - fRatio) * rTarget)
        end
        if (sRatio > 0.01) then
            sTarget = (sRatio * gaits['Strafing'].getPoint((sDirection * (curStep + leg.phase) % 1)) + (1 - sRatio) * rTarget)
        end
        if (yRatio > 0.01) then
            yTarget = (yRatio * gaits['Turning'].getPoint((yDirection * (curStep + leg.phase) % 1)) + (1 - yRatio) * rTarget)
        end

        local wTarget = nil
        if (fTarget ~= nil) then
            if (sTarget ~= nil) then
                wTarget = fTarget + sTarget - leg.gaitCenter
            else
                wTarget = fTarget
            end
        elseif sTarget ~= nil then
            wTarget = sTarget
        end

        if (wTarget ~= nil) then
            if (yTarget ~= nil) then
                return (wTarget + yTarget) - leg.gaitCenter
            else
                return wTarget
            end
        elseif yTarget ~= nil then
            return yTarget
        else
            return rTarget
        end
    end,
    -- create a controller that throttles using amplitude of movement
    newAmplitudeController = function(I, legs)
        local legGaits = Controller.buildLegsGaits(I, legs)
        return {
            legGaits = legGaits,
            cps = CONTROLLER_CPS, -- cycles per second
            comAdjuster = Adjuster.new(I, legs),
            lastTime = I:GetGameTime(),
            curTurn = 0,
            commandReader = InputReader.new(I),
            -- read the inputs, calculates the target for each leg and makes them move
            move = function(self, I)
                local curTime = I:GetGameTime()
                local curStep = (curTime * self.cps) % 1
                local inputs = self.commandReader:getCommands(I)
                I:Log(string.format('forward=%f, yaw=%f, strafe=%f', inputs.forward, inputs.yaw, inputs.strafe))
                local legTargets = {}
                for leg, gaits in pairs(self.legGaits) do
                    local target = Controller.calculateTarget(leg, gaits, curStep, inputs.forward, inputs.yaw, inputs.strafe)
                    legTargets[leg] = target
                end
                self.comAdjuster:calculateLegOffsets(I, legTargets, curTime - self.lastTime)
                for leg, target in pairs(legTargets) do
                    leg:moveLeg(I, target + self.comAdjuster:getForLeg(leg), curTime)
                    --leg:moveLeg(I, target, curTime)
                end
                self.lastTime = curTime
            end,
            -- check that all legs are still there
            checkIntegrity = function(self, I)
                local subConstructs = I:GetAllSubConstructs()
                local scIndex = {}
                for i = 1, #subConstructs do
                    scIndex[subConstructs[i]] = true
                end
                for leg, gaits in pairs(self.legGaits) do
                    if not scIndex[leg.segments[1].spinId] then
                        return false
                    end
                end
                return true
            end
        }
    end
}
-- wrap a value to smooth the output so that changes are not to drastic
Smoother = {
    new = function()
        return {
            target = 0, -- wanted value
            speed = SMOOTHER_SPEED, -- variation of value toward target per evaluation
            value = 0, -- current value
            update = function(self, val) -- change the target
                self.target = val
                return self
            end,
            evaluate = function(self) -- get the next value
                if (self.target ~= self.value) then
                    if (math.abs(self.target - self.value) < self.speed) then
                        self.value = self.target
                    else
                        local sign = Mathf.Sign(self.target - self.value)
                        self.value = self.value + sign * self.speed
                    end
                end
                return self.value
            end
        }
    end
}

-- handle configuring and reading inputs, especially strafe and forwards commands using a spinner as relay
InputReader = {
    new = function(I)
        local strafeId = -1
        local forwardId = -1
        local subConstructs = I:GetAllSubConstructs()
        for i = 1, #subConstructs do
            if I:IsSpinBlock(subConstructs[i]) then
                if I:IsSubConstructOnHull(subConstructs[i]) then
                    local blockInfo = I:GetSubConstructInfo(subConstructs[i])
                    if blockInfo.CustomName == INPUT_STRAFER_NAME then
                        strafeId = blockInfo.SubConstructIdentifier
                    elseif blockInfo.CustomName == INPUT_FORWARDER_NAME then
                        forwardId = blockInfo.SubConstructIdentifier
                    end
                end
            end
        end

        local commands = {
            forward = {
                forwardId = forwardId,
                smoother = Smoother.new(),
                read = function(self, I)
                    local drive = I:GetDrive(0)
                    if drive == 0 then
                        return InputReader.readSpinner(I, self.forwardId)
                    else
                        return drive
                    end
                end
            },
            yaw = {
                smoother = Smoother.new(),
                read = function(self, I)
                    return I:GetInput(0, 0) - I:GetInput(0, 1)
                end
            },
            strafe = {
                strafeId = strafeId,
                smoother = Smoother.new(),
                read = function(self, I)
                    return InputReader.readSpinner(I, self.strafeId)
                end
            }
        }

        return {
            commands = commands,
            getCommands = function(self, I)
                return {
                    forward = self.commands.forward.smoother:update(self.commands.forward:read(I)):evaluate(),
                    strafe = self.commands.strafe.smoother:update(self.commands.strafe:read(I)):evaluate(),
                    yaw = self.commands.yaw.smoother:update(self.commands.yaw:read(I)):evaluate(),
                }
            end
        }
    end,
    readSpinner = function(I, spinnerId)
        if spinnerId ~= -1 then
            local angle = Vector3.SignedAngle(Vector3.forward, I:GetSubConstructInfo(spinnerId).LocalForwards, Vector3.up)
            return angle / (180 * INPUT_SPINNER_POWER)
        end
        return 0
    end
}

-- contains all possible gaits for a leg, with handy creation method
Gait = {
    -- walking gait, composed of a flat line and an elliptic return to origin
    Walking = {
        new = function(position, yAngle, height, width)
            return {
                getPoint = function(t)
                    local groundRatio = GAIT_GROUND_RATIO -- ratio of time passed on ground over time returning to origin
                    t = (t + groundRatio / 2) % 1
                    local target
                    if (t < groundRatio) then
                        target = Vector3(
                                position.x + width / 2 - (t / groundRatio) * width,
                                position.y,
                                position.z)
                    else
                        local te = 0.5 - 0.5 * (t - groundRatio) / (1 - groundRatio)
                        target = Vector3(
                                position.x + math.cos(te * Mathf.PI * 2) * (width / 2),
                                position.y + math.sin(te * Mathf.PI * 2) * height,
                                position.z)
                    end
                    return rotateAround(target, yAngle, position)
                end
            }
        end,
        autoconfig = function(I, leg, angle)
            local posCenter
            if leg.gaitCenter == nil then
                local legDirection = I:GetSubConstructInfo(leg.segments[1].spinId).LocalPositionRelativeToCom
                posCenter = legDirection.normalized * ((leg.segments[1].len.z + leg.segments[2].len.z) * 1.2)
                posCenter.y = -leg.segments[3].len.z + leg.segments[1].len.y - 2
            else
                posCenter = leg.gaitCenter
            end

            local actionRay = math.sqrt(Mathf.Pow(leg.length, 2) - Mathf.Pow(posCenter.y, 2)) - math.sqrt(posCenter.x * posCenter.x + posCenter.z * posCenter.z)

            I:Log(string.format("creating walking gait for leg %s, centered at %s with width %f", tostring(leg.position), tostring(posCenter), actionRay * 2 * GAIT_SIZE_FACTOR))

            return Gait.Walking.new(posCenter,
                    angle, math.max(leg.segments[2].len.z * 0.5, GAIT_MIN_HEIGHT), actionRay * 2 * GAIT_SIZE_FACTOR)
        end
    },
    -- turning gait, composed of a circle arc on the ground and a elliptic circle arc to return to origin
    Turning = {
        new = function(center, radius, radStart, radLength, height)
            return {
                getPoint = function(t)
                    local groundRatio = GAIT_GROUND_RATIO
                    t = (t + groundRatio / 2) % 1
                    if (t < groundRatio) then
                        -- foot on ground
                        local tg = t / groundRatio
                        return Vector3(
                                center.x + radius * math.cos(radStart + (radLength * (tg))),
                                center.y,
                                center.z + radius * math.sin(radStart + (radLength * (tg))))
                    else
                        local tv = 0.5 * (t - groundRatio) / (1 - groundRatio)
                        local th = (1 + math.cos(tv * Mathf.PI * 2)) / 2;
                        return Vector3(
                                center.x + radius * math.cos(radStart + (radLength * th)),
                                center.y + math.sin(tv * Mathf.PI * 2) * height,
                                center.z + radius * math.sin(radStart + (radLength * th)));
                    end
                end
            }
        end,
        autoconfig = function(I, leg)
            local spinPosition = I:GetSubConstructInfo(leg.segments[1].spinId).LocalPositionRelativeToCom

            local rotCenter
            local rotRadius
            local angleOffset
            if leg.gaitCenter == nil then
                rotCenter = Vector3(-spinPosition.x, -leg.segments[3].len.z + leg.segments[1].len.y - 2, -spinPosition.z)
                rotRadius = math.sqrt(spinPosition.x * spinPosition.x + spinPosition.z * spinPosition.z) + leg.segments[1].len.z + leg.segments[2].len.z
                angleOffset = 2 * Mathf.PI * Vector3.SignedAngle(Vector3.ProjectOnPlane(spinPosition, Vector3.up), Vector3.right, Vector3.up) / 360
            else
                rotCenter = Vector3(-spinPosition.x, leg.gaitCenter.y, -spinPosition.z)
                rotRadius = Vector3.Distance(rotCenter, leg.gaitCenter)
                angleOffset = 2 * Mathf.PI * Vector3.SignedAngle(Vector3.ProjectOnPlane(spinPosition + leg.gaitCenter, Vector3.up), Vector3.right, Vector3.up) / 360
            end
            local Ta = math.sqrt(leg.gaitCenter.x * leg.gaitCenter.x + leg.gaitCenter.z * leg.gaitCenter.z);
            local actionRay = math.sqrt(Mathf.Pow(leg.length, 2) - Mathf.Pow(leg.gaitCenter.y, 2)) - Ta

            local angleTurning = Mathf.Atan2(actionRay, Ta + math.sqrt(spinPosition.x * spinPosition.x + spinPosition.z * spinPosition.z))

            I:Log(string.format("creating turning gait for leg %s, rotating around %s with radius %f, turning %f and offset %f", tostring(leg.position), tostring(rotCenter), rotRadius, angleTurning, angleOffset))

            return Gait.Turning.new(rotCenter,
                    rotRadius * GAIT_SIZE_FACTOR,
                    angleOffset + angleTurning,
                    -2 * angleTurning,
                    math.max(leg.segments[2].len.z * 0.5, GAIT_MIN_HEIGHT)
            )
        end
    },
    -- resting gait, legs immobile at center
    Resting = {
        new = function(position)
            return {
                getPoint = function(t)
                    return position
                end
            }
        end,
        autoconfig = function(I, leg)
            local position
            if leg.gaitCenter == nil then
                local legDirection = I:GetSubConstructInfo(leg.segments[1].spinId).LocalPositionRelativeToCom
                position = legDirection.normalized * ((leg.segments[1].len.z + leg.segments[2].len.z) * 1.2)
                position.y = -leg.segments[3].len.z + leg.segments[1].len.y - 2
            else
                position = leg.gaitCenter
            end
            I:Log(string.format("creating resting gait for leg %s, at %s", tostring(leg.position), tostring(position)))
            return Gait.Resting.new(position)
        end
    }
}

-- represent a segment of a leg, used to control the spinner of this segment
Segment = {
    new = function(spinId, len, spinOffset, spinDirection)
        return {
            lastUpdate = nil,
            lastAngle = nil,
            spinId = spinId,
            len = len,
            spinOffset = spinOffset,
            spinDirection = spinDirection,
            setAngle = function(self, I, angle, t)
                local rotSpeed = 0
                if (self.lastUpdate ~= nil and self.lastUpdate ~= t) then
                    local ellapsedTime = (t - self.lastUpdate + 1) % 1
                    rotSpeed = math.min(math.abs((angle - self.lastAngle) / (ellapsedTime)), 30)
                    --I:Log(string.format('spinner %d going from %f to %f in %f seconds, setting speed at %f rad/s', self.spinId, self.lastAngle, angle, ellapsedTime, rotSpeed))
                end
                self.lastUpdate = t
                self.lastAngle = angle
                I:SetSpinBlockContinuousSpeed(self.spinId, 30)
                local angleDeg = 360 * angle / (2 * Mathf.PI)
                I:SetSpinBlockRotationAngle(self.spinId, self.spinDirection * (angleDeg + self.spinOffset))
            end
        }
    end
}

-- helper method to build legs using a leg template and finding suitable spinners on construct
PrefabLegBuilder = {
    buildLegs = function(I, legTemplates)
        local subConstructs = I:GetAllSubConstructs()

        local scByOrientation = {}
        for i = 1, #subConstructs do
            local id = subConstructs[i]
            if I:IsSpinBlock(id) then
                if I:IsSubConstructOnHull(id) then
                    local scPos = I:GetSubConstructInfo(id).LocalPositionRelativeToCom
                    table.insert(scByOrientation, { id = id, ori = Vector3.SignedAngle(Vector3.forward, Vector3.ProjectOnPlane(scPos, Vector3.up), Vector3.up) })
                end
            end
        end
        table.sort(scByOrientation, function(a, b)
            return a.ori > b.ori
        end)

        local legs = {}
        local i = 1
        local defaultPhase = 0
        for pos, scSorted in pairs(scByOrientation) do
            local scId = scSorted.id
            if legTemplates[i] ~= nil then
                local leg = PrefabLegBuilder.buildLegFromTemplate(I, scId, legTemplates[i])
                if leg ~= nil then
                    if (legTemplates[i].phase == nil) then
                        leg.phase = defaultPhase
                    else
                        leg.phase = legTemplates[i].phase
                    end
                    defaultPhase = (defaultPhase + 0.5) % 1
                    legs[scId] = leg
                    i = i + 1
                    I:Log(string.format('built leg : %d segment, position %s, gaitCenter %s, segment length %f', #leg.segments, tostring(leg.position), tostring(leg.gaitCenter), leg.length))
                end
            end
        end

        return legs
    end,
    buildLegFromTemplate = function(I, baseScId, legTemplate)
        local sc = I:GetSubConstructInfo(baseScId)
        local length = 0
        for i = 1, #legTemplate.segments do
            length = length + legTemplate.segments[i].len.magnitude
        end
        local ikModel
        if legTemplate.ikModel == IK_MODEL.HUMAN then
            ikModel = IkLib.moveHumanoidLeg
        elseif legTemplate.ikModel == IK_MODEL.CHICKEN then
            ikModel = IkLib.moveChickenLeg
        else
            ikModel = IkLib.moveInsectoidLeg
        end
        local leg = {
            position = sc.LocalPositionRelativeToCom,
            segments = {},
            moveLeg = function(leg, I, target, t)
                local a0, a1, a2, a3 = ikModel(leg, I, target)

                I:Log(string.format('t=%f : moving humanoid leg %d at %s, offset=%d, spinDir=%d to %s with angle a0=%f, a1=%f, a2=%f',
                            t, leg.segments[1].spinId, tostring(leg.position), leg.segments[1].spinOffset, leg.segments[1].spinDirection, tostring(target), a0, a1, a2))

                leg.segments[1]:setAngle(I, a0, t)
                leg.segments[2]:setAngle(I, a1, t)
                leg.segments[3]:setAngle(I, a2, t)
                if leg.segments[4] ~= nil then
                    leg.segments[4]:setAngle(I, a3, t)
                end
            end,
            gaitCenter = legTemplate.gaitCenter,
            length = length
        }

        I:Log(string.format('found base spinner number %d', sc.SubConstructIdentifier))
        for i = 1, #legTemplate.segments do
            local segTemplate = legTemplate.segments[i]
            leg.segments[i] = Segment.new(
                    sc.SubConstructIdentifier,
                    segTemplate.len,
                    segTemplate.spinOffset,
                    segTemplate.spinDirection
            )

            if i < #legTemplate.segments then
                local scChildren = I:GetAllSubConstructChildren(sc.SubConstructIdentifier)
                if (#scChildren ~= 1) then
                    I:Log("not a leg, keeping searching")
                    return nil
                else
                    sc = I:GetSubConstructInfo(scChildren[1])
                end
            end
        end
        return leg
    end
}

-- helper lib for doing inverse kinematic using a leg config
IkLib = {
    moveInsectoidLeg = function(leg, I, target)
        if target.magnitude > leg.length then
            I:LogToHud(string.format("ERROR : target %s is too far for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        if target.magnitude < leg.segments[1].len.z then
            I:LogToHud(string.format("ERROR : target %s is too close for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        local a0 = (target.z > 0 and -1 or 1) * math.acos(target.x / math.sqrt(target.x * target.x + target.z * target.z))

        local rx = target.x / math.cos(a0) - leg.segments[1].len.z
        -- if segment 1 (the first one) is going up/down (has y component), it must be taken into account
        -- for calculating the up/down on the plane
        local ry = target.y - leg.segments[1].len.y
        if leg.segments[4] then
            ry = ry + leg.segments[4].len.z
        end
        local rMag = math.sqrt(rx * rx + ry * ry)

        local a1 = (ry > 0 and 1 or -1) * math.acos(rx / rMag)
                + math.acos(
                (leg.segments[2].len.z * leg.segments[2].len.z + rMag * rMag - leg.segments[3].len.z * leg.segments[3].len.z)
                        / (2 * leg.segments[2].len.z * rMag))

        local a2 = -math.acos((rMag * rMag - leg.segments[2].len.z * leg.segments[2].len.z - leg.segments[3].len.z * leg.segments[3].len.z)
                / (2 * leg.segments[2].len.z * leg.segments[3].len.z))

        return a0, a1, a2, Mathf.PI/2 - a1 - a2
    end,
    moveHumanoidLeg = function(leg, I, target)
        if target.magnitude > leg.length then
            I:LogToHud(string.format("ERROR : target %s is too far for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        if target.magnitude < leg.segments[1].len.z then
            I:LogToHud(string.format("ERROR : target %s is too close for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        local a0 = (target.x > 0 and 1 or -1) * math.acos(-target.y / math.sqrt(target.x * target.x + target.y * target.y))

        local rx = -target.y / math.cos(a0) - leg.segments[1].len.z
        if leg.segments[4] then
            rx = rx - math.cos(a0) * leg.segments[4].len.z
        end
        local ry = target.z - leg.segments[1].len.y
        local rMag = math.sqrt(rx * rx + ry * ry)

        local a1 = (ry > 0 and 1 or -1) * math.acos(rx / rMag)
                + math.acos(
                (leg.segments[2].len.z * leg.segments[2].len.z + rMag * rMag - leg.segments[3].len.z * leg.segments[3].len.z)
                        / (2 * leg.segments[2].len.z * rMag))

        local a2 = -math.acos((rMag * rMag - leg.segments[2].len.z * leg.segments[2].len.z - leg.segments[3].len.z * leg.segments[3].len.z)
                / (2 * leg.segments[2].len.z * leg.segments[3].len.z))

        return a0, a1, a2, - a1 - a2
    end,
    moveChickenLeg = function(leg, I, target)
        if target.magnitude > leg.length then
            I:LogToHud(string.format("ERROR : target %s is too far for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        if target.magnitude < leg.segments[1].len.z then
            I:LogToHud(string.format("ERROR : target %s is too close for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        local a0 = (target.x > 0 and 1 or -1) * math.acos(-target.y / math.sqrt(target.x * target.x + target.y * target.y))

        local rx = -target.y / math.cos(a0) - leg.segments[1].len.z
        if leg.segments[4] then
            rx = rx - math.cos(a0) * leg.segments[4].len.z
        end
        local ry = target.z - leg.segments[1].len.y
        local rMag = math.sqrt(rx * rx + ry * ry)

        local a1 = (ry > 0 and 1 or -1) * math.acos(rx / rMag)
                - math.acos(
                (leg.segments[2].len.z * leg.segments[2].len.z + rMag * rMag - leg.segments[3].len.z * leg.segments[3].len.z)
                        / (2 * leg.segments[2].len.z * rMag))

        local a2 = math.acos((rMag * rMag - leg.segments[2].len.z * leg.segments[2].len.z - leg.segments[3].len.z * leg.segments[3].len.z)
                / (2 * leg.segments[2].len.z * leg.segments[3].len.z))

        return a0, a1, a2, - a1 - a2
    end
}

function rotateAround(base, yAngle, rotationCenter)
    local cosA = math.cos(yAngle)
    local sinA = math.sin(yAngle)
    return Vector3(
            rotationCenter.x + cosA * (base.x - rotationCenter.x) + sinA * (base.z - rotationCenter.z),
            base.y,
            rotationCenter.z - sinA * (base.x - rotationCenter.x) + cosA * (base.z - rotationCenter.z)
    )
end
CONTROLLER_CPS = 1  -- cycles per second, this is the number of step taken per second. lower is more stable but higher is faster
SMOOTHER_SPEED = 0.05  -- rate of variation of the value toward target value per call (1/40s). shouldn't be changed
-- name of the strafer ad forwarder spinner to relay commands
INPUT_STRAFER_NAME = "strafer"
INPUT_FORWARDER_NAME = "forwarder"
INPUT_SPINNER_POWER = 0.5  -- the power expected on the input spinners (0 > power > 1, 0 and 1 won't work)
-- the ratio fo time a leg spend on ground vs returning to the start point.
-- higher is faster but risk maxing out the spinner speed and hitting ground too strongly (and looking weird)
GAIT_GROUND_RATIO = 0.7
GAIT_MIN_HEIGHT = 3  -- min height for the return move of the gait
IK_MODEL = {INSECT = 1, HUMAN = 2}

controller = nil

function Update(I)
    if controller == nil then
        controller = Controller.newAmplitudeController(I, buildArms(I))
    end
    controller:move(I)
end

function buildArms(I)
    --marauder 8 legs
    local segment0Right = { len = Vector3(0, -4, 4), spinOffset = 0, spinDirection = -1 }
    local segment0Left = { len = Vector3(0, -4, 4), spinOffset = 180, spinDirection = -1 }
    local segment1 = { len = Vector3(0, 0, 6), spinOffset = 0, spinDirection = 1 }
    local segment2 = { len = Vector3(0, 4, 11.5), spinOffset = -19.2, spinDirection = -1 }
    return PrefabLegBuilder.buildLegs(I,
            {
                { gaitCenter = Vector3(8.5, -7.5, -5), segments = { segment0Right, segment1, segment2 } }, -- bottom right
                { gaitCenter = Vector3(10, -7.5, 0), segments = { segment0Right, segment1, segment2 } }, -- middle bottom right
                { gaitCenter = Vector3(10, -7.5, 0), segments = { segment0Right, segment1, segment2 } }, -- middle top right
                { gaitCenter = Vector3(8.5, -7.5, 5), segments = { segment0Right, segment1, segment2 } }, -- top right
                { gaitCenter = Vector3(-8.5, -7.5, 5), segments = { segment0Left, segment1, segment2 } }, -- top left
                { gaitCenter = Vector3(-10, -7.5, 0), segments = { segment0Left, segment1, segment2 } }, -- middle top left
                { gaitCenter = Vector3(-10, -7.5, 0), segments = { segment0Left, segment1, segment2 } }, -- middle bottom left
                { gaitCenter = Vector3(-8.5, -7.5, -5), segments = { segment0Left, segment1, segment2 } }  -- bottom left
            })
end

-- WIP : adjust each leg height individually by checking the terrain height, acting as a manual suspension
Adjuster = {
    new = function(I, legs)
        return {
            comAdjust = Vector3.zero,
            maxOffset = legs[next(legs)].segments[2].len.z * 0.2,
            legOffsets = {},
            getForLeg = function(self, leg)
                local legOffset = self.legOffsets[leg]
                if (legOffset == nil) then
                    legOffset = Vector3.zero
                end
                return self.comAdjust + legOffset
            end,
            calculateLegOffsets = function(self, I, legTargets, t)
                local vehiculeWorldPosition = I:GetConstructCenterOfMass() + I:GetVelocityVector() * t
                --local vehiculeWorldRotation = Quaternion.LookRotation(I:GetConstructForwardVector(), I:GetConstructUpVector())
                local vehiculeWorldRotation = Quaternion.AngleAxis(I:GetConstructYaw(), Vector3.up)
                local n = 0
                local offset = 0
                self.legOffsets = {}
                for leg, target in pairs(legTargets) do
                    local targetLocalPosition = leg.position + target
                    local targetWorldPos = vehiculeWorldPosition + vehiculeWorldRotation * targetLocalPosition
                    local heightDiff = targetWorldPos.y - I:GetTerrainAltitudeForPosition(targetWorldPos.x, targetWorldPos.y, targetWorldPos.z)
                    self.legOffsets[leg] = heightDiff
                    offset = offset + heightDiff
                    n = n + 1
                end
                local meanOffset = offset / n
                local log = string.format("mean=%f, ", meanOffset)
                for leg, heightDiff in pairs(self.legOffsets) do
                    log = log .. string.format(" leg%d=%f ", leg.segments[1].spinId, heightDiff)
                end
                I:Log(log)
                for leg, heightDiff in pairs(self.legOffsets) do
                    local legOffset = heightDiff - meanOffset
                    if (math.abs(legOffset) > self.maxOffset) then
                        legOffset = self.maxOffset * legOffset / math.abs(legOffset)
                    end
                    self.legOffsets[leg] = Vector3(0, -legOffset, 0)
                end
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
                Walking = Gait.Walking.autoconfig(I, leg, -math.pi / 2),
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
        local fRatio = math.abs(forward)
        local sRatio = math.abs(strafe)
        local yRatio = math.abs(yaw)
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
                return (wTarget + yTarget) / 2
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
            --comAdjuster = Adjuster.new(I, legs),
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
                --self.comAdjuster:calculateLegOffsets(I, legTargets, curTime - self.lastTime)
                for leg, target in pairs(legTargets) do
                    --leg:moveLeg(I, target + self.comAdjuster:getForLeg(leg), curTime)
                    leg:moveLeg(I, target, curTime)
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

-- contains all possiblie gaits for a leg, with handy creation method
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
                                position.x + math.cos(te * math.pi * 2) * (width / 2),
                                position.y + math.sin(te * math.pi * 2) * height,
                                position.z)
                    end
                    return rotateAround(target, yAngle, position)
                end
            }
        end,
        autoconfig = function(I, leg, angle)
            local position
            if leg.gaitCenter == nil then
                local legDirection = I:GetSubConstructInfo(leg.segments[1].spinId).LocalPositionRelativeToCom
                position = legDirection.normalized * ((leg.segments[1].len.z + leg.segments[2].len.z) * 1.2)
                position.y = -leg.segments[3].len.z + leg.segments[1].len.y - 2
            else
                position = leg.gaitCenter
            end

            local actionRayon = math.sqrt(Mathf.Pow(position.y, 2) + Mathf.Pow(leg.length, 2)) - math.sqrt(position.x * position.x + position.z * position.z)

            I:Log(string.format("creating walking gait for leg %s, centered at %s with width %f", tostring(leg.position), tostring(position), actionRayon * 1.5))

            return Gait.Walking.new(position,
                    angle, math.max(leg.segments[2].len.z * 0.5, GAIT_MIN_HEIGHT), actionRayon * 1.5)
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
                        local th = (1 + math.cos(tv * math.pi * 2)) / 2;
                        return Vector3(
                                center.x + radius * math.cos(radStart + (radLength * th)),
                                center.y + math.sin(tv * math.pi * 2) * height,
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
                angleOffset = 2 * math.pi * Vector3.SignedAngle(Vector3.ProjectOnPlane(spinPosition, Vector3.up), Vector3.right, Vector3.up) / 360
            else
                rotCenter = Vector3(-spinPosition.x, leg.gaitCenter.y, -spinPosition.z)
                rotRadius = Vector3.Distance(rotCenter, leg.gaitCenter)
                angleOffset = 2 * math.pi * Vector3.SignedAngle(Vector3.ProjectOnPlane(spinPosition + leg.gaitCenter, Vector3.up), Vector3.right, Vector3.up) / 360
            end
            local Ta = math.sqrt(leg.gaitCenter.x * leg.gaitCenter.x + leg.gaitCenter.z * leg.gaitCenter.z);
            local actionRayon = math.sqrt(Mathf.Pow(leg.gaitCenter.y, 2) + Mathf.Pow(leg.length, 2)) - Ta

            local angleTurning = math.atan(actionRayon/Ta) * 0.5

            I:Log(string.format("creating turning gait for leg %s, rotating around %s with radius %f, turning %f and offset %f", tostring(leg.position), tostring(rotCenter), rotRadius, angleTurning, angleOffset))

            return Gait.Turning.new(rotCenter,
                    rotRadius,
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
                local angleDeg = 360 * angle / (2 * math.pi)
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
                    I:Log(string.format('built leg : position %s, gaitCenter %s, segment length %f', tostring(leg.position), tostring(leg.gaitCenter), leg.length))
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
        else
            ikModel = IkLib.moveInsectoidLeg
        end
        local leg = {
            position = sc.LocalPositionRelativeToCom,
            segments = {},
            moveLeg = ikModel,
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
    moveInsectoidLeg = function(leg, I, target, t)
        if target.magnitude > leg.length then
            I:LogToHud(string.format("ERROR : target %s is too far for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        if target.magnitude < leg.segments[1].len.z then
            I:LogToHud(string.format("ERROR : target %s is too close for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        local a0 = (target.z > 0 and -1 or 1) * math.acos(target.x / math.sqrt(target.x * target.x + target.z * target.z))

        local rx = target.x / math.cos(a0) - leg.segments[1].len.z
        local ry = target.y
        local rMag = math.sqrt(rx * rx + ry * ry)

        local a1 = (ry > 0 and 1 or -1) * math.acos(rx / rMag)
                + math.acos(
                (leg.segments[2].len.z * leg.segments[2].len.z + rMag * rMag - leg.segments[3].len.z * leg.segments[3].len.z)
                        / (2 * leg.segments[2].len.z * rMag))

        local a2 = -math.acos((rMag * rMag - leg.segments[2].len.z * leg.segments[2].len.z - leg.segments[3].len.z * leg.segments[3].len.z)
                / (2 * leg.segments[2].len.z * leg.segments[3].len.z))

        I:Log(string.format('t=%f : moving insectoid leg %d at %s, offset=%d, spinDir=%d to %s with angle a0=%f, a1=%f, a2=%f',
                t, leg.segments[1].spinId, tostring(leg.position), leg.segments[1].spinOffset, leg.segments[1].spinDirection, tostring(target), a0, a1, a2))

        leg.segments[1]:setAngle(I, a0, t)
        leg.segments[2]:setAngle(I, a1, t)
        leg.segments[3]:setAngle(I, a2, t)
    end,
    moveHumanoidLeg = function(leg, I, target, t)
        if target.magnitude > leg.length then
            I:LogToHud(string.format("ERROR : target %s is too far for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        if target.magnitude < leg.segments[1].len.z then
            I:LogToHud(string.format("ERROR : target %s is too close for leg %s !", tostring(target), leg.segments[1].spinId))
        end

        local a0 = (target.x > 0 and 1 or -1) * math.acos(-target.y / math.sqrt(target.x * target.x + target.y * target.y))

        local rx = -target.y / math.cos(a0) - leg.segments[1].len.z
        local ry = -target.z
        local rMag = math.sqrt(rx * rx + ry * ry)

        local a1 = (ry > 0 and 1 or -1) * math.acos(rx / rMag)
                + math.acos(
                (leg.segments[2].len.z * leg.segments[2].len.z + rMag * rMag - leg.segments[3].len.z * leg.segments[3].len.z)
                        / (2 * leg.segments[2].len.z * rMag))

        local a2 = -math.acos((rMag * rMag - leg.segments[2].len.z * leg.segments[2].len.z - leg.segments[3].len.z * leg.segments[3].len.z)
                / (2 * leg.segments[2].len.z * leg.segments[3].len.z))

        I:Log(string.format('t=%f : moving humanoid leg %d at %s, offset=%d, spinDir=%d to %s with angle a0=%f, a1=%f, a2=%f',
                t, leg.segments[1].spinId, tostring(leg.position), leg.segments[1].spinOffset, leg.segments[1].spinDirection, tostring(target), a0, a1, a2))

        leg.segments[1]:setAngle(I, a0, t)
        leg.segments[2]:setAngle(I, a1, t)
        leg.segments[3]:setAngle(I, a2, t)
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
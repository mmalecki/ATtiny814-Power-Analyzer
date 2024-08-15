import cq_queryabolt as queryabolt
import cadquery as cq

class Workplane(queryabolt.WorkplaneMixin, cq.Workplane):
    pass

fit = 0.1
t = 1.6

# Chamfer
c = 1
f = 5

bolt = "M2"
boltD = queryabolt.boltData(bolt)["diameter"]
boltWallT = 1.2
boltWallD = boltD + 2 * boltWallT

nutH = queryabolt.nutData(bolt)["thickness"]
nutW = queryabolt.nutData(bolt)["width"]
nutWallT = 1.6
nutWallH = nutH + 1.6
nutWallD = nutW + 1.6


mountW = 48.273
mountL = 40.640

padding = 5
wallT = 1.6

fanW = 30
fanMountW = 24

standoffH = 2.8

# An educated guess at a maximum size of a heatsink someone might want to put here
heatsinkClearance = 20 

fanMountT = 1.5 * nutWallH

fanOffset = 2.40

w = mountW + 2 * padding + 2 * wallT
l = mountL + heatsinkClearance + 2 * padding + 2 * wallT + (fanMountT - wallT) / 2

def mount(workplane):
    return workplane.rect(mountW, mountL).vertices()

def bottom():
    plate = Workplane("XY").rect(w, l).slot2D(20, 2, 90).extrude(2 * t).edges("|Z").fillet(f)

    plate.faces(">Z").workplane().tag("top")
    plate.faces(">Z").workplane(-t).tag("basketTop")
    plate.faces("<Z").workplane().tag("bottom")

    plate = plate.faces(">Z").edges().toPending().offset2D(-wallT).extrude(-t, combine='cut')

    plate = plate.workplaneFromTagged("basketTop").move(fanOffset, l / 2 - wallT / 2).rect(fanW, 1.5 * nutWallH).extrude(nutWallD)
    # Fan mount, of which I am not a fan
    plate = plate.faces(">Y").workplane().center(-fanOffset,nutWallD / 2).pushPoints([(fanMountW / 2, 0), (-fanMountW / 2, 0)]).boltHole(bolt)
    plate = plate.faces("<Y[3]").workplane().pushPoints([(fanMountW / 2, 0), (-fanMountW / 2, 0)]).nutcatchParallel(bolt)
    plate = plate.faces(">Y").workplane().pushPoints([(fanMountW / 2, 0), (-fanMountW / 2, 0)]).nutcatchParallel(bolt)
    plate = (plate.faces("<Z[2]").faces(">Y").edges("<Y").chamfer(t - 0.001))
    plate = plate.faces(">Z").workplane(centerOption="CenterOfMass").rect(fanW / 2, 1.5 * nutWallH).extrude(-nutWallD/2, combine='cut')


    # PCB standoffs and nutcatches
    plate = mount(plate.workplaneFromTagged("basketTop").move(0, -heatsinkClearance / 2)).cylinder(standoffH, boltWallD * 3/4, centered=(True, True, False))
    plate = mount(plate.workplaneFromTagged("bottom").move(0, heatsinkClearance / 2)).boltHole(bolt, clearance=fit)
    plate = mount(plate.workplaneFromTagged("bottom").move(0, heatsinkClearance / 2)).nutcatchParallel(bolt)
    plate = plate.edges(">Z and |Y").fillet(nutWallH / 4)
    plate = plate.edges("|Z and >Y").edges(">>X or <<X").chamfer(0.45)

    plate = plate.faces(">Z[1]").edges("%Circle").edges(cq.selectors.RadiusNthSelector(2)).fillet(boltWallD / 4)

    return plate

show_object(bottom(), name="bottom")
pcb = cq.importers.importStep("../hardware/PowerAnalyzer.step")
show_object(pcb.translate((0, -heatsinkClearance / 2, standoffH + t)), name="pcb")

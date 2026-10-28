# -*- coding: utf-8 -*-
"""Build analytical-rigid compression baselines from project JSON.

Run from a prepared case directory with case.json to build one case. Run from
the project root without case.json to build all configured cases.
No analysis is submitted by this model-generation script.
"""
from abaqus import *
from abaqusConstants import *
import math, os, json, re, sys
import regionToolset, mesh
for _module_name in ('part', 'material', 'section', 'assembly', 'step',
                     'interaction', 'load', 'mesh', 'job', 'sketch'):
    try:
        __import__(_module_name)
    except ImportError:
        pass

PROJECT = globals().get('PROJECT_DIR')
if not PROJECT:
    import inspect
    PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(
        inspect.currentframe().f_code.co_filename)))
GEOMETRY = os.path.join(PROJECT, 'geometry', 'v2_silicone.step')
MAGNETS = os.path.join(PROJECT, 'config', 'magnets.json')
CASES = os.path.join(PROJECT, 'config', 'indenter_cases.json')
OUT_ROOT = os.path.join(PROJECT, 'abaqus_output', 'analytic_rigid_cavities_mu05_cpus4')
POLE_OFFSET_MM = 0.001

def bbox(obj):
    try:
        box = obj.cells.getBoundingBox()
    except Exception:
        box = obj.vertices.getBoundingBox()
    return tuple(box['low']), tuple(box['high'])

def top_faces(part):
    high = bbox(part)[1]
    selected = None
    for index in range(len(part.faces)):
        face = part.faces[index:index + 1]
        box = face.getBoundingBox()
        if (box['high'][2] >= high[2] - 0.05 and
                box['high'][0] - box['low'][0] > 0.05 and
                box['high'][1] - box['low'][1] > 0.05):
            selected = face if selected is None else selected + face
    if selected is None:
        raise RuntimeError('Could not identify silicone upper surface')
    return selected

def contact_frame(part, target_angle, row_y):
    upper = top_faces(part)
    high_z = bbox(part)[1][2]
    def project(x):
        face, point = upper.getClosest(
            coordinates=((x, row_y, high_z + 0.1),),
            searchTolerance=100.0)[0]
        point = tuple(point)
        normal = tuple(face.getNormal(point=point))
        if normal[2] < 0.0:
            normal = tuple(-value for value in normal)
        norm = math.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2)
        return point, tuple(value / norm for value in normal)
    if abs(target_angle) < 1.0e-8:
        point, normal = project(0.0)
    else:
        sign = 1.0 if target_angle > 0.0 else -1.0
        low, high = 0.0, 25.0
        for unused in range(60):
            middle = 0.5 * (low + high)
            point, normal = project(sign * middle)
            angle = math.degrees(math.atan2(normal[0], normal[2]))
            if abs(angle) < abs(target_angle):
                low = middle
            else:
                high = middle
        point, normal = project(sign * 0.5 * (low + high))
    actual_angle = math.degrees(math.atan2(normal[0], normal[2]))
    if abs(actual_angle - target_angle) > 0.5:
        raise RuntimeError('Requested surface normal %.2f deg; found %.2f deg' %
                           (target_angle, actual_angle))
    return point, normal

def position_cylinder(assembly, name, center, direction, height):
    dx, dy, dz = direction
    axis = (-dy, dx, 0.0)
    length = math.sqrt(axis[0] ** 2 + axis[1] ** 2)
    if length > 1.0e-10:
        assembly.rotate(instanceList=(name,), axisPoint=(0, 0, 0),
                        axisDirection=(axis[0] / length, axis[1] / length, 0),
                        angle=math.degrees(math.atan2(length, dz)))
    elif dz < 0:
        assembly.rotate(instanceList=(name,), axisPoint=(0, 0, 0),
                        axisDirection=(1, 0, 0), angle=180.0)
    assembly.translate(instanceList=(name,), vector=tuple(
        center[i] - 0.5 * height * direction[i] for i in range(3)))

def create_cavities(model, silicone, records, radius, height):
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    assembly.Instance(name='silicone-raw', part=silicone, dependent=ON)
    sketch = model.ConstrainedSketch(name='Sketch-cutter', sheetSize=50.0)
    sketch.CircleByCenterPerimeter(center=(0, 0), point1=(radius, 0))
    cutter = model.Part(name='magnet-cutter', dimensionality=THREE_D,
                        type=DEFORMABLE_BODY)
    cutter.BaseSolidExtrude(sketch=sketch, depth=height)
    del model.sketches['Sketch-cutter']
    names = []
    for name, center, direction in records:
        instance_name = 'cutter-' + name
        assembly.Instance(name=instance_name, part=cutter, dependent=ON)
        position_cylinder(assembly, instance_name, center, direction, height)
        names.append(instance_name)
    assembly.regenerate()
    result = assembly.InstanceFromBooleanCut(
        name='silicone-with-cavities',
        instanceToBeCut=assembly.instances['silicone-raw'],
        cuttingInstances=tuple(assembly.instances[name] for name in names),
        originalInstances=DELETE)
    assembly.regenerate()
    cut_part = model.parts[result.partName]
    assembly.deleteFeatures((result.name,))
    return cut_part

def create_flat_plate(model, width, length):
    # A 3D analytical extruded surface is infinite along its extrusion axis.
    # The finite profile spans the 25 mm sensor width; the sensor itself is
    # contained within that width, so no plate edge enters the contact region.
    sketch = model.ConstrainedSketch(name='Sketch-flat-plate', sheetSize=60.0)
    sketch.Line(point1=(-width / 2.0, 0.0), point2=(width / 2.0, 0.0))
    part = model.Part(name='indenter-flat-%g' % width,
                      dimensionality=THREE_D, type=ANALYTIC_RIGID_SURFACE)
    part.AnalyticRigidSurfExtrude(sketch=sketch, depth=length)
    part.ReferencePoint(point=(0.0, 0.0, 0.0))
    part.Surface(name='Surf-indenter', side1Faces=part.faces[:])
    del model.sketches['Sketch-flat-plate']
    return part

def create_sphere(model, diameter):
    radius = diameter / 2.0
    sketch = model.ConstrainedSketch(name='Sketch-sphere', sheetSize=30.0)
    sketch.ConstructionLine(point1=(0.0, -radius), point2=(0.0, radius))
    pole_y = math.sqrt(radius * radius - POLE_OFFSET_MM * POLE_OFFSET_MM)
    lower_pole = (POLE_OFFSET_MM, -pole_y)
    equator = (radius, 0.0)
    sketch.ArcByCenterEnds(center=(0.0, 0.0), point1=lower_pole,
                           point2=equator, direction=COUNTERCLOCKWISE)
    part = model.Part(name='indenter-sphere-%g' % diameter,
                      dimensionality=THREE_D, type=ANALYTIC_RIGID_SURFACE)
    part.AnalyticRigidSurfRevolve(sketch=sketch)
    part.ReferencePoint(point=(0.0, 0.0, 0.0))
    part.Surface(name='Surf-indenter', side1Faces=part.faces[:])
    del model.sketches['Sketch-sphere']
    return part

def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])

def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def norm(a):
    return math.sqrt(dot(a, a))

def rotate_flat_to_frame(assembly, instance_name, normal):
    u, v = perpendicular_basis(normal)
    local_x = (1.0, 0.0, 0.0)
    axis = cross(local_x, u)
    axis_norm = norm(axis)
    if axis_norm > 1.0e-10:
        assembly.rotate(instanceList=(instance_name,), axisPoint=(0, 0, 0),
                        axisDirection=tuple(value / axis_norm for value in axis),
                        angle=math.degrees(math.atan2(axis_norm, dot(local_x, u))))
    elif dot(local_x, u) < 0.0:
        assembly.rotate(instanceList=(instance_name,), axisPoint=(0, 0, 0),
                        axisDirection=(0, 0, 1), angle=180.0)
    local_z = (0.0, 0.0, 1.0)
    first_axis = tuple(value / axis_norm for value in axis) if axis_norm > 1.0e-10 else (0, 0, 1)
    first_angle = (math.atan2(axis_norm, dot(local_x, u))
                   if axis_norm > 1.0e-10 else (math.pi if dot(local_x, u) < 0.0 else 0.0))
    z_rotated = tuple(
        local_z[i] * math.cos(first_angle) +
        cross(first_axis, local_z)[i] * math.sin(first_angle) +
        first_axis[i] * dot(first_axis, local_z) * (1.0 - math.cos(first_angle))
        for i in range(3))
    twist = math.atan2(dot(cross(z_rotated, v), u), dot(z_rotated, v))
    if abs(twist) > 1.0e-10:
        assembly.rotate(instanceList=(instance_name,), axisPoint=(0, 0, 0),
                        axisDirection=u, angle=math.degrees(twist))

def add_restart_writes(input_path, step_count):
    with open(input_path, 'r') as stream:
        lines = stream.readlines()
    output = []
    current_step = None
    compression_steps = set('STEP-COMPRESSION-%02d' % (i + 1)
                            for i in range(step_count))
    for line in lines:
        stripped = line.strip().upper()
        if stripped.startswith('*STEP'):
            match = re.search(r'NAME\s*=\s*([^,]+)', stripped)
            current_step = match.group(1).strip() if match else None
        if stripped.startswith('*RESTART') and current_step in compression_steps:
            continue
        if stripped.startswith('*END STEP') and current_step in compression_steps:
            output.append('*Restart, write, frequency=0\n')
        output.append(line)
        if stripped.startswith('*END STEP'):
            current_step = None
    with open(input_path, 'w') as stream:
        stream.writelines(output)

def rotate_about_y(assembly, instance_name, angle):
    if abs(angle) > 1.0e-10:
        assembly.rotate(instanceList=(instance_name,), axisPoint=(0, 0, 0),
                        axisDirection=(0, 1, 0), angle=angle)

def rotate_sphere_axis_to_normal(assembly, instance_name, normal):
    local_axis = (0.0, 1.0, 0.0)
    axis = cross(local_axis, normal)
    axis_norm = norm(axis)
    angle = math.degrees(math.atan2(axis_norm, dot(local_axis, normal)))
    if axis_norm > 1.0e-10:
        assembly.rotate(instanceList=(instance_name,), axisPoint=(0, 0, 0),
                        axisDirection=tuple(value / axis_norm for value in axis),
                        angle=angle)
    elif dot(local_axis, normal) < 0.0:
        assembly.rotate(instanceList=(instance_name,), axisPoint=(0, 0, 0),
                        axisDirection=(1, 0, 0), angle=180.0)

def perpendicular_basis(direction):
    seed = (0.0, 0.0, 1.0) if abs(direction[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = (seed[1] * direction[2] - seed[2] * direction[1],
         seed[2] * direction[0] - seed[0] * direction[2],
         seed[0] * direction[1] - seed[1] * direction[0])
    norm = math.sqrt(u[0] ** 2 + u[1] ** 2 + u[2] ** 2)
    u = tuple(value / norm for value in u)
    v = (direction[1] * u[2] - direction[2] * u[1],
         direction[2] * u[0] - direction[0] * u[2],
         direction[0] * u[1] - direction[1] * u[0])
    return u, v

def build_case(case, shared, magnet_records, radius, height, output_root,
               include_magnet_cavities=True):
    case_id = case['case_id']
    target_dir = output_root if os.path.isfile(prepared_case_path) else os.path.join(output_root, case_id)
    if os.path.isfile(os.path.join(target_dir, 'Job-' + case_id + '.inp')):
        raise RuntimeError('Input already exists; prepare a new output directory: ' + target_dir)
    model_name = 'Model-' + case_id
    if model_name in mdb.models.keys():
        del mdb.models[model_name]
    model = mdb.Model(name=model_name, modelType=STANDARD_EXPLICIT)
    geometry = mdb.openStep(GEOMETRY, scaleFromFile=OFF)
    model.PartFromGeometryFile(name='silicone', geometryFile=geometry,
                               bodyNum=1, combine=False,
                               dimensionality=THREE_D, type=DEFORMABLE_BODY)
    silicone = model.parts['silicone']
    sil_low, sil_high = bbox(silicone)
    if include_magnet_cavities:
        silicone = create_cavities(model, silicone, magnet_records, radius, height)
    assembly = model.rootAssembly
    assembly.Instance(name='silicone-1', part=silicone, dependent=ON)

    indenter = case['indenter']
    if indenter['type'] == 'sphere':
        part = create_sphere(model, float(indenter['diameter_mm']))
        characteristic = float(indenter['diameter_mm']) / 2.0
        analytical_indenter = True
    elif indenter['type'] == 'rectangular_flat':
        part = create_flat_plate(model, float(indenter['width_mm']),
                                 float(indenter['length_mm']))
        characteristic = 0.0
        analytical_indenter = True
    else:
        raise RuntimeError('Unsupported indenter type: ' + indenter['type'])
    assembly.Instance(name='indenter-1', part=part, dependent=ON)
    angle = float(case['surface_position']['normal_angle_deg'])
    y = float(case['surface_position']['y_mm'])
    surface_point, normal = contact_frame(silicone, angle, y)
    if indenter['type'] == 'sphere':
        rotate_sphere_axis_to_normal(assembly, 'indenter-1', normal)
    else:
        rotate_flat_to_frame(assembly, 'indenter-1', normal)
    if indenter['type'] == 'sphere':
        offset = float(shared['initial_gap_mm']) + characteristic
    else:
        offset = float(shared['initial_gap_mm'])
    translation = tuple(surface_point[i] + offset * normal[i]
                        for i in range(3))
    assembly.translate(instanceList=('indenter-1',), vector=translation)

    if include_magnet_cavities:
        for name, center, direction in magnet_records:
            feature = assembly.ReferencePoint(point=center)
            assembly.Set(name='Set-' + name + '-rp',
                         referencePoints=(assembly.referencePoints[feature.id],))
    low, high = bbox(assembly.instances['indenter-1'])
    part_rp_id = part.referencePoints.keys()[0]
    assembly.Set(name='Set-indenter-rp', referencePoints=(
        assembly.instances['indenter-1'].referencePoints[part_rp_id],))
    # Materials and sections (N-mm-MPa).
    model.Material(name='Material-silicone')
    model.materials['Material-silicone'].Hyperelastic(
        materialType=ISOTROPIC, testData=OFF, type=NEO_HOOKE,
        volumetricResponse=VOLUMETRIC_DATA, table=((0.0265, 0.39),))
    model.Material(name='Material-steel')
    model.materials['Material-steel'].Elastic(table=((210000.0, 0.30),))
    model.HomogeneousSolidSection(name='Section-silicone',
                                  material='Material-silicone', thickness=None)
    silicone.SectionAssignment(region=silicone.Set(name='Set-silicone-body', cells=silicone.cells[:]),
                               sectionName='Section-silicone')
    # The analytical rigid part associates its surface with its part-level RP.
    low_s, high_s = bbox(assembly.instances['silicone-1'])
    bottom_faces = assembly.instances['silicone-1'].faces.getByBoundingBox(
        zMin=low_s[2] - 0.05, zMax=low_s[2] + 0.05)
    assembly.Set(name='Set-silicone-bottom', faces=bottom_faces)
    base = assembly.ReferencePoint(point=(0.0, 0.0, low_s[2]))
    assembly.Set(name='Set-bottom-rp', referencePoints=(assembly.referencePoints[base.id],))
    assembly.Surface(name='Surf-bottom', side1Faces=bottom_faces)
    model.Coupling(name='Coupling-bottom',
                   controlPoint=assembly.sets['Set-bottom-rp'],
                   surface=assembly.surfaces['Surf-bottom'],
                   influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC,
                   localCsys=None, u1=ON, u2=ON, u3=ON,
                   ur1=ON, ur2=ON, ur3=ON)
    model.EncastreBC(name='BC-bottom', createStepName='Initial',
                     region=assembly.sets['Set-bottom-rp'])
    # Couple each cavity wall to its magnet-center RP for the baseline model.
    for name, center, direction in (magnet_records if include_magnet_cavities else []):
        u, v = perpendicular_basis(direction)
        probes = [tuple(center[i] + radius * vec[i] for i in range(3))
                  for vec in (u, v, tuple(-x for x in u), tuple(-x for x in v))]
        probes.extend(tuple(center[i] + sign * 0.5 * height * direction[i]
                            for i in range(3)) for sign in (-1.0, 1.0))
        selected = []
        selected_indices = set()
        for point in probes:
            face = assembly.instances['silicone-1'].faces.findAt((point,))[0]
            face_index = assembly.instances['silicone-1'].faces.index(face)
            if face_index not in selected_indices:
                selected.append(assembly.instances['silicone-1'].faces[face_index:face_index + 1])
                selected_indices.add(face_index)
        if not selected:
            raise RuntimeError('No cavity faces found for ' + name)
        cavity_faces = selected[0]
        for face_sequence in selected[1:]:
            cavity_faces = cavity_faces + face_sequence
        surface_name = 'Surf-' + name + '-cavity'
        assembly.Surface(name=surface_name, side1Faces=cavity_faces)
        model.Coupling(name='Coupling-' + name,
                       controlPoint=assembly.sets['Set-' + name + '-rp'],
                       surface=assembly.surfaces[surface_name],
                       influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC,
                       localCsys=None, u1=ON, u2=ON, u3=ON,
                       ur1=ON, ur2=ON, ur3=ON)
    # Contact uses the complete outer surfaces; the bottom is fixed separately.
    model.ContactProperty('IntProp-contact')
    model.interactionProperties['IntProp-contact'].NormalBehavior(
        pressureOverclosure=HARD, allowSeparation=ON)
    model.interactionProperties['IntProp-contact'].TangentialBehavior(
        formulation=PENALTY, directionality=ISOTROPIC,
        table=((float(shared.get('friction_coefficient', 0.5)),),),
        maximumElasticSlip=FRACTION, fraction=0.005)
    model.StaticStep(name='Step-contact', previous='Initial', maxNumInc=100,
                     initialInc=0.1, minInc=1.0e-5, maxInc=1.0, nlgeom=ON)
    sil_face_indices = []
    for face_index in range(len(assembly.instances['silicone-1'].faces)):
        face_box = assembly.instances['silicone-1'].faces[
            face_index:face_index + 1].getBoundingBox()
        if face_box['high'][2] > low_s[2] + 0.05:
            sil_face_indices.append(face_index)
    sil_faces = assembly.instances['silicone-1'].faces[
        sil_face_indices[0]:sil_face_indices[0] + 1]
    for face_index in sil_face_indices[1:]:
        sil_faces = sil_faces + assembly.instances['silicone-1'].faces[
            face_index:face_index + 1]
    indenter_surface = assembly.instances['indenter-1'].surfaces['Surf-indenter']
    assembly.Surface(name='Surf-silicone', side1Faces=sil_faces)
    try:
        model.SurfaceToSurfaceContactStd(
            name='Int-contact', createStepName='Step-contact',
            main=indenter_surface,
            secondary=assembly.surfaces['Surf-silicone'], sliding=FINITE,
            thickness=ON, interactionProperty='IntProp-contact')
    except TypeError:
        model.SurfaceToSurfaceContactStd(
            name='Int-contact', createStepName='Step-contact',
            master=indenter_surface,
            slave=assembly.surfaces['Surf-silicone'], sliding=FINITE,
            thickness=ON, interactionProperty='IntProp-contact')
    # Each baseline closes the initial gap, then compresses to six exact
    # checkpoint depths. Shear is created later as independent restart jobs.
    checkpoints = [float(value) for value in shared['compression_checkpoints_mm']]
    previous = 'Step-contact'
    for index, checkpoint in enumerate(checkpoints):
        name = 'Step-compression-%02d' % (index + 1)
        model.StaticStep(name=name, previous=previous, maxNumInc=100,
                         initialInc=0.02, minInc=1.0e-5, maxInc=0.1, nlgeom=ON)
        previous = name
    model.DisplacementBC(name='BC-indenter', createStepName='Step-contact',
                         region=assembly.sets['Set-indenter-rp'],
                         u1=0.0, u2=0.0, u3=0.0, ur1=0.0, ur2=0.0, ur3=0.0)
    gap = float(shared['initial_gap_mm'])
    model.boundaryConditions['BC-indenter'].setValuesInStep(
        stepName='Step-contact', u1=-gap * normal[0], u3=-gap * normal[2])
    for index, checkpoint in enumerate(checkpoints):
        model.boundaryConditions['BC-indenter'].setValuesInStep(
            stepName='Step-compression-%02d' % (index + 1),
            u1=-(gap + checkpoint) * normal[0],
            u3=-(gap + checkpoint) * normal[2])
    # Mesh the deformable silicone only; analytical indenters do not use elements.
    silicone.seedPart(size=1.0, deviationFactor=0.1, minSizeFactor=0.1)
    silicone.setMeshControls(regions=silicone.cells[:], elemShape=TET,
                             technique=FREE)
    silicone.setElementType(regions=(silicone.cells[:],),
                            elemTypes=(mesh.ElemType(elemCode=C3D10H,
                                                      elemLibrary=STANDARD),))
    silicone.generateMesh()
    # Field/history output and end-of-step restart records.
    model.FieldOutputRequest(name='F-Output-1', createStepName='Step-contact',
                             variables=('U', 'RF', 'S', 'LE', 'CSTRESS', 'CDISP', 'CSTATUS'))
    model.HistoryOutputRequest(name='H-Output-indenter', createStepName='Step-contact',
                               variables=('U1', 'U2', 'U3', 'RF1', 'RF2', 'RF3'),
                               region=assembly.sets['Set-indenter-rp'])
    model.HistoryOutputRequest(name='H-Output-bottom', createStepName='Step-contact',
                               variables=('RF1', 'RF2', 'RF3'),
                               region=assembly.sets['Set-bottom-rp'])
    if include_magnet_cavities:
        for name, center, direction in magnet_records:
            model.HistoryOutputRequest(
                name='H-Output-' + name + '-rp', createStepName='Step-contact',
                variables=('U1', 'U2', 'U3', 'UR1', 'UR2', 'UR3'),
                region=assembly.sets['Set-' + name + '-rp'])
    assembly.regenerate()
    output_dir = output_root if os.path.isfile(prepared_case_path) else os.path.join(output_root, case_id)
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    job_name = 'Job-' + case_id
    job = mdb.Job(name=job_name, model=model_name,
                  description='Analytical rigid compression baseline')
    old_cwd = os.getcwd()
    try:
        os.chdir(output_dir)
        job.writeInput(consistencyChecking=OFF)
        add_restart_writes(os.path.join(output_dir, job_name + '.inp'),
                           len(checkpoints))
    finally:
        os.chdir(old_cwd)
    path = os.path.join(output_dir, case_id + '_complete.cae')
    mdb.saveAs(pathName=path)
    print('Built %s: %s' % (case_id, path))
    print('  indenter bbox: %s - %s' % (str(low), str(high)))
    return path

with open(MAGNETS, 'r') as stream:
    magnet_config = json.load(stream)
with open(CASES, 'r') as stream:
    case_config = json.load(stream)
geom = magnet_config['magnet_geometry']
radius = float(geom['diameter']) / 2.0
height = float(geom['height'])
records = []
for item in magnet_config['magnets']:
    direction = tuple(float(value) for value in item['direction'])
    direction_norm = math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)
    records.append((item['id'], tuple(float(value) for value in item['position']),
                    tuple(value / direction_norm for value in direction)))

shared_loading = dict(case_config['shared_loading'])
prepared_case_path = os.path.join(os.getcwd(), 'case.json')
if os.path.isfile(prepared_case_path):
    with open(prepared_case_path, 'r') as stream:
        prepared_case = json.load(stream)
    case_id = prepared_case['case_id']
    selected_cases = [case for case in case_config['cases']
                      if case['case_id'] == case_id]
    if len(selected_cases) != 1:
        raise RuntimeError('case.json case_id is not configured: ' + case_id)
    for key, target in (('initial_gap_mm', 'initial_gap_mm'),
                        ('depth_mm', 'compression_depth_mm'),
                        ('friction', 'friction_coefficient'),
                        ('compression_checkpoints_mm', 'compression_checkpoints_mm')):
        if key in prepared_case:
            shared_loading[target] = prepared_case[key]
    include_magnet_cavities = bool(prepared_case.get('include_magnet_cavities', True))
    checkpoints = shared_loading.get('compression_checkpoints_mm', [])
    if (not checkpoints or
            abs(float(checkpoints[-1]) -
                float(shared_loading.get('compression_depth_mm', checkpoints[-1]))) > 1.0e-8):
        raise RuntimeError('Compression checkpoints must end at compression depth')
    output_root = os.getcwd()
else:
    selected_cases = case_config['cases']
    output_root = OUT_ROOT
    include_magnet_cavities = True

for case in selected_cases:
    build_case(case, shared_loading, records, radius, height, output_root,
               include_magnet_cavities)
print('Generated %d analytical-rigid compression models; no jobs submitted.' %
      len(selected_cases))

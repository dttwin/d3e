from sumolib.xml import parse_fast

d = parse_fast("../evropska_initialdemand.rou.xml", "vehicle", ["type"])
vehicle_set = set()
for vt in d:
    vehicle_set.add(vt.type)
for vt in sorted(vehicle_set):
    nc = list(vt.split('_'))
    del nc[1]
    ec = '_'.join(nc)
    print(f'    <vType class="passenger" id="{vt}" emissionClass="{ec}"/>')


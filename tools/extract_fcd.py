import lzma
from lxml import etree as et
from collections import defaultdict

VEHICLE_IDS = [
    '6.9'
]
BASE_FILE = 'notts_emission.out.xml.xz'

data = defaultdict(list)
with lzma.open(BASE_FILE) as cf:

    for event, element in et.iterparse(cf, tag="timestep"):
        time = float(element.get('time'))
        for child in element:
            vid = child.get('id')
            if vid in VEHICLE_IDS:
                data[vid].append([time] + list(child.attrib.values()))
        element.clear()

print(data)

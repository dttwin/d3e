from lxml import etree as et
import posixpath


DET_OUTPUT = posixpath.join('outputs', 'dets')

tree = et.parse('det_fixed.NET.add.xml')
root = tree.getroot()

for det in root:
    det_id = det.attrib['id']
    det_seq = det_id.split('-')
    intersection_id = det_seq[0]
    detector_id = det_seq[2]
    new_det_id = intersection_id + '_' + detector_id
    det.attrib['id'] = new_det_id
    det.attrib['freq'] = '60'
    det.attrib['file'] = posixpath.join(DET_OUTPUT, new_det_id + '.out.xml')
    print(new_det_id)

tree.write('evropska.det.add.xml', pretty_print=True)



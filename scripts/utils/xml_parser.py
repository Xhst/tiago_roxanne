import xml.etree.ElementTree as ET


class XMLParser:

    def __init__(self, xml_file):
        self.et = ET.parse(xml_file)

        if (self.et is None):
            raise Exception('Unable to parse XML document.')
        
        self.root = self.et.getroot()


    def has_attributes(self, element, attributes):
        if element is None or attributes is None:
            return False
        
        for k, v in attributes.items():
            if k not in element.attrib or element.attrib[k] != v:
                return False
        return True
    

    def find_element_from_root(self, tag, attributes = {}):
        return self.find_element(self.root, tag, attributes)


    def find_element(self, root, tag, attributes = {}):
        for child in root:
            if child.tag == tag and self.has_attributes(child, attributes):
                return child
            res = self.find_element(child, tag, attributes)
            if res is not None:
                return res
        return None 

"""Export the B9-8 STEP assembly for MuJoCo (optional cadquery-ocp dependency).

Run with: python cad/export_b98_sim.py
STEPCAF preserves occurrence names and assembly placements. OpenCascade reads
STEP length units as millimetres; exported coordinates and inertia use metres.
"""
from pathlib import Path
import gzip
import hashlib
import json

from OCP.BRep import BRep_Tool
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label

ROOT = Path(__file__).resolve().parents[1]


def main():
    source = ROOT / 'cad/B9_8/Robot_B9_8.step'
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise RuntimeError('Could not read B9-8 STEP assembly')
    doc = TDocStd_Document(TCollection_ExtendedString('B9-8 simulation export'))
    if not reader.Transfer(doc):
        raise RuntimeError('Could not transfer B9-8 STEP assembly')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    records = []

    def visit(label, parent_location):
        target = label
        if tool.IsReference_s(label):
            target = TDF_Label()
            tool.GetReferredShape_s(label, target)
        location = parent_location.Multiplied(tool.GetLocation_s(label))
        children = Sequence_TDF_Label()
        tool.GetComponents_s(target, children)
        if children.Length():
            for i in range(1, children.Length() + 1):
                visit(children.Value(i), location)
            return
        attr = TDataStd_Name()
        if not target.FindAttribute(TDataStd_Name.GetID_s(), attr):
            raise ValueError('Unnamed STEP component')
        name = attr.Get().ToExtString()
        shape = tool.GetShape_s(target).Located(location)
        solids = TopExp_Explorer(shape, TopAbs_SOLID)
        solid_index = 0
        while solids.More():
            solid = solids.Current()
            BRepMesh_IncrementalMesh(solid, 0.1, False, 0.2, False)
            vertices, indices = [], []
            faces = TopExp_Explorer(solid, TopAbs_FACE)
            while faces.More():
                face = TopoDS.Face(faces.Current())
                loc = TopLoc_Location()
                mesh = BRep_Tool.Triangulation_s(face, loc)
                if mesh is None:
                    raise ValueError(f'No tessellation for {name}')
                start = len(vertices)
                for i in range(1, mesh.NbNodes() + 1):
                    p = mesh.Node(i).Transformed(loc.Transformation())
                    vertices.append([p.X() / 1000, p.Y() / 1000, p.Z() / 1000])
                for i in range(1, mesh.NbTriangles() + 1):
                    tri = list(mesh.Triangle(i).Get())
                    if face.Orientation() == TopAbs_REVERSED:
                        tri.reverse()
                    indices.extend(start + j - 1 for j in tri)
                faces.Next()
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(solid, props)
            volume = props.Mass()
            if volume <= 0 or not indices:
                raise ValueError(f'Empty or reversed solid: {name}')
            com = props.CentreOfMass()
            inertia = props.MatrixOfInertia()
            records.append(dict(
                component=name, body=f'solid_{solid_index}', vertices=vertices,
                indices=indices, volume_m3=volume / 1e9,
                com_m=[com.X()/1000, com.Y()/1000, com.Z()/1000],
                # Inertia per unit mass about the solid COM, in m^2.
                unit_inertia_m2=[[inertia.Value(i, j) / volume / 1e6
                                  for j in range(1, 4)] for i in range(1, 4)],
            ))
            solid_index += 1
            solids.Next()
        if not solid_index:
            raise ValueError(f'No solids in {name}')

    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        visit(roots.Value(i), TopLoc_Location())
    export = dict(revision='B9-8', source=str(source.relative_to(ROOT)),
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  tessellation_tolerance_m=0.0001, bodies=records)
    output = ROOT / 'reference/b98_cad_meshes.json.gz'
    # Fixed gzip timestamp makes repeat exports reproducible.
    with output.open('wb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as zipped:
            zipped.write(json.dumps(export, separators=(',', ':')).encode())
    print(f'Exported {len(records)} B9-8 CAD solids to {output.relative_to(ROOT)}')


if __name__ == '__main__':
    main()
